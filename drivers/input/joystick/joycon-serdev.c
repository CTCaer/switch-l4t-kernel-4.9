// SPDX-License-Identifier: GPL-2.0+
/*
 * serial driver for Nintendo Switch Joy-Cons
 *
 * Copyright (c) 2019-2020 Daniel J. Ogorchock <djogorchock@gmail.com>
 * Copyright (c) 2021-2023 CTCaer <ctcaer@gmail.com>
 *
 * The following resources/projects were referenced for this driver:
 *   https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 *   https://gitlab.com/pjranki/joycon-linux-kernel (Peter Rankin)
 *   https://github.com/FrotBot/SwitchProConLinuxUSB
 *   https://github.com/MTCKC/ProconXInput
 *   https://github.com/Davidobot/BetterJoyForCemu
 *   hid-wiimote kernel hid driver
 *   hid-logitech-hidpp driver
 *   hid-sony driver
 *
 * This driver supports the Nintendo Switch Joy-Cons and Lite gamepad over uart.
 *
 * Much of this driver is ported from the hid-nintendo usb/bt joy-con driver.
 *
 * Large portions are based on Max Thomas' joycon.c serdev driver:
 * Copyright (c) 2018 Max Thomas
 *
 */

#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>

/*
 * Reference the url below for the following protocol defines:
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 */

#define DETECT_PIN_SETTLE_TIME_MS		32

/* UART header commands */
static const u8 JC_CMD_EXTSEND			= 0x91;
static const u8 JC_CMD_EXTRET			= 0x92;
static const u8 JC_CMD_INITRET			= 0x94;
static const u8 JC_CMD_HANDSHAKE		= 0xA5;
static const u8 JC_CMD_HORIINPUTREPORT		= 0x9A;

/* Used in handshake */
static const u8 JC_INIT_MAC			= 0x01;
static const u8 JC_INIT_HID_CONNECT		= 0x10;
static const u8 JC_INIT_HID_DISCONNECT		= 0x11;
static const u8 JC_INIT_SET_HID_RATE		= 0x12;
static const u8 JC_INIT_SET_BAUDRATE		= 0x20;

/* Input Reports */
static const u8 JC_INPUT_EXT_ACK		= 0x1F;
static const u8 JC_INPUT_BUTTON_EVENT		= 0x3F;
static const u8 JC_INPUT_SUBCMD_REPLY		= 0x21;
static const u8 JC_INPUT_IMU_DATA		= 0x30;
static const u8 JC_INPUT_MCU_DATA		= 0x31;
static const u8 JC_INPUT_USB_RESPONSE		= 0x81;

/* Output Reports */
static const u8 JC_OUTPUT_RUMBLE_AND_SUBCMD	= 0x01;
static const u8 JC_OUTPUT_FW_UPDATE_PKT		= 0x03;
static const u8 JC_OUTPUT_RUMBLE_ONLY		= 0x10;
static const u8 JC_OUTPUT_MCU_DATA		= 0x11;

/* Subcommand IDs */
static const u8 JC_SUBCMD_STATE			= 0x00;
static const u8 JC_SUBCMD_MANUAL_BT_PAIRING	= 0x01;
static const u8 JC_SUBCMD_REQ_DEV_INFO		= 0x02;
static const u8 JC_SUBCMD_SET_REPORT_MODE	= 0x03;
static const u8 JC_SUBCMD_TRIGGERS_ELAPSED	= 0x04;
static const u8 JC_SUBCMD_GET_PAGE_LIST_STATE	= 0x05;
static const u8 JC_SUBCMD_SET_HCI_STATE		= 0x06;
static const u8 JC_SUBCMD_RESET_PAIRING_INFO	= 0x07;
static const u8 JC_SUBCMD_LOW_POWER_MODE	= 0x08;
static const u8 JC_SUBCMD_SPI_FLASH_READ	= 0x10;
static const u8 JC_SUBCMD_SPI_FLASH_WRITE	= 0x11;
static const u8 JC_SUBCMD_REQUEST_REPORT	= 0x1F;
static const u8 JC_SUBCMD_RESET_MCU		= 0x20;
static const u8 JC_SUBCMD_SET_MCU_CONFIG	= 0x21;
static const u8 JC_SUBCMD_SET_MCU_STATE		= 0x22;
static const u8 JC_SUBCMD_SET_PLAYER_LIGHTS	= 0x30;
static const u8 JC_SUBCMD_GET_PLAYER_LIGHTS	= 0x31;
static const u8 JC_SUBCMD_SET_HOME_LIGHT	= 0x38;
static const u8 JC_SUBCMD_ENABLE_IMU		= 0x40;
static const u8 JC_SUBCMD_SET_IMU_SENSITIVITY	= 0x41;
static const u8 JC_SUBCMD_WRITE_IMU_REG		= 0x42;
static const u8 JC_SUBCMD_READ_IMU_REG		= 0x43;
static const u8 JC_SUBCMD_ENABLE_VIBRATION	= 0x48;
static const u8 JC_SUBCMD_GET_REGULATED_VOLTAGE	= 0x50;

/* Sio uart header commands */
static const u8 JC_SIO_OUTPUT_RPT		= 0x91;
static const u8 JC_SIO_INPUT_RPT		= 0x92;
static const u8  JC_SIO_CMD_ACK			= 0x80;
static const u8  JC_SIO_STATUS_OK		= 0x00;

/* Sio CMDs */
static const u8 JC_SIO_CMD_INIT			= 0x01;
static const u8 JC_SIO_CMD_VER_RPT		= 0x03;
static const u8 JC_SIO_CMD_INPUTREPORT		= 0x41;
static const u8 JC_SIO_CMD_IAP_VER		= 0x42;

/* Magic value denoting presence of user calibration */
static const u16 JC_CAL_USR_MAGIC_0		= 0xB2;
static const u16 JC_CAL_USR_MAGIC_1		= 0xA1;
static const u8 JC_CAL_USR_MAGIC_SIZE		= 2;

/* SPI storage addresses of user calibration data */
static const u16 JC_CAL_USR_LEFT_MAGIC_ADDR	= 0x8010;
static const u16 JC_CAL_USR_LEFT_DATA_ADDR	= 0x8012;
static const u16 JC_CAL_USR_LEFT_DATA_END	= 0x801A;
static const u16 JC_CAL_USR_RIGHT_MAGIC_ADDR	= 0x801B;
static const u16 JC_CAL_USR_RIGHT_DATA_ADDR	= 0x801D;
#define JC_CAL_STICK_DATA_SIZE \
	(JC_CAL_USR_LEFT_DATA_END - JC_CAL_USR_LEFT_DATA_ADDR + 1)

/* SPI storage addresses of factory calibration data */
static const u16 JC_CAL_FCT_DATA_LEFT_ADDR	= 0x603d;
static const u16 JC_CAL_FCT_DATA_RIGHT_ADDR	= 0x6046;

/* The raw analog joystick values will be mapped in terms of this magnitude */
static const u16 JC_MAX_STICK_MAG		= 32767;
static const u16 JC_STICK_FUZZ			= 250;
static const u16 JC_STICK_FLAT			= 500;

/* Under most circumstances IMU reports are pushed every 15ms; use as default */
static const u16 JC_IMU_DFLT_AVG_DELTA_MS	= 15;
/* How many samples to sum before calculating average IMU report delta */
static const u16 JC_IMU_SAMPLES_PER_DELTA_AVG	= 300;
/* Controls how many dropped IMU packets at once trigger a warning message */
static const u16 JC_IMU_DROPPED_PKT_WARNING	= 3;

/*
 * The controller's accelerometer has a sensor resolution of 16bits and is
 * configured with a range of +-8000 milliGs. Therefore, the resolution can be
 * calculated thus: (2^16-1)/(8000 * 2) = 4.096 digits per milliG
 * Resolution per G (rather than per millliG): 4.096 * 1000 = 4096 digits per G
 * Alternatively: 1/4096 = .0002441 Gs per digit
 */
static const s32 JC_IMU_MAX_ACCEL_MAG		= 32767;
static const u16 JC_IMU_ACCEL_RES_PER_G		= 4096;
static const u16 JC_IMU_ACCEL_FUZZ		= 10;
static const u16 JC_IMU_ACCEL_FLAT		/*= 0*/;

/*
 * The controller's gyroscope has a sensor resolution of 16bits and is
 * configured with a range of +-2000 degrees/second.
 * Digits per dps: (2^16 -1)/(2000*2) = 16.38375
 * dps per digit: 16.38375E-1 = .0610
 *
 * STMicro recommends in the datasheet to add 15% to the dps/digit. This allows
 * the full sensitivity range to be saturated without clipping. This yields more
 * accurate results, so it's the technique this driver uses.
 * dps per digit (corrected): .0610 * 1.15 = .0702
 * digits per dps (corrected): .0702E-1 = 14.247
 *
 * Now, 14.247 truncating to 14 loses a lot of precision, so we rescale the
 * min/max range by 1000.
 */
static const s32 JC_IMU_PREC_RANGE_SCALE	= 1000;
/* Note: change mag and res_per_dps if prec_range_scale is ever altered */
static const s32 JC_IMU_MAX_GYRO_MAG		= 32767000; /* (2^16-1)*1000 */
static const u16 JC_IMU_GYRO_RES_PER_DPS	= 14247; /* (14.247*1000) */
static const u16 JC_IMU_GYRO_FUZZ		= 10;
static const u16 JC_IMU_GYRO_FLAT		/*= 0*/;

/* Frequency/amplitude defines and tables for rumble */
static const u32 JC_RUMBLE_MASK			= 0xFF80FE00;
static const u32 JC_RUMBLE_ZERO_AMP		= 0x40000000;

#define JC_VIB_RPT_GET_BUF_STS_RIGHT(val)	((val) & 7)
#define JC_VIB_RPT_GET_BUF_STS_LEFT(val)	(((val) >> 4) & 7)

struct joycon_rumble_freq_data {
	u16 high;
	u8 low;
	u16 freq; /* Hz*/
};

struct joycon_rumble_amp_data {
	u8 high;
	u16 low;
	u16 amp;
};

/*
 * These tables are from
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/rumble_data_table.md
 */
static const struct joycon_rumble_freq_data joycon_rumble_frequencies[] = {
	/* high, low, freq */
	{ 0x0000, 0x01,   41 }, { 0x0000, 0x02,   42 }, { 0x0000, 0x03,   43 },
	{ 0x0000, 0x04,   44 }, { 0x0000, 0x05,   45 }, { 0x0000, 0x06,   46 },
	{ 0x0000, 0x07,   47 }, { 0x0000, 0x08,   48 }, { 0x0000, 0x09,   49 },
	{ 0x0000, 0x0A,   50 }, { 0x0000, 0x0B,   51 }, { 0x0000, 0x0C,   52 },
	{ 0x0000, 0x0D,   53 }, { 0x0000, 0x0E,   54 }, { 0x0000, 0x0F,   55 },
	{ 0x0000, 0x10,   57 }, { 0x0000, 0x11,   58 }, { 0x0000, 0x12,   59 },
	{ 0x0000, 0x13,   60 }, { 0x0000, 0x14,   62 }, { 0x0000, 0x15,   63 },
	{ 0x0000, 0x16,   64 }, { 0x0000, 0x17,   66 }, { 0x0000, 0x18,   67 },
	{ 0x0000, 0x19,   69 }, { 0x0000, 0x1A,   70 }, { 0x0000, 0x1B,   72 },
	{ 0x0000, 0x1C,   73 }, { 0x0000, 0x1D,   75 }, { 0x0000, 0x1e,   77 },
	{ 0x0000, 0x1f,   78 }, { 0x0000, 0x20,   80 }, { 0x0400, 0x21,   82 },
	{ 0x0800, 0x22,   84 }, { 0x0c00, 0x23,   85 }, { 0x1000, 0x24,   87 },
	{ 0x1400, 0x25,   89 }, { 0x1800, 0x26,   91 }, { 0x1c00, 0x27,   93 },
	{ 0x2000, 0x28,   95 }, { 0x2400, 0x29,   97 }, { 0x2800, 0x2a,   99 },
	{ 0x2c00, 0x2b,  102 }, { 0x3000, 0x2c,  104 }, { 0x3400, 0x2d,  106 },
	{ 0x3800, 0x2e,  108 }, { 0x3c00, 0x2f,  111 }, { 0x4000, 0x30,  113 },
	{ 0x4400, 0x31,  116 }, { 0x4800, 0x32,  118 }, { 0x4c00, 0x33,  121 },
	{ 0x5000, 0x34,  123 }, { 0x5400, 0x35,  126 }, { 0x5800, 0x36,  129 },
	{ 0x5c00, 0x37,  132 }, { 0x6000, 0x38,  135 }, { 0x6400, 0x39,  137 },
	{ 0x6800, 0x3a,  141 }, { 0x6c00, 0x3b,  144 }, { 0x7000, 0x3c,  147 },
	{ 0x7400, 0x3d,  150 }, { 0x7800, 0x3e,  153 }, { 0x7c00, 0x3f,  157 },
	{ 0x8000, 0x40,  160 }, { 0x8400, 0x41,  164 }, { 0x8800, 0x42,  167 },
	{ 0x8c00, 0x43,  171 }, { 0x9000, 0x44,  174 }, { 0x9400, 0x45,  178 },
	{ 0x9800, 0x46,  182 }, { 0x9c00, 0x47,  186 }, { 0xa000, 0x48,  190 },
	{ 0xa400, 0x49,  194 }, { 0xa800, 0x4a,  199 }, { 0xac00, 0x4b,  203 },
	{ 0xb000, 0x4c,  207 }, { 0xb400, 0x4d,  212 }, { 0xb800, 0x4e,  217 },
	{ 0xbc00, 0x4f,  221 }, { 0xc000, 0x50,  226 }, { 0xc400, 0x51,  231 },
	{ 0xc800, 0x52,  236 }, { 0xcc00, 0x53,  241 }, { 0xd000, 0x54,  247 },
	{ 0xd400, 0x55,  252 }, { 0xd800, 0x56,  258 }, { 0xdc00, 0x57,  263 },
	{ 0xe000, 0x58,  269 }, { 0xe400, 0x59,  275 }, { 0xe800, 0x5a,  281 },
	{ 0xec00, 0x5b,  287 }, { 0xf000, 0x5c,  293 }, { 0xf400, 0x5d,  300 },
	{ 0xf800, 0x5e,  306 }, { 0xfc00, 0x5f,  313 }, { 0x0001, 0x60,  320 },
	{ 0x0401, 0x61,  327 }, { 0x0801, 0x62,  334 }, { 0x0c01, 0x63,  341 },
	{ 0x1001, 0x64,  349 }, { 0x1401, 0x65,  357 }, { 0x1801, 0x66,  364 },
	{ 0x1c01, 0x67,  372 }, { 0x2001, 0x68,  381 }, { 0x2401, 0x69,  389 },
	{ 0x2801, 0x6a,  397 }, { 0x2c01, 0x6b,  406 }, { 0x3001, 0x6c,  415 },
	{ 0x3401, 0x6d,  424 }, { 0x3801, 0x6e,  433 }, { 0x3c01, 0x6f,  443 },
	{ 0x4001, 0x70,  453 }, { 0x4401, 0x71,  462 }, { 0x4801, 0x72,  473 },
	{ 0x4c01, 0x73,  483 }, { 0x5001, 0x74,  494 }, { 0x5401, 0x75,  504 },
	{ 0x5801, 0x76,  515 }, { 0x5c01, 0x77,  527 }, { 0x6001, 0x78,  538 },
	{ 0x6401, 0x79,  550 }, { 0x6801, 0x7a,  562 }, { 0x6c01, 0x7b,  574 },
	{ 0x7001, 0x7c,  587 }, { 0x7401, 0x7d,  600 }, { 0x7801, 0x7e,  613 },
	{ 0x7c01, 0x7f,  626 }, { 0x8001, 0x00,  640 }, { 0x8401, 0x00,  654 },
	{ 0x8801, 0x00,  668 }, { 0x8c01, 0x00,  683 }, { 0x9001, 0x00,  698 },
	{ 0x9401, 0x00,  713 }, { 0x9801, 0x00,  729 }, { 0x9c01, 0x00,  745 },
	{ 0xa001, 0x00,  761 }, { 0xa401, 0x00,  778 }, { 0xa801, 0x00,  795 },
	{ 0xac01, 0x00,  812 }, { 0xb001, 0x00,  830 }, { 0xb401, 0x00,  848 },
	{ 0xb801, 0x00,  867 }, { 0xbc01, 0x00,  886 }, { 0xc001, 0x00,  905 },
	{ 0xc401, 0x00,  925 }, { 0xc801, 0x00,  945 }, { 0xcc01, 0x00,  966 },
	{ 0xd001, 0x00,  987 }, { 0xd401, 0x00, 1009 }, { 0xd801, 0x00, 1031 },
	{ 0xdc01, 0x00, 1053 }, { 0xe001, 0x00, 1076 }, { 0xe401, 0x00, 1100 },
	{ 0xe801, 0x00, 1124 }, { 0xec01, 0x00, 1149 }, { 0xf001, 0x00, 1174 },
	{ 0xf401, 0x00, 1199 }, { 0xf801, 0x00, 1226 }, { 0xfc01, 0x00, 1253 }
};

#define JOYCON_RUMBLE_AMP_COEFF0	50
#define JOYCON_RUMBLE_AMP_COEFF1	15
#define JOYCON_RUMBLE_AMP_COEFF2	30
#define JOYCON_RUMBLE_AMP_COEFF3	10
#define JOYCON_RUMBLE_MAX_AMP	(1003) /* 100.3% */
static const struct joycon_rumble_amp_data joycon_rumble_amplitudes[] = {
	/* high, low, amp */
	{ 0x00, 0x0040,    0 },
	{ 0x02, 0x8040,   10 }, { 0x04, 0x0041,   12 }, { 0x06, 0x8041,   14 },
	{ 0x08, 0x0042,   17 }, { 0x0a, 0x8042,   20 }, { 0x0c, 0x0043,   24 },
	{ 0x0e, 0x8043,   28 }, { 0x10, 0x0044,   33 }, { 0x12, 0x8044,   40 },
	{ 0x14, 0x0045,   47 }, { 0x16, 0x8045,   56 }, { 0x18, 0x0046,   67 },
	{ 0x1a, 0x8046,   80 }, { 0x1c, 0x0047,   95 }, { 0x1e, 0x8047,  112 },
	{ 0x20, 0x0048,  117 }, { 0x22, 0x8048,  123 }, { 0x24, 0x0049,  128 },
	{ 0x26, 0x8049,  134 }, { 0x28, 0x004a,  140 }, { 0x2a, 0x804a,  146 },
	{ 0x2c, 0x004b,  152 }, { 0x2e, 0x804b,  159 }, { 0x30, 0x004c,  166 },
	{ 0x32, 0x804c,  173 }, { 0x34, 0x004d,  181 }, { 0x36, 0x804d,  189 },
	{ 0x38, 0x004e,  198 }, { 0x3a, 0x804e,  206 }, { 0x3c, 0x004f,  215 },
	{ 0x3e, 0x804f,  225 }, { 0x40, 0x0050,  230 }, { 0x42, 0x8050,  235 },
	{ 0x44, 0x0051,  240 }, { 0x46, 0x8051,  245 }, { 0x48, 0x0052,  251 },
	{ 0x4a, 0x8052,  256 }, { 0x4c, 0x0053,  262 }, { 0x4e, 0x8053,  268 },
	{ 0x50, 0x0054,  273 }, { 0x52, 0x8054,  279 }, { 0x54, 0x0055,  286 },
	{ 0x56, 0x8055,  292 }, { 0x58, 0x0056,  298 }, { 0x5a, 0x8056,  305 },
	{ 0x5c, 0x0057,  311 }, { 0x5e, 0x8057,  318 }, { 0x60, 0x0058,  325 },
	{ 0x62, 0x8058,  332 }, { 0x64, 0x0059,  340 }, { 0x66, 0x8059,  347 },
	{ 0x68, 0x005a,  355 }, { 0x6a, 0x805a,  362 }, { 0x6c, 0x005b,  370 },
	{ 0x6e, 0x805b,  378 }, { 0x70, 0x005c,  387 }, { 0x72, 0x805c,  395 },
	{ 0x74, 0x005d,  404 }, { 0x76, 0x805d,  413 }, { 0x78, 0x005e,  422 },
	{ 0x7a, 0x805e,  431 }, { 0x7c, 0x005f,  440 }, { 0x7e, 0x805f,  450 },
	{ 0x80, 0x0060,  460 }, { 0x82, 0x8060,  470 }, { 0x84, 0x0061,  480 },
	{ 0x86, 0x8061,  491 }, { 0x88, 0x0062,  501 }, { 0x8a, 0x8062,  512 },
	{ 0x8c, 0x0063,  524 }, { 0x8e, 0x8063,  535 }, { 0x90, 0x0064,  547 },
	{ 0x92, 0x8064,  559 }, { 0x94, 0x0065,  571 }, { 0x96, 0x8065,  584 },
	{ 0x98, 0x0066,  596 }, { 0x9a, 0x8066,  609 }, { 0x9c, 0x0067,  623 },
	{ 0x9e, 0x8067,  636 }, { 0xa0, 0x0068,  650 }, { 0xa2, 0x8068,  665 },
	{ 0xa4, 0x0069,  679 }, { 0xa6, 0x8069,  694 }, { 0xa8, 0x006a,  709 },
	{ 0xaa, 0x806a,  725 }, { 0xac, 0x006b,  741 }, { 0xae, 0x806b,  757 },
	{ 0xb0, 0x006c,  773 }, { 0xb2, 0x806c,  790 }, { 0xb4, 0x006d,  808 },
	{ 0xb6, 0x806d,  825 }, { 0xb8, 0x006e,  843 }, { 0xba, 0x806e,  862 },
	{ 0xbc, 0x006f,  881 }, { 0xbe, 0x806f,  900 }, { 0xc0, 0x0070,  920 },
	{ 0xc2, 0x8070,  940 }, { 0xc4, 0x0071,  960 }, { 0xc6, 0x8071,  981 },
	{ 0xc8, 0x0072, JOYCON_RUMBLE_MAX_AMP }
};

struct joycon_stick_cal {
	s32 max;
	s32 min;
	s32 center;
};

struct joycon_imu_cal {
	s16 offset[3];
	s16 scale[3];
};

/* States for controller state machine */
enum joycon_ctlr_state {
	JOYCON_CTLR_STATE_INIT,
	JOYCON_CTLR_STATE_READ,
};

enum joycon_ctlr_type {
	JOYCON_TYPE_UNKNOWN,
	JOYCON_TYPE_LEFT,
	JOYCON_TYPE_RIGHT,
	JOYCON_TYPE_SIO
};

/*
 * All the controller's button values are stored in a u32.
 * They can be accessed with bitwise ANDs.
 */
static const u32 JC_BTN_Y	= BIT(0);
static const u32 JC_BTN_X	= BIT(1);
static const u32 JC_BTN_B	= BIT(2);
static const u32 JC_BTN_A	= BIT(3);
static const u32 JC_BTN_SR_R	= BIT(4);
static const u32 JC_BTN_SL_R	= BIT(5);
static const u32 JC_BTN_R	= BIT(6);
static const u32 JC_BTN_ZR	= BIT(7);
static const u32 JC_BTN_MINUS	= BIT(8);
static const u32 JC_BTN_PLUS	= BIT(9);
static const u32 JC_BTN_RSTICK	= BIT(10);
static const u32 JC_BTN_LSTICK	= BIT(11);
static const u32 JC_BTN_HOME	= BIT(12);
static const u32 JC_BTN_CAP	= BIT(13); /* Capture button */
static const u32 JC_BTN_DOWN	= BIT(16);
static const u32 JC_BTN_UP	= BIT(17);
static const u32 JC_BTN_RIGHT	= BIT(18);
static const u32 JC_BTN_LEFT	= BIT(19);
static const u32 JC_BTN_SR_L	= BIT(20);
static const u32 JC_BTN_SL_L	= BIT(21);
static const u32 JC_BTN_L	= BIT(22);
static const u32 JC_BTN_ZL	= BIT(23);

static const u8 JC_CRC8_POLY	= 0x8D;
static const u8 JC_CRC8_INIT	= 0x00;

enum joycon_msg_type {
	JOYCON_MSG_TYPE_NONE,
	JOYCON_MSG_TYPE_SUBCMD,
	JOYCON_MSG_TYPE_UART_CMD,
};

static const u8 JC_UART_MAGIC_TX_0	= 0x19;
static const u8 JC_UART_MAGIC_TX_1	= 0x01;
static const u8 JC_UART_MAGIC_TX_2	= 0x03;
static const u8 JC_UART_MAGIC_RX_0	= 0x19;
static const u8 JC_UART_MAGIC_RX_1	= 0x81;
static const u8 JC_UART_MAGIC_RX_2	= 0x03;
struct joycon_uart_packet {
	u8 magic[3];
	u8 size;
	u8 pad;
	u8 command;
	u8 header_data[5];
	u8 crc;
	u8 data[0]; /* Length will vary by packet type */
} __packed;

struct joycon_subcmd_request {
	u8 output_id; /* Must be 0x01 for subcommand, 0x10 for rumble only */
	u8 packet_num; /* Incremented every send */
	u8 rumble_data[8];
	u8 subcmd_id;
	u8 data[]; /* Length depends on the subcommand */
} __packed;

struct joycon_subcmd_reply {
	u8 ack; /* MSB 1 for ACK, 0 for NACK */
	u8 id; /* id of requested subcmd */
	u8 data[0]; /* Will be at most 35 bytes */
} __packed;

struct joycon_input_report {
	u8 id;
	u8 timer;
	u8 bat_con; /* Battery and connection info */
	u8 button_status[3];
	u8 left_stick[3];
	u8 right_stick[3];
	u8 vibrator_report;

	/*
	 * If support for firmware updates, gyroscope data, and/or NFC/IR
	 * are added in the future, this can be swapped for a union.
	 */
	struct joycon_subcmd_reply reply;
} __packed;

struct sio_uart_packet {
	u8  cmd;
	u8  subcmd;
	u16 payload_len;
	u8  data[2];
	u8  crc_payload;
	u8  crc_hdr;
	u8  payload[];
} __packed;

struct joycon_imu_data {
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
} __packed;

struct sio_input_report
{
	u8 type;
	u8 pkt_id;
	u8 unk;
	u8 button_status[3];
	u8 left_stick[3];
	u8 right_stick[3];
	u8 sixaxis_report_num; /* Max 15. Each report is 800 us? */
	union {
		struct joycon_imu_data sixaxis[15];
		/* IMU input reports contain 3 samples */
		u8 imu_raw_bytes[sizeof(struct joycon_imu_data) * 15];
	};
} __packed;

struct joycon_led_queue_item {
	struct led_classdev *led;
	enum led_brightness brightness;
};

#define JC_MAX_RESP_SIZE		(sizeof(struct joycon_input_report) + 35)
#define JC_MAX_UART_PKT_SIZE		(sizeof(struct joycon_uart_packet) + JC_MAX_RESP_SIZE)
#define JC_NUM_LEDS			4
#define JC_RUMBLE_DATA_SIZE		8
#define JC_RUMBLE_QUEUE_SIZE		8
#define JC_LED_QUEUE_SIZE		20

#define JC_SIO_MAX_RESP_SIZE		(sizeof(struct sio_input_report) + 15)
#define JC_SIO_MAX_UART_PKT_SIZE	(sizeof(struct sio_uart_packet) + JC_SIO_MAX_RESP_SIZE)

static const u16 JC_RUMBLE_DFLT_LOW_FREQ = 160;
static const u16 JC_RUMBLE_DFLT_HIGH_FREQ = 320;
static const u16 JC_RUMBLE_PERIOD_MS = 50;

static const unsigned int JC_UART_BAUD_LOW = 1000000;
static const unsigned int JC_UART_BAUD_HIGH = 3000000;

/* Each physical controller is associated with a joycon_ctlr struct */
struct joycon_ctlr {
	struct serdev_device *sdev;
	struct input_dev *input;
	char *led_names[JC_NUM_LEDS];
	struct led_classdev leds[JC_NUM_LEDS]; /* Player leds */
	char *home_led_name;
	struct led_classdev home_led;
	enum joycon_ctlr_state ctlr_state;
	enum joycon_ctlr_type ctlr_type;
	spinlock_t lock;
	u8 mac_addr[6];
	char *mac_addr_str;
	bool suspending;

	/* Lite gamepad */
	bool is_sio;
	int sio_home_gpio;
	int sio_por_gpio;
	int sio_rst_gpio;
	struct clk *sio_clk;

	/* Third party protocol */
	bool is_hori;

	/* Used for crc8 computation */
	u8 joycon_crc_table[CRC8_TABLE_SIZE];

	/* Used for processing led brightness sets */
	struct joycon_led_queue_item led_queue[JC_LED_QUEUE_SIZE];
	int led_queue_head;
	int led_queue_tail;
	struct work_struct led_worker;

	/* The following members are used for synchronous sends/receives */
	enum joycon_msg_type msg_type;
	u8 subcmd_num;
	struct mutex output_mutex;
	u8 input_buf[JC_SIO_MAX_RESP_SIZE];
	wait_queue_head_t wait;
	bool received_resp;
	u8 subcmd_ack_match;
	u8 uart_cmd_match;
	bool received_input_report;
	bool ctlr_removed;

	/* Buffering partially received uart packets */
	u8 partial_pkt[JC_SIO_MAX_UART_PKT_SIZE];
	size_t partial_pkt_len;

	/* Joy-con detection */
	struct workqueue_struct *detection_queue;
	struct delayed_work detection_worker;
	int detection_irq_retries;
	int detect_en_gpio;
	bool detect_en_req;
	int detect_gpio;
	int detect_irq;

	/* Joy-con input polling */
	struct workqueue_struct *input_queue;
	struct delayed_work input_worker;
	unsigned int last_input_report_msecs;

	/* Factory calibration data */
	struct joycon_stick_cal stick_cal_x;
	struct joycon_stick_cal stick_cal_y;
	struct joycon_stick_cal right_stick_cal_x;
	struct joycon_stick_cal right_stick_cal_y;

	struct joycon_imu_cal accel_cal;
	struct joycon_imu_cal gyro_cal;

	/* Prevents needlessly recalculating these divisors every sample */
	s32 imu_cal_accel_divisor[3];
	s32 imu_cal_gyro_divisor[3];

	/* Power supply data */
	struct power_supply *battery;
	char *battery_desc_name;
	struct power_supply_desc battery_desc;
	u8 battery_capacity;
	bool battery_charging;
	bool host_powered;
	struct regulator *charger_reg;

	/* Rumble */
	u8 rumble_data[JC_RUMBLE_QUEUE_SIZE][JC_RUMBLE_DATA_SIZE];
	int rumble_queue_head;
	int rumble_queue_tail;
	struct workqueue_struct *rumble_queue;
	struct delayed_work rumble_worker;
	unsigned int rumble_msecs;
	u16 rumble_ll_freq;
	u16 rumble_lh_freq;
	u16 rumble_rl_freq;
	u16 rumble_rh_freq;
	u32 rumble_buf_avail;
	bool rumble_keep_alive;

	/* Imu */
	struct input_dev *imu_input;
	bool imu_first_packet_received; /* Helps in initiating timestamp */
	unsigned int imu_timestamp_us; /* Timestamp we report to userspace */
	unsigned int imu_last_pkt_ms; /* Used to calc imu report delta */
	/* The following are used to track the average imu report time delta */
	unsigned int imu_delta_samples_count;
	unsigned int imu_delta_samples_sum;
	unsigned int imu_avg_delta_ms;
};

static int joycon_serdev_send(struct joycon_ctlr *ctlr, u8 *data,
			      size_t len, u32 timeout)
{
	int ret;

	ret = serdev_device_write(ctlr->sdev, data, len, timeout);
	if (ret < 0) {
		dev_err(&ctlr->sdev->dev,
			"Failed to send serial data; ret=%d\n", ret);
		return ret;
	}
	return 0;
}

static int joycon_serdev_send_sync(struct joycon_ctlr *ctlr, u8 *data,
				   size_t len, u32 timeout)
{
	int ret;
	int tries = 2;

	/*
	 * The controller occasionally seems to drop subcommands. In testing,
	 * doing one retry after a timeout appears to always work.
	 */
	while (tries--) {
		/*
		 * If we are in the proper reporting mode, wait for an input
		 * report prior to sending the subcommand. This improves
		 * reliability considerably.
		 */
		if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
			unsigned long flags;

			spin_lock_irqsave(&ctlr->lock, flags);
			ctlr->received_input_report = false;
			spin_unlock_irqrestore(&ctlr->lock, flags);
			ret = wait_event_timeout(ctlr->wait,
						 ctlr->received_input_report,
						 HZ / 4);
			spin_lock_irqsave(&ctlr->lock, flags);
			if (!ret && !ctlr->ctlr_removed) {
				dev_warn(&ctlr->sdev->dev,
					 "Timeout waiting for input report\n");
				ret = -ETIMEDOUT;
				spin_unlock_irqrestore(&ctlr->lock, flags);
				goto err;
			}
			spin_unlock_irqrestore(&ctlr->lock, flags);
		}

		ret = joycon_serdev_send(ctlr, data, len, timeout);
		if (ret) {
			memset(ctlr->input_buf, 0, JC_SIO_MAX_RESP_SIZE);
			return ret;
		}

		ret = wait_event_timeout(ctlr->wait, ctlr->received_resp,
					 timeout);
		if (!ret) {
			if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
				dev_info(&ctlr->sdev->dev,
					"Synchronous send/receive timed out\n");
			}
			if (tries) {
				dev_dbg(&ctlr->sdev->dev,
					"Retrying sync send after timeout\n");
			}
			memset(ctlr->input_buf, 0, JC_SIO_MAX_RESP_SIZE);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
			break;
		}
	}

err:
	ctlr->received_resp = false;
	return ret;
}

static int joycon_send_packet(struct joycon_ctlr *ctlr, u8 command,
			      u8 *hdata, size_t hdata_size,
			      u8 *data, size_t data_size, u32 timeout,
			      bool sync)
{
	struct joycon_uart_packet *packet;
	struct device *dev = &ctlr->sdev->dev;
	int ret = 0;

	packet = kzalloc(sizeof(*packet) + data_size, GFP_KERNEL);
	if (!packet)
		return -ENOMEM;

	packet->magic[0] = JC_UART_MAGIC_TX_0;
	packet->magic[1] = JC_UART_MAGIC_TX_1;
	packet->magic[2] = JC_UART_MAGIC_TX_2;
	packet->size = 7 + data_size;
	packet->command = command;

	if (hdata) {
		if (hdata_size > sizeof(packet->header_data)) {
			dev_err(dev, "Header data size too large\n");
			ret = -EINVAL;
			goto err;
		}
		memcpy(packet->header_data, hdata, hdata_size);
	}

	if (command == JC_CMD_HANDSHAKE) {
		packet->crc = JC_CRC8_INIT;
	} else {
		packet->crc = crc8(ctlr->joycon_crc_table, &packet->pad, sizeof(packet->pad) + sizeof(packet->command) + sizeof(packet->header_data), JC_CRC8_INIT);
	}

	if (data)
		memcpy(packet->data, data, data_size);

	if (sync)
		ret = joycon_serdev_send_sync(ctlr, (u8 *)packet,
					      sizeof(*packet) + data_size,
					      timeout);
	else
		ret = joycon_serdev_send(ctlr, (u8 *)packet,
					 sizeof(*packet) + data_size, timeout);

	if (ret && ctlr->ctlr_state == JOYCON_CTLR_STATE_READ)
		dev_err(dev, "Failed sending uart packet; ret=%d\n", ret);

err:
	kfree(packet);
	return ret;
}


static int sio_send_packet(struct joycon_ctlr *ctlr, u8 command,
			      u8 *hdata, size_t hdata_size,
			      u8 *data, size_t data_size, u32 timeout,
			      bool sync)
{
	struct sio_uart_packet *packet;
	struct device *dev = &ctlr->sdev->dev;
	int ret = 0;

	packet = kzalloc(sizeof(*packet) + data_size, GFP_KERNEL);
	if (!packet)
		return -ENOMEM;

	packet->cmd = JC_SIO_OUTPUT_RPT;
	packet->subcmd = command;
	packet->payload_len = data_size;

	if (hdata) {
		if (hdata_size > sizeof(packet->data)) {
			dev_err(dev, "Header data size too large\n");
			ret = -EINVAL;
			goto err;
		}
		memcpy(packet->data, hdata, hdata_size);
	}

	if (data_size)
		packet->crc_payload = crc8(ctlr->joycon_crc_table, data, data_size, JC_CRC8_INIT);
	packet->crc_hdr = crc8(ctlr->joycon_crc_table, (u8 *)packet, sizeof(*packet) - 1, JC_CRC8_INIT);

	if (data)
		memcpy(packet->data, data, data_size);

	if (sync)
		ret = joycon_serdev_send_sync(ctlr, (u8 *)packet,
					      sizeof(*packet) + data_size,
					      timeout);
	else
		ret = joycon_serdev_send(ctlr, (u8 *)packet,
					 sizeof(*packet) + data_size, timeout);

	if (ret && ctlr->ctlr_state == JOYCON_CTLR_STATE_READ)
		dev_err(dev, "Failed sending uart packet; ret=%d\n", ret);

err:
	kfree(packet);
	return ret;
}

/* Caller must set ctlr->uart_cmd_match prior to calling */
static int joycon_send_command(struct joycon_ctlr *ctlr, u8 command,
			       u8 *data, size_t data_len, u32 timeout)
{
	struct device *dev = &ctlr->sdev->dev;
	int ret;

	ctlr->msg_type = JOYCON_MSG_TYPE_UART_CMD;
	ret = joycon_send_packet(ctlr, command, data, data_len, NULL, 0,
				 timeout, true);
	if (ret < 0)
		dev_dbg(dev, "Send command failed; ret=%d\n", ret);
	return ret;
}

static int joycon_send_subcmd(struct joycon_ctlr *ctlr,
			      struct joycon_subcmd_request *subcmd,
			      size_t data_len, u32 timeout)
{
	int ret;
	unsigned long flags;
	__be16 subcmd_size;
	struct device *dev = &ctlr->sdev->dev;

	spin_lock_irqsave(&ctlr->lock, flags);
	memcpy(subcmd->rumble_data, ctlr->rumble_data[ctlr->rumble_queue_tail],
	       JC_RUMBLE_DATA_SIZE);
	spin_unlock_irqrestore(&ctlr->lock, flags);

	subcmd->output_id = JC_OUTPUT_RUMBLE_AND_SUBCMD;
	subcmd->packet_num = ctlr->subcmd_num;
	if (++ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;
	ctlr->subcmd_ack_match = subcmd->subcmd_id;
	ctlr->msg_type = JOYCON_MSG_TYPE_SUBCMD;

	subcmd_size = cpu_to_be16(sizeof(*subcmd) + data_len);

	ret = joycon_send_packet(ctlr, JC_CMD_EXTRET,
				 (u8 *)&subcmd_size, sizeof(subcmd_size),
				 (u8 *)subcmd, sizeof(*subcmd) + data_len,
				 timeout, true);
	if (ret < 0)
		dev_dbg(dev, "Send subcommand failed; ret=%d\n", ret);
	return ret;
}

static int joycon_set_hci_state(struct joycon_ctlr *ctlr, u8 state)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_HCI_STATE;
	req->data[0] = state;

	dev_dbg(&ctlr->sdev->dev, "Setting hci state = %u\n", state);
	/* We won't get a reply. Just use a short timeout. */
	return joycon_send_subcmd(ctlr, req, 1, HZ/10);
}

static int joycon_get_ctlr_info(struct joycon_ctlr *ctlr)
{
	struct joycon_subcmd_request req = { 0 };
	struct device *dev = &ctlr->sdev->dev;
	struct joycon_input_report *report;
	int ret;

	req.subcmd_id = JC_SUBCMD_REQ_DEV_INFO;
	ret = joycon_send_subcmd(ctlr, &req, 0, HZ);
	if (ret) {
		dev_err(dev, "Failed to get Joy-con info; ret=%d\n", ret);
		return ret;
	}

	report = (struct joycon_input_report *)ctlr->input_buf;
	switch (report->reply.data[2]) {
	case 1:
		ctlr->ctlr_type = JOYCON_TYPE_LEFT;
		dev_info(dev, "Detected left joy-con\n");
		break;
	case 2:
		ctlr->ctlr_type = JOYCON_TYPE_RIGHT;
		dev_info(dev, "Detected right joy-con\n");
		break;
	default:
		dev_err(dev, "Invalid joy-con type = %u\n",
			report->reply.data[2]);
		return -EINVAL;
	}

	return 0;
}

/* Supply nibbles for flash and on. Ones correspond to active */
static int joycon_set_player_leds(struct joycon_ctlr *ctlr, u8 flash, u8 on)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_PLAYER_LIGHTS;
	req->data[0] = (flash << 4) | on;

	dev_dbg(&ctlr->sdev->dev, "Setting player leds\n");
	return joycon_send_subcmd(ctlr, req, 1, HZ/4);
}

/* The following was copied from the hid core's implementation */
static u32 field_extract(u8 *report, unsigned offset, int n)
{
	unsigned int idx = offset / 8;
	unsigned int bit_nr = 0;
	unsigned int bit_shift = offset % 8;
	int bits_to_copy = 8 - bit_shift;
	u32 value = 0;
	u32 mask = n < 32 ? (1U << n) - 1 : ~0U;

	while (n > 0) {
		value |= ((u32)report[idx] >> bit_shift) << bit_nr;
		n -= bits_to_copy;
		bit_nr += bits_to_copy;
		bits_to_copy = 8;
		bit_shift = 0;
		idx++;
	}

	return value & mask;
}

static int joycon_request_spi_flash_read(struct joycon_ctlr *ctlr,
					 u32 start_addr, u8 size, u8 **reply)
{
	struct joycon_subcmd_request *req;
	struct joycon_input_report *report;
	u8 buffer[sizeof(*req) + 5] = { 0 };
	struct device *dev = &ctlr->sdev->dev;
	u8 *data;
	int ret;

	if (!reply)
		return -EINVAL;

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SPI_FLASH_READ;
	data = req->data;
	data[0] = 0xFF & start_addr;
	data[1] = 0xFF & (start_addr >> 8);
	data[2] = 0xFF & (start_addr >> 16);
	data[3] = 0xFF & (start_addr >> 24);
	data[4] = size;

	dev_dbg(dev, "Requesting SPI flash data\n");
	ret = joycon_send_subcmd(ctlr, req, 5, HZ);
	if (ret) {
		dev_err(dev, "Failed reading SPI flash; ret=%d\n", ret);
	} else {
		report = (struct joycon_input_report *)ctlr->input_buf;
		/* The read data starts at the 6th byte */
		*reply = &report->reply.data[5];
	}
	return ret;
}

/*
 * User calibration's presence is denoted with a magic byte preceding it.
 * returns 0 if magic val is present, 1 if not present, < 0 on error
 */
static int joycon_check_for_cal_magic(struct joycon_ctlr *ctlr, u32 flash_addr)
{
	int ret;
	u8 *reply;

	ret = joycon_request_spi_flash_read(ctlr, flash_addr,
					    JC_CAL_USR_MAGIC_SIZE, &reply);
	if (ret)
		return ret;

	return reply[0] != JC_CAL_USR_MAGIC_0 || reply[1] != JC_CAL_USR_MAGIC_1;
}

static int joycon_read_stick_calibration(struct joycon_ctlr *ctlr, u16 cal_addr,
					 struct joycon_stick_cal *cal_x,
					 struct joycon_stick_cal *cal_y,
					 bool left_stick)
{
	s32 x_max_above;
	s32 x_min_below;
	s32 y_max_above;
	s32 y_min_below;
	u8 *raw_cal;
	int ret;

	ret = joycon_request_spi_flash_read(ctlr, cal_addr,
					    JC_CAL_STICK_DATA_SIZE, &raw_cal);
	if (ret)
		return ret;

	/* Stick calibration parsing: note the order differs based on stick */
	if (left_stick) {
		x_max_above = field_extract((raw_cal + 0), 0, 12);
		y_max_above = field_extract((raw_cal + 1), 4, 12);
		cal_x->center = field_extract((raw_cal + 3), 0, 12);
		cal_y->center = field_extract((raw_cal + 4), 4, 12);
		x_min_below = field_extract((raw_cal + 6), 0, 12);
		y_min_below = field_extract((raw_cal + 7), 4, 12);
	} else {
		cal_x->center = field_extract((raw_cal + 0), 0, 12);
		cal_y->center = field_extract((raw_cal + 1), 4, 12);
		x_min_below = field_extract((raw_cal + 3), 0, 12);
		y_min_below = field_extract((raw_cal + 4), 4, 12);
		x_max_above = field_extract((raw_cal + 6), 0, 12);
		y_max_above = field_extract((raw_cal + 7), 4, 12);
	}

	cal_x->max = cal_x->center + x_max_above;
	cal_x->min = cal_x->center - x_min_below;
	cal_y->max = cal_y->center + y_max_above;
	cal_y->min = cal_y->center - y_min_below;

	return 0;
}

static const u16 DFLT_STICK_CAL_CEN = 2048;
static const u16 DFLT_STICK_CAL_MAX = 3072;
static const u16 DFLT_STICK_CAL_MIN = 1024;
static const u16 HORI_STICK_CAL_CEN = 2048;
static const u16 HORI_STICK_CAL_MAX = 4095;
static const u16 HORI_STICK_CAL_MIN = 0;
static int joycon_request_calibration(struct joycon_ctlr *ctlr)
{
	u16 stick_addr = JC_CAL_FCT_DATA_LEFT_ADDR;
	struct device *dev = &ctlr->sdev->dev;
	enum joycon_ctlr_type type = ctlr->ctlr_type;
	int ret;

	dev_dbg(dev, "Requesting cal data\n");

	stick_addr = (type == JOYCON_TYPE_LEFT) ? JC_CAL_FCT_DATA_LEFT_ADDR
						: JC_CAL_FCT_DATA_RIGHT_ADDR;

	/* Check if user stick calibrations are present */
	if (type == JOYCON_TYPE_LEFT &&
	    !joycon_check_for_cal_magic(ctlr, JC_CAL_USR_LEFT_MAGIC_ADDR)) {
		stick_addr = JC_CAL_USR_LEFT_DATA_ADDR;
		dev_info(dev, "Using user cal for left stick\n");
	} else if (type == JOYCON_TYPE_RIGHT &&
		   !joycon_check_for_cal_magic(ctlr,
					       JC_CAL_USR_RIGHT_MAGIC_ADDR)) {
		stick_addr = JC_CAL_USR_RIGHT_DATA_ADDR;
		dev_info(dev, "Using user cal for right stick\n");
	} else {
		dev_info(dev, "Using factory cal for stick\n");
	}

	/* Read the stick calibration data */
	ret = joycon_read_stick_calibration(ctlr, stick_addr,
					    &ctlr->stick_cal_x,
					    &ctlr->stick_cal_y,
					    type == JOYCON_TYPE_LEFT);
	if (ret) {
		dev_warn(dev,
			 "Failed to read stick cal, using defaults; e=%d\n",
			 ret);

		ctlr->stick_cal_x.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_x.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_x.min = DFLT_STICK_CAL_MIN;

		ctlr->stick_cal_y.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_y.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_y.min = DFLT_STICK_CAL_MIN;
	}

	if (ctlr->stick_cal_x.center == 0 &&
		ctlr->stick_cal_x.max == 0 &&
		ctlr->stick_cal_x.min == 0) {
		ctlr->stick_cal_x.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_x.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_x.min = DFLT_STICK_CAL_MIN;
		dev_warn(dev, "Empty stick calibration for x axis, using defaults");
	}

	if (ctlr->stick_cal_y.center == 0 &&
		ctlr->stick_cal_y.max == 0 &&
		ctlr->stick_cal_y.min == 0) {
		ctlr->stick_cal_y.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_y.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_y.min = DFLT_STICK_CAL_MIN;
		dev_warn(dev, "Empty stick calibration for y axis, using defaults");
	}

	dev_dbg(&ctlr->sdev->dev, "calibration:\n"
				  "x_c=%d x_max=%d x_min=%d\n"
				  "y_c=%d y_max=%d y_min=%d\n",
				  ctlr->stick_cal_x.center,
				  ctlr->stick_cal_x.max,
				  ctlr->stick_cal_x.min,
				  ctlr->stick_cal_y.center,
				  ctlr->stick_cal_y.max,
				  ctlr->stick_cal_y.min);
	return 0;
}

static int joycon_enable_rumble(struct joycon_ctlr *ctlr, bool enable)
{
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };
	struct device *dev = &ctlr->sdev->dev;

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_ENABLE_VIBRATION;
	req->data[0] = enable ? 0x01 : 0x00;

	dev_dbg(dev, "%s rumble\n", enable ? "Enabling" : "Disabling");
	return joycon_send_subcmd(ctlr, req, 1, HZ/4);
}

static s32 joycon_map_stick_val(struct joycon_stick_cal *cal, s32 val)
{
	s32 center = cal->center;
	s32 min = cal->min;
	s32 max = cal->max;
	s32 new_val;

	if (val > center) {
		new_val = (val - center) * JC_MAX_STICK_MAG;
		new_val /= (max - center);
	} else {
		new_val = (center - val) * -JC_MAX_STICK_MAG;
		new_val /= (center - min);
	}
	new_val = clamp(new_val, (s32)-JC_MAX_STICK_MAG, (s32)JC_MAX_STICK_MAG);
	return new_val;
}

static void joycon_parse_report(struct joycon_ctlr *ctlr,
				struct joycon_input_report *rep)
{
	struct input_dev *dev = ctlr->input;
	unsigned long flags;
	u8 tmp;
	u32 btns;
	enum joycon_ctlr_type type = ctlr->ctlr_type;
	unsigned long msecs = jiffies_to_msecs(jiffies);

	dev_dbg(&ctlr->sdev->dev, "parse_report()\n");

	spin_lock_irqsave(&ctlr->lock, flags);

	switch (type) {
	case JOYCON_TYPE_LEFT:
		ctlr->rumble_buf_avail =
			JC_VIB_RPT_GET_BUF_STS_LEFT(rep->vibrator_report);
		break;
	case JOYCON_TYPE_RIGHT:
		ctlr->rumble_buf_avail =
			JC_VIB_RPT_GET_BUF_STS_RIGHT(rep->vibrator_report);
		break;
	default:
		ctlr->rumble_buf_avail = 0;
		break;
	}

	if (!ctlr->is_hori && IS_ENABLED(CONFIG_JOYCON_SERDEV_FF) &&
		(msecs - ctlr->rumble_msecs) >= JC_RUMBLE_PERIOD_MS &&
		(ctlr->rumble_queue_head != ctlr->rumble_queue_tail ||
		 ctlr->rumble_keep_alive))
		queue_delayed_work(ctlr->rumble_queue, &ctlr->rumble_worker, 0);

	ctlr->last_input_report_msecs = jiffies_to_msecs(jiffies);

	/* Parse the battery status */
	tmp = rep->bat_con;
	ctlr->host_powered = tmp & BIT(0);
	ctlr->battery_charging = tmp & BIT(4);
	tmp = tmp >> 5;
	switch (tmp) {
	case 0: /* Empty */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		break;
	case 1: /* Low */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		break;
	case 2: /* Medium */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case 3: /* High */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		break;
	case 4: /* Full */
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		break;
	default:
		ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		dev_warn(&ctlr->sdev->dev, "Invalid battery status\n");
		break;
	}
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/*
	 * When the battery is full, stop charging. This prevents long term
	 * damage due to unsophisticated charging circuitry in the joy-cons.
	 * We re-enable charging once the capacity drops below HIGH (to prevent
	 * oscillating between HIGH/FULL constantly).
	 */
	if (ctlr->battery_capacity == POWER_SUPPLY_CAPACITY_LEVEL_FULL) {
		/* Stop charging */
		if (!IS_ERR_OR_NULL(ctlr->charger_reg) &&
		    regulator_is_enabled(ctlr->charger_reg) > 0)
			regulator_disable(ctlr->charger_reg);
	} else if (ctlr->battery_capacity != POWER_SUPPLY_CAPACITY_LEVEL_HIGH) {
		/* Start charging */
		if (!IS_ERR_OR_NULL(ctlr->charger_reg) &&
		    !regulator_is_enabled(ctlr->charger_reg) &&
		    regulator_enable(ctlr->charger_reg))
			dev_err(&ctlr->sdev->dev, "Failed to enable charger\n");
	}

	/* Parse the buttons and sticks */
	btns = field_extract(rep->button_status, 0, 24);

	if (type == JOYCON_TYPE_LEFT) {
		u16 raw_x;
		u16 raw_y;
		s32 x;
		s32 y;

		/* Get raw stick values */
		raw_x = field_extract(rep->left_stick, 0, 12);
		raw_y = field_extract(rep->left_stick + 1, 4, 12);
		/* Map the stick values */
		x = joycon_map_stick_val(&ctlr->stick_cal_x, raw_x);
		y = -joycon_map_stick_val(&ctlr->stick_cal_y, raw_y);
		/* Report sticks */
		input_report_abs(dev, ABS_X, x);
		input_report_abs(dev, ABS_Y, y);

		/* Report buttons */
		input_report_key(dev, BTN_TL, btns & JC_BTN_L);
		input_report_key(dev, BTN_TL2, btns & JC_BTN_ZL);
		/* Report the S buttons as the non-existent triggers */
		input_report_key(dev, BTN_TR, btns & JC_BTN_SL_L);
		input_report_key(dev, BTN_TR2, btns & JC_BTN_SR_L);
		input_report_key(dev, BTN_SELECT, btns & JC_BTN_MINUS);
		input_report_key(dev, BTN_THUMBL, btns & JC_BTN_LSTICK);
		input_report_key(dev, BTN_Z, btns & JC_BTN_CAP);
		input_report_key(dev, BTN_DPAD_DOWN, btns & JC_BTN_DOWN);
		input_report_key(dev, BTN_DPAD_UP, btns & JC_BTN_UP);
		input_report_key(dev, BTN_DPAD_RIGHT, btns & JC_BTN_RIGHT);
		input_report_key(dev, BTN_DPAD_LEFT, btns & JC_BTN_LEFT);
	} else if (type == JOYCON_TYPE_RIGHT) {
		u16 raw_x;
		u16 raw_y;
		s32 x;
		s32 y;

		/* Get raw stick values */
		raw_x = field_extract(rep->right_stick, 0, 12);
		raw_y = field_extract(rep->right_stick + 1, 4, 12);
		/* Map stick values */
		x = joycon_map_stick_val(&ctlr->stick_cal_x, raw_x);
		y = -joycon_map_stick_val(&ctlr->stick_cal_y, raw_y);
		/* Report sticks */
		input_report_abs(dev, ABS_RX, x);
		input_report_abs(dev, ABS_RY, y);

		/* Report buttons */
		input_report_key(dev, BTN_TR, btns & JC_BTN_R);
		input_report_key(dev, BTN_TR2, btns & JC_BTN_ZR);
		/* Report the S buttons as the non-existent triggers */
		input_report_key(dev, BTN_TL, btns & JC_BTN_SL_R);
		input_report_key(dev, BTN_TL2, btns & JC_BTN_SR_R);
		input_report_key(dev, BTN_START, btns & JC_BTN_PLUS);
		input_report_key(dev, BTN_THUMBR, btns & JC_BTN_RSTICK);
		input_report_key(dev, BTN_MODE, btns & JC_BTN_HOME);
		input_report_key(dev, BTN_WEST, btns & JC_BTN_Y);
		input_report_key(dev, BTN_NORTH, btns & JC_BTN_X);
		input_report_key(dev, BTN_EAST, btns & JC_BTN_A);
		input_report_key(dev, BTN_SOUTH, btns & JC_BTN_B);
	}

	input_sync(dev);

	/*
	 * Immediately after receiving a report is the most reliable time to
	 * send a subcommand to the controller. Wake any subcommand senders
	 * waiting for a report.
	 */
	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->received_input_report = true;
	spin_unlock_irqrestore(&ctlr->lock, flags);
	wake_up(&ctlr->wait);
}

static void sio_input_report_parse_imu_data(struct joycon_ctlr *ctlr,
					       struct sio_input_report *rep,
					       struct joycon_imu_data *imu_data)
{
	u8 *raw = rep->imu_raw_bytes;
	int i;

	for (i = 0; i < rep->sixaxis_report_num; i++) {
		struct joycon_imu_data *data = &imu_data[i];

		data->accel_x = get_unaligned_le16(raw + 0);
		data->accel_y = get_unaligned_le16(raw + 2);
		data->accel_z = get_unaligned_le16(raw + 4);
		data->gyro_x = get_unaligned_le16(raw + 6);
		data->gyro_y = get_unaligned_le16(raw + 8);
		data->gyro_z = get_unaligned_le16(raw + 10);
		/* Point to next imu sample */
		raw += sizeof(struct joycon_imu_data);
	}
}

static void sio_parse_imu_report(struct joycon_ctlr *ctlr,
				    struct sio_input_report *rep)
{
	struct device *dev = &ctlr->sdev->dev;
	struct joycon_imu_data imu_data[15] = {0}; /* 15 reports per packet */
	struct input_dev *idev = ctlr->imu_input;
	unsigned int msecs = jiffies_to_msecs(jiffies);
	unsigned int last_msecs = ctlr->imu_last_pkt_ms;
	int i;
	int value[6];

	/*
	 * Only max of 15 is allowed for num of reports.
	 * On errors, that value is normally 0xF0;
	 */
	if (!rep->sixaxis_report_num || rep->sixaxis_report_num > 15)
		return;

	sio_input_report_parse_imu_data(ctlr, rep, imu_data);

	/*
	 * There are complexities surrounding how we determine the timestamps we
	 * associate with the samples we pass to userspace. The IMU input
	 * reports do not provide us with a good timestamp. There's a quickly
	 * incrementing 8-bit counter per input report, but it is not very
	 * useful for this purpose (it is not entirely clear what rate it
	 * increments at or if it varies based on packet push rate - more on
	 * the push rate below...).
	 *
	 * The reverse engineering work done on the joy-cons and pro controllers
	 * by the community seems to indicate the following:
	 * - The controller samples the IMU every 1.35ms. It then does some of
	 *   its own processing, probably averaging the samples out.
	 * - Each imu input report contains 3 IMU samples, (usually 5ms apart).
	 * - In the standard reporting mode (which this driver uses exclusively)
	 *   input reports are pushed from the controller as follows:
	 *      * joy-con (bluetooth): every 15 ms
	 *      * joy-cons (in charging grip via USB): every 15 ms
	 *      * pro controller (USB): every 15 ms
	 *      * pro controller (bluetooth): every 8 ms (this is the wildcard)
	 *
	 * Further complicating matters is that some bluetooth stacks are known
	 * to alter the controller's packet rate by hardcoding the bluetooth
	 * SSR for the switch controllers (android's stack currently sets the
	 * SSR to 11ms for both the joy-cons and pro controllers).
	 *
	 * In my own testing, I've discovered that my pro controller either
	 * reports IMU sample batches every 11ms or every 15ms. This rate is
	 * stable after connecting. It isn't 100% clear what determines this
	 * rate. Importantly, even when sending every 11ms, none of the samples
	 * are duplicates. This seems to indicate that the time deltas between
	 * reported samples can vary based on the input report rate.
	 *
	 * The solution employed in this driver is to keep track of the average
	 * time delta between IMU input reports. In testing, this value has
	 * proven to be stable, staying at 15ms or 11ms, though other hardware
	 * configurations and bluetooth stacks could potentially see other rates
	 * (hopefully this will become more clear as more people use the
	 * driver).
	 *
	 * Keeping track of the average report delta allows us to submit our
	 * timestamps to userspace based on that. Each report contains 3
	 * samples, so the IMU sampling rate should be avg_time_delta/3. We can
	 * also use this average to detect events where we have dropped a
	 * packet. The userspace timestamp for the samples will be adjusted
	 * accordingly to prevent unwanted behvaior.
	 */
	if (!ctlr->imu_first_packet_received) {
		ctlr->imu_timestamp_us = 0;
		ctlr->imu_delta_samples_count = 0;
		ctlr->imu_delta_samples_sum = 0;
		ctlr->imu_avg_delta_ms = JC_IMU_DFLT_AVG_DELTA_MS;
		ctlr->imu_first_packet_received = true;
	} else {
		unsigned int delta = msecs - last_msecs;
		unsigned int dropped_pkts;
		unsigned int dropped_threshold;

		/* Avg imu report delta housekeeping */
		ctlr->imu_delta_samples_sum += delta;
		ctlr->imu_delta_samples_count++;
		if (ctlr->imu_delta_samples_count >=
		    JC_IMU_SAMPLES_PER_DELTA_AVG) {
			ctlr->imu_avg_delta_ms = ctlr->imu_delta_samples_sum /
						 ctlr->imu_delta_samples_count;
			/* Don't ever want divide by zero shenanigans */
			if (ctlr->imu_avg_delta_ms == 0) {
				ctlr->imu_avg_delta_ms = 1;
				dev_warn(dev,
					 "Calculated avg imu delta of 0\n");
			}
			ctlr->imu_delta_samples_count = 0;
			ctlr->imu_delta_samples_sum = 0;
		}

		/* Useful for debugging IMU sample rate */
		dev_dbg(dev,
			"imu_report: ms=%u last_ms=%u delta=%u avg_delta=%u\n",
			msecs, last_msecs, delta, ctlr->imu_avg_delta_ms);

		/* Check if any packets have been dropped */
		dropped_threshold = ctlr->imu_avg_delta_ms * 3 / 2;
		dropped_pkts = (delta - min(delta, dropped_threshold)) /
				ctlr->imu_avg_delta_ms;
		ctlr->imu_timestamp_us += 1000 * ctlr->imu_avg_delta_ms;
		if (dropped_pkts > JC_IMU_DROPPED_PKT_WARNING) {
			dev_info(dev,
				 "Compensating for %u dropped IMU reports\n",
				 dropped_pkts);
			dev_info(dev,
				 "delta=%u avg_delta=%u\n",
				 delta, ctlr->imu_avg_delta_ms);
		}
	}
	ctlr->imu_last_pkt_ms = msecs;

	/* Each IMU input report contains 15 samples max */
	for (i = 0; i < rep->sixaxis_report_num; i++) {
		input_event(idev, EV_MSC, MSC_TIMESTAMP,
			    ctlr->imu_timestamp_us);

		/*
		 * These calculations (which use the controller's calibration
		 * settings to improve the final values) are based on those
		 * found in the community's reverse-engineering repo (linked at
		 * top of driver). For hid-nintendo, we make sure that the final
		 * value given to userspace is always in terms of the axis
		 * resolution we provided.
		 *
		 * Currently only the gyro calculations subtract the calibration
		 * offsets from the raw value itself. In testing, doing the same
		 * for the accelerometer raw values decreased accuracy.
		 *
		 * Note that the gyro values are multiplied by the
		 * precision-saving scaling factor to prevent large inaccuracies
		 * due to truncation of the resolution value which would
		 * otherwise occur. To prevent overflow (without resorting to 64
		 * bit integer math), the mult_frac macro is used.
		 */
		value[0] = mult_frac((JC_IMU_PREC_RANGE_SCALE *
				      (imu_data[i].gyro_x -
				       ctlr->gyro_cal.offset[0])),
				     ctlr->gyro_cal.scale[0],
				     ctlr->imu_cal_gyro_divisor[0]);
		value[1] = mult_frac((JC_IMU_PREC_RANGE_SCALE *
				      (imu_data[i].gyro_y -
				       ctlr->gyro_cal.offset[1])),
				     ctlr->gyro_cal.scale[1],
				     ctlr->imu_cal_gyro_divisor[1]);
		value[2] = mult_frac((JC_IMU_PREC_RANGE_SCALE *
				      (imu_data[i].gyro_z -
				       ctlr->gyro_cal.offset[2])),
				     ctlr->gyro_cal.scale[2],
				     ctlr->imu_cal_gyro_divisor[2]);

		value[3] = ((s32)imu_data[i].accel_x *
			    ctlr->accel_cal.scale[0]) /
			    ctlr->imu_cal_accel_divisor[0];
		value[4] = ((s32)imu_data[i].accel_y *
			    ctlr->accel_cal.scale[1]) /
			    ctlr->imu_cal_accel_divisor[1];
		value[5] = ((s32)imu_data[i].accel_z *
			    ctlr->accel_cal.scale[2]) /
			    ctlr->imu_cal_accel_divisor[2];

		dev_dbg(dev, "raw_gyro: g_x=%hd g_y=%hd g_z=%hd\n",
			imu_data[i].gyro_x, imu_data[i].gyro_y,
			imu_data[i].gyro_z);
		dev_dbg(dev, "raw_accel: a_x=%hd a_y=%hd a_z=%hd\n",
			imu_data[i].accel_x, imu_data[i].accel_y,
			imu_data[i].accel_z);

		/*
		 * The sio gamepad has 2 axes swapped, X and Y. This is due to
		 * the orientation of the IMU in the controller which is in
		 * in portrait. We also negate X axis value for that.
		 * Proper output based on orientation:
		 *   X: positive is pointing to right of console
		 *   Y: positive is pointing to top
		 *   Z: positive is pointing up (out of screen)
		 * The axes follow the right-hand rule with index pointing right.
		 */

		/* Set gyro */
		input_report_abs(idev, ABS_RX, -value[1]);
		input_report_abs(idev, ABS_RY, value[0]);
		input_report_abs(idev, ABS_RZ, value[2]);
		/* Set acceleration */
		input_report_abs(idev, ABS_X, -value[4]);
		input_report_abs(idev, ABS_Y, value[3]);
		input_report_abs(idev, ABS_Z, value[5]);
		input_sync(idev);

		/* Convert to micros and divide by samples per report. */
		ctlr->imu_timestamp_us += ctlr->imu_avg_delta_ms * 1000 /
					  rep->sixaxis_report_num;
	}
}

static void sio_parse_report(struct joycon_ctlr *ctlr,
			     struct sio_input_report *rep)
{
	struct input_dev *dev = ctlr->input;
	unsigned long flags;
	int btn_home = 0;
	u32 btns;
	u16 raw_x, raw_y, raw_rx, raw_ry;
	s32 x, y, rx, ry;

	dev_dbg(&ctlr->sdev->dev, "parse_report()\n");

	spin_lock_irqsave(&ctlr->lock, flags);

	ctlr->last_input_report_msecs = jiffies_to_msecs(jiffies);

	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* Parse the buttons and sticks */
	btns = field_extract(rep->button_status, 0, 24);
	btn_home = !gpio_get_value(ctlr->sio_home_gpio);

	/* Get raw stick values */
	raw_x = field_extract(rep->left_stick, 0, 12);
	raw_y = field_extract(rep->left_stick + 1, 4, 12);
	/* Map the stick values */
	x = joycon_map_stick_val(&ctlr->stick_cal_x, raw_x);
	y = joycon_map_stick_val(&ctlr->stick_cal_y, raw_y);
	/* Report sticks */
	input_report_abs(dev, ABS_X, x);
	input_report_abs(dev, ABS_Y, y);

	/* Get raw stick values */
	raw_rx = field_extract(rep->right_stick, 0, 12);
	raw_ry = field_extract(rep->right_stick + 1, 4, 12);
	/* Map stick values */
	rx = -joycon_map_stick_val(&ctlr->right_stick_cal_x, raw_rx);
	ry = -joycon_map_stick_val(&ctlr->right_stick_cal_y, raw_ry);
	/* Report sticks */
	input_report_abs(dev, ABS_RX, rx);
	input_report_abs(dev, ABS_RY, ry);

	/* Report buttons */
	input_report_key(dev, BTN_TL, btns & JC_BTN_L);
	input_report_key(dev, BTN_TL2, btns & JC_BTN_ZL);
	input_report_key(dev, BTN_SELECT, btns & JC_BTN_MINUS);
	input_report_key(dev, BTN_THUMBL, btns & JC_BTN_LSTICK);
	input_report_key(dev, BTN_Z, btns & JC_BTN_CAP);
	input_report_key(dev, BTN_DPAD_DOWN, btns & JC_BTN_DOWN);
	input_report_key(dev, BTN_DPAD_UP, btns & JC_BTN_UP);
	input_report_key(dev, BTN_DPAD_RIGHT, btns & JC_BTN_RIGHT);
	input_report_key(dev, BTN_DPAD_LEFT, btns & JC_BTN_LEFT);

	/* Report buttons */
	input_report_key(dev, BTN_TR, btns & JC_BTN_R);
	input_report_key(dev, BTN_TR2, btns & JC_BTN_ZR);
	input_report_key(dev, BTN_START, btns & JC_BTN_PLUS);
	input_report_key(dev, BTN_THUMBR, btns & JC_BTN_RSTICK);
	input_report_key(dev, BTN_MODE, btn_home);
	input_report_key(dev, BTN_WEST, btns & JC_BTN_Y);
	input_report_key(dev, BTN_NORTH, btns & JC_BTN_X);
	input_report_key(dev, BTN_EAST, btns & JC_BTN_A);
	input_report_key(dev, BTN_SOUTH, btns & JC_BTN_B);

	input_sync(dev);

	/*
	 * Immediately after receiving a report is the most reliable time to
	 * send a subcommand to the controller. Wake any subcommand senders
	 * waiting for a report.
	 */
	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->received_input_report = true;
	spin_unlock_irqrestore(&ctlr->lock, flags);
	wake_up(&ctlr->wait);

	/* Parse IMU data if present */
	sio_parse_imu_report(ctlr, rep);
}

static int joycon_request_input_report(struct joycon_ctlr *ctlr)
{
	u8 hdata[] = {0x00, 0x01, 0x00, 0x00, 0x69};
	u8 data[] = {JC_SUBCMD_REQUEST_REPORT};
	struct device *dev = &ctlr->sdev->dev;
	int ret;

	dev_dbg(dev, "Requesting input report\n");
	/*
	 * Intentionally don't lock the output mutex. We want to send no matter
	 * what. We don't want to deadlock with other senders which are waiting
	 * to be woken up from parse_report().
	 */
	ret = joycon_send_packet(ctlr, JC_CMD_EXTRET, hdata, sizeof(hdata),
				 data, sizeof(data), HZ/4, false);
	if (ret)
		dev_err(dev, "Failed to request input report; ret=%d\n", ret);
	return ret;
}

static int sio_request_input_report(struct joycon_ctlr *ctlr)
{
	struct device *dev = &ctlr->sdev->dev;
	int ret;

	dev_dbg(dev, "Requesting sio input report\n");
	/*
	 * Intentionally don't lock the output mutex. We want to send no matter
	 * what. We don't want to deadlock with other senders which are waiting
	 * to be woken up from parse_report().
	 */
	ctlr->msg_type = JOYCON_MSG_TYPE_SUBCMD;
	ctlr->uart_cmd_match = JC_SIO_CMD_INPUTREPORT | JC_SIO_CMD_ACK;
	ret = sio_send_packet(ctlr, JC_SIO_CMD_INPUTREPORT, NULL, 0, NULL, 0,
			      HZ/4, false);
	if (ret)
		dev_err(dev, "Failed to request input report; ret=%d\n", ret);
	return ret;
}

static int hori_request_input_report(struct joycon_ctlr *ctlr)
{
	int ret;
	u8 hdata[] = {0x01};
	struct device *dev = &ctlr->sdev->dev;

	dev_dbg(dev, "Requesting hori input report\n");
	/*
	 * Intentionally don't lock the output mutex. We want to send no matter
	 * what. We don't want to deadlock with other senders which are waiting
	 * to be woken up from parse_report().
	 */
	ret = joycon_send_packet(ctlr, JC_CMD_HORIINPUTREPORT, hdata, sizeof(hdata),
				 NULL, 0, HZ/4, false);
	if (ret)
		dev_err(dev, "Failed to request hori input report; ret=%d\n", ret);
	return ret;
}

static void joycon_disconnect(struct joycon_ctlr *ctlr)
{
	struct device *dev = &ctlr->sdev->dev;
	int i;
	unsigned long flags;

	dev_info(dev, "Joy-con disconnected - unregistering\n");

	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->ctlr_removed = true;
	ctlr->ctlr_state = JOYCON_CTLR_STATE_INIT;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	dev_info(dev, "Removing input device\n");
	/* Remove input imu device */
	if (ctlr->imu_input) {
		input_unregister_device(ctlr->imu_input);
		ctlr->imu_input = NULL;
	}
	/* Remove input device */
	if (ctlr->input) {
		input_unregister_device(ctlr->input);
		ctlr->input = NULL;
	}
	if (ctlr->mac_addr_str) {
		devm_kfree(dev, ctlr->mac_addr_str);
		ctlr->mac_addr_str = NULL;
	}

	if (!ctlr->is_hori && !ctlr->is_sio) {
		dev_info(dev, "Removing LEDs\n");
		flush_workqueue(ctlr->detection_queue);
		/* Remove LEDS */
		for (i = 0; i < JC_NUM_LEDS; i++) {
			struct led_classdev *led = &ctlr->leds[i];
			struct led_classdev empty = { 0 };

			devm_led_classdev_unregister(dev, led);
			ctlr->leds[i] = empty;
			if (ctlr->led_names[i]) {
				devm_kfree(dev, ctlr->led_names[i]);
				ctlr->led_names[i] = NULL;
			}
		}
		if (ctlr->ctlr_type == JOYCON_TYPE_RIGHT) {
			struct led_classdev empty = { 0 };

			devm_led_classdev_unregister(dev, &ctlr->home_led);
			ctlr->home_led = empty;
			if (ctlr->home_led_name) {
				devm_kfree(dev, ctlr->home_led_name);
				ctlr->home_led_name = NULL;
			}
		}

		dev_info(dev, "Removing power supply\n");
		/* Remove power supply */
		if (ctlr->battery) {
			power_supply_unregister(ctlr->battery);
			ctlr->battery = NULL;
			if (ctlr->battery_desc_name) {
				devm_kfree(dev, ctlr->battery_desc_name);
				ctlr->battery_desc_name = NULL;
			}
		}
	}

	/* Set POR for sio */
	if (ctlr->is_sio)
		gpio_set_value(ctlr->sio_por_gpio, 0);
}

struct serport {
	struct tty_port *port;
	struct tty_struct *tty;
	struct tty_driver *tty_drv;
	int tty_idx;
	unsigned long flags;
};

static int joycon_set_baudrate(struct joycon_ctlr *ctlr, unsigned int speed)
{
	struct serport *serport = serdev_controller_get_drvdata(ctlr->sdev->ctrl);
	struct tty_struct *tty = serport->tty;
	struct ktermios ktermios = tty->termios;

	int ret;

	/* Manually clear/set dual STOP bit depending on baudrate */
	if (speed != JC_UART_BAUD_HIGH)
		ktermios.c_cflag &= ~CSTOPB;
	else
		ktermios.c_cflag |= CSTOPB;

	tty_set_termios(tty, &ktermios);

	ret = serdev_device_set_baudrate(ctlr->sdev, speed);

	return ret;
}

static irqreturn_t joycon_detection_irq(int irq, void *dev_id)
{
	struct joycon_ctlr *ctlr = (struct joycon_ctlr *)dev_id;

	if (ctlr->ctlr_state != JOYCON_CTLR_STATE_INIT)
		return IRQ_HANDLED;

	/* Check if interrupt was caused by detection enable pin in SPIO */
	if (gpio_get_value(ctlr->detect_gpio))
		return IRQ_HANDLED;

	/* Disable interrupt */
	disable_irq_nosync(ctlr->detect_irq);

	/* Restore UART TX pin to SPIO */
	gpio_free(ctlr->detect_en_gpio);
	ctlr->detect_en_req = false;

	queue_delayed_work(ctlr->detection_queue,
			   &ctlr->detection_worker, 0);
	return IRQ_HANDLED;
}

/* Approx. 3 seconds which is the time it takes for Joy-Con to reboot */
#define JOYCON_IRQ_DETECTION_RETRIES 30

static int joycon_enter_detection(struct joycon_ctlr *ctlr)
{
	int ret;
	struct device *dev = &ctlr->sdev->dev;

	ret = joycon_set_baudrate(ctlr, JC_UART_BAUD_LOW);
	if (ret != JC_UART_BAUD_LOW) {
		dev_err(dev, "Failed to set initial serial baudrate; ret=%d\n",
			ret);
		return -EINVAL;
	}
	ctlr->ctlr_state = JOYCON_CTLR_STATE_INIT;

	/* Start charging */
	if (!IS_ERR_OR_NULL(ctlr->charger_reg) &&
	    !regulator_is_enabled(ctlr->charger_reg) &&
	    regulator_enable(ctlr->charger_reg))
		dev_err(dev, "Failed to enable charger\n");

	/* Enter interrupt based detection */
	if (gpio_is_valid(ctlr->detect_gpio) &&
	    gpio_is_valid(ctlr->detect_en_gpio)) {
		ret = gpio_request(ctlr->detect_en_gpio, "jc-detect-en");
		if (ret < 0) {
			dev_err(dev, "Failed to enable detect pin; ret=%d\n",
				ret);
			return ret;
		}

		ret = gpio_direction_input(ctlr->detect_en_gpio);
		if (ret < 0) {
			gpio_free(ctlr->detect_en_gpio);
			dev_err(dev, "Failed to set gpio en direction; ret=%d\n",
				ret);
			return ret;
		}

		/* Wait for detection pin to settle */
		msleep(DETECT_PIN_SETTLE_TIME_MS * 2);

		ctlr->detection_irq_retries = JOYCON_IRQ_DETECTION_RETRIES;

		/* Check if already plugged-in */
		if (!gpio_get_value(ctlr->detect_gpio)) {
			gpio_free(ctlr->detect_en_gpio);
			goto queue_detection;
		}

		/* IRQ based detection */
		ctlr->detect_en_req = true;
		enable_irq(ctlr->detect_irq);

		return 0;
	}

queue_detection:
	/* Always queue if no detection pin or Sio */
	queue_delayed_work(ctlr->detection_queue, &ctlr->detection_worker, 0);

	return 0;
}

static void joycon_input_poller(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work,
						  work);
	struct joycon_ctlr *ctlr = container_of(dwork, struct joycon_ctlr,
						input_worker);
	struct device *dev = &ctlr->sdev->dev;
	unsigned int timediff;
	bool suspending, removed;
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	suspending = ctlr->suspending;
	removed = ctlr->ctlr_removed;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	if (suspending || removed)
		return;

	timediff = jiffies_to_msecs(jiffies) - ctlr->last_input_report_msecs;
	if (timediff > 500) {
		dev_info(dev, "Joy-con disconnected\n");
		joycon_disconnect(ctlr);
		if (joycon_enter_detection(ctlr))
			dev_err(dev, "Failed to re-enter detection\n");
		return;
	}

	if (!ctlr->is_hori && !ctlr->is_sio) {
		joycon_request_input_report(ctlr);
	} else if (ctlr->is_sio) {
		sio_request_input_report(ctlr);
	} else {
		hori_request_input_report(ctlr);
	}
	queue_delayed_work(ctlr->input_queue, &ctlr->input_worker,
			   msecs_to_jiffies(!ctlr->is_sio ? 15 : 8));
}

static int joycon_send_rumble_data(struct joycon_ctlr *ctlr)
{
	int ret;
	unsigned long flags;
	__be16 subcmd_size;
	struct device *dev = &ctlr->sdev->dev;
	struct joycon_subcmd_request *subcmd;
	u8 buffer[sizeof(*subcmd)] = { 0 };
	u32 rumble[2];

	subcmd = (struct joycon_subcmd_request *)buffer;

	spin_lock_irqsave(&ctlr->lock, flags);
	/*
	 * If the controller has been removed, just return ENODEV so the LED
	 * subsystem doesn't print invalid errors on removal.
	 */
	if (ctlr->ctlr_removed) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return -ENODEV;
	}

	/* Check if device has available buffers */
	/*! TODO: find out how it works.
	if (!ctlr->rumble_buf_avail) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return -EBUSY;
	}
	*/

	memcpy(subcmd->rumble_data, ctlr->rumble_data[ctlr->rumble_queue_tail],
	       JC_RUMBLE_DATA_SIZE);
	/* Set keep alive flag */
	memcpy(&rumble, subcmd->rumble_data, sizeof(rumble));
	if (ctlr->ctlr_type == JOYCON_TYPE_LEFT) {
		ctlr->rumble_keep_alive = (rumble[0] & JC_RUMBLE_MASK) !=
					  JC_RUMBLE_ZERO_AMP;
	} else {
		ctlr->rumble_keep_alive = (rumble[1] & JC_RUMBLE_MASK) !=
					  JC_RUMBLE_ZERO_AMP;
	}
	spin_unlock_irqrestore(&ctlr->lock, flags);

	subcmd->output_id = JC_OUTPUT_RUMBLE_ONLY;
	subcmd->packet_num = ctlr->subcmd_num;
	if (++ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;

	subcmd_size = cpu_to_be16(sizeof(*subcmd) - 1);

	ret = joycon_send_packet(ctlr, JC_CMD_EXTRET,
				 (u8 *)&subcmd_size, sizeof(subcmd_size),
				 (u8 *)subcmd, sizeof(*subcmd) - 1,
				 ctlr->rumble_keep_alive ? HZ : HZ/4, false);
	if (ret < 0)
		dev_dbg(dev, "Send subcommand failed; ret=%d\n", ret);
	return ret;
}

static void joycon_rumble_worker(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work,
						  work);
	struct joycon_ctlr *ctlr = container_of(dwork, struct joycon_ctlr,
						rumble_worker);
	unsigned long flags;
	bool again = true;
	int ret = 0;

	ret = joycon_send_rumble_data(ctlr);
	/* -ENODEV means the controller was just unplugged
	 * -EBUSY  means the controller has no buffer available
	 */
	if (ret == -ENODEV)
		return;

	if (ret == -EBUSY)
		goto reschedule;

	spin_lock_irqsave(&ctlr->lock, flags);
	if (ret < 0) {
		dev_warn(&ctlr->sdev->dev,
			 "Failed to set rumble; e=%d", ret);
	}

	ctlr->rumble_msecs = jiffies_to_msecs(jiffies);
	if (ctlr->rumble_queue_tail != ctlr->rumble_queue_head) {
		if (++ctlr->rumble_queue_tail >= JC_RUMBLE_QUEUE_SIZE)
			ctlr->rumble_queue_tail = 0;
	} else {
		again = false;
	}
	spin_unlock_irqrestore(&ctlr->lock, flags);

reschedule:
	/* Max 1 rumble command per 5 ms. */
	if (again) {
		queue_delayed_work(ctlr->rumble_queue, &ctlr->rumble_worker,
				   msecs_to_jiffies(5));
	}
}

#if IS_ENABLED(CONFIG_JOYCON_SERDEV_FF)
static struct joycon_rumble_freq_data joycon_find_rumble_freq(u16 freq)
{
	const size_t length = ARRAY_SIZE(joycon_rumble_frequencies);
	const struct joycon_rumble_freq_data *data = joycon_rumble_frequencies;
	int i = 0;

	if (freq > data[0].freq) {
		for (i = 1; i < length - 1; i++) {
			if (freq > data[i - 1].freq && freq <= data[i].freq)
				break;
		}
	}

	return data[i];
}

static struct joycon_rumble_amp_data joycon_find_rumble_amp(u16 amp)
{
	const size_t length = ARRAY_SIZE(joycon_rumble_amplitudes);
	const struct joycon_rumble_amp_data *data = joycon_rumble_amplitudes;
	int i = 0;

	if (amp > data[0].amp) {
		for (i = 1; i < length - 1; i++) {
			if (amp > data[i - 1].amp && amp <= data[i].amp)
				break;
		}
	}

	return data[i];
}

static void joycon_encode_rumble(u8 *data,
				 u16 freq_low, u16 freq_high,
				 u16 amp, bool left)
{
	struct joycon_rumble_freq_data freq_data_low;
	struct joycon_rumble_freq_data freq_data_high;
	struct joycon_rumble_amp_data amp_data_h;
	struct joycon_rumble_amp_data amp_data_l;
	u16 amp_mod;

	freq_data_low = joycon_find_rumble_freq(freq_low);
	freq_data_high = joycon_find_rumble_freq(freq_high);

	if (left) {
		amp_mod = (u32)amp * JOYCON_RUMBLE_AMP_COEFF1 / 100;
		amp_data_h = joycon_find_rumble_amp(amp_mod);
		amp_mod = (u32)amp * JOYCON_RUMBLE_AMP_COEFF0 / 100;
		amp_data_l = joycon_find_rumble_amp(amp_mod);
	} else {
		amp_mod = (u32)amp * JOYCON_RUMBLE_AMP_COEFF2 / 100;
		amp_data_h = joycon_find_rumble_amp(amp_mod);
		amp_mod = (u32)amp * JOYCON_RUMBLE_AMP_COEFF3 / 100;
		amp_data_l = joycon_find_rumble_amp(amp_mod);
	}

	data[0] = (freq_data_high.high >> 8) & 0xFF;
	data[1] = (freq_data_high.high & 0xFF) | amp_data_h.high;

	data[2] = freq_data_low.low | ((amp_data_l.low >> 8) & 0xFF);
	data[3] = amp_data_l.low & 0xFF;
}

static const u16 JOYCON_MAX_RUMBLE_HIGH_FREQ	= 1253;
static const u16 JOYCON_MIN_RUMBLE_HIGH_FREQ	= 82;
static const u16 JOYCON_MAX_RUMBLE_LOW_FREQ	= 626;
static const u16 JOYCON_MIN_RUMBLE_LOW_FREQ	= 41;

static void joycon_clamp_rumble_freqs(struct joycon_ctlr *ctlr)
{
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->rumble_ll_freq = clamp(ctlr->rumble_ll_freq,
				     JOYCON_MIN_RUMBLE_LOW_FREQ,
				     JOYCON_MAX_RUMBLE_LOW_FREQ);
	ctlr->rumble_lh_freq = clamp(ctlr->rumble_lh_freq,
				     JOYCON_MIN_RUMBLE_HIGH_FREQ,
				     JOYCON_MAX_RUMBLE_HIGH_FREQ);
	ctlr->rumble_rl_freq = clamp(ctlr->rumble_rl_freq,
				     JOYCON_MIN_RUMBLE_LOW_FREQ,
				     JOYCON_MAX_RUMBLE_LOW_FREQ);
	ctlr->rumble_rh_freq = clamp(ctlr->rumble_rh_freq,
				     JOYCON_MIN_RUMBLE_HIGH_FREQ,
				     JOYCON_MAX_RUMBLE_HIGH_FREQ);
	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static int joycon_set_rumble(struct joycon_ctlr *ctlr, u16 amp_weak, u16 amp_strong,
			     bool schedule_now)
{
	u8 data[JC_RUMBLE_DATA_SIZE];
	u16 amp;
	u16 freq_r_low;
	u16 freq_r_high;
	u16 freq_l_low;
	u16 freq_l_high;
	bool removed;
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	freq_r_low = ctlr->rumble_rl_freq;
	freq_r_high = ctlr->rumble_rh_freq;
	freq_l_low = ctlr->rumble_ll_freq;
	freq_l_high = ctlr->rumble_lh_freq;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* Right joy-con */
	amp = amp_weak * (u32)JOYCON_RUMBLE_MAX_AMP / 65535;
	joycon_encode_rumble(data + 4, freq_r_low, freq_r_high, amp, false);

	/* Left joy-con */
	amp = amp_strong * (u32)JOYCON_RUMBLE_MAX_AMP / 65535;
	joycon_encode_rumble(data, freq_l_low, freq_l_high, amp, true);

	spin_lock_irqsave(&ctlr->lock, flags);
	if (++ctlr->rumble_queue_head >= JC_RUMBLE_QUEUE_SIZE)
		ctlr->rumble_queue_head = 0;
	memcpy(ctlr->rumble_data[ctlr->rumble_queue_head], data,
	       JC_RUMBLE_DATA_SIZE);
	removed = ctlr->ctlr_state == JOYCON_CTLR_STATE_INIT || ctlr->ctlr_removed;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* Don't wait for the periodic send (reduces latency) */
	if (schedule_now && !removed)
		queue_delayed_work(ctlr->rumble_queue, &ctlr->rumble_worker, 0);

	return 0;
}

static int joycon_play_effect(struct input_dev *dev, void *data,
						     struct ff_effect *effect)
{
	struct joycon_ctlr *ctlr = input_get_drvdata(dev);

	if (effect->type != FF_RUMBLE)
		return 0;

	return joycon_set_rumble(ctlr,
				 effect->u.rumble.weak_magnitude,
				 effect->u.rumble.strong_magnitude,
				 true);
}
#endif /* IS_ENABLED(CONFIG_JOYCON_SERDEV_FF) */

static const unsigned int joycon_button_inputs_left[] = {
	BTN_SELECT, BTN_Z, BTN_THUMBL,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
	BTN_TL, BTN_TL2,
	0 /* 0 signals end of array */
};

static const unsigned int joycon_button_inputs_right[] = {
	BTN_START, BTN_MODE, BTN_THUMBR,
	BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
	BTN_TR, BTN_TR2,
	0 /* 0 signals end of array */
};

static const unsigned int joycon_button_inputs_sio[] = {
	BTN_SELECT, BTN_Z, BTN_THUMBL,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
	BTN_TL, BTN_TL2,
	BTN_START, BTN_MODE, BTN_THUMBR,
	BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
	BTN_TR, BTN_TR2,
	0 /* 0 signals end of array */
};

static const s16 DFLT_ACCEL_OFFSET = 0;
static const s16 DFLT_ACCEL_SCALE = 16384;
static const s16 DFLT_GYRO_OFFSET = 0;
static const s16 DFLT_GYRO_SCALE  = 13371;
static void sio_set_imu_calibration(struct joycon_ctlr *ctlr)
{
	int i;

	for (i = 0; i < 3; i++) {
		ctlr->accel_cal.offset[i] = DFLT_ACCEL_OFFSET;
		ctlr->accel_cal.scale[i] = DFLT_ACCEL_SCALE;
		ctlr->gyro_cal.offset[i] = DFLT_GYRO_OFFSET;
		ctlr->gyro_cal.scale[i] = DFLT_GYRO_SCALE;
	}

	for (i = 0; i < 3; i++) {
		ctlr->imu_cal_accel_divisor[i] = ctlr->accel_cal.scale[i] -
						ctlr->accel_cal.offset[i];
		ctlr->imu_cal_gyro_divisor[i] = ctlr->gyro_cal.scale[i] -
						ctlr->gyro_cal.offset[i];
	}
}

static int joycon_input_create(struct joycon_ctlr *ctlr)
{
	struct device *dev = &ctlr->sdev->dev;
	enum joycon_ctlr_type type = ctlr->ctlr_type;
	const char *name;
	u16 pid = 0;
	int ret;
	int i;

	switch (type) {
	case JOYCON_TYPE_LEFT:
		name = "Nintendo Switch Left Joy-Con Serial";
		pid = 0x2006;
		break;
	case JOYCON_TYPE_RIGHT:
		name = "Nintendo Switch Right Joy-Con Serial";
		pid = 0x2007;
		break;
	case JOYCON_TYPE_SIO:
		name = "Nintendo Switch Lite Gamepad";
		pid = 0xF123; /* Custom PID */
		break;
	default: /* Should be impossible */
		dev_err(dev, "Invalid Joy-con type\n");
		return -EINVAL;
	}

	ctlr->input = devm_input_allocate_device(dev);
	if (!ctlr->input)
		return -ENOMEM;
	ctlr->input->id.bustype = BUS_VIRTUAL;
	ctlr->input->id.vendor = 0x57e;
	ctlr->input->id.product = pid;
	ctlr->input->id.version = 0;
	ctlr->input->name = name;
	ctlr->input->uniq = ctlr->mac_addr_str;
	input_set_drvdata(ctlr->input, ctlr);

	/* Set up inputs */
	if (type == JOYCON_TYPE_LEFT) {
		/* Analog stick */
		input_set_abs_params(ctlr->input, ABS_X,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);
		input_set_abs_params(ctlr->input, ABS_Y,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);

		/* Set up buttons */
		for (i = 0; joycon_button_inputs_left[i] > 0; i++)
			input_set_capability(ctlr->input, EV_KEY,
					     joycon_button_inputs_left[i]);
	} else if (type == JOYCON_TYPE_RIGHT) {
		/* Analog stick */
		input_set_abs_params(ctlr->input, ABS_RX,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);
		input_set_abs_params(ctlr->input, ABS_RY,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);

		/* Set up buttons */
		for (i = 0; joycon_button_inputs_right[i] > 0; i++)
			input_set_capability(ctlr->input, EV_KEY,
					     joycon_button_inputs_right[i]);
	} else if (type == JOYCON_TYPE_SIO) {
		/* Left analog stick */
		input_set_abs_params(ctlr->input, ABS_X,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);
		input_set_abs_params(ctlr->input, ABS_Y,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);
		/* Right analog stick */
		input_set_abs_params(ctlr->input, ABS_RX,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);
		input_set_abs_params(ctlr->input, ABS_RY,
				     -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				     JC_STICK_FUZZ, JC_STICK_FLAT);

		/* Set up buttons */
		for (i = 0; joycon_button_inputs_sio[i] > 0; i++)
			input_set_capability(ctlr->input, EV_KEY,
					     joycon_button_inputs_sio[i]);
	}

#if IS_ENABLED(CONFIG_JOYCON_SERDEV_FF)
	/* Set up rumble */
	if (!ctlr->is_hori && !ctlr->is_sio) {
		input_set_capability(ctlr->input, EV_FF, FF_RUMBLE);
		input_ff_create_memless(ctlr->input, NULL, joycon_play_effect);
		ctlr->rumble_ll_freq = JC_RUMBLE_DFLT_LOW_FREQ;
		ctlr->rumble_lh_freq = JC_RUMBLE_DFLT_HIGH_FREQ;
		ctlr->rumble_rl_freq = JC_RUMBLE_DFLT_LOW_FREQ;
		ctlr->rumble_rh_freq = JC_RUMBLE_DFLT_HIGH_FREQ;
		joycon_clamp_rumble_freqs(ctlr);
		joycon_set_rumble(ctlr, 0, 0, false);
		ctlr->rumble_msecs = jiffies_to_msecs(jiffies);
	}
#endif

	ret = input_register_device(ctlr->input);
	if (ret)
		return ret;

	if (ctlr->is_sio) {
		/* Configure the imu input device */
		ctlr->imu_input = devm_input_allocate_device(dev);
		if (!ctlr->imu_input)
			return -ENOMEM;

		/* Set calibration data */
		sio_set_imu_calibration(ctlr);

		ctlr->imu_input->dev.parent = dev;
		ctlr->imu_input->id.bustype = BUS_VIRTUAL;
		ctlr->imu_input->id.vendor = 0x57e;
		ctlr->imu_input->id.product = pid;
		ctlr->imu_input->id.version = 0;
		ctlr->imu_input->uniq = ctlr->mac_addr_str;
		ctlr->imu_input->name = "Nintendo Switch Lite Gamepad IMU";
		input_set_drvdata(ctlr->imu_input, ctlr);

		/* Configure imu axes */
		input_set_abs_params(ctlr->imu_input, ABS_X,
				     -JC_IMU_MAX_ACCEL_MAG, JC_IMU_MAX_ACCEL_MAG,
				     JC_IMU_ACCEL_FUZZ, JC_IMU_ACCEL_FLAT);
		input_set_abs_params(ctlr->imu_input, ABS_Y,
				     -JC_IMU_MAX_ACCEL_MAG, JC_IMU_MAX_ACCEL_MAG,
				     JC_IMU_ACCEL_FUZZ, JC_IMU_ACCEL_FLAT);
		input_set_abs_params(ctlr->imu_input, ABS_Z,
				     -JC_IMU_MAX_ACCEL_MAG, JC_IMU_MAX_ACCEL_MAG,
				     JC_IMU_ACCEL_FUZZ, JC_IMU_ACCEL_FLAT);
		input_abs_set_res(ctlr->imu_input, ABS_X, JC_IMU_ACCEL_RES_PER_G);
		input_abs_set_res(ctlr->imu_input, ABS_Y, JC_IMU_ACCEL_RES_PER_G);
		input_abs_set_res(ctlr->imu_input, ABS_Z, JC_IMU_ACCEL_RES_PER_G);

		input_set_abs_params(ctlr->imu_input, ABS_RX,
				     -JC_IMU_MAX_GYRO_MAG, JC_IMU_MAX_GYRO_MAG,
				     JC_IMU_GYRO_FUZZ, JC_IMU_GYRO_FLAT);
		input_set_abs_params(ctlr->imu_input, ABS_RY,
				     -JC_IMU_MAX_GYRO_MAG, JC_IMU_MAX_GYRO_MAG,
				     JC_IMU_GYRO_FUZZ, JC_IMU_GYRO_FLAT);
		input_set_abs_params(ctlr->imu_input, ABS_RZ,
				     -JC_IMU_MAX_GYRO_MAG, JC_IMU_MAX_GYRO_MAG,
				     JC_IMU_GYRO_FUZZ, JC_IMU_GYRO_FLAT);

		input_abs_set_res(ctlr->imu_input, ABS_RX, JC_IMU_GYRO_RES_PER_DPS);
		input_abs_set_res(ctlr->imu_input, ABS_RY, JC_IMU_GYRO_RES_PER_DPS);
		input_abs_set_res(ctlr->imu_input, ABS_RZ, JC_IMU_GYRO_RES_PER_DPS);

		__set_bit(EV_MSC, ctlr->imu_input->evbit);
		__set_bit(MSC_TIMESTAMP, ctlr->imu_input->mscbit);
		__set_bit(INPUT_PROP_ACCELEROMETER, ctlr->imu_input->propbit);

		ret = input_register_device(ctlr->imu_input);
		if (ret)
			return ret;
	}

	return 0;
}

static void joycon_led_brightness_set_noblock(struct led_classdev *led,
					      enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct serdev_device *sdev = to_serdev_device(dev);
	struct joycon_ctlr *ctlr;
	unsigned long flags;
	struct joycon_led_queue_item *queue_item;


	ctlr = serdev_device_get_drvdata(sdev);
	if (!ctlr) {
		dev_err(dev, "No controller data\n");
		return;
	}

	spin_lock_irqsave(&ctlr->lock, flags);
	if (ctlr->ctlr_state != JOYCON_CTLR_STATE_READ) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return;
	}
	queue_item = &ctlr->led_queue[ctlr->led_queue_head];
	queue_item->led = led;
	queue_item->brightness = brightness;
	if (++ctlr->led_queue_head >= JC_LED_QUEUE_SIZE)
		ctlr->led_queue_head = 0;
	queue_work(ctlr->detection_queue, &ctlr->led_worker);
	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static int joycon_player_led_brightness_set(struct led_classdev *led,
					    enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct serdev_device *sdev = to_serdev_device(dev);
	struct joycon_ctlr *ctlr;
	int val = 0;
	int i;
	int ret;
	int num;

	ctlr = serdev_device_get_drvdata(sdev);
	if (!ctlr) {
		dev_err(dev, "No controller data\n");
		return -ENODEV;
	}

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_INIT || ctlr->ctlr_removed)
		return 0;

	/* Determine which player led this is */
	for (num = 0; num < JC_NUM_LEDS; num++) {
		if (&ctlr->leds[num] == led)
			break;
	}
	if (num >= JC_NUM_LEDS)
		return -EINVAL;

	mutex_lock(&ctlr->output_mutex);
	for (i = 0; i < JC_NUM_LEDS; i++) {
		if (i == num)
			val |= brightness << i;
		else
			val |= ctlr->leds[i].brightness << i;
	}
	ret = joycon_set_player_leds(ctlr, 0, val);
	mutex_unlock(&ctlr->output_mutex);

	return ret;
}

static int joycon_home_led_brightness_set(struct led_classdev *led,
					  enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct serdev_device *sdev = to_serdev_device(dev);
	struct joycon_ctlr *ctlr;
	struct joycon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 5] = { 0 };
	u8 *data;
	int ret;

	ctlr = serdev_device_get_drvdata(sdev);
	if (!ctlr) {
		dev_err(dev, "No controller data\n");
		return -ENODEV;
	}

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_INIT || ctlr->ctlr_removed)
		return 0;

	req = (struct joycon_subcmd_request *)buffer;
	req->subcmd_id = JC_SUBCMD_SET_HOME_LIGHT;
	data = req->data;
	data[0] = 0x01;
	data[1] = brightness << 4;
	data[2] = brightness | (brightness << 4);
	data[3] = 0x11;
	data[4] = 0x11;

	dev_dbg(dev, "Setting home led brightness\n");
	mutex_lock(&ctlr->output_mutex);
	ret = joycon_send_subcmd(ctlr, req, 5, HZ/4);
	mutex_unlock(&ctlr->output_mutex);

	return ret;
}

static const char * const joycon_player_led_names[] = {
	"player1",
	"player2",
	"player3",
	"player4"
};

static void joycon_led_worker(struct work_struct *work)
{
	struct joycon_ctlr *ctlr = container_of(work, struct joycon_ctlr,
							led_worker);
	unsigned long flags;

	spin_lock_irqsave(&ctlr->lock, flags);
	while (ctlr->led_queue_tail != ctlr->led_queue_head) {
		struct joycon_led_queue_item *item;

		spin_unlock_irqrestore(&ctlr->lock, flags);
		item = ctlr->led_queue + ctlr->led_queue_tail;
		if (item->led == &ctlr->home_led)
			joycon_home_led_brightness_set(item->led,
						       item->brightness);
		else
			joycon_player_led_brightness_set(item->led,
							 item->brightness);
		spin_lock_irqsave(&ctlr->lock, flags);
		if (++ctlr->led_queue_tail >= JC_LED_QUEUE_SIZE)
			ctlr->led_queue_tail = 0;
	}
	spin_unlock_irqrestore(&ctlr->lock, flags);
}

static DEFINE_MUTEX(joycon_input_num_mutex);
#define LED_ON	(1)
static int joycon_leds_create(struct joycon_ctlr *ctlr)
{
	struct device *dev = &ctlr->sdev->dev;
	const char *d_name = dev_name(dev);
	struct led_classdev *led;
	char *name;
	int ret = 0;
	int i;
	static int input_num = 1;

	/* Set the default controller player leds based on controller number */
	mutex_lock(&joycon_input_num_mutex);
	mutex_lock(&ctlr->output_mutex);
	ret = joycon_set_player_leds(ctlr, 0, 0xF >> (4 - input_num));
	if (ret)
		dev_warn(dev, "Failed to set leds; ret=%d\n", ret);
	mutex_unlock(&ctlr->output_mutex);

	/* Configure the player LEDs */
	for (i = 0; i < JC_NUM_LEDS; i++) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s", d_name,
				      joycon_player_led_names[i]);
		if (!name) {
			mutex_unlock(&joycon_input_num_mutex);
			return -ENOMEM;
		}

		led = &ctlr->leds[i];
		ctlr->led_names[i] = name;
		led->name = name;
		led->brightness = ((i + 1) <= input_num) ? LED_ON : LED_OFF;
		led->max_brightness = LED_ON;
		led->brightness_set = joycon_led_brightness_set_noblock;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;

		ret = devm_led_classdev_register(dev, led);
		if (ret) {
			dev_err(dev, "Failed registering %s LED\n", led->name);
			mutex_unlock(&joycon_input_num_mutex);
			return ret;
		}
	}

	if (++input_num > 4)
		input_num = 1;
	mutex_unlock(&joycon_input_num_mutex);

	/* Configure the home LED */
	if (ctlr->ctlr_type == JOYCON_TYPE_RIGHT) {
		name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s", d_name, "home");
		if (!name)
			return ret;

		led = &ctlr->home_led;
		ctlr->home_led_name = name;
		led->name = name;
		led->brightness = 0;
		led->max_brightness = 0xF;
		led->brightness_set = joycon_led_brightness_set_noblock;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;
		ret = devm_led_classdev_register(dev, led);
		if (ret) {
			dev_err(dev, "Failed registering home led\n");
			return ret;
		}
		/* Set the home LED to 0 as default state */
		ret = joycon_home_led_brightness_set(led, 0);
		if (ret) {
			dev_err(dev, "Failed to set home LED dflt; ret=%d\n",
									ret);
			return ret;
		}
	}

	return 0;
}

static int joycon_battery_get_property(struct power_supply *supply,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct joycon_ctlr *ctlr = power_supply_get_drvdata(supply);
	unsigned long flags;
	int ret = 0;
	u8 capacity;
	bool charging;
	bool powered;

	spin_lock_irqsave(&ctlr->lock, flags);
	capacity = ctlr->battery_capacity;
	charging = ctlr->battery_charging;
	powered = ctlr->host_powered;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (capacity == POWER_SUPPLY_CAPACITY_LEVEL_FULL &&
			 powered)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property joycon_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
};

static int joycon_power_supply_create(struct joycon_ctlr *ctlr)
{
	struct device *dev = &ctlr->sdev->dev;
	struct power_supply_config supply_config = { .drv_data = ctlr, };
	const char * const name_fmt = "nintendo_switch_controller_battery_%s";
	int ret = 0;

	/* Set initially to unknown before receiving first input report */
	ctlr->battery_capacity = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	/* Configure the battery's description */
	ctlr->battery_desc.properties = joycon_battery_props;
	ctlr->battery_desc.num_properties =
					ARRAY_SIZE(joycon_battery_props);
	ctlr->battery_desc.get_property = joycon_battery_get_property;
	ctlr->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	ctlr->battery_desc.use_for_apm = 0;
	ctlr->battery_desc_name = devm_kasprintf(dev, GFP_KERNEL,
						 name_fmt,
						 dev_name(dev));
	if (!ctlr->battery_desc_name)
		return -ENOMEM;
	ctlr->battery_desc.name = ctlr->battery_desc_name;

	ctlr->battery = power_supply_register(dev,
					      &ctlr->battery_desc,
					      &supply_config);
	if (IS_ERR(ctlr->battery)) {
		ret = PTR_ERR(ctlr->battery);
		dev_err(dev, "Failed to register battery; ret=%d\n", ret);
		return ret;
	}
	power_supply_powers(ctlr->battery, dev);
	return 0;
}

static int joycon_read_mac(struct joycon_ctlr *ctlr)
{
	int ret;
	int i;
	int j;
	u8 request_mac_data[] = {JC_INIT_MAC};
	struct joycon_uart_packet *packet;

	ctlr->uart_cmd_match = JC_CMD_INITRET;
	ret = joycon_send_command(ctlr, JC_CMD_EXTSEND, request_mac_data,
				  sizeof(request_mac_data), HZ);
	if (ret) {
		dev_err(&ctlr->sdev->dev, "Failed to retrieve MAC; ret=%d\n",
			ret);
		return ret;
	}

	packet = (struct joycon_uart_packet *) ctlr->input_buf;
	if (packet->header_data[0] != JC_INIT_MAC) {
		dev_err(&ctlr->sdev->dev, "Invalid response to MAC request\n");
		return -EINVAL;
	}

	for (i = 5, j = 1; i >= 0; i--, j++)
		ctlr->mac_addr[i] = packet->data[j];

	/*
	 * Regular controller returns 0x01.
	 * HORI returns 0x22 or 0x21.
	 * If that is not enough we can also check:
	 *  mac == 00:00:00:00:00:00
	 */

	if (packet->data[0] == 0x21) {
		ctlr->ctlr_type = JOYCON_TYPE_LEFT;
		ctlr->is_hori = true;
		// Patch mac with L/R to ensure it's still somewhat unique
		ctlr->mac_addr[5] = JOYCON_TYPE_LEFT;
	} else if (packet->data[0] == 0x22) {
		ctlr->ctlr_type = JOYCON_TYPE_RIGHT;
		ctlr->is_hori = true;
		ctlr->mac_addr[5] = JOYCON_TYPE_RIGHT;
	} else {
		ctlr->is_hori = false;
	}

	ctlr->mac_addr_str = devm_kasprintf(&ctlr->sdev->dev, GFP_KERNEL,
					    "%02X:%02X:%02X:%02X:%02X:%02X",
					    ctlr->mac_addr[0],
					    ctlr->mac_addr[1],
					    ctlr->mac_addr[2],
					    ctlr->mac_addr[3],
					    ctlr->mac_addr[4],
					    ctlr->mac_addr[5]);
	if (!ctlr->mac_addr_str)
		return -ENOMEM;

	dev_info(&ctlr->sdev->dev, "Joy-con MAC = %s\n", ctlr->mac_addr_str);

	return 0;
}

/* Configures the Joy-con to use 3000000bps */
static int joycon_change_baud(struct joycon_ctlr *ctlr)
{
	int ret;
	u8 hdata[] = {JC_INIT_SET_BAUDRATE, 0x08, 0x00, 0x00, 0xBD};
	u8 data[] = {0xC0, 0xC6, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00}; /* 3Mbps */
	struct joycon_uart_packet *packet;

	dev_info(&ctlr->sdev->dev, "Increasing Joy-con baud to 3000000bps\n");

	ctlr->uart_cmd_match = JC_CMD_INITRET;
	ctlr->msg_type = JOYCON_MSG_TYPE_UART_CMD;
	ret = joycon_send_packet(ctlr, JC_CMD_EXTSEND,
				 hdata, sizeof(hdata), data, sizeof(data),
				 HZ, true);
	if (ret) {
		dev_err(&ctlr->sdev->dev, "Failed to set Joy-con baud; ret=%d",
			ret);
		return ret;
	}
	packet = (struct joycon_uart_packet *) ctlr->input_buf;
	if (packet->header_data[0] != JC_INIT_SET_BAUDRATE) {
		dev_err(&ctlr->sdev->dev,
			"Invalid baudrate set resp; val=0x%02x\n",
			packet->header_data[0]);
		return -EINVAL;
	}

	ret = joycon_set_baudrate(ctlr, JC_UART_BAUD_HIGH);
	if (ret != JC_UART_BAUD_HIGH) {
		dev_err(&ctlr->sdev->dev,
			"Failed to change serdev baudrate; ret=%d\n", ret);
		return -EINVAL;
	}
	return 0;
}

/* Send generic handshake step */
static int joycon_init_handshake(struct joycon_ctlr *ctlr, u8 handshake)
{
	int ret;
	u8 hdata[] = {handshake};
	struct joycon_uart_packet *packet;

	ctlr->uart_cmd_match = JC_CMD_INITRET;
	ret = joycon_send_command(ctlr, JC_CMD_EXTSEND, hdata,
				  sizeof(hdata), HZ);
	if (ret) {
		dev_err(&ctlr->sdev->dev,
			"Failed to retrieve handshake (0x%02x) resp; ret=%d\n",
			handshake, ret);
		return ret;
	}

	packet = (struct joycon_uart_packet *) ctlr->input_buf;
	if (packet->header_data[0] != handshake) {
		dev_err(&ctlr->sdev->dev,
			"Invalid handshake (0x%02x) resp; val=0x%02x\n",
			handshake, packet->header_data[0]);
		return -EINVAL;
	}

	return 0;
}

/* The final step in the handshake is to set hid interval */
static int joycon_init_hid_rate(struct joycon_ctlr *ctlr)
{
	int ret;
	u8 hdata[] = {JC_INIT_SET_HID_RATE, 0x04, 0x00, 0x00, 0x12};
	u8 data[] = {0x0F, 0x00, 0x00, 0x00};
	struct joycon_uart_packet *packet;

	ctlr->uart_cmd_match = JC_CMD_INITRET;
	ctlr->msg_type = JOYCON_MSG_TYPE_UART_CMD;
	ret = joycon_send_packet(ctlr, JC_CMD_EXTSEND,
				 hdata, sizeof(hdata), data, sizeof(data),
				 HZ, true);
	if (ret) {
		dev_err(&ctlr->sdev->dev,
			"Failed to finalize handshake; ret=%d",
			ret);
		return ret;
	}
	packet = (struct joycon_uart_packet *) ctlr->input_buf;
	if (packet->header_data[0] != JC_INIT_SET_HID_RATE) {
		dev_err(&ctlr->sdev->dev,
			"Invalid set rate resp; val=0x%02x\n",
			packet->header_data[0]);
		return -EINVAL;
	}
	return 0;
}

static int joycon_handshake(struct joycon_ctlr *ctlr)
{
	int ret;
	int baudret;
	struct device *dev = &ctlr->sdev->dev;
	u8 handshake_magic_start[] = {0xA1, 0xA2, 0xA3, 0xA4};
	u8 handshake_cmd_data[] = {0x02, 0x01, 0x7E};

	mutex_lock(&ctlr->output_mutex);

	/* Send the initial handshake magic bytes */
	ret = serdev_device_write(ctlr->sdev, handshake_magic_start,
				  sizeof(handshake_magic_start), HZ);
	if (ret < 0)
		goto exit;

	/* Send handshake command */
	ctlr->uart_cmd_match = JC_CMD_HANDSHAKE;
	ret = joycon_send_command(ctlr, JC_CMD_HANDSHAKE, handshake_cmd_data,
				  sizeof(handshake_cmd_data), HZ / 10);
	if (ret) {
		dev_dbg(dev, "Did not receive handshake response; ret=%d",
			ret);
		goto exit;
	}

	ret = joycon_read_mac(ctlr);
	if (ret)
		goto exit;

	if (!ctlr->is_hori) {
		/* Set higher baudrate */
		ret = joycon_change_baud(ctlr);
		if (ret)
			goto exit_restore_baud;

		/* Send hid disconnect */
		ret = joycon_init_handshake(ctlr, JC_INIT_HID_DISCONNECT);
		if (ret)
			goto exit_restore_baud;

		/* Send hid connect */
		ret = joycon_init_handshake(ctlr, JC_INIT_HID_CONNECT);
		if (ret)
			goto exit_restore_baud;

		/* Set hid interval */
		ret = joycon_init_hid_rate(ctlr);
		if (ret)
			goto exit_restore_baud;
		dev_info(dev, "completed handshake - Joycon\n");
		goto exit;
	} else {
		ctlr->stick_cal_x.center = HORI_STICK_CAL_CEN;
		ctlr->stick_cal_x.max = HORI_STICK_CAL_MAX;
		ctlr->stick_cal_x.min = HORI_STICK_CAL_MIN;

		ctlr->stick_cal_y.center = HORI_STICK_CAL_CEN;
		ctlr->stick_cal_y.max = HORI_STICK_CAL_MAX;
		ctlr->stick_cal_y.min = HORI_STICK_CAL_MIN;
		dev_info(dev, "completed handshake - Hori\n");
		goto exit;
	}

exit_restore_baud:
	dev_info(dev, "Returning to low baudrate for detection\n");
	baudret = joycon_set_baudrate(ctlr, JC_UART_BAUD_LOW);
	if (baudret != JC_UART_BAUD_LOW) {
		dev_err(&ctlr->sdev->dev,
			"Failed to change serdev baudrate; ret=%d\n", baudret);
	}
exit:
	mutex_unlock(&ctlr->output_mutex);
	return ret;
}

static int sio_handshake(struct joycon_ctlr *ctlr)
{
	int ret;
	struct device *dev = &ctlr->sdev->dev;
	u8 mac[] = {0x98, 0xb6, 0xe9, 0x53, 0x49, 0x4f};
	u8 version[] = {0x03, 0x04};

	mutex_lock(&ctlr->output_mutex);

	/* Set Sio POR low to reset. */
	gpio_set_value(ctlr->sio_por_gpio, 0);
	usleep_range(300, 300);
	gpio_set_value(ctlr->sio_por_gpio, 1);
	msleep(100);

	/* Send init command */
	ctlr->msg_type = JOYCON_MSG_TYPE_UART_CMD;
	ctlr->uart_cmd_match = JC_SIO_CMD_INIT | JC_SIO_CMD_ACK;
	ret = sio_send_packet(ctlr, JC_SIO_CMD_INIT, NULL, 0, NULL, 0, HZ, true);
	if (ret) {
		dev_err(dev, "Did not receive init response; ret=%d",
			ret);
		goto exit;
	}

	/* Send report version command */
	ctlr->msg_type = JOYCON_MSG_TYPE_UART_CMD;
	ctlr->uart_cmd_match = JC_SIO_CMD_VER_RPT | JC_SIO_CMD_ACK;
	ret = sio_send_packet(ctlr, JC_SIO_CMD_VER_RPT, version, sizeof(version),
			      NULL, 0, HZ, true);
	if (ret) {
		dev_err(dev, "Did not receive rpt version response; ret=%d",
			ret);
		goto exit;
	}

	ctlr->ctlr_type = JOYCON_TYPE_SIO;
	memcpy(ctlr->mac_addr, mac, sizeof(mac));
	ctlr->mac_addr_str = devm_kasprintf(&ctlr->sdev->dev, GFP_KERNEL,
					    "%02X:%02X:%02X:%02X:%02X:%02X",
					    ctlr->mac_addr[0],
					    ctlr->mac_addr[1],
					    ctlr->mac_addr[2],
					    ctlr->mac_addr[3],
					    ctlr->mac_addr[4],
					    ctlr->mac_addr[5]);
	if (!ctlr->mac_addr_str)
		return -ENOMEM;

	if (!ctlr->stick_cal_x.center) {
		ctlr->stick_cal_x.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_x.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_x.min = DFLT_STICK_CAL_MIN;

		ctlr->stick_cal_y.center = DFLT_STICK_CAL_CEN;
		ctlr->stick_cal_y.max = DFLT_STICK_CAL_MAX;
		ctlr->stick_cal_y.min = DFLT_STICK_CAL_MIN;
	}

	if (!ctlr->right_stick_cal_x.center) {
		ctlr->right_stick_cal_x.center = DFLT_STICK_CAL_CEN;
		ctlr->right_stick_cal_x.max = DFLT_STICK_CAL_MAX;
		ctlr->right_stick_cal_x.min = DFLT_STICK_CAL_MIN;

		ctlr->right_stick_cal_y.center = DFLT_STICK_CAL_CEN;
		ctlr->right_stick_cal_y.max = DFLT_STICK_CAL_MAX;
		ctlr->right_stick_cal_y.min = DFLT_STICK_CAL_MIN;
	}

	dev_info(dev, "Completed handshake - Sio\n");

exit:
	mutex_unlock(&ctlr->output_mutex);
	return ret;
}

static int joycon_post_handshake(struct joycon_ctlr *ctlr)
{
	int ret;
	int baudret;
	struct device *dev = &ctlr->sdev->dev;

	if (!ctlr->is_hori && !ctlr->is_sio) {
		mutex_lock(&ctlr->output_mutex);
		/* Determine which type of controller this is */
		ret = joycon_get_ctlr_info(ctlr);
		if (ret) {
			mutex_unlock(&ctlr->output_mutex);
			goto error;
		}

		/* Get controller calibration data, and parse it */
		ret = joycon_request_calibration(ctlr);
		if (ret) {
			/*
			 * We can function with default calibration, but it may be
			 * inaccurate. Provide a warning, and continue on.
			 */
			dev_warn(dev, "Analog stick positions may be inaccurate\n");
		}

		/* Enable rumble */
		ret = joycon_enable_rumble(ctlr, true);
		mutex_unlock(&ctlr->output_mutex);
		if (ret) {
			dev_err(dev, "Failed to enable rumble; ret=%d\n", ret);
			goto error;
		}

		/* Initialize the leds */
		ret = joycon_leds_create(ctlr);
		if (ret) {
			dev_err(dev, "Failed to create leds; ret=%d\n", ret);
			goto error;
		}

		/* Initialize the battery power supply */
		ret = joycon_power_supply_create(ctlr);
		if (ret) {
			dev_err(dev, "Failed to create power_supply; ret=%d\n", ret);
			goto error;
		}
	} else {
		/* SIO and HORI doesn't have any of:
		    - home, player leds
		    - rumble
		    - battery
		   and doesn't seem to require calibration.
		*/
	}

	ret = joycon_input_create(ctlr);
	if (ret) {
		dev_err(dev, "Failed to create input device; ret=%d\n", ret);
		goto error;
	}

	ctlr->last_input_report_msecs = jiffies_to_msecs(jiffies);
	ctlr->ctlr_state = JOYCON_CTLR_STATE_READ;
	ctlr->ctlr_removed = false;
	queue_delayed_work(ctlr->input_queue, &ctlr->input_worker, 0);

	dev_info(dev, "Post detect - success\n");
	return 0;

error:
	dev_info(dev, "Returning to low baudrate for detection\n");
	baudret = joycon_set_baudrate(ctlr, JC_UART_BAUD_LOW);
	if (baudret != JC_UART_BAUD_LOW) {
		dev_err(&ctlr->sdev->dev,
			"Failed to change serdev baudrate; ret=%d\n", baudret);
	}
	return ret;
}

static void joycon_detection_poller(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work,
						  work);
	struct joycon_ctlr *ctlr = container_of(dwork, struct joycon_ctlr,
							detection_worker);
	struct device *dev = &ctlr->sdev->dev;
	int ret;

	dev_dbg(dev, "Polling for Joy-con\n");

	if (!ctlr->is_sio)
		ret = joycon_handshake(ctlr);
	else
		ret = sio_handshake(ctlr);

	if (ret) {
		dev_dbg(dev, "Handshake failed; ret=%d\n", ret);
		goto retry;
	}
	ret = joycon_post_handshake(ctlr);
	if (ret) {
		dev_err(dev, "Failed post handshake procedure; ret=%d\n", ret);
		goto retry;
	}
	return;

retry:
	/* If irq based detection, decrease retries */
	if (ctlr->detection_irq_retries && !(--ctlr->detection_irq_retries)) {
		/* Stop trying and re-enter irq-based detection */
		joycon_enter_detection(ctlr);
		return;
	}

	queue_delayed_work(ctlr->detection_queue, &ctlr->detection_worker,
			   msecs_to_jiffies(!ctlr->is_sio ? 100 : 500));
}

static int joycon_serdev_receive_buf(struct serdev_device *serdev,
				     const unsigned char *buf, size_t len)
{
	struct joycon_ctlr *ctlr = serdev_device_get_drvdata(serdev);
	struct joycon_uart_packet *packet = (struct joycon_uart_packet *) buf;
	struct device *dev = &ctlr->sdev->dev;
	struct joycon_input_report *r;

	dev_dbg(dev, "Received uart data of size=%lu\n", len);
	/* Check if this is beginning of new packet */
	if (!ctlr->partial_pkt_len) {
		/* Obird workaround (they send some zeros in the beginning). */
		int j = 0;
		while (buf[j] == 0 && j < len)
			j++;
		packet = (struct joycon_uart_packet *) (buf + j);

		/* Have we received the entire packet? */
		if (len-j >= 4 && packet->size + 5 <= len-j) {
			if (packet->magic[0] != JC_UART_MAGIC_RX_0 ||
			    packet->magic[1] != JC_UART_MAGIC_RX_1 ||
			    packet->magic[2] != JC_UART_MAGIC_RX_2) {
				/* Toss out this packet if the magic is wrong */
				dev_warn(dev, "Received pkt has wrong magic\n");
				return len;
			}
			/* Proceed to process the packet without buffering */
			dev_dbg(dev, "Received whole uart packet\n");
		} else if (len-j > JC_MAX_RESP_SIZE) {
			/* Toss out this packet if malformed */
			dev_warn(dev, "Received pkt is malformed\n");
			return len;
		} else {
			/* This isn't yet the whole packet. */
			memcpy(ctlr->partial_pkt, buf+j, len-j);
			ctlr->partial_pkt_len = len-j;
			return len;
		}
	} else {
		if (ctlr->partial_pkt_len + len > JC_MAX_UART_PKT_SIZE) {
			dev_warn(dev, "Packet length too large; ignoring\n");
			ctlr->partial_pkt_len = 0;
			return len;
		}
		memcpy(ctlr->partial_pkt + ctlr->partial_pkt_len, buf, len);
		ctlr->partial_pkt_len += len;

		/* Need to be able to check packet size */
		if (ctlr->partial_pkt_len < 4)
			return len;

		packet = (struct joycon_uart_packet *) ctlr->partial_pkt;
		if (packet->size + 5 <= ctlr->partial_pkt_len) {
			ctlr->partial_pkt_len = 0;
			if (packet->magic[0] != JC_UART_MAGIC_RX_0 ||
			    packet->magic[1] != JC_UART_MAGIC_RX_1 ||
			    packet->magic[2] != JC_UART_MAGIC_RX_2) {
				/* Toss out this packet if the magic is wrong */
				dev_warn(dev, "Received pkt has wrong magic\n");
				return len;
			}
			/* Proceed to process the packet */
			dev_dbg(dev, "Finished receiving uart packet\n");
		} else {
			/* Packet still in progress */
			dev_dbg(dev, "rx of packet still in progress\n");
			return len;
		}
	}

	/* Could check package crc here,
	   calculation is the same as for sending */

	if (unlikely(mutex_is_locked(&ctlr->output_mutex))) {
		switch (ctlr->msg_type) {
		case JOYCON_MSG_TYPE_UART_CMD:
			if (packet->command == ctlr->uart_cmd_match) {
				memcpy(ctlr->input_buf, buf,
				       min(len, (size_t)JC_MAX_RESP_SIZE));
				ctlr->msg_type = JOYCON_MSG_TYPE_NONE;
				ctlr->received_resp = true;
				wake_up(&ctlr->wait);
				return len;
			}
			break;
		case JOYCON_MSG_TYPE_SUBCMD:
			if (packet->command == JC_CMD_EXTRET &&
			    packet->data[0] == JC_INPUT_SUBCMD_REPLY) {
				r = (struct joycon_input_report *)packet->data;
				if (r->reply.id != ctlr->subcmd_ack_match)
					break;
				memcpy(ctlr->input_buf, (u8 *)r,
				       min(len - sizeof(*packet),
					   (size_t)JC_MAX_RESP_SIZE));
				ctlr->msg_type = JOYCON_MSG_TYPE_NONE;
				ctlr->received_resp = true;
				wake_up(&ctlr->wait);
				return len;
			}
			break;
		default:
			break;
		}
	}

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
		if (packet->command == JC_CMD_EXTRET) {
			dev_dbg(dev, "JC_CMD_EXTRET\n");
			r = (struct joycon_input_report *)packet->data;
			if (r->id == JC_INPUT_SUBCMD_REPLY ||
			    r->id == JC_INPUT_IMU_DATA ||
			    r->id == JC_INPUT_MCU_DATA)
				joycon_parse_report(ctlr, r);
		} else if (packet->command == JC_CMD_INITRET) {
			dev_dbg(dev, "JC_CMD_INITRET\n");
		} else if (packet->command == JC_CMD_HANDSHAKE) {
			dev_dbg(dev, "JC_CMD_HANDSHAKE\n");
		} else if(packet->command == JC_CMD_HORIINPUTREPORT) {
			joycon_parse_report(ctlr, (struct joycon_input_report*)packet->data);
		} else {
			dev_dbg(dev, "Unknown uart cmd=%u\n", packet->command);
		}
	}

	return len;
}

static int sio_serdev_receive_buf(struct serdev_device *serdev,
				  const unsigned char *buf, size_t len)
{
	struct joycon_ctlr *ctlr = serdev_device_get_drvdata(serdev);
	struct sio_uart_packet *packet = (struct sio_uart_packet *) buf;
	struct device *dev = &ctlr->sdev->dev;
	u8 subcmd = 0;

	dev_dbg(dev, "Received uart data of size=%lu\n", len);
	/* Check if this is beginning of new packet */
	if (!ctlr->partial_pkt_len) {
		/* Have we received the entire packet? */
		if (len >= 4 && packet->payload_len + 8 <= len) {
			if (packet->cmd != JC_SIO_INPUT_RPT) {
				/* Toss out this packet if the magic is wrong */
				dev_warn(dev, "Received pkt has wrong magic\n");
				return len;
			} else if (packet->payload_len > JC_SIO_MAX_RESP_SIZE) {
				/* Toss out this packet if too big */
				dev_warn(dev, "Received pkt payload too big\n");
				return len;
			}

			/* Proceed to process the packet without buffering */
			dev_dbg(dev, "Received whole uart packet\n");
		} else if (len > JC_SIO_MAX_RESP_SIZE) {
			/* Toss out this packet if malformed */
			dev_warn(dev, "Received pkt is malformed\n");
			return len;
		} else {
			/* This isn't yet the whole packet. */
			memcpy(ctlr->partial_pkt, buf, len);
			ctlr->partial_pkt_len = len;
			return len;
		}
	} else {
		if (ctlr->partial_pkt_len + len > JC_SIO_MAX_UART_PKT_SIZE) {
			dev_warn(dev, "Packet length too large; ignoring\n");
			ctlr->partial_pkt_len = 0;
			return len;
		}
		memcpy(ctlr->partial_pkt + ctlr->partial_pkt_len, buf, len);
		ctlr->partial_pkt_len += len;

		/* Need to be able to check packet size */
		if (ctlr->partial_pkt_len < 4)
			return len;

		packet = (struct sio_uart_packet *) ctlr->partial_pkt;
		if (packet->payload_len + 8 <= ctlr->partial_pkt_len) {
			ctlr->partial_pkt_len = 0;
			if (packet->cmd != JC_SIO_INPUT_RPT) {
				/* Toss out this packet if the magic is wrong */
				dev_warn(dev, "Received pkt has wrong magic\n");
				return len;
			}
			/* Proceed to process the packet */
			dev_dbg(dev, "Finished receiving uart packet\n");
		} else {
			/* Packet still in progress */
			dev_dbg(dev, "rx of packet still in progres\n");
			return len;
		}
	}

	/* Could check package crc here,
	   calculation is the same as for sending */

	if (unlikely(mutex_is_locked(&ctlr->output_mutex))) {
		switch (ctlr->msg_type) {
		case JOYCON_MSG_TYPE_UART_CMD:
			if (packet->subcmd == ctlr->uart_cmd_match &&
			    packet->data[0] == JC_SIO_STATUS_OK) {
				memcpy(ctlr->input_buf, buf,
				       min(len, (size_t)JC_SIO_MAX_RESP_SIZE));
				ctlr->msg_type = JOYCON_MSG_TYPE_NONE;
				ctlr->received_resp = true;
				wake_up(&ctlr->wait);
				return len;
			}
			break;
		case JOYCON_MSG_TYPE_SUBCMD:
			if (packet->subcmd != ctlr->uart_cmd_match ||
			    packet->data[0] != JC_SIO_STATUS_OK) {
				return len;
			}
			ctlr->msg_type = JOYCON_MSG_TYPE_NONE;
			break;
		default:
			break;
		}
	}

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
		subcmd = packet->subcmd & (~JC_SIO_CMD_ACK);
		if (subcmd == JC_SIO_CMD_INPUTREPORT) {
			dev_dbg(dev, "JC_SIO_CMD_INPUTREPORT\n");
			sio_parse_report(ctlr,
				    (struct sio_input_report *)packet->payload);
		} else if (subcmd == JC_SIO_CMD_INIT) {
			dev_dbg(dev, "JC_SIO_CMD_INIT\n");
		} else if (subcmd == JC_SIO_CMD_VER_RPT) {
			dev_dbg(dev, "JC_SIO_CMD_VER_RPT\n");
		} else if (subcmd == JC_SIO_CMD_IAP_VER) {
			dev_dbg(dev, "JC_SIO_CMD_IAP_VER\n");
		} else {
			dev_dbg(dev, "Unknown uart cmd=%u\n", subcmd);
		}
	}

	return len;
}

static struct serdev_device_ops joycon_serdev_ops = {
	.receive_buf = joycon_serdev_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static struct serdev_device_ops sio_serdev_ops = {
	.receive_buf = sio_serdev_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int joycon_serdev_probe(struct serdev_device *serdev)
{
	struct joycon_ctlr *ctlr;
	struct device *dev = &serdev->dev;
	struct clk *sio_pclk_pll = NULL, *sio_pclk = NULL, *sio_pclk_mux = NULL;
	u32 stick_cal[6];
	int ret = 0, pclk_rate = 0;

	dev_info(dev, "joycon_serdev_probe\n");
	ctlr = devm_kzalloc(dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr)
		return -ENOMEM;

	ctlr->is_sio = of_property_read_bool(dev->of_node, "nintendo,sio");
	if (ctlr->is_sio) {
		ctlr->sio_home_gpio = of_get_named_gpio(dev->of_node, "nintendo,sio-home-gpio", 0);
		if (!gpio_is_valid(ctlr->sio_home_gpio)) {
			dev_err(dev, "Could not get sio-home-gpio\n");
			return ctlr->sio_home_gpio;
		}
		ctlr->sio_por_gpio = of_get_named_gpio(dev->of_node, "nintendo,sio-por-gpio", 0);
		if (!gpio_is_valid(ctlr->sio_por_gpio)) {
			dev_err(dev, "Could not get sio-por-gpio\n");
			return ctlr->sio_por_gpio;
		}
		ctlr->sio_rst_gpio = of_get_named_gpio(dev->of_node, "nintendo,sio-rst-gpio", 0);
		if (!gpio_is_valid(ctlr->sio_rst_gpio)) {
			dev_err(dev, "Could not get sio-rst-gpio\n");
			return ctlr->sio_rst_gpio;
		}
		sio_pclk_pll = devm_clk_get(dev, "sio_pclk_pll");
		if (IS_ERR(sio_pclk_pll)) {
			dev_err(dev, "Could not get sio parent clock pll\n");
			return PTR_ERR(sio_pclk);
		}
		sio_pclk = devm_clk_get(dev, "sio_pclk");
		if (IS_ERR(sio_pclk)) {
			dev_err(dev, "Could not get sio parent clock\n");
			return PTR_ERR(sio_pclk);
		}
		sio_pclk_mux = devm_clk_get(dev, "sio_pclk_mux");
		if (IS_ERR(sio_pclk_mux)) {
			dev_err(dev, "Could not get sio parent clock mux\n");
			return PTR_ERR(sio_pclk_mux);
		}
		ctlr->sio_clk = devm_clk_get(dev, "sio_clk");
		if (IS_ERR(ctlr->sio_clk)) {
			dev_err(dev, "Could not get sio clock\n");
			return PTR_ERR(ctlr->sio_clk);
		}

		ret = of_property_read_u32(dev->of_node, "sio-clock-rate",
					   &pclk_rate);
		if (ret) {
			dev_err(dev, "Could not get sio clock rate\n");
			return ret;
		}

		/* Sio stores factory/user calibration to prodinfo/settings */
		ret = of_property_read_u32_array(dev->of_node,
						 "sio-stick-cal-l",
						  stick_cal, 6);
		if (!ret) {
			ctlr->stick_cal_x.min = stick_cal[0];
			ctlr->stick_cal_x.center = stick_cal[1];
			ctlr->stick_cal_x.max = stick_cal[2];

			ctlr->stick_cal_y.min = stick_cal[3];
			ctlr->stick_cal_y.center = stick_cal[4];
			ctlr->stick_cal_y.max = stick_cal[5];
		} else
			ctlr->stick_cal_x.center = 0;

		ret = of_property_read_u32_array(dev->of_node,
						 "sio-stick-cal-r",
						 stick_cal, 6);
		if (!ret) {
			ctlr->right_stick_cal_x.min = stick_cal[0];
			ctlr->right_stick_cal_x.center = stick_cal[1];
			ctlr->right_stick_cal_x.max = stick_cal[2];

			ctlr->right_stick_cal_y.min = stick_cal[3];
			ctlr->right_stick_cal_y.center = stick_cal[4];
			ctlr->right_stick_cal_y.max = stick_cal[5];
		} else
			ctlr->right_stick_cal_x.center = 0;
	}

	crc8_populate_msb(ctlr->joycon_crc_table, JC_CRC8_POLY);

	ctlr->charger_reg = devm_regulator_get_optional(dev, "charger");
	if (PTR_ERR(ctlr->charger_reg) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (IS_ERR_OR_NULL(ctlr->charger_reg))
		dev_info(dev, "Continuing without charger regulator\n");

	ctlr->detect_gpio = of_get_named_gpio(dev->of_node, "detect-gpios", 0);
	ctlr->detect_en_gpio = of_get_named_gpio(dev->of_node, "detect-en-gpios", 0);

	ctlr->sdev = serdev;
	serdev_device_set_drvdata(serdev, ctlr);
	mutex_init(&ctlr->output_mutex);
	init_waitqueue_head(&ctlr->wait);
	spin_lock_init(&ctlr->lock);
	ctlr->rumble_queue = alloc_ordered_workqueue("joycon-serdev-rumble_wq",
					     WQ_FREEZABLE | WQ_MEM_RECLAIM);
	if (!ctlr->rumble_queue) {
		ret = -ENOMEM;
		goto err;
	}
	INIT_DELAYED_WORK(&ctlr->rumble_worker, joycon_rumble_worker);
	ctlr->detection_queue = alloc_ordered_workqueue("joycon-serdev-detection_wq",
						 WQ_MEM_RECLAIM);
	if (!ctlr->detection_queue) {
		ret = -ENOMEM;
		goto err;
	}
	INIT_DELAYED_WORK(&ctlr->detection_worker, joycon_detection_poller);
	ctlr->input_queue = alloc_ordered_workqueue("joycon-serdev-input_wq",
					    WQ_MEM_RECLAIM);
	if (!ctlr->input_queue) {
		ret = -ENOMEM;
		goto err;
	}
	INIT_DELAYED_WORK(&ctlr->input_worker, joycon_input_poller);
	INIT_WORK(&ctlr->led_worker, joycon_led_worker);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(dev, "Failed to open serdev device; ret=%d\n", ret);
		goto err;
	}
	serdev_device_set_client_ops(serdev, !ctlr->is_sio ?
				     &joycon_serdev_ops : &sio_serdev_ops);
	serdev_device_set_flow_control(serdev, true);

	if (ctlr->is_sio) {
		ret = devm_gpio_request(dev, ctlr->sio_home_gpio, "jc-sio-home");
		if (ret) {
			dev_err(dev, "Failed to request gpio jc-sio-home\n");
			goto err_sdev_close;
		}
		ret = devm_gpio_request(dev, ctlr->sio_por_gpio, "jc-por-home");
		if (ret) {
			dev_err(dev, "Failed to request gpio jc-por-home\n");
			goto err_sdev_close;
		}
		ret = devm_gpio_request(dev, ctlr->sio_rst_gpio, "jc-rst-home");
		if (ret) {
			dev_err(dev, "Failed to request gpio jc-rst-home\n");
			goto err_sdev_close;
		}

		ret = clk_set_parent(sio_pclk, sio_pclk_pll);
		if (ret) {
			dev_err(dev, "Failed to set SIO clock parent pll\n");
			goto err_sdev_close;
		}
		devm_clk_put(dev, sio_pclk_pll);
		ret = clk_set_rate(sio_pclk, pclk_rate);
		if (ret) {
			dev_err(dev, "Failed to set SIO clock parent rate\n");
			goto err_sdev_close;
		}

		ret = clk_prepare_enable(sio_pclk);
		if (ret) {
			dev_err(dev, "Failed to enable SIO clock parent\n");
			goto err_sdev_close;
		}
		ret = clk_set_parent(sio_pclk_mux, sio_pclk);
		if (ret) {
			dev_err(dev, "Failed to set SIO clock parent\n");
			goto err_sdev_close;
		}
		ret = clk_prepare_enable(ctlr->sio_clk);
		if (ret) {
			dev_err(dev, "Failed to enable SIO clock\n");
			goto err_sdev_close;
		}

		gpio_direction_input(ctlr->sio_home_gpio);
		gpio_direction_input(ctlr->sio_rst_gpio);
		gpio_direction_output(ctlr->sio_por_gpio, 0);
	}

	if (gpio_is_valid(ctlr->detect_gpio) &&
	    gpio_is_valid(ctlr->detect_en_gpio)) {
		if (devm_gpio_request(dev, ctlr->detect_gpio, "jc-detect") < 0) {
			dev_err(dev, "Failed to request detect gpio\n");
			goto polling_mode;
		}
		if (gpio_direction_input(ctlr->detect_gpio) < 0) {
			dev_err(dev, "Failed to set detect gpio direction\n");
			devm_gpio_free(dev, ctlr->detect_gpio);
			goto polling_mode;
		}
		/* Set debounce to avoid misfires */
		gpio_set_debounce(ctlr->detect_gpio,
				  DETECT_PIN_SETTLE_TIME_MS * 1000);

		ctlr->detect_irq = gpio_to_irq(ctlr->detect_gpio);
		if (devm_request_threaded_irq(dev, ctlr->detect_irq,
					      NULL, joycon_detection_irq,
					      IRQF_TRIGGER_FALLING |
					      IRQF_ONESHOT,
					      "jc-detect", ctlr)) {
			dev_err(dev, "Failed to request detect irq\n");
			gpio_set_debounce(ctlr->detect_gpio, 0);
			devm_gpio_free(dev, ctlr->detect_gpio);
			goto polling_mode;
		}
		dev_info(dev, "Interrupt based detection\n");
		disable_irq(ctlr->detect_irq);
	} else {
polling_mode:
		/* Invalidate detection gpios */
		ctlr->detect_gpio = -EINVAL;
		ctlr->detect_en_gpio = -EINVAL;
		dev_info(dev, "Polling based detection\n");
	}

	ret = joycon_enter_detection(ctlr);
	if (ret) {
		dev_err(dev,
			"Failed to enter initial detection mode; ret=%d\n",
			ret);
		goto err_sdev_close;
	}

	dev_info(dev, "Probe complete\n");
	return 0;

err_sdev_close:
	serdev_device_close(serdev);
err:
	return ret;
}

static void joycon_stop_queues(struct joycon_ctlr *ctlr)
{
	dev_info(&ctlr->sdev->dev, "Stopping queues\n");
	if (ctlr->input_queue) {
		cancel_delayed_work_sync(&ctlr->input_worker);
		flush_workqueue(ctlr->input_queue);
	}
	if (ctlr->rumble_queue) {
		cancel_delayed_work_sync(&ctlr->rumble_worker);
		flush_workqueue(ctlr->rumble_queue);
	}
	if (ctlr->detection_queue) {
		cancel_delayed_work_sync(&ctlr->detection_worker);
		flush_workqueue(ctlr->detection_queue);
	}
}

static void joycon_free_queues(struct joycon_ctlr *ctlr)
{
	dev_info(&ctlr->sdev->dev, "Freeing queues\n");
	if (ctlr->input_queue)
		destroy_workqueue(ctlr->input_queue);
	if (ctlr->rumble_queue)
		destroy_workqueue(ctlr->rumble_queue);
	if (ctlr->detection_queue)
		destroy_workqueue(ctlr->detection_queue);
}

static void joycon_serdev_remove(struct serdev_device *serdev)
{
	struct joycon_ctlr *ctlr = serdev_device_get_drvdata(serdev);

	dev_info(&serdev->dev, "Remove\n");

	if (ctlr->detect_en_req) {
		disable_irq(ctlr->detect_irq);
		gpio_free(ctlr->detect_en_gpio);
		ctlr->detect_en_req = false;
	}

	serdev_device_close(serdev);
	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ)
		joycon_disconnect(ctlr);

	joycon_stop_queues(ctlr);
	joycon_free_queues(ctlr);
	ctlr->ctlr_removed = true;
	if (ctlr->battery)
		power_supply_unregister(ctlr->battery);

	/* Stop charging */
	if (!IS_ERR_OR_NULL(ctlr->charger_reg) &&
	    regulator_is_enabled(ctlr->charger_reg) > 0)
		regulator_disable(ctlr->charger_reg);

	if (ctlr->is_sio)
		gpio_direction_output(ctlr->sio_rst_gpio, 0);
}

static int __maybe_unused joycon_serdev_suspend(struct device *dev)
{
	struct joycon_ctlr *ctlr = dev_get_drvdata(dev);
	unsigned long flags;

	dev_info(dev, "Suspend\n");

	if (ctlr->detect_en_req) {
		disable_irq(ctlr->detect_irq);
		gpio_free(ctlr->detect_en_gpio);
		ctlr->detect_en_req = false;
	}

	spin_lock_irqsave(&ctlr->lock, flags);
	if (ctlr->suspending) {
		spin_unlock_irqrestore(&ctlr->lock, flags);
		return 0;
	}
	ctlr->suspending = true;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* Stop charging */
	if (!IS_ERR_OR_NULL(ctlr->charger_reg) &&
	    regulator_is_enabled(ctlr->charger_reg) > 0)
		regulator_disable(ctlr->charger_reg);

	if (ctlr->ctlr_state == JOYCON_CTLR_STATE_READ) {
		/* Attempt telling the joy-con to sleep to decrease battery drain */
		if (!ctlr->is_sio)
			joycon_set_hci_state(ctlr, 0);
		joycon_disconnect(ctlr);

		if (ctlr->is_sio)
			gpio_direction_output(ctlr->sio_rst_gpio, 0);
	}

	joycon_stop_queues(ctlr);

	return 0;
}

static int __maybe_unused joycon_serdev_resume(struct device *dev)
{
	struct joycon_ctlr *ctlr = dev_get_drvdata(dev);
	unsigned long flags;

	dev_info(dev, "Resume\n");
	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->suspending = false;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	if (ctlr->is_sio)
		gpio_direction_input(ctlr->sio_rst_gpio);

	return joycon_enter_detection(ctlr);
}

#ifdef CONFIG_OF
static const struct of_device_id joycon_serdev_of_match[] = {
	{ .compatible = "nintendo,joycon-serdev" },
	{ },
};
MODULE_DEVICE_TABLE(of, joycon_serdev_of_match);
#endif

static const struct dev_pm_ops joycon_serdev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(joycon_serdev_suspend, joycon_serdev_resume)
};

static struct serdev_device_driver joycon_serdev_driver = {
	.probe = joycon_serdev_probe,
	.remove = joycon_serdev_remove,
	.driver = {
		.name = "joycon_serdev",
		.of_match_table = of_match_ptr(joycon_serdev_of_match),
		.pm = &joycon_serdev_pm_ops,
	},
};

int __init joycon_serdev_init(void)
{
	printk(KERN_INFO "joycon-serdev: init\n");
	return serdev_device_driver_register(&joycon_serdev_driver);
}

void __exit joycon_serdev_deinit(void)
{
	printk(KERN_INFO "joycon-serdev: deinit\n");
	serdev_device_driver_unregister(&joycon_serdev_driver);
}

module_init(joycon_serdev_init);
module_exit(joycon_serdev_deinit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel J. Ogorchock <djogorchock@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch Joy-Cons over UART");
