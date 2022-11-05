/******************************** FingerTipS 4CD60D ****************************
*
* File Name		: fts.c
* Authors		: Copyright (c) 2012 STMicroelectronics, Analog Mems Sensor Team
*                 Copyright (c) 2019-2020 Billy Laws <blaws05@gmail.com>
*                 Copyright (c) 2019-2021 Kostas Missos <ctcaer@gmail.com>
* Description	: FTS Capacitive touch screen controller (FingerTipS)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*******************************************************************************/

#ifndef _LINUX_FTM4_TS_H_
#define _LINUX_FTM4_TS_H_

#include <linux/device.h>
#include <linux/atomic.h>

#include <linux/printk.h>
#define tsp_debug_dbg(dev, fmt, ...)	dev_dbg(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_info(dev, fmt, ...)	dev_info(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_err(dev, fmt, ...)	dev_err(dev, fmt, ## __VA_ARGS__)
#ifdef CONFIG_TOUCHSCREEN_FTM4_SHOW_EVENTS
#define tsp_debug_event(dev, fmt, ...)	dev_dbg(dev, fmt, ## __VA_ARGS__)
#else
#define tsp_debug_event(dev, fmt, ...)
#endif

#define FTS_INPUT_OPEN_DWORK_TIME	10

#define FTS_TS_DRV_NAME			"fts_touch"
#define FTS_TS_DRV_VERSION		"4CD60D"

#define STM_DEVICE_NAME			"STM"

#define FTS_ID0				0x36
#define FTS_ID1				0x70

#define FTS_DIGITAL_REV_1		0x01
#define FTS_DIGITAL_REV_2		0x02
#define FTS_FIFO_MAX			32
#define FTS_EVENT_SIZE			8
#define FTS_FW_UPDATE_RETRY		3
#define FTS_LOCKDOWNCODE_SIZE		13

#define PRESSURE_MIN			0
#define PRESSURE_MAX			500
#define FINGER_MAX			10

#define EVENTID_NO_EVENT		0x00
#define EVENTID_ENTER_POINTER		0x03
#define EVENTID_LEAVE_POINTER		0x04
#define EVENTID_HOS_EVENT		0x04
#define EVENTID_MOTION_POINTER		0x05
#define EVENTID_HOVER_ENTER_POINTER	0x07
#define EVENTID_HOVER_LEAVE_POINTER	0x08
#define EVENTID_MOTION_POINTER		0x05
#define EVENTID_HOVER_MOTION_POINTER	0x09
#define EVENTID_PROXIMITY_IN		0x0B
#define EVENTID_PROXIMITY_OUT		0x0C
#define EVENTID_MSKEY			0x0E
#define EVENTID_ERROR			0x0F
#define EVENTID_CONTROLLER_READY	0x10
#define EVENTID_SLEEPOUT_CONTROLLER_READY	0x11
#define EVENTID_RESULT_READ_REGISTER		0x12
#define EVENTID_STATUS_REQUEST_COMP		0x13

#define EVENTID_STATUS_EVENT			0x16
#define EVENTID_INTERNAL_RELEASE_INFO		0x14
#define EVENTID_EXTERNAL_RELEASE_INFO		0x15

#define EVENTID_FROM_STRING			0x80
#define EVENTID_GESTURE				0x20

#define EVENTID_SIDE_SCROLL			0x40
/* side touch event-id for debug, remove after f/w fixed */
#define EVENTID_SIDE_TOUCH_DEBUG		0xDB
#define EVENTID_SIDE_TOUCH			0x0B

#define EVENTID_ERROR_FLASH_CORRUPTION		0x03

/* define flash corruption type */
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_1	0x01
#define EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_2	0x02
#define EVENTID_ERROR_CX_FLASH_CORRUPTION	0x03

#define EVENTID_LOCKDOWN_CODE			0x1E
#define EVENTID_ERROR_LOCKDOWN			0x0B

#define STATUS_EVENT_MS_CX_TUNING_DONE		0x01
#define STATUS_EVENT_SS_CX_TUNING_DONE		0x02
#define STATUS_EVENT_FLASH_WRITE_CONFIG		0x03
#define STATUS_EVENT_FLASH_WRITE_CXTUNE_VALUE	0x04
#define STATUS_EVENT_FORCE_CAL_MUTUAL_SELF	0x05
#define STATUS_EVENT_FORCE_CAL_DONE		0x06

#define STATUS_EVENT_FORCE_CAL_MUTUAL		0x15
#define STATUS_EVENT_FORCE_CAL_SELF		0x06
#define	STATUS_EVENT_PARAM1_FCAL_MS_SS_DONE	0x23
#define STATUS_EVENT_WATERMODE_ON		0x07
#define STATUS_EVENT_WATERMODE_OFF		0x08
#define STATUS_EVENT_RTUNE_MUTUAL		0x09
#define STATUS_EVENT_RTUNE_SELF			0x0A
#define STATUS_EVENT_PANEL_TEST_RESULT		0x0B
#define STATUS_EVENT_GLOVE_MODE			0x0C
#define STATUS_EVENT_RAW_DATA_READY		0x0D
#define STATUS_EVENT_MUTUAL_CAL_FRAME_CHECK	0xC1
#define STATUS_EVENT_SELF_CAL_FRAME_CHECK	0xC2
#define STATUS_EVENT_CHARGER_CONNECTED		0xCC
#define STATUS_EVENT_CHARGER_DISCONNECTED	0xCD
#define STATUS_EVENT_PURE_AUTOTUNE_FLAG_WRITE_FINISH 0x10
#define STATUS_EVENT_PURE_AUTOTUNE_FLAG_CLEAR_FINISH 0x11

#define INT_ENABLE			0x48
#define INT_DISABLE			0x08

#define FTS_READ_STATUS			0x84
#define FTS_READ_ONE_EVENT		0x85
#define FTS_READ_ALL_EVENT		0x86

#define FTS_CMD_SENSEOFF		0x92
#define FTS_CMD_SENSEON			0x93
#define FTS_CMD_HOVER_OFF		0x94
#define FTS_CMD_HOVER_ON		0x95

#define FTS_CMD_MSKEY_AUTOTUNE		0x96
#define FTS_CMD_TRIM_LOW_POWER_OSCILLATOR	0x97

#define FTS_CMD_KEY_SENSE_OFF		0x9A
#define FTS_CMD_KEY_SENSE_ON		0x9B
#define FTS_CMD_SET_FAST_GLOVE_MODE	0x9D

#define FTS_CMD_MSHOVER_OFF		0x9E
#define FTS_CMD_MSHOVER_ON		0x9F
#define FTS_CMD_SET_NOR_GLOVE_MODE	0x9F

#define FTS_CMD_FLUSHBUFFER		0xA1
#define FTS_CMD_FORCECALIBRATION	0xA2
#define FTS_CMD_MS_CX_TUNING		0xA3
#define FTS_CMD_SS_CX_TUNING		0xA4

#define FTS_CMD_ITO_CHECK		0xA7

#define FTS_CMD_CHARGER_PLUGGED		0xA8
#define FTS_CMD_CHARGER_UNPLUGGED	0xAB

#define FTS_CMD_RELEASEINFO		0xAA
#define FTS_CMD_STYLUS_OFF		0xAB
#define FTS_CMD_STYLUS_ON		0xAC
#define FTS_CMD_LOWPOWER_MODE		0xAD

#define FTS_CMD_WRITE_REG		0xB6

#define FTS_CMS_ENABLE_FEATURE		0xC1
#define FTS_CMS_DISABLE_FEATURE		0xC2
#define FTS_CMD_SWITCH_SENSE_MODE	0xC3
#define FTS_CMD_LOCKDOWN_READ		0xC4

#define FTS_CMD_WRITE_PRAM		0xF0
#define FTS_CMD_BURN_PROG_FLASH		0xF2
#define FTS_CMD_ERASE_PROG_FLASH	0xF3
#define FTS_CMD_READ_FLASH_STAT		0xF4
#define FTS_CMD_UNLOCK_FLASH		0xF7
#define FTS_CMD_SAVE_FWCONFIG		0xFB
#define FTS_CMD_SAVE_CX_TUNING		0xFC

#define FTS_CMD_FAST_SCAN		0x01
#define FTS_CMD_SLOW_SCAN		0x02
#define FTS_CMD_USLOW_SCAN		0x03

#define FTS_STYLUS_MODE			0x00
#define FTS_FINGER_MODE			0x01
#define FTS_HOVER_MODE			0x02

#define FTS_REPORT_RATE_90HZ		0
#define FTS_REPORT_RATE_60HZ		1
#define FTS_REPORT_RATE_30HZ		2

#define FTS_CMD_STRING_ACCESS		0xEC00
#define FTS_CMD_NOTIFY			0xC0

#define FTS_RETRY_COUNT			10

/* QUICK SHOT : Quick Camera Launching */
#define FTS_STRING_EVENT_REAR_CAM		(1 << 0)
#define FTS_STRING_EVENT_FRONT_CAM		(1 << 1)

/* SCRUB : Display Watch, Event Status / Fast Access Event */
#define FTS_STRING_EVENT_WATCH_STATUS		(1 << 2)
#define FTS_STRING_EVENT_FAST_ACCESS		(1 << 3)
#define FTS_STRING_EVENT_DIRECT_INDICATOR	((1 << 3) | (1 << 2))
#define FTS_STRING_EVENT_SPAY			(1 << 4)
#define FTS_STRING_EVENT_SPAY1			(1 << 5)
#define FTS_STRING_EVENT_SPAY2			((1 << 4) | (1 << 5))

#define FTS_SIDEGESTURE_EVENT_SINGLE_STROKE	0xE0
#define FTS_SIDEGESTURE_EVENT_DOUBLE_STROKE	0xE1
#define FTS_SIDEGESTURE_EVENT_INNER_STROKE	0xE3

#define FTS_SIDETOUCH_EVENT_LONG_PRESS		0xBB
#define FTS_SIDETOUCH_EVENT_REBOOT_BY_ESD	0xED

#define FTS_ENABLE		1
#define FTS_DISABLE		0

#define FTS_MODE_QUICK_SHOT		(1 << 0)
#define FTS_MODE_SCRUB			(1 << 1)
#define FTS_MODE_SPAY			(1 << 1)
#define FTS_MODE_QUICK_APP_ACCESS	(1 << 2)
#define FTS_MODE_DIRECT_INDICATOR	(1 << 3)

#define TSP_BUF_SIZE 2048
#define CMD_STR_LEN 32
#define CMD_RESULT_STR_LEN 2048
#define CMD_PARAM_NUM 8

#define FTS_LOWP_FLAG_QUICK_CAM		(1 << 0)
#define FTS_LOWP_FLAG_2ND_SCREEN	(1 << 1)
#define FTS_LOWP_FLAG_BLACK_UI		(1 << 2)
#define FTS_LOWP_FLAG_QUICK_APP_ACCESS	(1 << 3)
#define FTS_LOWP_FLAG_DIRECT_INDICATOR	(1 << 4)
#define FTS_LOWP_FLAG_SPAY		(1 << 5)
#define FTS_LOWP_FLAG_TEMP_CMD		(1 << 6)

enum fts_error_return {
	FTS_NOT_ERROR = 0,
	FTS_ERROR_INVALID_CHIP_ID,
	FTS_ERROR_INVALID_CHIP_VERSION_ID,
	FTS_ERROR_INVALID_SW_VERSION,
	FTS_ERROR_EVENT_ID,
	FTS_ERROR_TIMEOUT,
	FTS_ERROR_FW_UPDATE_FAIL,
};
#define RAW_MAX	3750
/**
 * struct fts_finger - Represents fingers.
 * @ state: finger status (Event ID).
 * @ mcount: moving counter for debug.
 */
struct fts_finger {
	unsigned char state;
	int lx;
	int ly;
};

enum tsp_power_mode {
	FTS_POWER_STATE_ACTIVE = 0,
	FTS_POWER_STATE_POWERDOWN,
};

enum fts_customer_feature {
	FTS_FEATURE_ORIENTATION_GESTURE = 1,
	FTS_FEATURE_STYLUS,
	FTS_FEATURE_QUICK_SHORT_CAMERA_ACCESS,
	FTS_FEATURE_SIDE_GUSTURE,
	FTS_FEATURE_COVER_GLASS,
	FTS_FEATURE_COVER_WALLET,
	FTS_FEATURE_COVER_LED,
	FTS_FEATURE_COVER_CLEAR_FLIP,
	FTS_FEATURE_DUAL_SIDE_GUSTURE,
	FTS_FEATURE_CUSTOM_COVER_GLASS_ON,
};

struct fts_flash_corruption_info {
	bool fw_broken;
	bool cfg_broken;
	bool cx_broken;
};

struct fts_i2c_platform_data {
	bool disable_tuning;
	bool delayed_open;
	unsigned int delayed_open_time;
	int max_x;
	int max_y;

	struct regulator *regulator_dvdd;
	struct regulator *regulator_avdd;

	int (*power)(void *data, bool on);

	unsigned gpio;
	int vdd_gpio;
	int vio_gpio;
	int irq_type;

	int coord_factor;
	int x_axis_real_max;
	int y_axis_real_max;
	int x_axis_edge_offset;
	int y_axis_edge_offset;
};

struct fts_ts_info {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;

	int irq;
	int irq_type;
	atomic_t irq_enabled;
	struct fts_i2c_platform_data *board;

	bool run_autotune;

	int power_state;
	int touch_count;
	struct fts_finger finger[FINGER_MAX];
	bool palm_pressed;
	int palm_touch_id;

	int touch_mode;

	struct delayed_work open_work;

	struct mutex i2c_mutex;
	struct mutex device_mutex;
	spinlock_t lock;

	unsigned char data[FTS_EVENT_SIZE * FTS_FIFO_MAX];

	struct fts_flash_corruption_info flash_corruption_info;
};

#define WRITE_CHUNK_SIZE			32
#define FLASH_CHUNK				(64 * 1024)
#define DMA_CHUNK				32

#define FW_HEADER_SIZE				64
#define FW_HEADER_FTB_SIGNATURE			0xAA55AA55
#define FW_FTB_VER				0x00000001
#define FW_BYTES_ALLIGN				4
#define FW_BIN_VER_OFFSET			16
#define FW_BIN_CONFIG_VER_OFFSET		20

/* Command for flash */
#define FLASH_CMD_UNLOCK			0xF7
#define FLASH_CMD_WRITE_64K			0xF8
#define FLASH_CMD_READ_REGISTER			0xF9
#define FLASH_CMD_WRITE_REGISTER		0xFA

/* Parameters for commands */
#define ADDR_WARM_BOOT				0x001E
#define WARM_BOOT_VALUE				0x38
#define FLASH_ADDR_CODE				0x00000000
#define FLASH_ADDR_CONFIG			0x0000FC00

#define FLASH_UNLOCK_CODE0			0x74
#define FLASH_UNLOCK_CODE1			0x45

#define FLASH_ERASE_UNLOCK_CODE0		0x72
#define FLASH_ERASE_UNLOCK_CODE1		0x03
#define FLASH_ERASE_UNLOCK_CODE2		0x02
#define FLASH_ERASE_CODE0			0x02
#define FLASH_ERASE_CODE1			0xC0
#define FLASH_DMA_CODE0				0x05
#define FLASH_DMA_CODE1				0xC0
#define FLASH_DMA_CONFIG			0x06

#endif /* _LINUX_FTM4_TS_H_ */
