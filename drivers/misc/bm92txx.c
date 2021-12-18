/*
 * bm92txx.c
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All Rights Reserved.
 * Copyright (c) 2020-2021 CTCaer <ctcaer@gmail.com>
 *
 * Authors:
 *     Adam Jiang <chaoj@nvidia.com>
 *     CTCaer <ctcaer@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/extcon.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>

/* Registers */
#define ALERT_STATUS_REG    0x02
#define STATUS1_REG         0x03
#define STATUS2_REG         0x04
#define COMMAND_REG         0x05 /* Send special command */
#define CONFIG1_REG         0x06 /* Controller Configuration 1 */
#define DEV_CAPS_REG        0x07
#define READ_PDOS_SRC_REG   0x08 /* Data size: 28 */
#define CONFIG2_REG         0x17 /* Controller Configuration 2 */
#define DP_STATUS_REG       0x18
#define DP_ALERT_EN_REG     0x19
#define VENDOR_CONFIG_REG   0x1A /* Vendor Configuration 1 */
#define AUTO_NGT_FIXED_REG  0x20 /* Data size: 4 */
#define AUTO_NGT_BATT_REG   0x23 /* Data size: 4 */
#define SYS_CONFIG1_REG     0x26 /* System Configuration 1 */
#define SYS_CONFIG2_REG     0x27 /* System Configuration 2 */
#define CURRENT_PDO_REG     0x28 /* Data size: 4 */
#define CURRENT_RDO_REG     0x2B /* Data size: 4 */
#define ALERT_ENABLE_REG    0x2E
#define SYS_CONFIG3_REG     0x2F /* System Configuration 3 */
#define SET_RDO_REG         0x30 /* Data size: 4 */
#define PDOS_SNK_CONS_REG   0x33 /* PDO Sink Consumer. Data size: 16 */
#define PDOS_SRC_PROV_REG   0x3C /* PDO Source Provider. Data size: 28 */
#define FW_TYPE_REG         0x4B
#define FW_REVISION_REG     0x4C
#define MAN_ID_REG          0x4D
#define DEV_ID_REG          0x4E
#define REV_ID_REG          0x4F
#define INCOMING_VDM_REG    0x50 /* Max data size: 28 */
#define OUTGOING_VDM_REG    0x60 /* Max data size: 28 */

/* ALERT_STATUS_REG */
#define ALERT_SNK_FAULT     BIT(0)
#define ALERT_SRC_FAULT     BIT(1)
#define ALERT_CMD_DONE      BIT(2)
#define ALERT_PLUGPULL      BIT(3)
#define ALERT_DP_EVENT      BIT(6)
#define ALERT_DR_SWAP       BIT(10)
#define ALERT_VDM_RECEIVED  BIT(11)
#define ALERT_CONTRACT      BIT(12)
#define ALERT_SRC_PLUGIN    BIT(13)
#define ALERT_PDO           BIT(14)

/* STATUS1_REG */
#define STATUS1_FAULT_MASK    (3 << 0)
#define STATUS1_SPDSRC2       BIT(3) /* VBUS2 enabled */
#define STATUS1_LASTCMD_SHIFT 4
#define STATUS1_LASTCMD_MASK  (7 << STATUS1_LASTCMD_SHIFT)
#define STATUS1_INSERT        BIT(7)  /* Cable inserted */
#define STATUS1_DR_SHIFT      8
#define STATUS1_DR_MASK       (3 << STATUS1_DR_SHIFT)
#define STATUS1_VSAFE         BIT(10) /* 0: No power, 1: VSAFE 5V or PDO */
#define STATUS1_CSIDE         BIT(11) /* Type-C Plug Side. 0: CC1 Side Valid, 1: CC2 Side Valid */
#define STATUS1_SRC_MODE      BIT(12) /* 0: Sink Mode, 1: Source mode (OTG) */
#define STATUS1_CMD_BUSY      BIT(13) /* Command in progress */
#define STATUS1_SPDSNK        BIT(14) /* Sink mode */
#define STATUS1_SPDSRC1       BIT(15) /* VBUS enabled */

#define LASTCMD_COMPLETE   0
#define LASTCMD_ABORTED    2
#define LASTCMD_INVALID    4
#define LASTCMD_REJECTED   6
#define LASTCMD_TERMINATED 7

#define DATA_ROLE_NONE  0
#define DATA_ROLE_UFP   1
#define DATA_ROLE_DFP   2
#define DATA_ROLE_ACC   3

/* STATUS2_REG */
#define STATUS2_PDOI_MASK    BIT(3)
#define STATUS2_VCONN_ON     BIT(9)
#define STATUS2_ACC_SHIFT    10
#define STATUS2_ACC_MASK     (3 << STATUS2_ACC_SHIFT) /* Accesory mode */
#define STATUS2_EM_CABLE     BIT(12) /* Electronically marked cable. Safe for 1.3A */
#define STATUS2_OTG_INSERT   BIT(13)

#define PDOI_SRC_OR_NO  0
#define PDOI_SNK        1

#define ACC_DISABLED    0
#define ACC_AUDIO       1
#define ACC_DEBUG       2
#define ACC_VCONN       3

/* DP_STATUS_REG */
#define DP_STATUS_SNK_CONN  BIT(1)
#define DP_STATUS_SIGNAL_ON BIT(7)
#define DP_STATUS_INSERT    BIT(14)
#define DP_STATUS_HPD       BIT(15)

/* CONFIG1_REG */
#define CONFIG1_AUTO_DR_SWAP          BIT(1)
#define CONFIG1_SLEEP_REQUEST         BIT(4)
#define CONFIG1_AUTONGTSNK_VAR_EN     BIT(5)
#define CONFIG1_AUTONGTSNK_FIXED_EN   BIT(6)
#define CONFIG1_AUTONGTSNK_EN         BIT(7)
#define CONFIG1_AUTONGTSNK_BATT_EN    BIT(8)
#define CONFIG1_VINOUT_DELAY_EN       BIT(9) /* VIN/VOUT turn on delay enable */
#define CONFIG1_VINOUT_TIME_ON_SHIFT  10 /* VIN/VOUT turn on delay */
#define CONFIG1_VINOUT_TIME_ON_MASK   (3 << CONFIG1_VINOUT_TIME_ON_SHIFT)
#define CONFIG1_SPDSRC_SHIFT          14
#define CONFIG1_SPDSRC_MASK           (3 << CONFIG1_SPDSRC_SHIFT)

#define VINOUT_TIME_ON_1MS    0
#define VINOUT_TIME_ON_5MS    1
#define VINOUT_TIME_ON_10MS   2
#define VINOUT_TIME_ON_20MS   3

#define SPDSRC12_ON           0 /* SPDSRC 1/2 on */
#define SPDSRC2_ON            1
#define SPDSRC1_ON            2
#define SPDSRC12_OFF          3 /* SPDSRC 1/2 off */

/* CONFIG2_REG */
#define CONFIG2_PR_SWAP_MASK      (3 << 0)
#define CONFIG2_DR_SWAP_SHIFT     2
#define CONFIG2_DR_SWAP_MASK      (3 << CONFIG2_DR_SWAP_SHIFT)
#define CONFIG2_VSRC_SWAP         BIT(4) /* VCONN source swap. 0: Reject, 1: Accept */
#define CONFIG2_NO_USB_SUSPEND    BIT(5)
#define CONFIG2_EXT_POWERED       BIT(7)
#define CONFIG2_TYPEC_AMP_SHIFT   8
#define CONFIG2_TYPEC_AMP_MASK    (3 << CONFIG2_TYPEC_AMP_SHIFT)

#define PR_SWAP_ALWAYS_REJECT         0
#define PR_SWAP_ACCEPT_SNK_REJECT_SRC 1 /* Accept when power sink */
#define PR_SWAP_ACCEPT_SRC_REJECT_SNK 2 /* Accept when power source */
#define PR_SWAP_ALWAYS_ACCEPT         3

#define DR_SWAP_ALWAYS_REJECT         0
#define DR_SWAP_ACCEPT_UFP_REJECT_DFP 1 /* Accept when device */
#define DR_SWAP_ACCEPT_DFP_REJECT_UFP 2 /* Accept when host */
#define DR_SWAP_ALWAYS_ACCEPT         3

#define TYPEC_AMP_0_5A_5V   0
#define TYPEC_AMP_1_5A_5V   1
#define TYPEC_AMP_3_0A_5V   2

/* SYS_CONFIG1_REG */
#define SYS_CONFIG1_PLUG_MASK           (0xF << 0)
#define SYS_CONFIG1_USE_AUTONGT         BIT(6)
#define SYS_CONFIG1_PDO_SNK_CONS        BIT(8)
#define SYS_CONFIG1_PDO_SNK_CONS_SHIFT  9 /* Number of Sink PDOs */
#define SYS_CONFIG1_PDO_SNK_CONS_MASK   (7 << SYS_CONFIG1_PDO_SNK_CONS_SHIFT)
#define SYS_CONFIG1_PDO_SRC_PROV        BIT(12)
#define SYS_CONFIG1_DOUT4_SHIFT         13
#define SYS_CONFIG1_DOUT4_MASK          (3 << SYS_CONFIG1_DOUT4_SHIFT)
#define SYS_CONFIG1_WAKE_ON_INSERT      BIT(15)

#define PLUG_TYPE_C      9
#define PLUG_TYPE_C_3A   10
#define PLUG_TYPE_C_5A   11

#define DOUT4_PDO4       0
#define DOUT4_PDO5       1
#define DOUT4_PDO6       2
#define DOUT4_PDO7       3

/* SYS_CONFIG2_REG */
#define SYS_CONFIG2_NO_COMM_UFP          BIT(0) /* Force no USB comms Capable UFP */
#define SYS_CONFIG2_NO_COMM_DFP          BIT(1) /* Force no USB comms Capable DFP */
#define SYS_CONFIG2_NO_COMM_ON_NO_BATT   BIT(2) /* Force no USB comms on dead battery */
#define SYS_CONFIG2_AUTO_SPDSNK_EN       BIT(6) /* Enable SPDSNK without SYS_RDY */
#define SYS_CONFIG2_BST_EN               BIT(8)
#define SYS_CONFIG2_PDO_SRC_PROV_SHIFT   9 /* Number of Source provisioned PDOs */
#define SYS_CONFIG2_PDO_SRC_PROV_MASK    (7 << SYS_CONFIG2_PDO_SRC_PROV_SHIFT)

/* VENDOR_CONFIG_REG */
#define VENDOR_CONFIG_OCP_DISABLE  BIT(2) /* Disable Over-current protection */

/* DEV_CAPS_REG */
#define DEV_CAPS_ALERT_STS  BIT(0)
#define DEV_CAPS_ALERT_EN   BIT(1)
#define DEV_CAPS_VIN_EN     BIT(2)
#define DEV_CAPS_VOUT_EN0   BIT(3)
#define DEV_CAPS_SPDSRC2    BIT(4)
#define DEV_CAPS_SPDSRC1    BIT(5)
#define DEV_CAPS_SPRL       BIT(6)
#define DEV_CAPS_SPDSNK     BIT(7)
#define DEV_CAPS_OCP        BIT(8)  /* Over current protection */
#define DEV_CAPS_DP_SRC     BIT(9)  /* DisplayPort capable Source */
#define DEV_CAPS_DP_SNK     BIT(10) /* DisplayPort capable Sink */
#define DEV_CAPS_VOUT_EN1   BIT(11)

/* COMMAND_REG command list */
#define ABORT_LASTCMD_SENT_CMD    0x0101
#define PR_SWAP_CMD               0x0303 /* Power Role swap request */
#define PS_RDY_CMD                0x0505 /* Power supply ready */
#define GET_SRC_CAP_CMD           0x0606 /* Get Source capabilities */
#define SEND_RDO_CMD              0x0707
#define PD_HARD_RST_CMD           0x0808 /* Hard reset link */
#define STORE_SYSCFG_CMD          0x0909 /* Store system configuration */
#define UPDATE_PDO_SRC_PROV_CMD   0x0A0A /* Update PDO Source Provider */
#define GET_SNK_CAP_CMD           0x0B0B /* Get Sink capabilities */
#define STORE_CFG2_CMD            0x0C0C /* Store controller configuration 2 */
#define SYS_RESET_CMD             0x0D0D /* Full USB-PD IC reset */
#define RESET_PS_RDY_CMD          0x1010 /* Reset power supply ready */
#define SEND_VDM_CMD              0x1111 /* Send VMD SOP */
#define SEND_VDM_1_CMD            0x1212 /* Send VMD SOP'  EM cable near end */
#define SEND_VDM_2_CMD            0x1313 /* Send VMD SOP'' EM cable far end */
#define SEND_VDM_1_DBG_CMD        0x1414 /* Send VMD SOP'  debug */
#define SEND_VDM_2_DBG_CMD        0x1515 /* Send VMD SOP'' debug */
#define ACCEPT_VDM_CMD            0x1616 /* Receive VDM */
#define MODE_ENTERED_CMD          0x1717 /* Alt mode entered */
#define DR_SWAP_CMD               0x1818 /* Data Role swap request */
#define VC_SWAP_CMD               0x1919 /* VCONN swap request */
#define BIST_REQ_CARR_M2_CMD      0x2424 /* Request BIST carrier mode 2 */
#define BIST_TEST_DATA_CMD        0x2B2B /* Send BIST test data */
#define PD_SOFT_RST_CMD           0x2C2C /* Reset power and get new PDO/Contract */
#define BIST_CARR_M2_CONT_STR_CMD 0x2F2F /* Send BIST carrier mode 2 continuous string */
#define DP_ENTER_MODE_CMD         0x3131 /* Discover DP Alt mode */
#define DP_STOP_CMD               0x3232 /* Cancel DP Alt mode discovery */
#define START_HPD_CMD             0x3434 /* Start handling HPD */
#define DP_CFG_AND_START_HPD_CMD  0x3636 /* Configure and enter selected DP Alt mode and start handling HPD */
#define STOP_HPD_CMD              0x3939 /* Stop handling HPD */
#define STOP_HPD_EXIT_DP_CMD      0x3B3B /* Stop handling HPD and exit DP Alt mode */

/* General defines */
#define PDO_TYPE_FIXED  0
#define PDO_TYPE_BATT   1
#define PDO_TYPE_VAR    2

#define PDO_INFO_DR_DATA   (1 << 5)
#define PDO_INFO_USB_COMM  (1 << 6)
#define PDO_INFO_EXT_POWER (1 << 7)
#define PDO_INFO_HP_CAP    (1 << 8)
#define PDO_INFO_DR_POWER  (1 << 9)

/* VDM/VDO */
#define VDM_CMD_RESERVED    0x00
#define VDM_CMD_DISC_ID     0x01
#define VDM_CMD_DISC_SVID   0x02
#define VDM_CMD_DISC_MODE   0x03
#define VDM_CMD_ENTER_MODE  0x04
#define VDM_CMD_EXIT_MODE   0x05
#define VDM_CMD_ATTENTION   0x06
#define VDM_CMD_DP_STATUS   0x10
#define VDM_CMD_DP_CONFIG   0x11

#define VDM_ACK   0x40
#define VDM_NAK   0x80
#define VDM_BUSY  0xC0
#define VDM_UNSTRUCTURED   0x00
#define VDM_STRUCTURED     0x80

/* VDM Discover ID */
#define VDO_ID_TYPE_NONE        0
#define VDO_ID_TYPE_PD_HUB      1
#define VDO_ID_TYPE_PD_PERIPH   2
#define VDO_ID_TYPE_PASS_CBL    3
#define VDO_ID_TYPE_ACTI_CBL    4
#define VDO_ID_TYPE_ALTERNATE   5

/* VDM Discover Mode Caps [From device (UFP_U) to host (DFP_U)] */
#define VDO_DP_UFP_D       BIT(0) /* DisplayPort Sink */
#define VDO_DP_DFP_D       BIT(1) /* DisplayPort Source */
#define VDO_DP_SUPPORT     BIT(2)
#define VDO_DP_RECEPTACLE  BIT(6)

/* VDM DP Configuration [From host (DFP_U) to device (UFP_U)] */
#define VDO_DP_U_DFP_D     BIT(0) /* UFP_U as DisplayPort Source */
#define VDO_DP_U_UFP_D     BIT(1) /* UFP_U as DisplayPort Sink */
#define VDO_DP_SUPPORT     BIT(2)
#define VDO_DP_RECEPTACLE  BIT(6)

/* VDM Mode Caps and DP Configuration pins */
#define VDO_DP_PIN_A   BIT(0)
#define VDO_DP_PIN_B   BIT(1)
#define VDO_DP_PIN_C   BIT(2)
#define VDO_DP_PIN_D   BIT(3)
#define VDO_DP_PIN_E   BIT(4)
#define VDO_DP_PIN_F   BIT(5)

/* Known VID/SVID */
#define VID_NINTENDO      0x057E
#define PID_NIN_DOCK      0x2003
#define PID_NIN_CHARGER   0x2004

#define SVID_NINTENDO     VID_NINTENDO
#define SVID_DP           0xFF01

/* Nintendo dock VDM Commands */
#define VDM_NCMD_LED_CONTROL         0x01 /* Reply size 12 */
#define VDM_NCMD_DEVICE_STATE        0x16 /* Reply size 12 */
#define VDM_NCMD_DP_SIGNAL_DISABLE   0x1C /* Reply size 8 */
#define VDM_NCMD_HUB_RESET           0x1E /* Reply size 8 */
#define VDM_NCMD_HUB_CONTROL         0x20 /* Reply size 8 */

/* Nintendo dock VDM Request Type */
#define VDM_ND_READ    0
#define VDM_ND_WRITE   1

/* Nintendo dock VDM Reply Status */
#define VDM_ND_BUSY    1

/* Nintendo dock VDM Request/Reply Source */
#define VDM_ND_HOST    1
#define VDM_ND_DOCK    2

/* Nintendo dock VDM Message Type */
#define VDM_ND_REQST   0x00
#define VDM_ND_REPLY   0x40

/* Nintendo dock identifiers and limits */
#define DOCK_ID_VOLTAGE_MV  5000u
#define DOCK_ID_CURRENT_MA  500u
#define DOCK_INPUT_VOLTAGE_MV             15000u
#define DOCK_INPUT_CURRENT_LIMIT_MIN_MA   2600u
#define DOCK_INPUT_CURRENT_LIMIT_MAX_MA   3000u

/* Power limits */
#define PD_05V_CHARGING_CURRENT_LIMIT_MA   2000u
#define PD_09V_CHARGING_CURRENT_LIMIT_MA   2000u
#define PD_12V_CHARGING_CURRENT_LIMIT_MA   1500u
#define PD_15V_CHARGING_CURRENT_LIMIT_MA   1200u

#define NON_PD_POWER_RESERVE_UA   2500000u
#define PD_POWER_RESERVE_UA       4500000u

#define PD_INPUT_CURRENT_LIMIT_MIN_MA   0u
#define PD_INPUT_CURRENT_LIMIT_MAX_MA   3000u
#define PD_INPUT_VOLTAGE_LIMIT_MAX_MV   17000u

/* All states with ND are for Nintendo Dock */
enum bm92t_state_type {
	INIT_STATE = 0,
	NEW_PDO,
	PS_RDY_SENT,
	DR_SWAP_SENT,
	VDM_DISC_ID_SENT,
	VDM_ACCEPT_DISC_ID_REPLY,
	VDM_DISC_SVID_SENT,
	VDM_ACCEPT_DISC_SVID_REPLY,
	VDM_DISC_MODE_SENT,
	VDM_ACCEPT_DISC_MODE_REPLY,
	VDM_ENTER_ND_ALT_MODE_SENT,
	VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY,
	DP_DISCOVER_MODE,
	DP_CFG_START_HPD_SENT,
	VDM_ND_QUERY_DEVICE_SENT,
	VDM_ACCEPT_ND_QUERY_DEVICE_REPLY,
	VDM_ND_ENABLE_USBHUB_SENT,
	VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY,
	VDM_ND_LED_ON_SENT,
	VDM_ACCEPT_ND_LED_ON_REPLY,
	VDM_ND_CUSTOM_CMD_SENT,
	VDM_ACCEPT_ND_CUSTOM_CMD_REPLY,
	VDM_CUSTOM_CMD_SENT,
	VDM_ACCEPT_CUSTOM_CMD_REPLY,
	NINTENDO_CONFIG_HANDLED,
	NORMAL_CONFIG_HANDLED
};

struct __attribute__((packed)) pd_object {
	unsigned int amp:10;
	unsigned int volt:10;
	unsigned int info:10;
	unsigned int type:2;
};

struct __attribute__((packed)) rd_object {
	unsigned int max_amp:10;
	unsigned int op_amp:10;
	unsigned int info:6;
	unsigned int usb_comms:1;
	unsigned int mismatch:1;
	unsigned int obj_no:4;
};

struct __attribute__((packed)) vd_object {
	unsigned int vid:16;
	unsigned int rsvd:10;
	unsigned int modal:1;
	unsigned int type:3;
	unsigned int ufp:1;
	unsigned int dfp:1;

	unsigned int xid;

	unsigned int bcd:16;
	unsigned int pid:16;

	unsigned int prod_type;
};

struct bm92t_device {
	int pdo_no;
	unsigned int charging_limit;
	bool drd_support;
	bool is_nintendo_dock;
	struct pd_object pdo;
	struct vd_object vdo;
};

struct bm92t_platform_data {
	bool dp_signal_toggle_on_resume;
	bool led_static_on_suspend;
	bool dock_power_limit_disable;
	bool dp_alerts_enable;

	unsigned int pd_5v_current_limit;
	unsigned int pd_9v_current_limit;
	unsigned int pd_12v_current_limit;
	unsigned int pd_15v_current_limit;
};

struct bm92t_info {
	struct i2c_client *i2c_client;
	struct bm92t_platform_data *pdata;
	struct work_struct work;
	struct workqueue_struct *event_wq;
	struct completion cmd_done;

	int state;
	bool first_init;

	struct extcon_dev edev;
	struct delayed_work oneshot_work;
	struct delayed_work power_work;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
	struct regulator *batt_chg_reg;
	struct regulator *vbus_reg;
	bool charging_enabled;
	unsigned int fw_type;
	unsigned int fw_revision;

	struct bm92t_device cable;
};

static const char * const states[] = {
	"INIT_STATE",
	"NEW_PDO",
	"PS_RDY_SENT",
	"DR_SWAP_SENT",
	"VDM_DISC_ID_SENT",
	"VDM_ACCEPT_DISC_ID_REPLY",
	"VDM_DISC_SVID_SENT",
	"VDM_ACCEPT_DISC_SVID_REPLY",
	"VDM_DISC_MODE_SENT",
	"VDM_ACCEPT_DISC_MODE_REPLY",
	"VDM_ENTER_ND_ALT_MODE_SENT",
	"VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY",
	"DP_DISCOVER_MODE",
	"DP_CFG_START_HPD_SENT",
	"VDM_ND_QUERY_DEVICE_SENT",
	"VDM_ACCEPT_ND_QUERY_DEVICE_REPLY",
	"VDM_ND_ENABLE_USBHUB_SENT",
	"VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY",
	"VDM_ND_LED_ON_SENT",
	"VDM_ACCEPT_ND_LED_ON_REPLY",
	"VDM_ND_CUSTOM_CMD_SENT",
	"VDM_ACCEPT_ND_CUSTOM_CMD_REPLY",
	"VDM_CUSTOM_CMD_SENT",
	"VDM_ACCEPT_CUSTOM_CMD_REPLY",
	"NINTENDO_CONFIG_HANDLED",
	"NORMAL_CONFIG_HANDLED"
};

static const unsigned int bm92t_extcon_cable[] = {
	EXTCON_USB_HOST, /* Id */
	EXTCON_USB,      /* Vbus */
	EXTCON_USB_PD,   /* USB-PD */
	EXTCON_DISP_DP,  /* DisplayPort. Handled by HPD so not used. */
	EXTCON_NONE
};

struct bm92t_extcon_cables {
	unsigned int cable;
	char *name;
};

static const struct bm92t_extcon_cables bm92t_extcon_cable_names[] = {
	{ EXTCON_USB_HOST, "USB HOST"},
	{ EXTCON_USB,      "USB"},
	{ EXTCON_USB_PD,   "USB-PD"},
	{ EXTCON_DISP_DP,  "DisplayPort"},
	{ EXTCON_NONE,     "None"},
	{ -1,              "Unknown"}
};

/* bq2419x current input limits */
static const unsigned int current_input_limits[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000
};

/* USB-PD common VDMs */
unsigned char vdm_discover_id_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_ID, VDM_STRUCTURED, 0x00, 0xFF};

unsigned char vdm_discover_svid_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_SVID, VDM_STRUCTURED, 0x00, 0xFF};

unsigned char vdm_discover_mode_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_MODE, VDM_STRUCTURED, 0x01, 0xFF}; /* DisplayPort Alt Mode */

unsigned char vdm_exit_dp_alt_mode_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_EXIT_MODE, VDM_STRUCTURED | 1, 0x01, 0xFF};

unsigned char vdm_enter_nin_alt_mode_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_ENTER_MODE, VDM_STRUCTURED | 1, 0x7E, 0x05};

/* Nintendo Dock VDMs */
unsigned char vdm_query_device_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_READ,  VDM_ND_HOST, VDM_NCMD_DEVICE_STATE, 0x00};

unsigned char vdm_usbhub_enable_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_WRITE, VDM_ND_HOST, VDM_NCMD_HUB_CONTROL, 0x00};

unsigned char vdm_usbhub_disable_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_READ,  VDM_ND_HOST, VDM_NCMD_HUB_CONTROL, 0x00};

unsigned char vdm_usbhub_reset_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_READ,  VDM_ND_HOST, VDM_NCMD_HUB_RESET, 0x00};

unsigned char vdm_usbhub_dp_sleep_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	0x00,         VDM_ND_HOST, VDM_NCMD_DP_SIGNAL_DISABLE, 0x00};

unsigned char vdm_usbhub_led_msg[14] = {OUTGOING_VDM_REG, 12,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_WRITE, VDM_ND_HOST, VDM_NCMD_LED_CONTROL, 0x00,
	0x00, 0x00, 0x00, 0x00}; /* Fade, Time off, Time on, Duty */

static int bm92t_write_reg(struct bm92t_info *info,
			   unsigned char *buf, unsigned len)
{
	struct i2c_msg xfer_msg[1];

	xfer_msg[0].addr = info->i2c_client->addr;
	xfer_msg[0].len = len;
	xfer_msg[0].flags = I2C_M_NOSTART;
	xfer_msg[0].buf = buf;

	dev_dbg(&info->i2c_client->dev,
		     "write reg cmd = 0x%02X len = %d\n", buf[0], len);
	return (i2c_transfer(info->i2c_client->adapter, xfer_msg, 1) != 1);
}

static int bm92t_read_reg(struct bm92t_info *info,
			  unsigned char reg, unsigned char *buf, int num)
{
	struct i2c_msg xfer_msg[2];
	int err;
	unsigned char reg_addr;

	reg_addr = reg;

	xfer_msg[0].addr = info->i2c_client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = &reg_addr;

	xfer_msg[1].addr = info->i2c_client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	err = i2c_transfer(info->i2c_client->adapter, xfer_msg, 2);
	if (err < 0)
		dev_err(&info->i2c_client->dev,
		      "%s: transfer error %d\n", __func__, err);
	return (err != 2);
}

static int bm92t_send_cmd(struct bm92t_info *info, unsigned short *cmd)
{
	int ret;
	unsigned char reg;
	unsigned char *_cmd = (unsigned char *) cmd;
	unsigned char msg[3];

	if (!cmd)
		return -EINVAL;

	reg = COMMAND_REG;

	msg[0] = reg;
	msg[1] = _cmd[0];
	msg[2] = _cmd[1];

	ret = bm92t_write_reg(info, msg, 3);
	dev_dbg(&info->i2c_client->dev,
		 "Sent cmd 0x%02X 0x%02X return value %d\n",
		 _cmd[0], _cmd[1], ret);
	return ret;
}

static inline bool bm92t_is_success(const short alert_data)
{
	return (alert_data & ALERT_CMD_DONE);
}

static inline bool bm92t_received_vdm(const short alert_data)
{
	return (alert_data & ALERT_VDM_RECEIVED);
}

static inline bool bm92t_is_plugged(const short status1_data)
{
	return (status1_data & STATUS1_INSERT);
}

static inline bool bm92t_is_ufp(const short status1_data)
{
	return (((status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT) ==
				DATA_ROLE_UFP);
}

static inline bool bm92t_is_dfp(const short status1_data)
{
	return (((status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT) ==
				DATA_ROLE_DFP);
}

static inline bool bm92t_is_lastcmd_ok(struct bm92t_info *info,
	const char* cmd, const short status1_data)
{
	unsigned int lastcmd_status =
		(status1_data & STATUS1_LASTCMD_MASK) >> STATUS1_LASTCMD_SHIFT;

	switch (lastcmd_status) {
	case LASTCMD_COMPLETE:
		break;
	case LASTCMD_ABORTED:
		dev_err(&info->i2c_client->dev, "%s aborted!", cmd);
		break;
	case LASTCMD_INVALID:
		dev_err(&info->i2c_client->dev, "%s invalid!", cmd);
		break;
	case LASTCMD_REJECTED:
		dev_err(&info->i2c_client->dev, "%s rejected!", cmd);
		break;
	case LASTCMD_TERMINATED:
		dev_err(&info->i2c_client->dev, "%s terminated!", cmd);
		break;
	default:
		dev_err(&info->i2c_client->dev, "%s failed! (%d)",
			cmd, lastcmd_status);
	}

	return (lastcmd_status == LASTCMD_COMPLETE);
}

static int bm92t_handle_dp_config_and_hpd(struct bm92t_info *info)
{
	int err;
	bool pin_valid = false;
	unsigned char msg[5];
	unsigned short cmd = DP_CFG_AND_START_HPD_CMD;
	unsigned char cfg[6] = {OUTGOING_VDM_REG, 0x04,
		VDO_DP_SUPPORT | VDO_DP_U_UFP_D, 0x00, 0x00, 0x00};

	/* Read DisplayPort Capabilities */
	err = bm92t_read_reg(info, INCOMING_VDM_REG,
			     msg, sizeof(msg));

	/* Prepare UFP_U as UFP_D configuration */
	if (info->cable.is_nintendo_dock) {
		/* Dock reports Plug but uses Receptacle */
		/* Both plug & receptacle pin assignment work, */
		/* because dock ignores them. Use the latter though. */
		if (msg[3] & VDO_DP_PIN_D) {
			cfg[3] = 0x00;
			cfg[4] = VDO_DP_PIN_D;
			pin_valid = true;
		}
	} else if (!(msg[1] & VDO_DP_RECEPTACLE)) { /* Plug */
		/* Set Plug pin assignment */
		if (msg[2] & VDO_DP_PIN_D) { /* 2 DP Lanes */
			cfg[3] = VDO_DP_PIN_D;
			cfg[4] = 0x00;
			pin_valid = true;
		} else if (msg[2] & VDO_DP_PIN_C) { /* 4 DP Lanes - 2 Unused */
			cfg[3] = VDO_DP_PIN_C;
			cfg[4] = 0x00;
			pin_valid = true;
		}
	} else if (msg[1] & VDO_DP_RECEPTACLE) { /* Receptacle */
		/* Set Receptacle pin assignment */
		if (msg[3] & VDO_DP_PIN_D) { /* 2 DP Lanes */
			cfg[3] = VDO_DP_PIN_D;
			cfg[4] = 0x00;
			pin_valid = true;
		} else if (msg[3] & VDO_DP_PIN_C) { /* 4 DP Lanes - 2 Unused */
			cfg[3] = VDO_DP_PIN_C;
			cfg[4] = 0x00;
			pin_valid = true;
		}
	}

	/* Check that UFP_U/UFP_D Pin D assignment is supported */
	if (!err && msg[0] == 4 && pin_valid) {
		/* Send DisplayPort Configuration */
		err = bm92t_write_reg(info, (unsigned char *) cfg, sizeof(cfg));
		if (err) {
			dev_err(&info->i2c_client->dev, "Writing DP cfg failed!\n");
			return -ENODEV;
		}
		/* Configure DP Alt mode and start handling HPD */
		bm92t_send_cmd(info, &cmd);
	} else {
		dev_err(&info->i2c_client->dev,
			"No compatible DP Pin assignment (%d: %02X %02X %02X)!\n",
			msg[0], msg[1], msg[2], msg[3]);
		return -ENODEV;
	}

	return 0;
}

static int
	bm92t_set_current_limit(struct bm92t_info *info, int max_ua)
{
	int ret = 0;

	dev_info(&info->i2c_client->dev,
		"Set Charging Current Limit %dma\n", max_ua/1000);
	if (info->batt_chg_reg != NULL)
		ret = regulator_set_current_limit(info->batt_chg_reg,
						  0, max_ua);

	return ret;
}

static int bm92t_set_vbus_enable(struct bm92t_info *info, bool enable)
{
	int ret = 0;
	bool is_enabled;

	dev_dbg(&info->i2c_client->dev,
		"%s VBUS\n", enable ? "Enabling" : "Disabling");
	if (info->vbus_reg != NULL) {
		is_enabled = regulator_is_enabled(info->vbus_reg);
		if (enable && !is_enabled)
			ret = regulator_enable(info->vbus_reg);
		else if (is_enabled)
			ret = regulator_disable(info->vbus_reg);
	}

	return ret;
}

static int bm92t_set_source_mode(struct bm92t_info *info, unsigned int role)
{
	int err = 0;
	unsigned short value;
	unsigned char msg[3] = {CONFIG1_REG, 0, 0};

	err = bm92t_read_reg(info, CONFIG1_REG,
		(unsigned char *) &value, sizeof(value));
	if (err < 0)
		return err;

	if (((value & CONFIG1_SPDSRC_MASK) >> CONFIG1_SPDSRC_SHIFT) != role)
	{
		value &= ~CONFIG1_SPDSRC_MASK;
		value |= role << CONFIG1_SPDSRC_SHIFT;
		msg[1] = value & 0xFF;
		msg[2] = (value >> 8) & 0xFF;
		err = bm92t_write_reg(info, msg, sizeof(msg));
	}

	return err;
}

static int bm92t_set_dp_alerts(struct bm92t_info *info, bool enable)
{
	int err = 0;
	unsigned char msg[3] = {DP_ALERT_EN_REG, 0, 0};

	msg[1] = enable ? 0xFF : 0x00;
	msg[2] = enable ? 0xFF : 0x00;
	err = bm92t_write_reg(info, msg, sizeof(msg));

	return err;
}

static int bm92t_enable_ocp(struct bm92t_info *info)
{
	int err = 0;
	unsigned short value;
	unsigned char msg[3] = {VENDOR_CONFIG_REG, 0, 0};

	bm92t_read_reg(info, VENDOR_CONFIG_REG,
		(unsigned char *) &value, sizeof(value));
	if (value & VENDOR_CONFIG_OCP_DISABLE) {
		value &= ~VENDOR_CONFIG_OCP_DISABLE;
		msg[1] = value & 0xFF;
		msg[2] = (value >> 8) & 0xFF;
		bm92t_write_reg(info, msg, sizeof(msg));
	}

	return err;
}

static int bm92t_system_reset_auto(struct bm92t_info *info, bool force)
{
	int err = 0;
	unsigned short cmd = SYS_RESET_CMD;
	unsigned short alert_data, status1_data, dp_data;

	if (force) {
		dev_info(&info->i2c_client->dev, "SYS Reset requested!\n");
		bm92t_send_cmd(info, &cmd);
		msleep(33);

		/* Clear alerts */
		err = bm92t_read_reg(info, ALERT_STATUS_REG,
			   	 (unsigned char *) &alert_data,
			   	 sizeof(alert_data));
		goto ret;
	}

	err = bm92t_read_reg(info, STATUS1_REG,
			     (unsigned char *) &status1_data,
			     sizeof(status1_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, DP_STATUS_REG,
			     (unsigned char *) &dp_data,
			     sizeof(dp_data));
	if (err < 0)
		goto ret;

	/* Check if UFP is in invalid state */
	if (bm92t_is_plugged(status1_data)) {
		if (bm92t_is_dfp(status1_data) ||
			dp_data & DP_STATUS_HPD ||
			!bm92t_is_lastcmd_ok(info, "Unknown cmd", status1_data)) {
			dev_err(&info->i2c_client->dev,
				"Invalid state, initiating SYS Reset!\n");
			bm92t_send_cmd(info, &cmd);
			msleep(100);

			/* Clear alerts */
			err = bm92t_read_reg(info, ALERT_STATUS_REG,
			    	 (unsigned char *) &alert_data,
			    	 sizeof(alert_data));
		}
	}

ret:
	return err;
}

static char * bm92t_extcon_cable_get_name(const unsigned int cable)
{
	int i, count;

	count = ARRAY_SIZE(bm92t_extcon_cable_names);

	for (i = 0; i < count; i++)
		if (bm92t_extcon_cable_names[i].cable == cable)
			return bm92t_extcon_cable_names[i].name;

	return bm92t_extcon_cable_names[count - 1].name;
}

static void bm92t_extcon_cable_update(struct bm92t_info *info,
	const unsigned int cable, bool is_attached)
{
	int state = extcon_get_cable_state_(&info->edev, cable);

	if (state != is_attached) {
		dev_info(&info->i2c_client->dev, "extcon cable (%02d: %s) %s\n",
			cable, bm92t_extcon_cable_get_name(cable),
			is_attached ? "attached" : "detached");
		extcon_set_cable_state_(&info->edev, cable, is_attached);
	}
}

static inline void bm92t_state_machine(struct bm92t_info *info, int state)
{
	info->state = state;
	dev_dbg(&info->i2c_client->dev, "state = %s\n", states[state]);
}

static void bm92t_calculate_current_limit(struct bm92t_info *info,
	unsigned int voltage, unsigned int amperage)
{
	int i;
	unsigned int charging_limit = amperage;
	struct bm92t_platform_data *pdata = info->pdata;

	/* Subtract a USB2 or USB3 port current */
	if (voltage > 5000)
		charging_limit -= (PD_POWER_RESERVE_UA / voltage);
	else
		charging_limit -= (NON_PD_POWER_RESERVE_UA / voltage);

	/* Set limits */
	switch (voltage) {
	case 5000:
		charging_limit = min(charging_limit, pdata->pd_5v_current_limit);
		break;
	case 9000:
		charging_limit = min(charging_limit, pdata->pd_9v_current_limit);
		break;
	case 12000:
		charging_limit = min(charging_limit, pdata->pd_12v_current_limit);
		break;
	case 15000:
	default:
		charging_limit = min(charging_limit, pdata->pd_15v_current_limit);
		break;
	}

	/* Set actual amperage */
	for (i = ARRAY_SIZE(current_input_limits) - 1; i >= 0; i--)
	{
		if (charging_limit >= current_input_limits[i]) {
			charging_limit = current_input_limits[i];
			break;
		}
	}

	info->cable.charging_limit = charging_limit;
}

static void bm92t_power_work(struct work_struct *work)
{
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, power_work);

	bm92t_set_current_limit(info, info->cable.charging_limit * 1000u);
	info->charging_enabled = true;

	extcon_set_cable_state_(&info->edev, EXTCON_USB_PD, true);
}

static void
	bm92t_extcon_cable_set_init_state(struct work_struct *work)
{
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, oneshot_work);

	dev_info(&info->i2c_client->dev,
		 "extcon cable is set to init state\n");

	disable_irq(info->i2c_client->irq);

	bm92t_set_vbus_enable(info, false);

	/* In case UFP is in an invalid state, request a SYS reset */
	bm92t_system_reset_auto(info, false);

	/* Enable over current protection */
	bm92t_enable_ocp(info);

	/* Enable power to SPDSRC for supporting both OTG and Charger */
	bm92t_set_source_mode(info, SPDSRC12_ON);

	/* Enable DisplayPort Alerts */
	if (info->pdata->dp_alerts_enable)
		bm92t_set_dp_alerts(info, true);

	bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
	bm92t_extcon_cable_update(info, EXTCON_USB, true);

	msleep(1000); /* WAR: Allow USB device enumeration at boot. */
	queue_work(info->event_wq, &info->work);
}

static bool bm92t_check_pdo(struct bm92t_info *info)
{
	int i, err, pdos_no;
	struct device *dev;
	unsigned char pdos[29];
	struct pd_object pdo[7];
	unsigned int prev_wattage = 0;
	unsigned int amperage, voltage, wattage, type;

	dev = &info->i2c_client->dev;

	memset(&info->cable, 0, sizeof(struct bm92t_device));

	err = bm92t_read_reg(info, READ_PDOS_SRC_REG, pdos, sizeof(pdos));
	pdos_no = pdos[0] / 4;
	if (err || pdos_no < 2)
		return 0;

	dev_info(dev, "Supported PDOs:\n");
	memcpy(pdo , pdos + 1, pdos[0]);
	for (i = 0; i < pdos_no; ++i) {
		dev_info(dev, "PDO %d: %4dmA %5dmV %s\n",
			i + 1, pdo[i].amp * 10, pdo[i].volt * 50,
			(pdo[i].info & PDO_INFO_DR_DATA) ? "DRD" : "No DRD");
	}

	if (pdo[0].info & PDO_INFO_DR_DATA)
		info->cable.drd_support = true;

	/* Check for dock mode */
	if (!info->pdata->dock_power_limit_disable &&
		pdos_no == 2 &&
		(pdo[0].volt * 50) == DOCK_ID_VOLTAGE_MV  &&
		(pdo[0].amp * 10)  == DOCK_ID_CURRENT_MA)
	{
		/* Only accept 15V, >= 2.6A for dock mode. */
		if (pdo[1].type == PDO_TYPE_FIXED &&
			(pdo[1].volt * 50) == DOCK_INPUT_VOLTAGE_MV &&
			(pdo[1].amp * 10)  >= DOCK_INPUT_CURRENT_LIMIT_MIN_MA &&
			(pdo[1].amp * 10)  <= DOCK_INPUT_CURRENT_LIMIT_MAX_MA)
		{
			dev_info(dev, "Device in Nintendo mode\n");
			info->cable.pdo_no = 2;
			memcpy(&info->cable.pdo, &pdo[1], sizeof(struct pd_object));
			return 1;
		}

		dev_info(dev, "Adapter in dock mode with improper current\n");
		return 0;
	}

	/* Not in dock mode. Check for max possible wattage */
	for (i = 0; i < pdos_no; ++i) {
		type = pdo[i].type;
		voltage = pdo[i].volt * 50;
		amperage = pdo[i].amp * 10;
		wattage = voltage * amperage;

		/* Only USB-PD defined voltages with max 15V. */
		switch (voltage) {
		case 5000:
		case 9000:
		case 12000:
		case 15000:
			break;
		default:
			continue;
		}

		/* Only accept <= 3A and select max wattage with max voltage. */
		if (type == PDO_TYPE_FIXED &&
			amperage >= PD_INPUT_CURRENT_LIMIT_MIN_MA &&
			amperage <= PD_INPUT_CURRENT_LIMIT_MAX_MA)
		{
			if (wattage > prev_wattage ||
			   (voltage > (info->cable.pdo.volt * 50) &&
				wattage && wattage == prev_wattage) ||
			   (!info->cable.pdo_no && !amperage && voltage == 5000))
			{
				prev_wattage = wattage;
				info->cable.pdo_no = i + 1;
				memcpy(&info->cable.pdo, &pdo[i], sizeof(struct pd_object));
			}
		}
	}

	if (info->cable.pdo_no) {
		dev_info(&info->i2c_client->dev, "Device in powered mode\n");
		return 1;
	}

	return 0;
}

static int bm92t_send_rdo(struct bm92t_info *info)
{
	int err;

	struct rd_object rdo = { 0 };
	unsigned char msg[6] = { SET_RDO_REG, 0x04, 0x00, 0x00, 0x00, 0x00};
	unsigned short cmd = SEND_RDO_CMD;

	/* Calculate operating current */
	bm92t_calculate_current_limit(info, info->cable.pdo.volt * 50,
							info->cable.pdo.amp * 10);

	dev_info(&info->i2c_client->dev,
		"Requesting %d: min %dmA, max %4dmA, %5dmV\n",
		info->cable.pdo_no, info->cable.charging_limit,
		info->cable.pdo.amp * 10,
		info->cable.pdo.volt * 50);

	rdo.usb_comms = 1;
	rdo.obj_no = info->cable.pdo_no;
	rdo.max_amp = info->cable.pdo.amp;
	rdo.op_amp = info->cable.charging_limit / 10;

	memcpy(&msg[2], &rdo, sizeof(struct rd_object));

	err = bm92t_write_reg(info, msg, sizeof(msg));
	if (!err)
		bm92t_send_cmd(info, &cmd);

	if (err) {
		dev_err(&info->i2c_client->dev, "Send RDO failure!\n");
		return -ENODEV;
	}
	return 0;
}

static int bm92t_send_vdm(struct bm92t_info *info, unsigned char *msg,
							unsigned int len)
{
	int err;
	unsigned short cmd = SEND_VDM_CMD;

	err = bm92t_write_reg(info, msg, len);
	if (!err)
		bm92t_send_cmd(info, &cmd);

	if (err) {
		dev_err(&info->i2c_client->dev, "Send VDM failure!\n");
		return -ENODEV;
	}
	return 0;
}

static void bm92t_usbhub_led_cfg(struct bm92t_info *info,
	unsigned char duty, unsigned char time_on,
	unsigned char time_off, unsigned char fade)
{
	vdm_usbhub_led_msg[10] = fade;
	vdm_usbhub_led_msg[11] = time_off;
	vdm_usbhub_led_msg[12] = time_on;
	vdm_usbhub_led_msg[13] = duty;

	bm92t_send_vdm(info, vdm_usbhub_led_msg, sizeof(vdm_usbhub_led_msg));
}

static void bm92t_usbhub_led_cfg_wait(struct bm92t_info *info,
	unsigned char duty, unsigned char time_on,
	unsigned char time_off, unsigned char fade)
{
	int retries = 100;
	if (info->state == NINTENDO_CONFIG_HANDLED) {
		bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
		bm92t_usbhub_led_cfg(info, duty, time_on, time_off, fade);
		while (info->state != NINTENDO_CONFIG_HANDLED) {
			retries--;
			if (retries < 0)
				break;
			msleep(1);
		}
	}
}

static void bm92t_usbhub_dp_sleep(struct bm92t_info *info, bool sleep)
{
	int retries = 100;
	if (info->state == NINTENDO_CONFIG_HANDLED ||
		info->state == NORMAL_CONFIG_HANDLED) {

		if (info->state == NINTENDO_CONFIG_HANDLED)
			bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
		else
			bm92t_state_machine(info, VDM_CUSTOM_CMD_SENT);

		vdm_usbhub_dp_sleep_msg[6] = sleep ? 1 : 0;

		bm92t_send_vdm(info, vdm_usbhub_dp_sleep_msg,
			sizeof(vdm_usbhub_dp_sleep_msg));

		while (info->state != NINTENDO_CONFIG_HANDLED ||
			   info->state != NORMAL_CONFIG_HANDLED) {
			retries--;
			if (retries < 0)
				break;
			msleep(1);
		}
	}
}

static void bm92t_print_dp_dev_info(struct device *dev,
	struct vd_object *vdo)
{
	dev_info(dev, "Connected PD device:\n");
	dev_info(dev, "VID: %04X, PID: %04X\n", vdo->vid, vdo->pid);

	switch (vdo->type) {
	case VDO_ID_TYPE_NONE:
		dev_info(dev, "Type: Undefined\n");
		break;
	case VDO_ID_TYPE_PD_HUB:
		dev_info(dev, "Type: PD HUB\n");
		break;
	case VDO_ID_TYPE_PD_PERIPH:
		dev_info(dev, "Type: PD Peripheral\n");
		break;
	case VDO_ID_TYPE_PASS_CBL:
		dev_info(dev, "Type: Passive Cable\n");
		break;
	case VDO_ID_TYPE_ACTI_CBL:
		dev_info(dev, "Type: Active Cable\n");
		break;
	case VDO_ID_TYPE_ALTERNATE:
		dev_info(dev, "Type: Alternate Mode Adapter\n");
		break;
	default:
		dev_info(dev, "Type: Unknown (%d)\n", vdo->type);
		break;
	}
}

static void bm92t_event_handler(struct work_struct *work)
{
	static bool sys_reset = false;
	static int retries_usbhub = 10;
	int i, err;
	struct bm92t_info *info;
	struct device *dev;
	struct pd_object curr_pdo;
	struct rd_object curr_rdo;
	unsigned short cmd;
	unsigned short alert_data;
	unsigned short status1_data;
	unsigned short status2_data;
	unsigned short dp_data;
	unsigned char vdm[29], pdo[5], rdo[5];

	info = container_of(work, struct bm92t_info, work);
	dev = &info->i2c_client->dev;

	/* Read status registers at 02h, 03h and 04h */
	err = bm92t_read_reg(info, ALERT_STATUS_REG,
			     (unsigned char *) &alert_data,
			     sizeof(alert_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, STATUS1_REG,
			     (unsigned char *) &status1_data,
			     sizeof(status1_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, STATUS2_REG,
			     (unsigned char *) &status2_data,
			     sizeof(status2_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, DP_STATUS_REG,
			     (unsigned char *) &dp_data,
			     sizeof(dp_data));
	if (err < 0)
		goto ret;

	dev_info_ratelimited(dev,
		"Alert= 0x%04X Status1= 0x%04X Status2= 0x%04X DP= 0x%04X State= %s\n",
		alert_data, status1_data, status2_data, dp_data, states[info->state]);

	/* Report sink error */
	if (alert_data & ALERT_SNK_FAULT)
		dev_err(dev, "Sink fault occurred!\n");

	/* Report source error */
	if (alert_data & ALERT_SRC_FAULT)
		dev_err(dev, "Source fault occurred!\n");

	/* TODO: DP event handling */
	if (alert_data == ALERT_DP_EVENT)
		goto ret;

	/* Check for errors */
	err = status1_data & STATUS1_FAULT_MASK;
	if (err) {
		dev_err(dev, "Internal error occurred. Ecode = %d\n", err);
		bm92t_state_machine(info, INIT_STATE);
		bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
		bm92t_extcon_cable_update(info, EXTCON_USB, false);
		bm92t_set_vbus_enable(info, false);
		if (bm92t_is_plugged(status1_data) || alert_data & ALERT_SNK_FAULT || alert_data == 0) {
			bm92t_system_reset_auto(info, true);
			sys_reset = true;
		}
		goto ret;
	}

	/* Check if sys reset happened */
	if (sys_reset) {
		sys_reset = false;
		msleep(100);

		/* Enable over current protection */
		bm92t_enable_ocp(info);

		/* Enable power to SPDSRC for supporting both OTG and Charger */
		bm92t_set_source_mode(info, SPDSRC12_ON);
	}

	/* Do a PD hard reset in case of a source fault */
	if (alert_data & ALERT_SRC_FAULT) {
		cmd = PD_HARD_RST_CMD;
		bm92t_send_cmd(info, &cmd);
		goto src_fault;
	}

	/* Check if cable removed */
	if (alert_data & ALERT_PLUGPULL) {
		if (!bm92t_is_plugged(status1_data)) { /* Pull out event */
src_fault:
			/* Cancel any pending charging enable work */
			cancel_delayed_work(&info->power_work);

			/* Disable VBUS in case it's enabled */
			bm92t_set_vbus_enable(info, false);

			/* Disable charging */
			if (info->charging_enabled) {
				bm92t_set_current_limit(info, 0);
				info->charging_enabled = false;
				bm92t_extcon_cable_update(info, EXTCON_USB_PD, false);
			}

			/* Reset USB modes and state */
			retries_usbhub = 10;
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_state_machine(info, INIT_STATE);
			goto ret;
		} else if (status1_data & STATUS1_SRC_MODE && /* OTG plug-in event */
				   status2_data & STATUS2_OTG_INSERT) {
			/* Enable VBUS for sourcing power to OTG device */
			bm92t_set_vbus_enable(info, true);

			/* Set USB to host mode */
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			goto ret;
		} else if (alert_data & ALERT_CONTRACT && !info->first_init) {
			/* When there's a plug-in wake-up, check if a new contract */
			/* was received. If yes continue with init. */

			/* In case of no new PDO, wait for it. Otherwise PD will fail. */
			/* In case of non-PD charger, this doesn't affect the result. */
			if (!(alert_data & ALERT_PDO))
				msleep(500);
		} else /* Simple plug-in event */
			goto ret;
	}

	switch (info->state) {
	case INIT_STATE:
		if (alert_data & ALERT_SRC_PLUGIN) {
			dev_info(dev, "Device in OTG mode\n");
			info->first_init = false;
			if (bm92t_is_dfp(status1_data)) {
				/* Reset cable info */
				memset(&info->cable, 0, sizeof(struct bm92t_device));

				bm92t_send_vdm(info, vdm_discover_id_msg,
					sizeof(vdm_discover_id_msg));
				bm92t_state_machine(info, VDM_DISC_ID_SENT);
			}
			break;
		}

		if (status1_data & STATUS1_SRC_MODE &&
				status2_data & STATUS2_OTG_INSERT) {
			info->first_init = false;
			dev_info(dev, "Device in OTG mode (no alert)\n");
			break;
		}

		if ((alert_data & ALERT_CONTRACT) || info->first_init) {
			/* Disable USB if first init and unplugged */
			if (!bm92t_is_plugged(status1_data)) {
				bm92t_extcon_cable_update(info, EXTCON_USB, false);
				goto init_contract_out;
			}

			/* Check if sink mode is enabled for first init */
			/* If not, exit and wait for next alert */
			if (info->first_init &&
				 !(alert_data & ALERT_CONTRACT) &&
				 !(status1_data & STATUS1_SPDSNK)) {
				goto init_contract_out;
			}

			/* Negotiate new power profile */
			if (!bm92t_check_pdo(info)) {
				dev_err(dev, "Power Negotiation failed\n");
				bm92t_state_machine(info, INIT_STATE);
				msleep(550); /* WAR: BQ2419x good power test */
				bm92t_extcon_cable_update(info, EXTCON_USB, true);
				goto init_contract_out;
			}

			/* Power negotiation succeeded */
			bm92t_send_rdo(info);
			bm92t_state_machine(info, NEW_PDO);
			msleep(20);

init_contract_out:
			info->first_init = false;
			break;
		}

		/* Check if forced workqueue and unplugged */
		if (!alert_data && !bm92t_is_plugged(status1_data))
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
		break;

	case NEW_PDO:
		if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in NEW_PDO state\n");

		if (alert_data & ALERT_CONTRACT) {
			/* Check PDO/RDO */
			err = bm92t_read_reg(info, CURRENT_PDO_REG,
				pdo, sizeof(pdo));
			memcpy(&curr_pdo, &pdo[1], sizeof(struct pd_object));
			err = bm92t_read_reg(info, CURRENT_RDO_REG,
				rdo, sizeof(rdo));
			memcpy(&curr_rdo, &rdo[1], sizeof(struct rd_object));

			dev_info(dev, "New PD Contract:\n");
			dev_info(dev, "PDO: %d: %dmA, %dmV\n",
				info->cable.pdo_no, curr_pdo.amp * 10, curr_pdo.volt * 50);
			dev_info(dev, "RDO: op %dmA, %dmA max\n",
				curr_rdo.op_amp * 10, curr_rdo.max_amp * 10);

			if (curr_rdo.mismatch)
				dev_err(dev, "PD mismatch!\n");

			cmd = PS_RDY_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, PS_RDY_SENT);
		}
		break;

	case PS_RDY_SENT:
		if (bm92t_is_success(alert_data)) {
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			schedule_delayed_work(&info->power_work,
				msecs_to_jiffies(2000));

			if (bm92t_is_ufp(status1_data)) {
				/* Check if Dual-Role Data is supported */
				if (!info->cable.drd_support) {
					dev_err(dev, "Device in UFP and DRD not supported!\n");
					break;
				}

				cmd = DR_SWAP_CMD;
				err = bm92t_send_cmd(info, &cmd);
				bm92t_state_machine(info, DR_SWAP_SENT);
			}
			else if (bm92t_is_dfp(status1_data)) {
				dev_dbg(dev, "Already in DFP mode\n");
				bm92t_send_vdm(info, vdm_discover_id_msg,
					sizeof(vdm_discover_id_msg));
				bm92t_state_machine(info, VDM_DISC_ID_SENT);
			}
		}
		break;

	case DR_SWAP_SENT:
		if ((bm92t_is_success(alert_data) &&
			 bm92t_is_plugged(status1_data) &&
			 bm92t_is_lastcmd_ok(info, "DR_SWAP_CMD", status1_data) &&
			 bm92t_is_dfp(status1_data))) {
			bm92t_send_vdm(info, vdm_discover_id_msg,
				sizeof(vdm_discover_id_msg));
			bm92t_state_machine(info, VDM_DISC_ID_SENT);
		}
		break;

	case VDM_DISC_ID_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_ID_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_ID_SENT\n");
		break;

	case VDM_ACCEPT_DISC_ID_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG,
				vdm, sizeof(vdm));

			memcpy(&info->cable.vdo, &vdm[5], sizeof(struct vd_object));

			bm92t_print_dp_dev_info(dev, &info->cable.vdo);

			/* Check if Nintendo dock. */
			if (!(info->cable.vdo.type == VDO_ID_TYPE_ALTERNATE &&
				  info->cable.vdo.vid == VID_NINTENDO &&
				  info->cable.vdo.pid == PID_NIN_DOCK)) {
				dev_err(dev, "VID/PID not Nintendo Dock\n");
				bm92t_send_vdm(info, vdm_discover_svid_msg,
					sizeof(vdm_discover_svid_msg));
				bm92t_state_machine(info, VDM_DISC_SVID_SENT);
			} else {
				info->cable.is_nintendo_dock = true;
				bm92t_send_vdm(info, vdm_enter_nin_alt_mode_msg,
					sizeof(vdm_enter_nin_alt_mode_msg));
				bm92t_state_machine(info, VDM_ENTER_ND_ALT_MODE_SENT);
			}
		}
		break;

	case VDM_DISC_SVID_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_SVID_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_SVID_SENT\n");
		break;

	case VDM_ACCEPT_DISC_SVID_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check discovered SVIDs */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			if (vdm[1] == (VDM_ACK | VDM_CMD_DISC_SVID))
			{
				dev_info(dev, "Supported SVIDs:\n");
				for (i = 0; i < ((vdm[0] - 4) / 2); i++)
					dev_info(dev, "SVID%d %04X\n",
						i, vdm[5 + i * 2] | (vdm[6 + i * 2] << 8));

				/* Request DisplayPort Alt mode support SVID (0xFF01) */
				bm92t_send_vdm(info, vdm_discover_mode_msg,
					sizeof(vdm_discover_mode_msg));
				bm92t_state_machine(info, VDM_DISC_MODE_SENT);
			}
		}
		break;

	case VDM_DISC_MODE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_MODE_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_MODE_SENT\n");
		break;

	case VDM_ACCEPT_DISC_MODE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			/* Check if DisplayPort Alt mode is supported */
			if (vdm[0] > 4 && /* Has VDO objects */
				vdm[1] == (VDM_ACK | VDM_CMD_DISC_MODE) &&
				vdm[2] == VDM_STRUCTURED &&
				vdm[3] == 0x01 && vdm[4] == 0xFF && /* SVID DisplayPort */
				vdm[5] & VDO_DP_UFP_D &&
				vdm[5] & VDO_DP_SUPPORT)
			{
				dev_info(dev, "DisplayPort Alt Mode supported");
				for (i = 0; i < ((vdm[0] - 4) / 4); i++)
					dev_info(dev, "DPCap%d %08X\n",
						i, vdm[5 + i * 4] | (vdm[6 + i * 4] << 8) |
						(vdm[7 + i * 4] << 16) | (vdm[8 + i * 4] << 24));

				/* Enter automatic DisplayPort handling */
				msleep(100);
				cmd = DP_ENTER_MODE_CMD;
				err = bm92t_send_cmd(info, &cmd);
				msleep(100); /* WAR: may not need to wait */
				bm92t_state_machine(info, DP_DISCOVER_MODE);
			}
		}
		break;

	case VDM_ENTER_ND_ALT_MODE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ENTER_ND_ALT_MODE_SENT\n");
		break;

	case VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			/* Check if supported. */
			if (!(vdm[1] == (VDM_ACK | VDM_CMD_ENTER_MODE) &&
				vdm[2] == (VDM_STRUCTURED | 1) &&
				vdm[3] == 0x7e && vdm[4] == 0x05)) {
				dev_err(dev, "Failed to enter Nintendo Alt Mode!\n");
				break;
			}

			/* Enter automatic DisplayPort handling */
			msleep(100);
			cmd = DP_ENTER_MODE_CMD;
			err = bm92t_send_cmd(info, &cmd);
			msleep(100); /* WAR: may not need to wait */
			bm92t_state_machine(info, DP_DISCOVER_MODE);
		}
		break;

	case DP_DISCOVER_MODE:
		if (bm92t_is_success(alert_data)) {
			err = bm92t_handle_dp_config_and_hpd(info);
			if (!err)
				bm92t_state_machine(info, DP_CFG_START_HPD_SENT);
			else
				bm92t_state_machine(info, INIT_STATE);
		}
		break;

	case DP_CFG_START_HPD_SENT:
		if (bm92t_is_success(alert_data)) {
			if (bm92t_is_plugged(status1_data) &&
				bm92t_is_lastcmd_ok(info, "DP_CFG_AND_START_HPD_CMD",
					status1_data)) {
				if (info->cable.is_nintendo_dock) {
					bm92t_send_vdm(info, vdm_query_device_msg,
						sizeof(vdm_query_device_msg));
					bm92t_state_machine(info, VDM_ND_QUERY_DEVICE_SENT);
				} else
					bm92t_state_machine(info, NORMAL_CONFIG_HANDLED);
			}
		}
		break;

	/* Nintendo Dock VDMs */
	case VDM_ND_QUERY_DEVICE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_QUERY_DEVICE_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_QUERY_DEVICE_SENT\n");
		break;

	case VDM_ACCEPT_ND_QUERY_DEVICE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			if (!err && vdm[6] == VDM_ND_DOCK &&
				 vdm[7] == (VDM_NCMD_DEVICE_STATE + 1)) {
				/* Check if USB HUB is supported */
				if (vdm[11] & 0x02) {
					bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
					msleep(500);
					bm92t_extcon_cable_update(info, EXTCON_USB, true);
					dev_err(dev, "Dock has old FW!\n");
				}
				dev_info(dev, "device state: %02X %02X %02X %02X\n",
					vdm[9], vdm[10], vdm[11], vdm[12]);
			} else
				dev_err(dev, "Failed to get dock state reply!");

			/* Set dock LED */
			bm92t_usbhub_led_cfg(info, 128, 0, 0, 64);
			bm92t_state_machine(info, VDM_ND_LED_ON_SENT);
		}
		break;

	case VDM_ND_LED_ON_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_LED_ON_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_LED_ON_SENT\n");
		break;

	case VDM_ACCEPT_ND_LED_ON_REPLY:
		if (bm92t_is_success(alert_data)) {
			msleep(500); /* Wait for hub to power up */
			bm92t_send_vdm(info, vdm_usbhub_enable_msg,
				sizeof(vdm_usbhub_enable_msg));
			bm92t_state_machine(info, VDM_ND_ENABLE_USBHUB_SENT);
		}
		break;

	case VDM_ND_ENABLE_USBHUB_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_ENABLE_USBHUB_SENT\n");
		break;

	case VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			if ((vdm[6] == VDM_ND_DOCK &&
				 vdm[7] == (VDM_NCMD_HUB_CONTROL + 1) &&
				 retries_usbhub)) {
				if (vdm[5] & VDM_ND_BUSY) {
					msleep(250);
					dev_info(dev, "Retrying USB HUB enable...\n");
					bm92t_send_vdm(info, vdm_usbhub_enable_msg,
						sizeof(vdm_usbhub_enable_msg));
					bm92t_state_machine(info, VDM_ND_ENABLE_USBHUB_SENT);
					retries_usbhub--;
					break;
				}
			} else if (!retries_usbhub)
				dev_err(dev, "USB HUB enable timed out!\n");
			else
				dev_err(dev, "USB HUB enable failed!\n");

			bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;

	case VDM_ND_CUSTOM_CMD_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_CUSTOM_CMD_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_CUSTOM_CMD_SENT\n");
		break;

	case VDM_ACCEPT_ND_CUSTOM_CMD_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Read incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));
			bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;
	/* End of Nintendo Dock VDMs */

	case VDM_CUSTOM_CMD_SENT:
		if (bm92t_received_vdm(alert_data)) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_CUSTOM_CMD_REPLY);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_CUSTOM_CMD_SENT\n");
		break;

	case VDM_ACCEPT_CUSTOM_CMD_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Read incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));
			bm92t_state_machine(info, NORMAL_CONFIG_HANDLED);
		}
		break;

	case NORMAL_CONFIG_HANDLED:
	case NINTENDO_CONFIG_HANDLED:
		break;

	default:
		dev_err(dev, "Invalid state!\n");
		break;
	}

ret:
	enable_irq(info->i2c_client->irq);
}

static irqreturn_t bm92t_interrupt_handler(int irq, void *handle)
{
	struct bm92t_info *info = handle;

	disable_irq_nosync(info->i2c_client->irq);
	queue_work(info->event_wq, &info->work);
	return IRQ_HANDLED;
}

static int bm92t_remove(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

	dev_info(&info->i2c_client->dev, "%s\n", __func__);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(info->debugfs_root);
#endif
	return 0;
}

static void bm92t_shutdown(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

	dev_info(&info->i2c_client->dev, "%s\n", __func__);

	/* Disable Dock LED if enabled */
	bm92t_usbhub_led_cfg_wait(info, 0, 0, 0, 128);

	/* Disable SPDSRC */
	bm92t_set_source_mode(info, SPDSRC12_OFF);

	/* Disable DisplayPort Alerts */
	if (info->pdata->dp_alerts_enable)
		bm92t_set_dp_alerts(info, false);
}

#ifdef CONFIG_DEBUG_FS
static int bm92t_regs_print(struct seq_file *s, const char *reg_name,
	unsigned char reg_addr, int size)
{
	int err;
	unsigned char msg[5];
	unsigned short reg_val16;
	unsigned short reg_val32;
	struct bm92t_info *info = (struct bm92t_info *) (s->private);

	switch (size) {
	case 2:
		err = bm92t_read_reg(info, reg_addr,
			    (unsigned char *) &reg_val16, sizeof(reg_val16));
		if (!err)
			seq_printf(s, "%s 0x%04X\n", reg_name, reg_val16);
		break;
	case 4:
		err = bm92t_read_reg(info, reg_addr,
			    (unsigned char *) &reg_val32, sizeof(reg_val32));
		if (!err)
			seq_printf(s, "%s 0x%08X\n", reg_name, reg_val32);
		break;
	case 5:
		err = bm92t_read_reg(info, reg_addr, msg, sizeof(msg));
		if (!err)
			seq_printf(s, "%s 0x%02X%02X%02X%02X\n",
				reg_name, msg[4], msg[3], msg[2], msg[1]);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		dev_err(&info->i2c_client->dev, "Cannot read 0x%02X\n", reg_addr);

	return err;
}

static int bm92t_regs_show(struct seq_file *s, void *data)
{
	int err;

	err = bm92t_regs_print(s, "ALERT_STATUS:  ", ALERT_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "STATUS1:       ", STATUS1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "STATUS2:       ", STATUS2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DP_STATUS:     ", DP_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CONFIG1:       ", CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CONFIG2:       ", CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG1:   ", SYS_CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG2:   ", SYS_CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG3:   ", SYS_CONFIG3_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "VENDOR_CONFIG: ", VENDOR_CONFIG_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DEV_CAPS:      ", DEV_CAPS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "ALERT_ENABLE:  ", ALERT_ENABLE_REG, 4);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DP_ALERT_EN:   ", DP_ALERT_EN_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "AUTO_NGT_FIXED:", AUTO_NGT_FIXED_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "AUTO_NGT_BATT: ", AUTO_NGT_BATT_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CURRENT_PDO:   ", CURRENT_PDO_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CURRENT_RDO:   ", CURRENT_RDO_REG, 5);
	if (err)
		return err;

	return 0;
}

static int bm92t_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_regs_show, inode->i_private);
}

static const struct file_operations bm92t_regs_fops = {
	.open = bm92t_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bm92t_state_show(struct seq_file *s, void *data)
{
	struct bm92t_info *info = (struct bm92t_info *) (s->private);

	seq_printf(s, "%s\n", states[info->state]);
	return 0;
}
static int bm92t_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_state_show, inode->i_private);
}

static const struct file_operations bm92t_state_fops = {
	.open = bm92t_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bm92t_fw_info_show(struct seq_file *s, void *data)
{
	struct bm92t_info *info = (struct bm92t_info *) (s->private);

	seq_printf(s, "fw_type: 0x%02X, fw_revision: 0x%02X\n",
		info->fw_type, info->fw_revision);
	return 0;
}
static int bm92t_fw_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_fw_info_show, inode->i_private);
}

static const struct file_operations bm92t_fwinfo_fops = {
	.open = bm92t_fw_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t bm92t_led_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *) (file->private_data);
	unsigned duty, time_on, time_off, fade;
	char buf[32];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%i %i %i %i",
		&duty, &time_on, &time_off, &fade);

	if (ret == 4) {
		if (info->state == VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY ||
			info->state == NINTENDO_CONFIG_HANDLED)
		{
			bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
			bm92t_usbhub_led_cfg(info, duty, time_on, time_off, fade);
		}
		else
			dev_err(&info->i2c_client->dev,
				"Led is not supported\n");
	} else {
		dev_err(&info->i2c_client->dev,
				"Led syntax is: duty time_on time_off fade\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_led_fops = {
	.open = simple_open,
	.write = bm92t_led_write,
};

static ssize_t bm92t_cmd_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *) (file->private_data);
	unsigned val;
	unsigned short cmd;
	char buf[8];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%i", &val);

	if (ret == 1) {
		cmd = val;
		bm92t_send_cmd(info, &cmd);
	} else {
		dev_err(&info->i2c_client->dev, "Cmd syntax is: cmd\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_cmd_fops = {
	.open = simple_open,
	.write = bm92t_cmd_write,
};

static ssize_t bm92t_usbhub_dp_sleep_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *) (file->private_data);
	unsigned val;
	char buf[8];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%i", &val);

	if (ret == 1)
		bm92t_usbhub_dp_sleep(info, val ? true : false);
	else {
		dev_err(&info->i2c_client->dev, "Syntax is: 0 or number\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_usbhub_dp_sleep_fops = {
	.open = simple_open,
	.write = bm92t_usbhub_dp_sleep_write,
};

static int bm92t_debug_init(struct bm92t_info *info)
{
	dev_info(&info->i2c_client->dev, "%s\n", __func__);

	info->debugfs_root = debugfs_create_dir("bm92txx", NULL);
	if (!info->debugfs_root)
		goto failed;

	if (!debugfs_create_file("regs", S_IRUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_regs_fops))
		goto failed;

	if (!debugfs_create_file("state", S_IRUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_state_fops))
		goto failed;

	if (!debugfs_create_file("fw_info", S_IRUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_fwinfo_fops))
		goto failed;

	if (!debugfs_create_file("led", S_IWUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_led_fops))
		goto failed;

	if (!debugfs_create_file("cmd", S_IWUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_cmd_fops))
		goto failed;

	if (!debugfs_create_file("sleep", S_IWUGO,
				 info->debugfs_root,
				 info,
				 &bm92t_usbhub_dp_sleep_fops))
		goto failed;

	return 0;

failed:
	dev_err(&info->i2c_client->dev, "%s: failed\n", __func__);
	debugfs_remove_recursive(info->debugfs_root);
	return -ENOMEM;
}
#endif

static const struct of_device_id bm92t_of_match[] = {
	{ .compatible = "rohm,bm92t", },
	{ },
};

MODULE_DEVICE_TABLE(of, bm92t_of_match);

#ifdef CONFIG_OF
static struct bm92t_platform_data *bm92t_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct bm92t_platform_data *pdata;
	int ret = 0;

	if (!np)
		return dev->platform_data;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->dp_signal_toggle_on_resume = of_property_read_bool(np,
						"rohm,dp-signal-toggle-on-resume");
	pdata->led_static_on_suspend = of_property_read_bool(np,
						"rohm,led-static-on-suspend");
	pdata->dock_power_limit_disable = of_property_read_bool(np,
						"rohm,dock-power-limit-disable");
	pdata->dp_alerts_enable = of_property_read_bool(np,
						"rohm,dp-alerts-enable");

	ret = of_property_read_u32(np, "rohm,pd-5v-current-limit-ma",
			&pdata->pd_5v_current_limit);
	if (ret)
		pdata->pd_5v_current_limit = PD_05V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-9v-current-limit-ma",
			&pdata->pd_9v_current_limit);
	if (ret)
		pdata->pd_9v_current_limit = PD_09V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-12v-current-limit-ma",
			&pdata->pd_12v_current_limit);
	if (ret)
		pdata->pd_12v_current_limit = PD_12V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-15v-current-limit-ma",
			&pdata->pd_15v_current_limit);
	if (ret)
		pdata->pd_15v_current_limit = PD_15V_CHARGING_CURRENT_LIMIT_MA;

	return pdata;
}
#else
static struct bm92t_platform_data *bm92t_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int bm92t_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct bm92t_info *info;
	struct regulator *batt_chg_reg;
	struct regulator *vbus_reg;
	int err;
	unsigned short reg_value;

	dev_info(&client->dev, "%s\n", __func__);

	/* Get Battery Charger and VBUS regulators */
	batt_chg_reg = devm_regulator_get(&client->dev, "pd_bat_chg");
	if (IS_ERR(batt_chg_reg)) {
		err = PTR_ERR(batt_chg_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_err(&client->dev,
				"pd_bat_chg reg not registered: %d\n", err);
		batt_chg_reg = NULL;
	}

	vbus_reg = devm_regulator_get(&client->dev, "vbus");
	if (IS_ERR(vbus_reg)) {
		err = PTR_ERR(vbus_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_err(&client->dev,
				"vbus reg not registered: %d\n", err);
		vbus_reg = NULL;
	}

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		info->pdata = bm92t_parse_dt(&client->dev);
		if (IS_ERR(info->pdata))
			return PTR_ERR(info->pdata);
	} else {
		info->pdata = client->dev.platform_data;
		if (!info->pdata) {
			dev_err(&client->dev, "no platform data provided\n");
			return -EINVAL;
		}
	}

	i2c_set_clientdata(client, info);

	info->i2c_client = client;
	
	info->batt_chg_reg = batt_chg_reg;
	info->vbus_reg = vbus_reg;

	/* Initialized state */
	info->state = INIT_STATE;
	info->first_init = true;

	/* Register extcon */
	info->edev.supported_cable = bm92t_extcon_cable;
	info->edev.name = "bm92t_extcon";
	info->edev.dev.parent = &client->dev;
	err = extcon_dev_register(&info->edev);
	if (err < 0) {
		dev_err(&client->dev, "Cannot register extcon device\n");
		return err;
	}

	/* Create workqueue */
	info->event_wq = create_singlethread_workqueue("bm92t-event-queue");
	if (!info->event_wq) {
		dev_err(&client->dev, "Cannot create work queue\n");
		return -ENOMEM;
	}

	err = bm92t_read_reg(info, FW_TYPE_REG,
			 (unsigned char *) &reg_value, sizeof(reg_value));
	info->fw_type = reg_value;

	err = bm92t_read_reg(info, FW_REVISION_REG,
			 (unsigned char *) &reg_value, sizeof(reg_value));
	info->fw_revision = reg_value;

	dev_info(&info->i2c_client->dev, "fw_type: 0x%02X, fw_revision: 0x%02X\n",
		info->fw_type, info->fw_revision);

	if (info->fw_revision <= 0x644) {
		return -EINVAL;
	}

	/* Disable Source mode at boot */
	bm92t_set_source_mode(info, SPDSRC12_OFF);

	INIT_WORK(&info->work, bm92t_event_handler);
	INIT_DELAYED_WORK(&info->oneshot_work,
		bm92t_extcon_cable_set_init_state);

	INIT_DELAYED_WORK(&info->power_work, bm92t_power_work);

	if (client->irq > 0) {
		if (request_irq(client->irq, bm92t_interrupt_handler,
				IRQF_TRIGGER_LOW, "bm92t",
				info)) {
			dev_err(&client->dev, "Request irq failed\n");
			destroy_workqueue(info->event_wq);
			return -EBUSY;
		}
	}

	schedule_delayed_work(&info->oneshot_work, msecs_to_jiffies(5000));

#ifdef CONFIG_DEBUG_FS
	bm92t_debug_init(info);
#endif

	dev_info(&client->dev, "bm92txx driver loading done\n");
	return 0;
}

#ifdef CONFIG_PM
static int bm92t_pm_suspend(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_client;

	/* Dim or breathing Dock LED */
	if (info->pdata->led_static_on_suspend)
		bm92t_usbhub_led_cfg_wait(info, 16, 0, 0, 128);
	else
		bm92t_usbhub_led_cfg_wait(info, 32, 1, 255, 255);

	if (client->irq > 0) {
		disable_irq(client->irq);
		enable_irq_wake(client->irq);
	}

	return 0;
}

static int bm92t_pm_resume(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_client;
	bool enable_led = info->state == NINTENDO_CONFIG_HANDLED;

	if (client->irq > 0) {
		enable_irq(client->irq);
		disable_irq_wake(client->irq);
	}

	/*
	 * Toggle DP signal
	 * Do a toggle on resume instead of disable in suspend
	 * and enable in resume, because this also disables the
	 * led effects.
	 */
	if (info->pdata->dp_signal_toggle_on_resume) {
		bm92t_usbhub_dp_sleep(info, true);
		bm92t_usbhub_dp_sleep(info, false);
	}

	/* Set Dock LED to ON state */
	if (enable_led)
		bm92t_usbhub_led_cfg_wait(info, 128, 0, 0, 64);

	return 0;
}

static const struct dev_pm_ops bm92t_pm_ops = {
	.suspend = bm92t_pm_suspend,
	.resume = bm92t_pm_resume,
};
#endif

static const struct i2c_device_id bm92t_id[] = {
	{ "bm92t", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bm92t_id);

static struct i2c_driver bm92t_i2c_driver = {
	.driver = {
		.name = "bm92t",
		.owner = THIS_MODULE,
		.of_match_table = bm92t_of_match,
#ifdef CONFIG_PM
		.pm = &bm92t_pm_ops,
#endif
	},
	.id_table = bm92t_id,
	.probe = bm92t_probe,
	.remove = bm92t_remove,
	.shutdown = bm92t_shutdown,
};

static int __init bm92t_init(void)
{
	return i2c_add_driver(&bm92t_i2c_driver);
}
subsys_initcall_sync(bm92t_init);

static void __exit bm92t_exit(void)
{
	return i2c_del_driver(&bm92t_i2c_driver);
}
module_exit(bm92t_exit);

MODULE_LICENSE("GPL v2");
