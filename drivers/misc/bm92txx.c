/*
 * bm92txx.c
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All Rights Reserved.
 * Copyright (c) 2020 CTCaer <ctcaer@gmail.com>
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

//#define DEBUG 1

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

#define ALERT_STATUS_REG    0x2
#define STATUS1_REG         0x3
#define STATUS2_REG         0x4
#define COMMAND_REG         0x5
#define CONFIG1_REG         0x6
#define DEV_CAPS_REG        0x7
#define READ_PDOS_SRC_REG   0x8
#define CONFIG2_REG         0x17
#define DP_STATUS_REG       0x18
#define DP_ALERT_EN_REG     0x19
#define VENDOR_CONFIG_REG   0x1A
#define AUTO_NGT_FIXED_REG  0x20
#define AUTO_NGT_BATT_REG   0x23
#define SYS_CONFIG1_REG     0x26
#define SYS_CONFIG2_REG     0x27
#define CURRENT_PDO_REG     0x28
#define CURRENT_RDO_REG     0x2B
#define ALERT_ENABLE_REG    0x2E
#define SYS_CONFIG3_REG     0x2F
#define SET_RDO_REG         0x30
#define PDOS_SNK_REG        0x33
#define PDOS_SRC_PROV_REG   0x3C
#define FW_TYPE_REG         0x4B
#define FW_REVISION_REG     0x4C
#define MAN_ID_REG          0x4D
#define DEV_ID_REG          0x4E
#define REV_ID_REG          0x4F
#define INCOMING_VDM_REG    0x50
#define OUTGOING_VDM_REG    0x60

#define ALERT_SNK_FAULT     BIT(0)
#define ALERT_SRC_FAULT     BIT(1)
#define ALERT_CMD_DONE      BIT(2)
#define ALERT_PLUGPULL      BIT(3)
#define ALERT_DP_EVENT      BIT(6)
#define ALERT_DRSWAPPED     BIT(10)
#define ALERT_VDM_RECEIVED  BIT(11)
#define ALERT_CONTRACT      BIT(12)
#define ALERT_SRC_PLUGIN    BIT(13)
#define ALERT_PDO           BIT(14)

/* STATUS1_REG */
#define STATUS1_SPDSRC2     BIT(3)
#define STATUS1_LASTCMD     BIT(4)
#define STATUS1_INSERT      BIT(7)  /* Cable inserted */
#define STATUS1_VSAFE       BIT(10) /* 0: No power, 1: VSAFE 5V or PDO */
#define STATUS1_CSIDE       BIT(11) /* Type-C Plug Side. 0: CC1 Side Valid, 1: CC2 Side Valid */
#define STATUS1_SRC_MODE    BIT(12) /* Source mode (OTG) */
#define STATUS1_CMD_BUSY    BIT(13) /* Command in progress */
#define STATUS1_SPDSNK      BIT(14) /* Sink mode */
#define STATUS1_SPDSRC1     BIT(15) /* VBUS enabled */

#define STATUS1_FAULT_MASK     3
#define STATUS1_DR_SHIFT       8
#define STATUS1_DR_MASK        (3 << STATUS1_DR_SHIFT)
#define STATUS1_LASTCMD_MASK   (8 << 4)

#define DATA_ROLE_UFP   1
#define DATA_ROLE_DFP   2
#define DATA_ROLE_ACC   3

/* STATUS2_REG */
#define STATUS2_VCONN_ON     BIT(9)
#define STATUS2_EM_CABLE     BIT(12) /* Electronically marked cable. Safe for 1.3A */
#define STATUS2_OTG_INSERT   BIT(13)

#define STATUS2_PDOI_SHIFT   3
#define STATUS2_PDOI_MASK    (1 << STATUS2_PDO_SHIFT)
#define STATUS2_ACC_SHIFT    10
#define STATUS2_ACC_MASK     (3 << STATUS2_ACC_SHIFT) /* Accesory mode */

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
#define DP_STATUS_SRC_CONN  BIT(15)

/* CONFIG1_REG */
#define CONFIG1_AUTO_DR_SWAP          BIT(1)
#define CONFIG1_SLEEP_REQUEST         BIT(4)
#define CONFIG1_AUTONGTSNK_VAR_EN     BIT(5)
#define CONFIG1_AUTONGTSNK_FIXED_EN   BIT(6)
#define CONFIG1_AUTONGTSNK_EN         BIT(7)
#define CONFIG1_AUTONGTSNK_BATT_EN    BIT(8)
#define CONFIG1_VINOUT_DELAY_OFF      BIT(9) /* VIN/VOUT turn on delay. 0: 250us, 1: 1ms */

#define CONFIG1_VINOUT_TIME_ON_SHIFT  10 /* VIN/VOUT turn on delay */
#define CONFIG1_VINOUT_TIME_ON_MASK   (3 << CONFIG1_VINOUT_DELAY_SHIFT)
#define CONFIG1_SPDSRC_SHIFT          14
#define CONFIG1_SPDSRC_MASK           (3 << CONFIG1_SPDSRC_SHIFT)

#define VINOUT_TIME_ON_1MS    0
#define VINOUT_TIME_ON_5MS    1
#define VINOUT_TIME_ON_10MS   2
#define VINOUT_TIME_ON_20MS   3

#define SPDSRC12_AUTO         0 /* SPDSRC1/2 Auto on/off */
#define SPDSRC2_AUTO          1
#define SPDSRC1_AUTO          2
#define SPDSRC12_OFF          3 /* SPDSRC1/2 off */

/* CONFIG2_REG */
#define CONFIG2_VSRC_SWAP         BIT(4) /* VCONN source swap. 0: Reject, 1: Accept */
#define CONFIG2_NO_USB_SUSPEND    BIT(5)
#define CONFIG2_EXT_POWERED       BIT(7)

#define CONFIG2_PR_SWAP_MASK      (3 << 0)
#define CONFIG2_DR_SWAP_SHIFT     2
#define CONFIG2_DR_SWAP_MASK      (3 << CONFIG2_DR_SWAP_SHIFT)
#define CONFIG2_TYPEC_AMP_SHIFT   14
#define CONFIG2_TYPEC_AMP_MASK    (3 << CONFIG1_TYPEC_AMP_SHIFT)

#define PR_SWAP_ALWAYS_REJECT         0
#define PR_SWAP_ACCEPT_SNK_REJECT_SRC 1 /* Accept when power sink */
#define PR_SWAP_ACCEPT_SRC_REJECT_SNK 2 /* Accept when power source */
#define PR_SWAP_ALWAYS_ACCEPT         3

#define DR_SWAP_ALWAYS_REJECT         0
#define DR_SWAP_ACCEPT_UFP_REJECT_DFP 1 /* Accept when device */
#define DR_SWAP_ACCEPT_DFP_REJECT_UFP 2 /* Accept when host */
#define DR_SWAP_ALWAYS_ACCEPT         3

#define TYPEC_AMP_0_5A_5V   9
#define TYPEC_AMP_1_5A_5V   10
#define TYPEC_AMP_3_0A_5V   11

/* SYS_CONFIG1_REG */
#define SYS_CONFIG1_USE_AUTONGT_HOST    BIT(6)
#define SYS_CONFIG1_PDO_SNK_CONS        BIT(8)
#define SYS_CONFIG1_PDO_SRC_PROV        BIT(12)
#define SYS_CONFIG1_WAKE_ON_INSERT      BIT(15)

#define SYS_CONFIG1_PLUG_MASK           (0xF << 0)
#define SYS_CONFIG1_PDO_SNK_CONS_SHIFT  9 /* Number of Sink PDOs */
#define SYS_CONFIG1_PDO_SNK_CONS_MASK   (7 << SYS_CONFIG1_PDO_SNK_CONS_NO_SHIFT)
#define SYS_CONFIG1_PDO4_SHIFT          13
#define SYS_CONFIG1_PDO4_MASK           (0x3 << SYS_CONFIG1_PDO4_SHIFT)

#define PLUG_TYPE_C      9
#define PLUG_TYPE_C_3A   10
#define PLUG_TYPE_C_5A   11

#define PDO4_PDO4        0
#define PDO4_PDO5        1
#define PDO4_PDO6        2
#define PDO4_PDO7        3

/* SYS_CONFIG2_REG */
#define SYS_CONFIG2_NO_COMM_UFP          BIT(0) /* Force no USB comms Capable UFP */
#define SYS_CONFIG2_NO_COMM_DFP          BIT(1) /* Force no USB comms Capable DFP */
#define SYS_CONFIG2_NO_COMM_ON_NO_BATT   BIT(2) /* Force no USB comms on dead battery */
#define SYS_CONFIG2_AUTO_SPDSNK_EN       BIT(6) /* Enable without SYS_RDY */
#define SYS_CONFIG2_BST_EN               BIT(8)

#define SYS_CONFIG2_PDO_SRC_PROV_SHIFT   9 /* Number of Source provisioned PDOs */
#define SYS_CONFIG2_PDO_SRC_PROV_MASK    (7 << SYS_CONFIG2_PDO_SRC_PROV_SHIFT)

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

/* Commands */
#define SYS_RDY_CMD       0x0505
#define SEND_RDO_CMD      0x0707
#define STORE_SYSCFG_CMD  0x0909
#define PD_HARD_RST_CMD   0x0D0D
#define SEND_VDM_CMD      0x1111
#define ACCEPT_VDM_CMD    0x1616
#define DR_SWAP_CMD       0x1818
#define DP_MODE_CMD       0x3131 /* Discover DP Alt mode */
#define DP_STOP_CMD       0x3232
#define DP_START_CMD      0x3636 /* Configure and enter selected DP mode */

/* General defines */
#define PDO_TYPE_FIXED  0
#define PDO_TYPE_BATT   1
#define PDO_TYPE_VAR    2

#define PDO_INFO_DR_DATA   (1 << 5)
#define PDO_INFO_USB_COMM  (1 << 6)
#define PDO_INFO_EXT_POWER (1 << 7)
#define PDO_INFO_HP_CAP    (1 << 8)
#define PDO_INFO_DR_POWER  (1 << 9)

#define PD_05V_CHARGING_CURRENT_LIMIT_MA 2000u
#define PD_09V_CHARGING_CURRENT_LIMIT_MA 2000u
#define PD_12V_CHARGING_CURRENT_LIMIT_MA 1500u
#define PD_15V_CHARGING_CURRENT_LIMIT_MA 1200u

#define NON_PD_SUB_POWER_UA 2500000u
#define PD_SUB_POWER_UA 4500000u

#define PD_INPUT_CURRENT_LIMIT_MIN_MA 0u
#define PD_INPUT_CURRENT_LIMIT_MAX_MA 3000u
#define PD_INPUT_VOLTAGE_LIMIT_MAX_MV 17000u

#define DOCK_ID_VOLTAGE_MV 5000u
#define DOCK_ID_CURRENT_MA 500u
#define DOCK_INPUT_VOLTAGE_MV 15000u
#define DOCK_INPUT_CURRENT_LIMIT_MIN_MA 2600u
#define DOCK_INPUT_CURRENT_LIMIT_MAX_MA 3000u

/* VDM/VDO */
#define VDM_CMD_RESERVED   0
#define VDM_CMD_DISC_ID    1
#define VDM_CMD_DISC_SVID  2
#define VDM_CMD_DISC_MODE  3
#define VDM_CMD_ENTER_MODE 4
#define VDM_CMD_EXIT_MODE  5
#define VDM_CMD_ATTENTION  6
#define VDM_CMD_DP_STATUS  16
#define VDM_CMD_DP_CONFIG  17

#define VDM_ACK 0x40
#define VDM_NAK 0x80
#define VDM_UNSTRUCTURED   0x00
#define VDM_STRUCTURED     0x80

#define VDO_ID_TYPE_NONE        0
#define VDO_ID_TYPE_PD_HUB      1
#define VDO_ID_TYPE_PD_PERIPH   2
#define VDO_ID_TYPE_PASS_CBL    3
#define VDO_ID_TYPE_ACTI_CBL    4
#define VDO_ID_TYPE_ALTERNATE   5

#define VDO_DP_UFP_D       BIT(0)
#define VDO_DP_DFP_D       BIT(1)
#define VDO_DP_SUPPORT     BIT(2)
#define VDO_DP_RECEPTACLE  BIT(6)

#define VDO_DP_UFP_PIN_D   BIT(3)

#define VID_NINTENDO      0x057E
#define PID_NIN_DOCK      0x2003
#define PID_NIN_CHARGER   0x2004

#define SVID_NINTENDO VID_NINTENDO
#define SVID_DP       0xFF01


/* All states with ND are for Nintendo Dock */
enum bm92t_state_type {
	INIT_STATE = 0,
	NEW_PDO,
	SYS_RDY_SENT,
	DR_SWAP_SENT,
	VDM_DISC_ID_SENT,
	VDM_ACCEPT_DISC_ID_SENT,
	VDM_DISC_SVID_SENT,
	VDM_ACCEPT_DISC_SVID_SENT,
	VDM_DISC_MODE_SENT,
	VDM_ACCEPT_DISC_MODE_SENT,
	VDM_ENTER_ND_ALT_MODE_SENT,
	VDM_ACCEPT_ENTER_NIN_ALT_MODE_SENT,
	DP_DISCOVER_MODE,
	DP_CONFIG_ENTER_HANDLED,
	VDM_ND_QUERY_DEVICE_SENT,
	VDM_ACCEPT_ND_QUERY_DEVICE_SENT,
	VDM_ND_ENABLE_USBHUB_SENT,
	VDM_ACCEPT_ND_ENABLE_USBHUB_SENT,
	VDM_ND_LED_ON_SENT,
	VDM_ACCEPT_ND_LED_ON_SENT,
	VDM_ND_LED_CUSTOM_SENT,
	VDM_ACCEPT_ND_LED_CUSTOM_SENT,
	VDM_EXIT_DP_MODE_SENT,
	VDM_ACCEPT_EXIT_DP_MODE_SENT,
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

struct bm92t_info {
	struct i2c_client *i2c_client;

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
	"SYS_RDY_SENT",
	"DR_SWAP_SENT",
	"VDM_DISC_ID_SENT",
	"VDM_ACCEPT_DISC_ID_SENT",
	"VDM_DISC_SVID_SENT",
	"VDM_ACCEPT_DISC_SVID_SENT",
	"VDM_DISC_MODE_SENT",
	"VDM_ACCEPT_DISC_MODE_SENT",
	"VDM_ENTER_ND_ALT_MODE_SENT",
	"VDM_ACCEPT_ENTER_NIN_ALT_MODE_SENT",
	"DP_DISCOVER_MODE",
	"DP_CONFIG_ENTER_HANDLED",
	"VDM_ND_QUERY_DEVICE_SENT",
	"VDM_ACCEPT_ND_QUERY_DEVICE_SENT",
	"VDM_ND_ENABLE_USBHUB_SENT",
	"VDM_ACCEPT_ND_ENABLE_USBHUB_SENT",
	"VDM_ND_LED_ON_SENT",
	"VDM_ACCEPT_ND_LED_ON_SENT",
	"VDM_ND_LED_CUSTOM_SENT",
	"VDM_ACCEPT_ND_LED_CUSTOM_SENT",
	"VDM_EXIT_DP_MODE_SENT",
	"VDM_ACCEPT_EXIT_DP_MODE_SENT",
	"NINTENDO_CONFIG_HANDLED",
	"NORMAL_CONFIG_HANDLED"
};

static const unsigned int bm92t_extcon_cable[] = {
	EXTCON_USB_HOST, /* Id */
	EXTCON_USB,      /* Vbus */
	EXTCON_USB_PD,   /* USB-PD */
	EXTCON_DISP_DP,  /* DisplayPort. Must be declared. */
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

unsigned char vdm_discover_id_msg[6] = {OUTGOING_VDM_REG,
	0x04, VDM_CMD_DISC_ID, VDM_STRUCTURED, 0x00, 0xFF};

unsigned char vdm_discover_svid_msg[6] = {OUTGOING_VDM_REG,
	0x04, VDM_CMD_DISC_SVID, VDM_STRUCTURED, 0x00, 0xFF};

unsigned char vdm_discover_mode_msg[6] = {OUTGOING_VDM_REG,
	0x04, VDM_CMD_DISC_MODE, VDM_STRUCTURED, 0x01, 0xFF}; /* DisplayPort Alt Mode */

unsigned char vdm_exit_dp_alt_mode_msg[6] = {OUTGOING_VDM_REG,
	0x04, VDM_CMD_EXIT_MODE, VDM_STRUCTURED | 1, 0x01, 0xFF}; /* DisplayPort Alt Mode*/

unsigned char vdm_enter_nin_alt_mode_msg[6] = {OUTGOING_VDM_REG,
	0x04, VDM_CMD_ENTER_MODE, VDM_STRUCTURED | 1, 0x7E, 0x05}; /* Nintendo Alt Mode */

unsigned char vdm_query_device_msg[10] = {OUTGOING_VDM_REG,
	0x08, VDM_CMD_RESERVED, VDM_UNSTRUCTURED, 0x7E, 0x05, 0x00, 0x01, 0x16, 0x00};

unsigned char vdm_enable_usbhub_msg[10] = {OUTGOING_VDM_REG,
	0x08, VDM_CMD_RESERVED, VDM_UNSTRUCTURED, 0x7E, 0x05, 0x01, 0x01, 0x20, 0x00};

unsigned char vdm_usbhub_led_msg[14] = {OUTGOING_VDM_REG,
	0x0c, VDM_CMD_RESERVED, VDM_UNSTRUCTURED, 0x7E, 0x05, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00}; // Fade, Time off, Time on, Duty.

static int bm92t_write_reg(struct bm92t_info *info,
			   unsigned char *buf, unsigned len)
{
	struct i2c_msg xfer_msg[1];

	xfer_msg[0].addr = info->i2c_client->addr;
	xfer_msg[0].len = len;
	xfer_msg[0].flags = I2C_M_NOSTART;
	xfer_msg[0].buf = buf;

	dev_dbg(&info->i2c_client->dev,
		     "write reg cmd = 0x%02x len = %d\n", buf[0], len);
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
		 "Sent cmd 0x%02x 0x%02x return value %d\n",
		 _cmd[0], _cmd[1], ret);
	return ret;
}

static inline bool bm92t_is_success(const short alert_data)
{
	return (alert_data & ALERT_CMD_DONE);
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

static int bm92t_handle_dp_config_enter(struct bm92t_info *info)
{
	int err;
	bool cfg_valid = false;
	unsigned char msg[5];
	unsigned char cfg[6] = {OUTGOING_VDM_REG, 0x04, 0x06, 0x00, 0x00, 0x00};
	unsigned short cmd = DP_START_CMD;

	err = bm92t_read_reg(info, INCOMING_VDM_REG,
			     msg, sizeof(msg));

	/* Prepare configuration */
	if (info->cable.is_nintendo_dock) {
		 /* Dock reports plug but uses Receptactle */
		if (msg[3] & VDO_DP_UFP_PIN_D) {
			cfg[3] = 0x00;
			cfg[4] = VDO_DP_UFP_PIN_D;
			cfg_valid = true;
		}
	} else if (!(msg[1] & VDO_DP_RECEPTACLE) && /* Plug */
				 msg[2] & VDO_DP_UFP_PIN_D)
	{
		cfg[3] = VDO_DP_UFP_PIN_D;
		cfg[4] = 0x00;
		cfg_valid = true;
	} else if (msg[1] & VDO_DP_RECEPTACLE && /* Receptactle */
			   msg[3] & VDO_DP_UFP_PIN_D)
	{
		cfg[3] = 0x00;
		cfg[4] = VDO_DP_UFP_PIN_D;
		cfg_valid = true;
	}

	/* Check that UFP_U/UFP_D Pin D assignment is supported */
	if (!err && msg[0] == 4 && cfg_valid) {
		/* Set DP configuration */
		err = bm92t_write_reg(info, (unsigned char *)cfg,
				      sizeof(cfg));
		/* Configure and enter DP Alt mode */
		if (!err)
			bm92t_send_cmd(info, &cmd);
		else {
			dev_err(&info->i2c_client->dev, "Writing VDM failed\n");
			return -ENODEV;
		}
	} else {
		dev_err(&info->i2c_client->dev,
			"Cannot handle DP configure (%d: %02X %02X %02X).\n",
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
	bool is_enabled = regulator_is_enabled(info->vbus_reg);

	dev_dbg(&info->i2c_client->dev,
		"%s VBUS\n", enable ? "Enabling" : "Disabling");
	if (info->vbus_reg != NULL) {
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

	err = bm92t_read_reg(info, CONFIG1_REG, (unsigned char *) &value, sizeof(value));
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

static int bm92t_hard_pd_reset_auto(struct bm92t_info *info, bool force)
{
	int err = 0;
	unsigned short cmd = PD_HARD_RST_CMD;
	unsigned short alert_data, status1_data, dp_data;

	if (force) {
		dev_info(&info->i2c_client->dev, "PD Hard Reset requested!\n");
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
			dp_data & DP_STATUS_SRC_CONN ||
			status1_data & 0x70) {
			dev_err(&info->i2c_client->dev,
				"Invalid state, initiating PD Hard Reset!!\n");
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

	/* Subtract a USB2 or USB3 port current */
	if (voltage > 5000)
		charging_limit -= (PD_SUB_POWER_UA / voltage);
	else
		charging_limit -= (NON_PD_SUB_POWER_UA / voltage);

	/* Set limits */
	switch (voltage)
	{
	case 5000:
		charging_limit = min(charging_limit, PD_05V_CHARGING_CURRENT_LIMIT_MA);
		break;
	case 9000:
		charging_limit = min(charging_limit, PD_09V_CHARGING_CURRENT_LIMIT_MA);
		break;
	case 12000:
		charging_limit = min(charging_limit, PD_12V_CHARGING_CURRENT_LIMIT_MA);
		break;
	case 15000:
	default:
		charging_limit = min(charging_limit, PD_15V_CHARGING_CURRENT_LIMIT_MA);
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
	unsigned short value;
	unsigned char msg[3] = {VENDOR_CONFIG_REG, 0, 0};
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, oneshot_work);

	dev_info(&info->i2c_client->dev,
		 "extcon cable is set to init state\n");

	disable_irq(info->i2c_client->irq);

	bm92t_set_vbus_enable(info, false);

	/* In case UFP is in an invalid state, request a PD hard reset */
	bm92t_hard_pd_reset_auto(info, false);

	/* Enable OTG detection */
	bm92t_read_reg(info, VENDOR_CONFIG_REG, (unsigned char *) &value, sizeof(value));
	if (value & 4) {
		value &= 0xFFFB;
		msg[1] = value & 0xFF;
		msg[2] = (value >> 8) & 0xFF;
		bm92t_write_reg(info, msg, sizeof(msg));
	}

	/* Enable auto power to SPDSRC for supporting both OTG and Charger */
	bm92t_set_source_mode(info, SPDSRC12_AUTO);

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
	if (pdos_no == 2 &&
		(pdo[0].volt * 50) == DOCK_ID_VOLTAGE_MV  &&
		(pdo[0].amp * 10)  == DOCK_ID_CURRENT_MA)
	{
		/* Only accept 15V, >= 2.6A for dock mode. */
		if (pdo[1].type == PDO_TYPE_FIXED &&
			(pdo[1].volt * 50) == DOCK_INPUT_VOLTAGE_MV &&
			(pdo[1].amp * 10)  >= DOCK_INPUT_CURRENT_LIMIT_MIN_MA &&
			(pdo[1].amp * 10)  <= DOCK_INPUT_CURRENT_LIMIT_MAX_MA)
		{
			dev_info(dev, "Adapter in nintendo dock\n");
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
		switch (voltage)
		{
		case 5000:
		case 9000:
		case 12000:
		case 15000:
			break;
		default:
			continue;
		}

		/* Only accept <= 3A and select max wattage with max voltage possible. */
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
		dev_info(&info->i2c_client->dev, "Adapter in charger/hub mode\n");
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
		dev_err(&info->i2c_client->dev, "Send RDO failure!!\n");
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
		dev_err(&info->i2c_client->dev, "Send VDM failure!!\n");
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

static void bm92t_print_dp_dev_info(struct device *dev,
	struct vd_object *vdo)
{
	dev_info(dev, "Connected PD device:\n");
	dev_info(dev, "VID: %04X, PID: %04X\n", vdo->vid, vdo->pid);

	switch (vdo->type)
	{
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

	static int retries_usbhub = 10;

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
		"Alert= 0x%04x Status1= 0x%04x Status2= 0x%04x DP= 0x%04x State=%s\n",
		alert_data, status1_data, status2_data, dp_data, states[info->state]);

	/* Check for errors */
	err = status1_data & STATUS1_FAULT_MASK;
	if (err) {
		dev_err(dev,
			"Internal error occurred. Ecode = %d\n", err);
		bm92t_state_machine(info, INIT_STATE);
		bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
		bm92t_extcon_cable_update(info, EXTCON_USB, false);
		bm92t_set_vbus_enable(info, false);
		if (bm92t_is_plugged(status1_data) || alert_data == 0)
			bm92t_hard_pd_reset_auto(info, true);
		goto ret;
	}

	/* Check if power source was connected while VBUS was enabled */
	if (alert_data & ALERT_SRC_FAULT) {
		dev_err(dev, "VBUS error occurred.\n");
		goto ret;
	}

	/* Check if cable removed */
	if (alert_data & ALERT_PLUGPULL) {
		if (!bm92t_is_plugged(status1_data)) {
			cancel_delayed_work(&info->power_work);

			bm92t_set_vbus_enable(info, false);

			if (info->charging_enabled) {
				bm92t_set_current_limit(info, 0);
				info->charging_enabled = false;
				bm92t_extcon_cable_update(info, EXTCON_USB_PD, false);
			}

			retries_usbhub = 10;
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_state_machine(info, INIT_STATE);
		} else if (status1_data & STATUS1_SRC_MODE && /* OTG event */
				   status2_data & STATUS2_OTG_INSERT) {
			bm92t_set_vbus_enable(info, true);
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
		}
		goto ret;
	}

	switch (info->state) {
	case INIT_STATE:
		if (alert_data & ALERT_SRC_PLUGIN) {
			dev_info(dev, "Source/OTG HUB plug-in\n");
			info->first_init = false;
			if (bm92t_is_dfp(status1_data)) {
				/* Reset cable info */
				memset(&info->cable, 0, sizeof(struct bm92t_device));

				bm92t_send_vdm(info, vdm_discover_id_msg,
					sizeof(vdm_discover_id_msg));
				bm92t_state_machine(info, VDM_DISC_ID_SENT);
			}
			goto ret;
		}

		if (status1_data & STATUS1_SRC_MODE &&
				status2_data & STATUS2_OTG_INSERT) {
			info->first_init = false;
			dev_info(dev, "OTG adapter\n");
			goto ret;
		}

		if ((alert_data & ALERT_CONTRACT) || info->first_init) {
			info->first_init = false;

			/* Disable USB if first init and unplugged */
			if (!bm92t_is_plugged(status1_data)) {
				bm92t_extcon_cable_update(info, EXTCON_USB, false);
				goto ret;
			}

			/* Negotiate new power profile */
			if (!bm92t_check_pdo(info)) {
				dev_err(dev, "Power Negotiation failed\n");
				bm92t_state_machine(info, INIT_STATE);
				msleep(550); /* WAR: BQ2419x good power test */
				bm92t_extcon_cable_update(info, EXTCON_USB, true);
				goto ret;
			}
			bm92t_send_rdo(info);
			bm92t_state_machine(info, NEW_PDO);
			msleep(20);
			goto ret;
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

			cmd = SYS_RDY_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, SYS_RDY_SENT);
		}
		break;

	case SYS_RDY_SENT:
		if (bm92t_is_success(alert_data)) {
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			schedule_delayed_work(&info->power_work,
				msecs_to_jiffies(2000));

			if (bm92t_is_ufp(status1_data)) {
				/* Check if Dual-Role Data is supported */
				if (!info->cable.drd_support) {
					dev_dbg(dev,
						"DRD not supported, cmd done in SYS_RDY_SENT\n");
					goto ret;
				}

				cmd = DR_SWAP_CMD;
				err = bm92t_send_cmd(info, &cmd);
				bm92t_state_machine(info, DR_SWAP_SENT);
			}
			else if (bm92t_is_dfp(status1_data)) {
				dev_dbg(dev, "Already in DFP mode\n");
				if ((status1_data & 0xff) == STATUS1_INSERT) {
					bm92t_send_vdm(info, vdm_discover_id_msg,
						sizeof(vdm_discover_id_msg));
					bm92t_state_machine(info, VDM_DISC_ID_SENT);
				}
			}
		}
		break;

	case DR_SWAP_SENT:
		if (bm92t_is_success(alert_data) &&
			((status1_data & 0xff) == STATUS1_INSERT) &&
			bm92t_is_dfp(status1_data)) {
			bm92t_send_vdm(info, vdm_discover_id_msg,
				sizeof(vdm_discover_id_msg));
			bm92t_state_machine(info, VDM_DISC_ID_SENT);
		}
		break;

	case VDM_DISC_ID_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_ID_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_ID_SENT\n");
		break;

	case VDM_ACCEPT_DISC_ID_SENT:
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
				dev_err(dev, "VID/PID not recognized\n");
				bm92t_send_vdm(info, vdm_discover_svid_msg,
					sizeof(vdm_discover_svid_msg));
				bm92t_state_machine(info, VDM_DISC_SVID_SENT);
				goto ret;
			}

			info->cable.is_nintendo_dock = true;
			bm92t_send_vdm(info, vdm_enter_nin_alt_mode_msg,
				sizeof(vdm_enter_nin_alt_mode_msg));
			bm92t_state_machine(info, VDM_ENTER_ND_ALT_MODE_SENT);
		}
		break;

	case VDM_DISC_SVID_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_SVID_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_SVID_SENT\n");
		break;

	case VDM_ACCEPT_DISC_SVID_SENT:
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
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_MODE_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_DISC_MODE_SENT\n");
		break;

	case VDM_ACCEPT_DISC_MODE_SENT:
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
				cmd = DP_MODE_CMD;
				err = bm92t_send_cmd(info, &cmd);
				msleep(100); /* WAR: may not need to wait */
				bm92t_state_machine(info, DP_DISCOVER_MODE);
			}
		}
		break;

	case VDM_ENTER_ND_ALT_MODE_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ENTER_NIN_ALT_MODE_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ENTER_ND_ALT_MODE_SENT\n");
		break;

	case VDM_ACCEPT_ENTER_NIN_ALT_MODE_SENT:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));

			/* Check if supported. */
			if (!(vdm[1] == (VDM_ACK | VDM_CMD_ENTER_MODE) &&
				vdm[2] == (VDM_STRUCTURED | 1) &&
				vdm[3] == 0x7e && vdm[4] == 0x05)) {
				dev_err(dev, "Failed to enter Nintendo Alt Mode\n");
				goto ret;
			}

			/* Enter automatic DisplayPort handling */
			msleep(100);
			cmd = DP_MODE_CMD;
			err = bm92t_send_cmd(info, &cmd);
			msleep(100); /* WAR: may not need to wait */
			bm92t_state_machine(info, DP_DISCOVER_MODE);
		}
		break;

	case DP_DISCOVER_MODE:
		if (bm92t_is_success(alert_data)) {
			err = bm92t_handle_dp_config_enter(info);
			if (!err)
				bm92t_state_machine(info, DP_CONFIG_ENTER_HANDLED);
			else
				bm92t_state_machine(info, INIT_STATE);
		}
		break;

	case DP_CONFIG_ENTER_HANDLED:
		if (bm92t_is_success(alert_data)) {
			if ((status1_data & 0xff) == STATUS1_INSERT) {
				if (info->cable.is_nintendo_dock) {
					bm92t_send_vdm(info, vdm_query_device_msg,
						sizeof(vdm_query_device_msg));
					bm92t_state_machine(info, VDM_ND_QUERY_DEVICE_SENT);
				} else
					bm92t_state_machine(info, NORMAL_CONFIG_HANDLED);
			} else
				dev_err(dev, "Failed to enter DP Alt mode\n");
		}
		break;

	/* Nintendo Dock VDMs */
	case VDM_ND_QUERY_DEVICE_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_QUERY_DEVICE_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_QUERY_DEVICE_SENT\n");
		break;

	case VDM_ACCEPT_ND_QUERY_DEVICE_SENT:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG,
				vdm, sizeof(vdm));
			dev_info(dev, "device state = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
				vdm[9], vdm[10], vdm[11], vdm[12]);

			if (vdm[11] & 0x02) {
				bm92t_extcon_cable_update(info,
					EXTCON_USB_HOST, false);
				msleep(500);
				bm92t_extcon_cable_update(info,
					EXTCON_USB, true);
			}

			bm92t_usbhub_led_cfg(info, 128, 0, 0, 64);
			bm92t_state_machine(info, VDM_ND_LED_ON_SENT);
		}
		break;

	case VDM_ND_LED_ON_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_LED_ON_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_LED_ON_SENT\n");
		break;

	case VDM_ACCEPT_ND_LED_ON_SENT:
		if (bm92t_is_success(alert_data)) {
			bm92t_send_vdm(info, vdm_enable_usbhub_msg,
				sizeof(vdm_enable_usbhub_msg));
			bm92t_state_machine(info, VDM_ND_ENABLE_USBHUB_SENT);
		}
		break;

	case VDM_ND_ENABLE_USBHUB_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_ENABLE_USBHUB_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_ENABLE_USBHUB_SENT\n");
		break;

	case VDM_ACCEPT_ND_ENABLE_USBHUB_SENT:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG,
				vdm, sizeof(vdm));

			if ((vdm[5] & 0x1) && retries_usbhub) {
				msleep(250);
				dev_info(dev, "Retrying enabling USB HUB.\n");
				bm92t_send_vdm(info, vdm_enable_usbhub_msg,
					sizeof(vdm_enable_usbhub_msg));
				bm92t_state_machine(info,
					VDM_ND_ENABLE_USBHUB_SENT);
				retries_usbhub--;
			} else if (!retries_usbhub)
				dev_err(dev, "USB HUB check failed\n");
			else
				bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;

	case VDM_ND_LED_CUSTOM_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_ND_LED_CUSTOM_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_ND_LED_CUSTOM_SENT\n");
		break;

	case VDM_ACCEPT_ND_LED_CUSTOM_SENT:
		if (bm92t_is_success(alert_data)) {
			/* Read incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, sizeof(vdm));
			bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;
	/* End of Nintendo Dock VDMs */

	case VDM_EXIT_DP_MODE_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_EXIT_DP_MODE_SENT);
		} else if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in VDM_EXIT_DP_MODE_SENT\n");
		break;

	case VDM_ACCEPT_EXIT_DP_MODE_SENT:
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
		dev_err(dev, "Invalid state\n");
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
	int retries = 100;
	struct bm92t_info *info = i2c_get_clientdata(client);

	dev_info(&info->i2c_client->dev, "%s\n", __func__);

	/* Disable Dock LED if enabled */
	if (info->state == NINTENDO_CONFIG_HANDLED) {
		bm92t_state_machine(info, VDM_ND_LED_CUSTOM_SENT);
		bm92t_usbhub_led_cfg(info, 0, 0, 0, 128);
		while (info->state != NINTENDO_CONFIG_HANDLED)
		{
			retries--;
			if (retries < 0)
				break;
			msleep(1);
		}
	}

	/* Disable SPDSRC */
	bm92t_set_source_mode(info, SPDSRC12_OFF);
}

#ifdef CONFIG_DEBUG_FS
static int bm92t_regs_print(struct seq_file *s, unsigned char reg_addr, int size)
{
	int err;
	unsigned char msg[5];
	unsigned short reg_val16;
	unsigned short reg_val32;
	struct bm92t_info *info = (struct bm92t_info *) (s->private);

	switch (size)
	{
	case 2:
		err = bm92t_read_reg(info, reg_addr,
			    (unsigned char *) &reg_val16, sizeof(reg_val16));
		if (!err)
			seq_printf(s, "0x%02x: 0x%04x\n", reg_addr, reg_val16);
		break;
	case 4:
		err = bm92t_read_reg(info, reg_addr,
			    (unsigned char *) &reg_val32, sizeof(reg_val32));
		if (!err)
			seq_printf(s, "0x%02x: 0x%08x\n", reg_addr, reg_val32);
		break;
	case 5:
		err = bm92t_read_reg(info, reg_addr, msg, sizeof(msg));
		if (!err)
			seq_printf(s, "0x%02x: 0x%02x%02x%02x%02x\n",
				reg_addr, msg[4], msg[3], msg[2], msg[1]);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		dev_err(&info->i2c_client->dev,
				"debugfs cannot read 0x%02x\n", reg_addr);

	return err;
}

static int bm92t_regs_show(struct seq_file *s, void *data)
{
	int err;

	err = bm92t_regs_print(s, ALERT_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, STATUS1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, STATUS2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, DEV_CAPS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, DP_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, DP_ALERT_EN_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, VENDOR_CONFIG_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, AUTO_NGT_FIXED_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, AUTO_NGT_BATT_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, SYS_CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, SYS_CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, CURRENT_PDO_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, CURRENT_RDO_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, ALERT_ENABLE_REG, 4);
	if (err)
		return err;
	err = bm92t_regs_print(s, SYS_CONFIG3_REG, 2);
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

	seq_printf(s, "fw_type: 0x%02x, fw_revision: 0x%02x\n",
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
		if (info->state == VDM_ACCEPT_ND_ENABLE_USBHUB_SENT ||
			info->state == VDM_ACCEPT_ND_LED_CUSTOM_SENT)
		{
			bm92t_state_machine(info, VDM_ND_LED_CUSTOM_SENT);
			bm92t_usbhub_led_cfg(info, duty, time_on, time_off, fade);
		}
		else
			dev_err(&info->i2c_client->dev,
				"Led is not supported\n");
	}
	else
	{
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
	}
	else
	{
		dev_err(&info->i2c_client->dev,
				"CMD syntax is: duty time_on time_off fade\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_cmd_fops = {
	.open = simple_open,
	.write = bm92t_cmd_write,
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

static int bm92t_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct bm92t_info *info;
	int err;
	unsigned short reg_value;

	dev_info(&client->dev, "%s\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}
	i2c_set_clientdata(client, info);

	info->i2c_client = client;
	info->batt_chg_reg = devm_regulator_get(&client->dev, "pd_bat_chg");
	if (IS_ERR(info->batt_chg_reg)) {
		err = PTR_ERR(info->batt_chg_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_info(&client->dev,
				"pd_bat_chg reg not registered: %d\n", err);
		info->batt_chg_reg = NULL;
	}

	info->vbus_reg = devm_regulator_get(&client->dev, "vbus");
	if (IS_ERR(info->vbus_reg)) {
		err = PTR_ERR(info->vbus_reg);

		dev_info(&client->dev,
				"vbus reg not registered: %d\n", err);
		info->vbus_reg = NULL;
	}

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

	dev_info(&info->i2c_client->dev, "fw_type: 0x%02x, fw_revision: 0x%02x\n",
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

	if (client->irq) {
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
