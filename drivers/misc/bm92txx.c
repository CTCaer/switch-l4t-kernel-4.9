/*
 * bm92txx.c
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All Rights Reserved.
 *
 * Authors:
 *     Adam Jiang <chaoj@nvidia.com>
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
#define DEBUG 1
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
#include "bm92txx.h"

#define ALERT_STATUS    0x2
#define STATUS1         0x3
#define STATUS2         0x4
#define COMMAND         0x5
#define CONFIG          0x6
#define DP_STATUS       0x18
#define CURRENT_PDO     0x28
#define CURRENT_RDO     0x2B
#define INCOMING_VDM    0x50
#define OUTGOING_VDM    0x60
#define FW_TYPE         0x4B
#define FW_REVISION     0x4C
#define READ_PDOS       0x08
#define SET_RDO         0x30

#define ALERT_CMD_DONE      BIT(2)
#define ALERT_PLUGPULL      BIT(3)
#define ALERT_DRSWAPPED     BIT(10)
#define ALERT_VDM_RECEIVED  BIT(11)
#define ALERT_CONTR         BIT(12)
#define ALERT_PDO           BIT(14)

#define STATUS1_LASTCMD     BIT(4)
#define STATUS1_INSERT      BIT(7)
#define STATUS1_SPDSNK      BIT(14)

#define STATUS1_DR_SHIFT        8
#define STATUS1_DR_MASK         (3 << STATUS1_DR_SHIFT)
#define STATUS1_LASTCMD_MASK    (8 << 4)

#define SYS_RDY_CMD       0x0505
#define SEND_RDO_CMD      0x0707
#define SEND_VDM_CMD      0x1111
#define ACCEPT_VDM_CMD    0x1616
#define DR_SWAP_CMD       0x1818
#define DP_MODE_CMD       0x3131
#define DP_START_CMD      0x3636

#define DATA_ROLE_UFP   1
#define DATA_ROLE_DFP   2

#define PD_CHARGING_CURRENT_LIMIT_UA 1500000u

enum bm92t_state_type {
	INIT_STATE = 0,
	NEW_PDO,
	SYS_RDY_SENT,
	DR_SWAP_SENT,
	VDM_ID_PHASE1_SENT,
	VDM_ACCEPT_PHASE1_SENT,
	VDM_ID_PHASE2_SENT,
	VDM_ACCEPT_PHASE2_SENT,
	ENTER_DP_MODE,
	HPD_HANDLED,
	VDM_QUERY_DEVICE_SENT,
	VDM_ACCEPT_QUERY_DEVICE_SENT,
	VDM_CHECK_USBHUB_SENT,
	VDM_ACCEPT_CHECK_USBHUB_SENT
};

enum bm92t_extcon_cable_type {
	USB_HOST = 0,
	USB,
	USB_PD
};

enum bm92t_vdm_id_type {
	VDM_IDENTIFICATION_PHASE1 = 0,
	VDM_IDENTIFICATION_PHASE2,
};

struct bm92t_info {
	struct i2c_client *i2c_client;
	struct bm92t_platform_data *pdata;

	struct work_struct work;
	struct workqueue_struct *event_wq;
	struct completion cmd_done;

	int state;

	struct extcon_dev edev;
	struct delayed_work oneshot_work;
	struct delayed_work power_work;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
	struct regulator *batt_chg_reg;
	bool charging_enabled;
	unsigned int fw_type;
	unsigned int fw_revision;
};

static const char * const states[] = {
	"INIT_STATE",
	"NEW_PDO",
	"SYS_RDY_SENT",
	"DR_SWAP_SENT",
	"VDM_ID_PHASE1_SENT",
	"VDM_ACCEPT_PHASE1_SENT",
	"VDM_ID_PHASE2_SENT",
	"VDM_ACCEPT_PHASE2_SENT",
	"ENTER_DP_MODE",
	"HPD_HANDLED",
	"VDM_QUERY_DEVICE_SENT",
	"VDM_ACCEPT_QUERY_DEVICE_SENT",
	"VDM_CHECK_USBHUB_SENT",
	"VDM_ACCEPT_CHECK_USBHUB_SENT"
};

struct bm92t_platform_data bm92t_dflt_pdata = {
	.irq_gpio = -1,
};

static const unsigned int bm92t_extcon_cable[] = {
	EXTCON_USB_HOST, /* id */
	EXTCON_USB, /* vbus */
	EXTCON_USB_PD, /* usb-pd */
	EXTCON_NONE,
};

unsigned char vdm_id_phase1_msg[6] = {OUTGOING_VDM,
	0x04, 0x01, 0x80, 0x00, 0xFF};
unsigned char vdm_id_phase2_msg[6] = {OUTGOING_VDM,
	0x04, 0x04, 0x81, 0x7E, 0x05};
unsigned char vdm_query_device_msg[10] = {OUTGOING_VDM,
	0x08, 0x00, 0x00, 0x7E, 0x05, 0x00, 0x01, 0x16, 0x00};
unsigned char vdm_check_usbhub_msg[10] = {OUTGOING_VDM,
	0x08, 0x00, 0x00, 0x7E, 0x05, 0x01, 0x01, 0x20, 0x00};

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

	reg = COMMAND;

	msg[0] = reg;
	msg[1] = _cmd[0];
	msg[2] = _cmd[1];

	ret = bm92t_write_reg(info, msg, 3);
	dev_info(&info->i2c_client->dev,
		 "Sent cmd 0x%02x 0x%02x return value %d\n",
		 _cmd[0], _cmd[1], ret);
	return ret;
}

static int bm92t_handle_hpd(struct bm92t_info *info)
{
	int err;
	unsigned char msg[5];
	unsigned short hpd[3] = { 0x0460, 0x0006, 0x0008 };
	unsigned short cmd = DP_START_CMD;

	err = bm92t_read_reg(info, INCOMING_VDM,
			     msg, sizeof(msg));
	if (!err && msg[0] == 4 && (msg[3] & 0x08) == 0x08) {
		err = bm92t_write_reg(info, (unsigned char *) hpd,
				      sizeof(hpd));
		if (!err)
			bm92t_send_cmd(info, &cmd);
		else {
			dev_err(&info->i2c_client->dev, "Writing VDM failed");
			return -ENODEV;
		}
	} else {
		dev_err(&info->i2c_client->dev, "Cannot handle HPD event.\n");
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

static void bm92t_extcon_cable_update(struct bm92t_info *info,
	const unsigned int cable, bool is_attached)
{
	int state = extcon_get_cable_state_(&info->edev, cable);

	if (state != is_attached) {
		dev_info(&info->i2c_client->dev, "extcon cable(%d) %s\n", cable,
			 is_attached ? "attached" : "detached");
		extcon_set_cable_state_(&info->edev, cable, is_attached);
	}
}

static inline void bm92t_state_machine(struct bm92t_info *info, int state)
{
	info->state = state;
	dev_dbg(&info->i2c_client->dev, "state = %s\n", states[state]);
}

static inline bool bm92t_is_success(const short alert_data,
			      const short status1_data)
{
	return alert_data & ALERT_CMD_DONE;
}

static void bm92t_power_work(struct work_struct *work)
{
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, power_work);

	bm92t_set_current_limit(info, PD_CHARGING_CURRENT_LIMIT_UA);
	info->charging_enabled = true;

	extcon_set_cable_state_(&info->edev, EXTCON_USB_PD, true);
}

static inline void bm92t_swap_data_role(struct bm92t_info *info,
					const short status1_data)
{
	unsigned char dr =
		(status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT;

	dev_info(&info->i2c_client->dev, "DataRole is %s\n",
		 (dr == 2) ? "DFP" : "UFP ");

	switch (dr) {
	case DATA_ROLE_DFP:
		bm92t_extcon_cable_update(info,
			EXTCON_USB_HOST, true);
		break;
	case DATA_ROLE_UFP:
		bm92t_extcon_cable_update(info,
			EXTCON_USB, true);
		break;
	}
}

static void
	bm92t_extcon_cable_set_init_state(struct work_struct *work)
{
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, oneshot_work);

	unsigned short status1_data;
	int err;
	unsigned char dr;

	if (info->state > INIT_STATE)
		goto ret;

	dev_info(&info->i2c_client->dev,
		 "extcon cable is set to init state\n");

	err = bm92t_read_reg(info, STATUS1,
			 (unsigned char *) &status1_data, sizeof(status1_data));

	dev_info(&info->i2c_client->dev, "addr: 0x%02x, val: 0x%04x\n",
		STATUS1, status1_data);

	dr = (status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT;

	if (dr == DATA_ROLE_UFP) {
		bm92t_extcon_cable_update(info, EXTCON_USB, true);
		goto ret;
	} else if (dr == DATA_ROLE_DFP) {
		bm92t_set_current_limit(info, PD_CHARGING_CURRENT_LIMIT_UA);
		info->charging_enabled = true;
		bm92t_extcon_cable_update(info,
			EXTCON_USB_PD, true);
		bm92t_extcon_cable_update(info,
			EXTCON_USB_HOST, true);
		bm92t_state_machine(info, HPD_HANDLED);
	}
ret:
	return;
}

static void
	bm92t_extcon_cable_set_init_state2(struct work_struct *work)
{
	struct bm92t_info *info = container_of(
		to_delayed_work(work), struct bm92t_info, oneshot_work);

	dev_info(&info->i2c_client->dev,
		 "extcon cable is set to init state\n");

	bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
	bm92t_extcon_cable_update(info, EXTCON_USB, true);
}

static bool bm92t_check_pdo(struct bm92t_info *info)
{
	int err;
	unsigned char pdos[28];

	err = bm92t_read_reg(info, READ_PDOS,
			     pdos, sizeof(pdos));

	dev_dbg(&info->i2c_client->dev, "B05=0x%x, B06=0x%x, B7=0x%x, B8=0x%x\n",
			pdos[5], pdos[6], pdos[7], pdos[8]);

	if (pdos[5] == 0x04 && pdos[6] == 0xb1 &&
		pdos[7] == 0x04 && pdos[8] == 0x00)
		return 1;

	return 0;
}

static int bm92t_send_rdo(struct bm92t_info *info)
{
	int err;
	unsigned char msg[6] = { SET_RDO, 0x04, 0x04, 0x11, 0x04, 0x20};
	unsigned short cmd = SEND_RDO_CMD;

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

static void bm92t_event_handler(struct work_struct *work)
{
	int err;
	struct bm92t_info *info;
	struct device *dev;
	unsigned short cmd;
	unsigned short alert_data;
	unsigned short status1_data;

	info = container_of(work, struct bm92t_info, work);
	dev = &info->i2c_client->dev;

	/* read status registers at 02h, 03h and 04h */
	err = bm92t_read_reg(info, ALERT_STATUS,
			     (unsigned char *) &alert_data,
			     sizeof(alert_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, STATUS1,
			     (unsigned char *) &status1_data,
			     sizeof(status1_data));
	dev_info_ratelimited(dev, "AlertStatus = 0x%04x, Status1 = 0x%04x\n",
		alert_data, status1_data);

	if (!err && !(status1_data & 0x3)) {
		/* Check initialization completed */
		if (alert_data & ALERT_CMD_DONE && !status1_data) {
			bm92t_state_machine(info, INIT_STATE);
			goto ret;
		}

		/* Check if cable removed */
		if (alert_data & ALERT_PLUGPULL) {
			if (!(status1_data & STATUS1_INSERT)) {
				cancel_delayed_work(&info->power_work);
				if (info->charging_enabled) {
					bm92t_set_current_limit(info, 0);
					info->charging_enabled = false;
					bm92t_extcon_cable_update(info,
						EXTCON_USB_PD, false);
				}

				if (info->state == INIT_STATE)
					bm92t_extcon_cable_update(info,
						EXTCON_USB, false);
				else if (info->state >= DR_SWAP_SENT)
					bm92t_extcon_cable_update(info,
						EXTCON_USB_HOST, false);

				bm92t_state_machine(info, INIT_STATE);
			}
			goto ret;
		}

		/* Check if we are in the last state */
		if (info->state != HPD_HANDLED) {
			/* Check Power Negotiation */
			if (alert_data & ALERT_PDO) {
				bm92t_state_machine(info, NEW_PDO);
				goto ret;
			}

			if ((alert_data & ALERT_CONTR) &&
			    (info->state == NEW_PDO)) {
				msleep(550);
				cmd = SYS_RDY_CMD;
				err = bm92t_send_cmd(info, &cmd);
				bm92t_state_machine(info, SYS_RDY_SENT);
				goto ret;
			}

			if ((alert_data & ALERT_CONTR) &&
				(info->state == INIT_STATE)) {
				bm92t_swap_data_role(info, status1_data);
				goto ret;
			}

			/* Check System Ready */
			if (bm92t_is_success(alert_data, status1_data) &&
			    (info->state == SYS_RDY_SENT)) {
				schedule_delayed_work(&info->power_work,
					msecs_to_jiffies(500));
				cmd = DR_SWAP_CMD;
				err = bm92t_send_cmd(info, &cmd);
				bm92t_state_machine(info, DR_SWAP_SENT);
				goto ret;
			}

			/* Send Data-role swap */
			if (bm92t_is_success(alert_data, status1_data) &&
			    ((status1_data & 0xff) == 0x80) &&
			    (info->state == DR_SWAP_SENT)) {
				bm92t_swap_data_role(info, status1_data);

				cmd = DP_MODE_CMD;
				err = bm92t_send_cmd(info, &cmd);
				msleep(100); /* WAR: may not need to wait */
				bm92t_state_machine(info, ENTER_DP_MODE);
				goto ret;
			}

			/* Handle HPD */
			if (bm92t_is_success(alert_data, status1_data) &&
			    (info->state == ENTER_DP_MODE)) {
				err = bm92t_handle_hpd(info);
				if (!err)
					bm92t_state_machine(info, HPD_HANDLED);
				else
					bm92t_state_machine(info, INIT_STATE);
				goto ret;
			}
		}
	} else
		dev_err(&info->i2c_client->dev, "Internal error occurred.\n");

ret:
	enable_irq(info->i2c_client->irq);
}

static void bm92t_event_handler2(struct work_struct *work)
{
	int err;
	struct bm92t_info *info;
	struct device *dev;
	unsigned short cmd;
	unsigned short alert_data;
	unsigned short status1_data;
	unsigned char vdm[28], pdo[4], rdo[4];

	info = container_of(work, struct bm92t_info, work);
	dev = &info->i2c_client->dev;

	/* read status registers at 02h, 03h and 04h */
	err = bm92t_read_reg(info, ALERT_STATUS,
			     (unsigned char *) &alert_data,
			     sizeof(alert_data));
	if (err < 0)
		goto ret;
	err = bm92t_read_reg(info, STATUS1,
			     (unsigned char *) &status1_data,
			     sizeof(status1_data));
	if (err < 0)
		goto ret;

	dev_info_ratelimited(dev, "Alert= 0x%04x Status1= 0x%04x State=%s\n",
		alert_data, status1_data, states[info->state]);

	err = status1_data & 0x3;
	if (err) {
		dev_err(dev,
			"Internal error occurred. Ecode = %d\n", err);
		bm92t_state_machine(info, INIT_STATE);
		bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
		bm92t_extcon_cable_update(info, EXTCON_USB, false);
		goto ret;
	}

	/* Check if cable removed */
	if (alert_data & ALERT_PLUGPULL) {
		if (!(status1_data & STATUS1_INSERT)) {
			cancel_delayed_work(&info->power_work);
			if (info->charging_enabled) {
				bm92t_set_current_limit(info, 0);
				info->charging_enabled = false;
				bm92t_extcon_cable_update(info,
					EXTCON_USB_PD, false);
			}

			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_state_machine(info, INIT_STATE);
		}
		goto ret;
	}

	switch (info->state) {
	case INIT_STATE:
		if (alert_data & ALERT_CONTR) {
			if (!bm92t_check_pdo(info)) {
				dev_err(dev, "Power Nego failed\n");
				bm92t_state_machine(info, INIT_STATE);
				bm92t_extcon_cable_update(info,
					EXTCON_USB, true);
				goto ret;
			}
			bm92t_send_rdo(info);
			bm92t_state_machine(info, NEW_PDO);
			msleep(20);
		}
		break;
	case NEW_PDO:
		if (bm92t_is_success(alert_data, status1_data))
			dev_dbg(dev, "cmd done in NEW_PDO state\n");

		if (alert_data & ALERT_CONTR) {
			/* check PDO/RDO */
			err = bm92t_read_reg(info, CURRENT_PDO,
			pdo, sizeof(pdo));
			dev_dbg(dev, "current PDO B0=0x%x, B1=0x%x, B2=0x%x, B3=0x%x\n",
			pdo[0], pdo[1], pdo[2], pdo[3]);

			err = bm92t_read_reg(info, CURRENT_RDO,
			rdo, sizeof(rdo));
			dev_dbg(dev, "current RDO B0=0x%x, B1=0x%x, B2=0x%x, B3=0x%x\n",
			rdo[0], rdo[1], rdo[2], rdo[3]);

			cmd = SYS_RDY_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, SYS_RDY_SENT);
		}
		break;
	case SYS_RDY_SENT:
		if (bm92t_is_success(alert_data, status1_data)) {
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			msleep(550); /* reguired? */
			schedule_delayed_work(&info->power_work,
				msecs_to_jiffies(2000));
			cmd = DR_SWAP_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, DR_SWAP_SENT);
		}
		break;
	case DR_SWAP_SENT:
		if (bm92t_is_success(alert_data, status1_data) &&
			((status1_data & 0xff) == 0x80)) {
			bm92t_send_vdm(info, vdm_id_phase1_msg,
				sizeof(vdm_id_phase1_msg));
			bm92t_state_machine(info, VDM_ID_PHASE1_SENT);
		}
		break;
	case VDM_ID_PHASE1_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_PHASE1_SENT);
		} else if (alert_data & ALERT_CMD_DONE)
			dev_dbg(dev, "cmd done in VDM_ID_PHASE1_SENT\n");
		break;
	case VDM_ACCEPT_PHASE1_SENT:
		if (alert_data & ALERT_CMD_DONE) {
			/* check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM,
			vdm, sizeof(vdm));

			if (!(vdm[5] == 0x7e && vdm[6] == 0x05 &&
				vdm[15] == 0x03 && vdm[16] == 0x20)) {
				dev_dbg(dev, "unexpected VDM\n");
				goto ret;
			}
			bm92t_send_vdm(info, vdm_id_phase2_msg,
				sizeof(vdm_id_phase2_msg));
			bm92t_state_machine(info, VDM_ID_PHASE2_SENT);
		}
		break;
	case VDM_ID_PHASE2_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_PHASE2_SENT);
		} else if (alert_data & ALERT_CMD_DONE)
			dev_dbg(dev, "cmd done in VDM_ID_PHASE2_SENT\n");
		break;
	case VDM_ACCEPT_PHASE2_SENT:
		if (alert_data & ALERT_CMD_DONE) {
			/* check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM,
					vdm, 28);
			if (!(vdm[1] == 0x44 && vdm[2] == 0x81 &&
				vdm[3] == 0x7e && vdm[4] == 0x05)) {
				dev_dbg(dev, "unexpected VDM\n");
				goto ret;
			}
			msleep(100);
			cmd = DP_MODE_CMD;
			err = bm92t_send_cmd(info, &cmd);
			msleep(100); /* WAR: may not need to wait */
			bm92t_state_machine(info, ENTER_DP_MODE);
		}
		break;
	case ENTER_DP_MODE:
		if (bm92t_is_success(alert_data, status1_data)) {
			err = bm92t_handle_hpd(info);
			if (!err)
				bm92t_state_machine(info, HPD_HANDLED);
			else
				bm92t_state_machine(info, INIT_STATE);
		}
		break;
	case HPD_HANDLED:
		if (bm92t_is_success(alert_data, status1_data) &&
			((status1_data & 0xff) == 0x80)) {
			bm92t_send_vdm(info, vdm_query_device_msg,
				sizeof(vdm_query_device_msg));
			bm92t_state_machine(info, VDM_QUERY_DEVICE_SENT);
		}
		break;
	case VDM_QUERY_DEVICE_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_QUERY_DEVICE_SENT);
		} else if (alert_data & ALERT_CMD_DONE)
			dev_dbg(dev, "cmd done in VDM_QUERY_DEVICE_SENT\n");
		break;
	case VDM_ACCEPT_QUERY_DEVICE_SENT:
		if (alert_data & ALERT_CMD_DONE) {
			/* check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM,
			vdm, sizeof(vdm));
			pr_info("device state = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
				vdm[9], vdm[10], vdm[11], vdm[12]);

			if (vdm[11] & 0x02) {
				bm92t_extcon_cable_update(info,
					EXTCON_USB_HOST, false);
				msleep(500);
				bm92t_extcon_cable_update(info,
					EXTCON_USB, true);
			}

			bm92t_send_vdm(info, vdm_check_usbhub_msg,
				sizeof(vdm_check_usbhub_msg));
			bm92t_state_machine(info, VDM_CHECK_USBHUB_SENT);
		}
		break;
	case VDM_CHECK_USBHUB_SENT:
		if (alert_data & ALERT_VDM_RECEIVED) {
			cmd = ACCEPT_VDM_CMD;
			err = bm92t_send_cmd(info, &cmd);
			bm92t_state_machine(info, VDM_ACCEPT_CHECK_USBHUB_SENT);
		} else if (alert_data & ALERT_CMD_DONE)
			dev_dbg(dev, "cmd done in VDM_CHECK_USBHUB_SENT\n");
		break;
	case VDM_ACCEPT_CHECK_USBHUB_SENT:
		if (alert_data & ALERT_CMD_DONE) {
			/* check incoming VDM */
			err = bm92t_read_reg(info, INCOMING_VDM,
			vdm, sizeof(vdm));

			if (vdm[5] & 0x1) {
				pr_info("Sending vdm again.\n");
				bm92t_send_vdm(info, vdm_check_usbhub_msg,
					sizeof(vdm_check_usbhub_msg));
				bm92t_state_machine(info,
					VDM_CHECK_USBHUB_SENT);
			}
		}
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

#ifdef CONFIG_DEBUG_FS
static int bm92t_regs_show(struct seq_file *s, void *data)
{
	int err;
	unsigned short reg_value;
	unsigned char reg_addr;
	struct bm92t_info *info = (struct bm92t_info *) (s->private);

	reg_addr = STATUS1;
	err = bm92t_read_reg(info, reg_addr,
			     (unsigned char *) &reg_value,
			     sizeof(reg_value));
	if (!err)
		seq_printf(s, "0x%02x: 0x%04x\n", reg_addr, reg_value);
	else {
		dev_err(&info->i2c_client->dev,
			"debugfs cannot read 0x%02x", reg_addr);
		return err;
	}

	reg_addr = STATUS2;
	err = bm92t_read_reg(info, reg_addr,
			     (unsigned char *) &reg_value,
			     sizeof(reg_value));
	if (!err)
		seq_printf(s, "0x%02x: 0x%04x\n", reg_addr, reg_value);
	else {
		dev_err(&info->i2c_client->dev,
			"debugfs cannot read 0x%02x", reg_addr);
		return err;
	}

	reg_addr = DP_STATUS;
	err = bm92t_read_reg(info, reg_addr,
			     (unsigned char *) &reg_value,
			     sizeof(reg_value));
	if (!err)
		seq_printf(s, "0x%02x: 0x%04x\n", reg_addr, reg_value);
	else {
		dev_err(&info->i2c_client->dev,
			"debugfs cannot read 0x%02x", reg_addr);
		return err;
	}

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

static struct bm92t_platform_data *bm92t_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct bm92t_platform_data *pdata;
	int num;

	dev_info(&client->dev, "%s: %s\n", __func__, np->full_name);
	num = sizeof(*pdata);
	pdata = devm_kzalloc(&client->dev, num, GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	if (client->dev.platform_data)
		memcpy(pdata, client->dev.platform_data, sizeof(*pdata));
	else
		memcpy(pdata, &bm92t_dflt_pdata, sizeof(*pdata));

	pdata->disable_power_nego = of_property_read_bool(np,
						"disable-power-nego");

	return pdata;
}

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
	if (client->dev.of_node) {
		info->pdata = bm92t_parse_dt(client);
		if (IS_ERR(info->pdata)) {
			err = PTR_ERR(info->pdata);
			dev_err(&client->dev,
				"Failed to parse OF node: %d\n", err);
			return err;
		}
	} else if (client->dev.platform_data)
		info->pdata = client->dev.platform_data;
	else {
		info->pdata  = &bm92t_dflt_pdata;
		dev_info(&client->dev,
			 "%s No platform data. Using defaults.\n",
			 __func__);
	}

	/* initialized state */
	info->state = INIT_STATE;

	/* register extcon */
	info->edev.supported_cable = bm92t_extcon_cable;
	info->edev.name = "bm92t_extcon";
	info->edev.dev.parent = &client->dev;
	err = extcon_dev_register(&info->edev);
	if (err < 0) {
		dev_err(&client->dev, "Cannot register extcon device\n");
		return err;
	}

	/* create workqueue */
	info->event_wq = create_singlethread_workqueue("bm92t-event-queue");
	if (!info->event_wq) {
		dev_err(&client->dev, "Cannot create work queue\n");
		return -ENOMEM;
	}

	err = bm92t_read_reg(info, FW_TYPE,
			 (unsigned char *) &reg_value, sizeof(reg_value));
	info->fw_type = reg_value;

	err = bm92t_read_reg(info, FW_REVISION,
			 (unsigned char *) &reg_value, sizeof(reg_value));
	info->fw_revision = reg_value;

	dev_info(&info->i2c_client->dev, "fw_type: 0x%02x, fw_revision: 0x%02x\n",
		info->fw_type, info->fw_revision);

	if (info->fw_revision <= 0x644) {
		INIT_WORK(&info->work, bm92t_event_handler);
		INIT_DELAYED_WORK(&info->oneshot_work,
			bm92t_extcon_cable_set_init_state);
	} else {
		INIT_WORK(&info->work, bm92t_event_handler2);
		INIT_DELAYED_WORK(&info->oneshot_work,
			bm92t_extcon_cable_set_init_state2);
	}

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
	info->batt_chg_reg = devm_regulator_get(&client->dev, "pd_bat_chg");
	if (IS_ERR(info->batt_chg_reg)) {
		err = PTR_ERR(info->batt_chg_reg);
		dev_info(&client->dev,
				"pd_bat_chg reg not registered: %d\n", err);
		info->batt_chg_reg = NULL;
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
