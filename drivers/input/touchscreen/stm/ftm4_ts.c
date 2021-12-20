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
********************************************************************************
* REVISON HISTORY
* DATE		| DESCRIPTION
* 03/09/2012| First Release
* 08/11/2012| Code migration
* 09/04/2013| Support Blob Information
* 09/04/2013| Support Blob Information
* 10/19/2019| Port to Icosa platform
* 02/06/2020| Adjust x,y based on actual touch panel margins
* 02/16/2020| Enable Suspend/Resume routines
* 02/17/2020| Clean various hacks and reinstate proper Suspend/Resume routines
* 06/21/2020| Auto tune on boot/resume and set sense mode
* 08/23/2020| Correct pressure value
* 08/27/2020| Add tool orientation support
* 10/19/2020| Power off in suspend
* 02/20/2021| Complete refactor
*           | - Clean and remove anything unneeded or not supported
*           | - Name properly all stm/vendor fts commands and values
*           | - Fix all boot/suspend/resume routines
*           | - Proper checks on power management
*           | - Allow more device tree based customization instead of ifdefs
*           | - Speed up boot and resume by 1-2s
*           | - Fix palm handling and raise pressure detection to 500 from 254
*           | - Fix issues with delayed resume
*           | - Fix issues with hanging on suspend
*           | - Fix issues/hangs because of incorrect interrupt management
*           | - Ensure power off on suspend for saving power
*           | - Correct logging to debug level instead of info/error
* 02/26/2021| Better palm removal detection
* 12/20/2021| Allow calibration on init disable via device tree or sysfs
*******************************************************************************/

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include <linux/input/mt.h>
#include "ftm4_ts.h"

static struct i2c_driver fts_i2c_driver;

static void fts_input_open_work(struct work_struct *work);
static int  fts_input_open(struct input_dev *dev);
static void fts_input_close(struct input_dev *dev);
static void fts_release_all_finger(struct fts_ts_info *info);

int fts_write_reg(struct fts_ts_info *info, unsigned char *reg, unsigned short num_com)
{
	struct i2c_msg xfer_msg[2];
	int ret = 0;

	if (info->power_state == FTS_POWER_STATE_POWERDOWN) {
		tsp_debug_err(&info->client->dev, "%s: Sensor stopped\n", __func__);
		goto exit;
	}

	mutex_lock(&info->i2c_mutex);

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = num_com;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	ret = i2c_transfer(info->client->adapter, xfer_msg, 1);

	mutex_unlock(&info->i2c_mutex);
	return ret;

 exit:
	return -EIO;
}

int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum, unsigned char *buf, int num)
{
	struct i2c_msg xfer_msg[2];
	int ret = 0;

	if (info->power_state == FTS_POWER_STATE_POWERDOWN) {
		tsp_debug_err(&info->client->dev, "%s: Sensor stopped\n", __func__);
		goto exit;
	}

	mutex_lock(&info->i2c_mutex);

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = info->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	ret = i2c_transfer(info->client->adapter, xfer_msg, 2);

	mutex_unlock(&info->i2c_mutex);

	return ret;

 exit:
	return -EIO;
}

void fts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

void fts_command(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd = 0;
	int ret = 0;

	regAdd = cmd;
	ret = fts_write_reg(info, &regAdd, 1);
	tsp_debug_dbg(&info->client->dev, "FTS Command (%02X) , ret = %d\n", cmd, ret);
}

int fts_systemreset(struct fts_ts_info *info)
{
	int ret = 0;
	unsigned char addr[4] = {FTS_CMD_WRITE_REG, 0x00, 0x28, 0x80};

	tsp_debug_info(&info->client->dev, "FTS SystemReset\n");
	ret = fts_write_reg(info, &addr[0], 4);
	fts_delay(10);

	return ret;
}

static void fts_interrupt_set(struct fts_ts_info *info, int enable)
{
	unsigned char regAdd[4] = {FTS_CMD_WRITE_REG, 0x00, 0x2C, INT_ENABLE};

	if (enable == INT_ENABLE) {
		tsp_debug_dbg(&info->client->dev, "FTS INT Enable\n");
	} else {
		regAdd[3] = INT_DISABLE;
		tsp_debug_dbg(&info->client->dev, "FTS INT Disable\n");
	}

	fts_write_reg(info, &regAdd[0], 4);
}

static void fts_switch_sense_mode(struct fts_ts_info *info, int mode)
{
	unsigned char reg[2] = {FTS_CMD_SWITCH_SENSE_MODE, 0};

	/* Set sense mode. */
	reg[1] = mode;
	fts_write_reg(info, &reg[0], 2);
	fts_delay(10);
}

int fts_read_chip_id(struct fts_ts_info *info)
{
	unsigned char regAdd[3] = {FTS_CMD_WRITE_REG, 0x00, 0x04};
	unsigned char val[7] = {0};
	int ret = 0;

	ret = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 7);
	if (ret < 0) {
		tsp_debug_err(&info->client->dev, "%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	tsp_debug_err(&info->client->dev,
		"FTS %02X%02X%02X =  %02X %02X %02X %02X %02X %02X\n",
		regAdd[0], regAdd[1], regAdd[2],
		val[1], val[2], val[3], val[4],
		val[5], val[6]);

	if ((val[1] == FTS_ID0) && (val[2] == FTS_ID1)) {
		if ((val[5] == 0x00) && (val[6] == 0x00)) {
			tsp_debug_err(&info->client->dev,
				"[fts_read_chip_id] Error - No FW : %02X %02X\n",
				val[5], val[6]);
			info->flash_corruption_info.fw_broken = true;
		}  else {
			tsp_debug_info(&info->client->dev, "FTS Chip ID : %02X %02X\n", val[1], val[2]);
		}
	} else
		return -FTS_ERROR_INVALID_CHIP_ID;

	return ret;
}

int fts_wait_for_ready(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char addr;
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;
	int err_cnt = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	addr = FTS_READ_ONE_EVENT;

	while (fts_read_reg(info, &addr, 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if (data[0] == EVENTID_CONTROLLER_READY) {
			rc = 0;
			break;
		}

		if (data[0] == EVENTID_ERROR) {
			if (data[1] == EVENTID_ERROR_FLASH_CORRUPTION) {
				rc = -FTS_ERROR_EVENT_ID;

				tsp_debug_err(&info->client->dev,
					"%s: flash corruption:%02X,%02X,%02X\n",
					__func__, data[0],
					data[1], data[2]);

				switch (data[2]) {
				case EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_1:
					info->flash_corruption_info.cfg_broken = true;
					break;
				case EVENTID_ERROR_CONFIG_FLASH_CORRUPTION_2:
					info->flash_corruption_info.cfg_broken = true;
					break;
				case EVENTID_ERROR_CX_FLASH_CORRUPTION:
					info->flash_corruption_info.cx_broken = true;
					break;
				default:
					break;
				}
			}

			if (err_cnt++ > 32) {
				rc = -FTS_ERROR_EVENT_ID;
				break;
			}
			continue;
		}

		if (retry++ > FTS_RETRY_COUNT) {
			rc = -FTS_ERROR_TIMEOUT;
			tsp_debug_err(&info->client->dev, "%s: Time Over\n", __func__);

			break;
		}
		fts_delay(20);
	}

	tsp_debug_dbg(&info->client->dev,
		"%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
		__func__, data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7]);

	return rc;
}

int fts_get_channel_info(struct fts_ts_info *info)
{
	int rc = 0;
	unsigned char cmd[4] = { 0xB2, 0x00, 0x14, 0x02 };
	unsigned char data[FTS_EVENT_SIZE];
	int retry = 0;

	memset(data, 0x0, FTS_EVENT_SIZE);

	fts_write_reg(info, &cmd[0], 4);
	cmd[0] = FTS_READ_ONE_EVENT;
	while (fts_read_reg(info, &cmd[0], 1, (unsigned char *)data, FTS_EVENT_SIZE)) {
		if (data[0] == EVENTID_RESULT_READ_REGISTER) {
			if ((data[1] == cmd[1]) && (data[2] == cmd[2])) {
				tsp_debug_err(&info->client->dev, "FTS Channel Sense: %d, Force: %d\n",
					data[3], data[4]);
				rc = 0;
				break;
			}
		}
		if (retry++ > 30) {
			rc = -1;
			tsp_debug_err(&info->client->dev, "Time over - wait for channel info\n");
			break;
		}
		fts_delay(5);
	}
	return rc;
}

int fts_cmd_completion_check(struct fts_ts_info *info, uint8_t event1, uint8_t event2, uint8_t event3)
{
	unsigned char val[8];
	unsigned char reg[2] = {FTS_READ_ONE_EVENT, 0};
	int retry = 200;
	int rc = 0;

	while (retry--) {
		fts_delay(10);
		fts_read_reg(info, &reg[0], 1, &val[0], FTS_EVENT_SIZE);
		if ((val[0] == event1) && (val[1] == event2) && (val[2] == event3)) {
			tsp_debug_dbg(&info->client->dev,
				"[fts_cmd_completion_check] OK [%02x][%02x][%02x]\n", val[0], val[1], val[2]);
			return rc;
		} else if (val[0] == 0x0F) {
			tsp_debug_err(&info->client->dev,
				"[fts_cmd_completion_check] Error - [%02x][%02x][%02x]\n", val[0], val[1], val[2]);
		}
	}

	rc = -1;
	if (retry <= 0)
		tsp_debug_err(&info->client->dev,
			"[fts_cmd_completion_check] Error - Time Over [%02x][%02x][%02x]\n", event1, event2, event3);
	return rc;
}

static void fts_auto_calibration(struct fts_ts_info *info)
{
	/* Trim low power oscillator */
	fts_command(info, FTS_CMD_TRIM_LOW_POWER_OSCILLATOR);
	msleep(200);

	/* Execute MS CX auto calibration */
	fts_command(info, FTS_CMD_MS_CX_TUNING);
	fts_cmd_completion_check(info, EVENTID_STATUS_EVENT, STATUS_EVENT_MS_CX_TUNING_DONE, 0);

	/* Execute SS CX auto calibration */
	fts_command(info, FTS_CMD_SS_CX_TUNING);
	fts_cmd_completion_check(info, EVENTID_STATUS_EVENT, STATUS_EVENT_SS_CX_TUNING_DONE, 0);
}

static int fts_init(struct fts_ts_info *info)
{
	unsigned char val[16];
	unsigned char regAdd[8];
	int rc = 0;

	msleep(20);
	fts_systemreset(info);

	rc = fts_wait_for_ready(info);
	if (rc == -FTS_ERROR_EVENT_ID)
		tsp_debug_err(&info->client->dev, "%s: Failed to fts_wait_for_ready\n", __func__);

	rc = fts_read_chip_id(info);
	if (rc < 0)
		tsp_debug_err(&info->client->dev, "%s: Failed to fts_read_chip_id\n", __func__);

	/* Calibrate touch panel */
	if (!info->board->disable_tuning)
		fts_auto_calibration(info);

	/* Enable finger sense mode */
	fts_switch_sense_mode(info, FTS_FINGER_MODE);

	/* Get Channel Sense and Force levels */
	fts_get_channel_info(info);

	fts_command(info, FTS_CMD_SENSEON);
	fts_command(info, FTS_CMD_FLUSHBUFFER);

	/* fts driver set functional feature */
	info->touch_count = 0;
	info->palm_pressed = false;

	memset(val, 0x0, 4);
	memset(regAdd, 0x0, 8);
	regAdd[0] = FTS_READ_STATUS;
	fts_read_reg(info, regAdd, 1, (unsigned char *)val, 4);
	tsp_debug_dbg(&info->client->dev, "FTS ReadStatus(0x84) : %02X %02X %02X %02X\n",
		val[0], val[1], val[2], val[3]);

	tsp_debug_info(&info->client->dev, "FTS Initialized\n");

	fts_interrupt_set(info, INT_ENABLE);

	return 0;
}

static void fts_debug_msg_event_handler(struct fts_ts_info *info,
				      unsigned char data[])
{
	tsp_debug_err(&info->client->dev,
		"%s: %02X %02X %02X %02X "
		"%02X %02X %02X %02X\n", __func__,
		data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7]);
}

/**
  * Adjust coordinates based on actual hw margins.
  */
static void fts_coordinates_factor(struct fts_ts_info *info, int *x, int *y)
{
	unsigned int x_work;
	unsigned int y_work;
	unsigned int x_adj;
	unsigned int y_adj;

	x_work = *x;
	y_work = *y;

	/* Ensure minimum value */
	x_work = max((unsigned int) x_work, (unsigned int) info->board->x_axis_edge_offset);
	y_work = max((unsigned int) y_work, (unsigned int) info->board->y_axis_edge_offset);

	/* Ensure maximum value */
	x_work = min((unsigned int) x_work, (unsigned int) info->board->x_axis_real_max);
	y_work = min((unsigned int) y_work, (unsigned int) info->board->y_axis_real_max);

	/* Adjust with edge offset */
	x_work -= info->board->x_axis_edge_offset;
	y_work -= info->board->y_axis_edge_offset;

	/* Calculate coordinates factor */
	x_adj = (info->board->max_x * 1000) /
		(info->board->x_axis_real_max - info->board->x_axis_edge_offset);
	y_adj = (info->board->max_y * 1000) /
		(info->board->y_axis_real_max - info->board->y_axis_edge_offset);

	/* Calculate the adjusted coordinates */
	x_work = x_work * x_adj / 1000;
	y_work = y_work * y_adj / 1000;

	*x = x_work;
	*y = y_work;
}

static unsigned char fts_event_handler_type_b(struct fts_ts_info *info,
					      unsigned char data[],
					      unsigned char LeftEvent)
{
	unsigned char EventNum = 0;
	unsigned char NumTouches = 0;
	unsigned char TouchID = 0, EventID = 0, status = 0;
	unsigned char LastLeftEvent = 0;
	unsigned tmp = 0;
	int x = 0, y = 0, z = 0;
	int palm = 0, orient = 0;

	for (EventNum = 0; EventNum < LeftEvent; EventNum++) {
#ifdef DEBUG
		tsp_debug_dbg(&info->client->dev,
			"%d %2x %2x %2x %2x %2x %2x %2x %2x\n",
			EventNum,
			data[EventNum * FTS_EVENT_SIZE],
			data[EventNum * FTS_EVENT_SIZE+1],
			data[EventNum * FTS_EVENT_SIZE+2],
			data[EventNum * FTS_EVENT_SIZE+3],
			data[EventNum * FTS_EVENT_SIZE+4],
			data[EventNum * FTS_EVENT_SIZE+5],
			data[EventNum * FTS_EVENT_SIZE+6],
			data[EventNum * FTS_EVENT_SIZE+7]);
		tsp_debug_dbg(&info->client->dev, "power_state (%d)\n", info->power_state );
#endif

		EventID = data[EventNum * FTS_EVENT_SIZE] & 0x0F;

		if ((EventID >= 3) && (EventID <= 5)) {
			LastLeftEvent = 0;
			NumTouches = 1;
			TouchID = (data[EventNum * FTS_EVENT_SIZE] >> 4) & 0x0F;
		} else {
			LastLeftEvent = data[7 + EventNum * FTS_EVENT_SIZE] & 0x0F;
			NumTouches = (data[1 + EventNum * FTS_EVENT_SIZE] & 0xF0) >> 4;
			TouchID = data[1 + EventNum * FTS_EVENT_SIZE] & 0x0F;
			EventID = data[EventNum * FTS_EVENT_SIZE] & 0xFF;
			status = data[1 + EventNum * FTS_EVENT_SIZE] & 0xFF;
		}

		switch (EventID) {
		case EVENTID_NO_EVENT:
			break;

		case EVENTID_ERROR:
			if (data[1 + EventNum * FTS_EVENT_SIZE] == 0x08) {
				/* Get Auto tune fail event */
				if (data[2 + EventNum * FTS_EVENT_SIZE] == 0x00) {
					tsp_debug_err(&info->client->dev, "[FTS] Fail Mutual Auto tune\n");
				} else if (data[2 + EventNum * FTS_EVENT_SIZE] == 0x01) {
					tsp_debug_err(&info->client->dev, "[FTS] Fail Self Auto tune\n");
				}
			} else if (data[1 + EventNum * FTS_EVENT_SIZE] == 0x09)
				/* Get detect SYNC fail event */
				tsp_debug_err(&info->client->dev, "[FTS] Fail detect SYNC\n");
			break;

		case EVENTID_HOVER_ENTER_POINTER:
		case EVENTID_HOVER_MOTION_POINTER:
			break;

		case EVENTID_HOVER_LEAVE_POINTER:
			input_mt_slot(info->input_dev, 0);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
			break;

		case EVENTID_ENTER_POINTER:
			info->touch_count++;

		case EVENTID_MOTION_POINTER:
			if (info->touch_count == 0) {
				tsp_debug_dbg(&info->client->dev, "%s: count 0\n", __func__);
				fts_release_all_finger(info);
				break;
			}

			if ((EventID == EVENTID_MOTION_POINTER) &&
				(info->finger[TouchID].state == EVENTID_LEAVE_POINTER)) {
				tsp_debug_dbg(&info->client->dev, "%s: state leave but point is moved.\n", __func__);
				break;
			}

			if (info->palm_pressed)
				break;

			/* Decode X/Y coordinates */
			x = ((data[1 + EventNum * FTS_EVENT_SIZE] & 0xFF) << 4) +
				((data[3 + EventNum * FTS_EVENT_SIZE] & 0xF0) >> 4);
			y = ((data[2 + EventNum * FTS_EVENT_SIZE] & 0xFF) << 4) +
				(data[3 + EventNum * FTS_EVENT_SIZE] & 0xF);

			/* Decode pressure base */
			z = data[4 + EventNum * FTS_EVENT_SIZE] |
				(data[5 + EventNum * FTS_EVENT_SIZE] << 8);
			z <<= 6;

			/* Factor pressure gain */
			tmp = 0x40;
			if ((data[6 + EventNum * FTS_EVENT_SIZE] & 0x3F) != 1 &&
				(data[6 + EventNum * FTS_EVENT_SIZE] & 0x3F) != 0x3F)
			{
				tmp = data[6 + EventNum * FTS_EVENT_SIZE] & 0x3F;
			}
			z /= tmp + 0x40;

			/* Palm rejection */
			if (z > PRESSURE_MAX) {
				tsp_debug_err(&info->client->dev, "Palm Detected\n");
				tsp_debug_dbg(&info->client->dev,
					"%s: [ID:%2d  X:%4d  Y:%4d  Z:%4d  Orient:%2d  tc:%2d]\n", __func__,
					TouchID, x, y, z, orient, info->touch_count);

				fts_release_all_finger(info);

				info->palm_touch_id = TouchID;
				info->palm_pressed = true;

				break;
			}

			/* Compensate edges */
			if (info->board->coord_factor)
				fts_coordinates_factor(info, &x, &y);

			/* Set orientation */
			tmp = ((data[7 + EventNum * FTS_EVENT_SIZE] & 0x80) >> 5) |
				(data[6 + EventNum * FTS_EVENT_SIZE] >> 6);
			switch (tmp) {
			case 0:
				orient =  0;
				break;
			case 1:
				orient =  45000;
				break;
			case 2:
				orient =  90000;
				break;
			case 3:
				orient = -45000;
				break;
			case 4:
				orient =  22500;
				break;
			case 5:
				orient =  67500;
				break;
			case 6:
				orient = -22500;
				break;
			case 7:
				orient = -67500;
				break;
			default:
				break;
			}

			input_mt_slot(info->input_dev, TouchID);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1 + (palm << 1));

			input_report_key(info->input_dev, BTN_TOUCH, 1);
			input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);

			input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);

			input_report_abs(info->input_dev, ABS_MT_ORIENTATION, orient);

			info->finger[TouchID].lx = x;
			info->finger[TouchID].ly = y;
			break;

		case EVENTID_LEAVE_POINTER:
			if (info->palm_pressed) {
				if (info->palm_touch_id == TouchID) {
					tsp_debug_err(&info->client->dev, "Palm Released\n");
					fts_release_all_finger(info);
					info->palm_pressed = false;
				}
				break;
			}

			if (info->touch_count <= 0) {
				tsp_debug_dbg(&info->client->dev, "%s: count 0\n", __func__);
				fts_release_all_finger(info);
				break;
			}

			info->touch_count--;

			input_mt_slot(info->input_dev, TouchID);

			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

			if (info->touch_count == 0) {
				/* Clear BTN_TOUCH when All touch are released  */
				input_report_key(info->input_dev, BTN_TOUCH, 0);
				input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

			}
			break;

		case EVENTID_STATUS_EVENT:
			if (status == STATUS_EVENT_GLOVE_MODE) {

			} else if (status == STATUS_EVENT_RAW_DATA_READY) {
				unsigned char regAdd[4] = {0xB0, 0x01, 0x29, 0x01};

				fts_write_reg(info, &regAdd[0], 4);

				tsp_debug_dbg(&info->client->dev, "[FTS] Received the Raw Data Ready Event\n");
			} else if (status == STATUS_EVENT_FORCE_CAL_MUTUAL) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Force Calibration Mutual only Event\n");
			} else if (status == STATUS_EVENT_FORCE_CAL_SELF) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Force Calibration Self only Event\n");
			} else if (status == STATUS_EVENT_WATERMODE_ON) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Water Mode On Event\n");
			} else if (status == STATUS_EVENT_WATERMODE_OFF) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Water Mode Off Event\n");
			} else if (status == STATUS_EVENT_MUTUAL_CAL_FRAME_CHECK) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Mutual Calib Frame Check Event\n");
			} else if (status == STATUS_EVENT_SELF_CAL_FRAME_CHECK) {
				tsp_debug_dbg(&info->client->dev, "[FTS] Received Self Calib Frame Check Event\n");
			} else {
				fts_debug_msg_event_handler(info, &data[EventNum * FTS_EVENT_SIZE]);
			}
			break;

		case EVENTID_RESULT_READ_REGISTER:
		default:
			fts_debug_msg_event_handler(info, &data[EventNum * FTS_EVENT_SIZE]);
			continue;
		}

		if (EventID == EVENTID_ENTER_POINTER) {
			tsp_debug_event(&info->client->dev,
				"[P] tID:%d x:%d y:%d z:%d p:%d tc:%d tm:%d\n",
				TouchID, x, y, z, palm, info->touch_count);
		} else if (EventID == EVENTID_HOVER_ENTER_POINTER) {
			tsp_debug_event(&info->client->dev,
				"[HP] tID:%d x:%d y:%d z:%d\n",
				TouchID, x, y, z);
		} else if (EventID == EVENTID_LEAVE_POINTER) {
			tsp_debug_event(&info->client->dev,
				"[R] tID:%d mc: %d tc:%d lx: %d ly: %d\n",
				TouchID, info->finger[TouchID].mcount, info->touch_count,
				info->finger[TouchID].lx,
				info->finger[TouchID].ly);

			info->finger[TouchID].mcount = 0;
		} else if (EventID == EVENTID_HOVER_LEAVE_POINTER) {
			tsp_debug_event(&info->client->dev, "[HR] tID:%d\n", TouchID);

			info->finger[TouchID].mcount = 0;
		} else if (EventID == EVENTID_MOTION_POINTER) {
			info->finger[TouchID].mcount++;
		}

		if ((EventID == EVENTID_ENTER_POINTER) ||
			(EventID == EVENTID_MOTION_POINTER) ||
			(EventID == EVENTID_LEAVE_POINTER))
			info->finger[TouchID].state = EventID;
	}

	input_sync(info->input_dev);

	return LastLeftEvent;
}

/**
 * fts_interrupt_handler()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;
	unsigned char regAdd[4] = {FTS_CMD_WRITE_REG, 0x00, 0x23, FTS_READ_ALL_EVENT};
	unsigned short evtcount = 0;

	evtcount = 0;

	fts_read_reg(info, &regAdd[0], 3, (unsigned char *)&evtcount, 2);
	evtcount = evtcount >> 8;
	evtcount = evtcount / 2;

	if (evtcount > FTS_FIFO_MAX)
		evtcount = FTS_FIFO_MAX;

	if (evtcount > 0) {
		memset(info->data, 0x0, FTS_EVENT_SIZE * evtcount);
		fts_read_reg(info, &regAdd[3], 1, (unsigned char *)info->data, FTS_EVENT_SIZE * evtcount);
		fts_event_handler_type_b(info, info->data, evtcount);
	}
	return IRQ_HANDLED;
}

static void fts_irq_enable(struct fts_ts_info *info, bool enable)
{
	spin_lock(&info->lock);

	if (enable) {
		if (atomic_cmpxchg(&info->irq_enabled, 0, 1) == 0) {
			tsp_debug_dbg(info->dev, "enable_irq\n");
			enable_irq(info->irq);
		}
	} else {
		if (atomic_cmpxchg(&info->irq_enabled, 1, 0) == 1) {
			tsp_debug_dbg(info->dev, "disable_irq\n");
			disable_irq_nosync(info->irq);
		}
	}

	spin_unlock(&info->lock);
}

#ifdef CONFIG_OF
static int fts_power_ctrl(void *data, bool on)
{
	struct fts_ts_info *info = (struct fts_ts_info *)data;
	const struct fts_i2c_platform_data *pdata = info->board;
	struct device *dev = &info->client->dev;
	struct regulator *regulator_dvdd = NULL;
	struct regulator *regulator_avdd = NULL;
	static bool enabled = false;
	int retval = 0;

	if (enabled == on)
		return retval;

	/* touch power init */
	if (gpio_is_valid(pdata->vdd_gpio)) {
		gpio_request(pdata->vdd_gpio, "touch-vdd");
	} else {
		regulator_avdd = regulator_get(dev, pdata->regulator_avdd);
		if (IS_ERR_OR_NULL(regulator_avdd)) {
			tsp_debug_err(dev, "%s: Failed to get %s regulator.\n", __func__, pdata->regulator_avdd);
			goto out;
		}
	}
	if (gpio_is_valid(pdata->vio_gpio)) {
		gpio_request(pdata->vio_gpio, "touch-vio");
	} else {
		regulator_dvdd = regulator_get(dev, pdata->regulator_dvdd);
		if (IS_ERR_OR_NULL(regulator_dvdd)) {
			tsp_debug_err(dev, "%s: Failed to get %s regulator.\n", __func__, pdata->regulator_dvdd);
			retval = -EPROBE_DEFER;
			goto out;
		}
	}

	tsp_debug_info(dev, "%s: %s\n", __func__, on ? "on" : "off");

	if (on) {
		if (gpio_is_valid(pdata->vdd_gpio)) {
			retval = gpio_direction_output(pdata->vdd_gpio, 1);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable vdd: %d\n", __func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_avdd)) {
			retval = regulator_enable(regulator_avdd);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable avdd: %d\n", __func__, retval);
				goto out;
			}
		}

		if (gpio_is_valid(pdata->vio_gpio)) {
			retval = gpio_direction_output(pdata->vio_gpio, 1);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable vio: %d\n", __func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_dvdd)) {
			retval = regulator_enable(regulator_dvdd);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable dvdd: %d\n", __func__, retval);
				goto out;
			}
		}


		fts_delay(5);
	} else {
		if (gpio_is_valid(pdata->vio_gpio)) {
			retval = gpio_direction_output(pdata->vio_gpio, 0);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable vio: %d\n", __func__, retval);
			}
		} else if (!IS_ERR_OR_NULL(regulator_dvdd)) {
			retval = regulator_disable(regulator_dvdd);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable dvdd: %d\n", __func__, retval);
				goto out;
			}
		}

		if (gpio_is_valid(pdata->vdd_gpio)) {
			retval = gpio_direction_output(pdata->vdd_gpio, 0);
			if (retval)
				tsp_debug_err(dev, "%s: Failed to enable vdd: %d\n", __func__, retval);
		} else if (!IS_ERR_OR_NULL(regulator_avdd)) {
			retval = regulator_disable(regulator_avdd);
			if (retval) {
				tsp_debug_err(dev, "%s: Failed to enable avdd: %d\n", __func__, retval);
				goto out;
			}
		}
	}

	enabled = on;

out:
	regulator_put(regulator_dvdd);
	regulator_put(regulator_avdd);

	return retval;
}

static int fts_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fts_i2c_platform_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	u32 coords[2];
	int retval = 0;

	if (of_property_read_u32(np, "stm,irq_type", &pdata->irq_type)) {
		tsp_debug_err(dev, "Failed to get irq_type property\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "stm,max_coords", coords, 2)) {
		tsp_debug_err(dev, "Failed to get max_coords property\n");
		return -EINVAL;
	}
	pdata->max_x = coords[0];
	pdata->max_y = coords[1];

	pdata->coord_factor = 0;
	if (!of_property_read_u32_array(np, "stm,max-real-coords", coords, 2)) {
		pdata->coord_factor = 1;
		pdata->x_axis_real_max = coords[0];
		pdata->y_axis_real_max = coords[1];

		if (!of_property_read_u32_array(np, "stm,edge-offset", coords, 2)) {
			pdata->x_axis_edge_offset = coords[0];
			pdata->y_axis_edge_offset = coords[1];
		} else {
			pdata->x_axis_edge_offset = 0;
			pdata->y_axis_edge_offset = 0;
		}
	}

	if (of_property_read_string(np, "stm,regulator_dvdd", &pdata->regulator_dvdd))
		tsp_debug_err(dev, "Failed to get regulator_dvdd name property\n");

	if (of_property_read_string(np, "stm,regulator_avdd", &pdata->regulator_avdd))
		tsp_debug_err(dev, "Failed to get regulator_avdd name property\n");

	pdata->vdd_gpio = of_get_named_gpio(np, "stm,vdd-gpio", 0);
	if (gpio_is_valid(pdata->vdd_gpio))
		tsp_debug_err(dev, "vdd_gpio : %s\n", gpio_get_value(pdata->vdd_gpio) ? "On" : "Off");
	else
		tsp_debug_err(dev, "Failed to get vdd_gpio gpio\n");

	pdata->vio_gpio = of_get_named_gpio(np, "stm,vio-gpio", 0);
	if (gpio_is_valid(pdata->vio_gpio))
		tsp_debug_err(dev, "vio_gpio : %s\n", gpio_get_value(pdata->vio_gpio) ? "On" : "Off");
	else
		tsp_debug_err(dev, "Failed to get vio_gpio gpio\n");

	pdata->delayed_open = of_property_read_bool(np, "stm,delayed-open");

	if (of_property_read_u32(np, "stm,delayed-open-time", &pdata->delayed_open_time)) {
		tsp_debug_err(dev, "Failed to get delayed-open-time property\n");
		pdata->delayed_open_time = FTS_INPUT_OPEN_DWORK_TIME;
	}

	pdata->disable_tuning = of_property_read_bool(np, "stm,disable-tuning");

	pdata->power = fts_power_ctrl;

	return retval;
}
#endif

static int fts_setup_drv_data(struct i2c_client *client)
{
	int retval = 0;
	struct fts_i2c_platform_data *pdata;
	struct fts_ts_info *info = NULL;

	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct fts_i2c_platform_data), GFP_KERNEL);

		if (!pdata) {
			tsp_debug_err(&client->dev, "Failed to allocate platform data\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
		retval = fts_parse_dt(client);
		if (retval) {
			tsp_debug_err(&client->dev, "Failed to parse dt\n");
			goto error;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		tsp_debug_err(&client->dev, "No platform data found\n");
			return -EINVAL;
	}
	if (!pdata->power) {
		tsp_debug_err(&client->dev, "No power control found\n");
			retval = -EINVAL;
			goto error;
	}

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		tsp_debug_err(&client->dev, "%s: Failed to alloc mem for info\n", __func__);
		retval = -ENOMEM;
		goto error;
	}

	info->client = client;
	info->board = pdata;
	info->irq = client->irq;
	info->irq_type = info->board->irq_type;
	atomic_set(&info->irq_enabled, 0);

	INIT_DELAYED_WORK(&info->open_work, fts_input_open_work);

	i2c_set_clientdata(client, info);

	return 0;

error:
	kfree(info);
	if (client->dev.of_node) {
		kfree(pdata);
		client->dev.platform_data = NULL;
	}
	return retval;
}

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	int retval = 0;
	struct fts_ts_info *info = NULL;
	static char fts_ts_phys[64] = { 0 };
	int i = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tsp_debug_err(&client->dev, "FTS err = EIO!\n");
		return -EIO;
	}

	/* Build up driver data */
	retval = fts_setup_drv_data(client);
	if (retval < 0) {
		tsp_debug_err(&client->dev, "%s: Failed to set up driver data\n", __func__);
		goto err_setup_drv_data;
	}

	info = (struct fts_ts_info *)i2c_get_clientdata(client);
	if (!info) {
		tsp_debug_err(&client->dev, "%s: Failed to get driver data\n", __func__);
		retval = -ENODEV;
		goto err_get_drv_data;
	}

	if (info->board->power)
		retval = info->board->power(info, true);

	info->power_state = FTS_POWER_STATE_ACTIVE;

	if (retval < 0) {
		retval = -EPROBE_DEFER;
		goto err_get_drv_data;
	}

	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		tsp_debug_err(&info->client->dev, "FTS err = ENOMEM!\n");
		retval = -ENOMEM;
		goto err_input_allocate_device;
	}

	info->input_dev->dev.parent = &client->dev;
	info->input_dev->name = "touchscreen";
	snprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input1", info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = BUS_I2C;
	info->input_dev->open = fts_input_open;
	info->input_dev->close = fts_input_close;

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(BTN_TOUCH, info->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, info->input_dev->keybit);

	input_mt_init_slots(info->input_dev, FINGER_MAX, INPUT_MT_DIRECT);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X, 0, info->board->max_x, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y, 0, info->board->max_y, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_ORIENTATION, -67500, 90000, 0, 0);

	mutex_init(&info->device_mutex);
	mutex_init(&info->i2c_mutex);
	spin_lock_init(&info->lock);

	mutex_lock(&info->device_mutex);
	retval = fts_init(info);
	mutex_unlock(&info->device_mutex);
	if (retval < 0) {
		tsp_debug_err(&info->client->dev, "FTS fts_init fail!\n");
		goto err_fts_init;
	}

	input_set_drvdata(info->input_dev, info);
	i2c_set_clientdata(client, info);

	retval = input_register_device(info->input_dev);
	if (retval) {
		tsp_debug_err(&info->client->dev, "FTS input_register_device fail!\n");
		goto err_register_input;
	}

	for (i = 0; i < FINGER_MAX; i++) {
		info->finger[i].state = EVENTID_LEAVE_POINTER;
		info->finger[i].mcount = 0;
	}

	retval = request_threaded_irq(info->irq, NULL,
		fts_interrupt_handler, info->board->irq_type,
		FTS_TS_DRV_NAME, info);

	if (retval < 0) {
		tsp_debug_err(&info->client->dev, "%s: Failed to enable attention interrupt\n", __func__);
		goto err_enable_irq;
	}
#ifdef CONFIG_PM
	device_enable_async_suspend(&info->client->dev);
#endif

	atomic_set(&info->irq_enabled, 1);

	return 0;

err_enable_irq:
	input_unregister_device(info->input_dev);
	info->input_dev = NULL;

err_register_input:
	if (info->input_dev)
		input_free_device(info->input_dev);

err_fts_init:
	mutex_destroy(&info->device_mutex);
	mutex_destroy(&info->i2c_mutex);

err_input_allocate_device:
	info->board->power(info, false);
	kfree(info);

err_get_drv_data:
err_setup_drv_data:
	return retval;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "FTS removed\n");

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FTS_CMD_FLUSHBUFFER);

	fts_irq_enable(info, false);
	free_irq(info->irq, info);

	input_mt_destroy_slots(info->input_dev);

	input_unregister_device(info->input_dev);
	info->input_dev = NULL;

	info->board->power(info, false);

	kfree(info);

	return 0;
}

static void fts_release_all_finger(struct fts_ts_info *info)
{
	int i;

	for (i = 0; i < FINGER_MAX; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);

		if ((info->finger[i].state == EVENTID_ENTER_POINTER) ||
			(info->finger[i].state == EVENTID_MOTION_POINTER)) {
			info->touch_count--;
			if (info->touch_count < 0)
				info->touch_count = 0;

			tsp_debug_dbg(&info->client->dev,
				"[RA] tID:%d mc: %d tc:%d\n",
				i, info->finger[i].mcount, info->touch_count);
		}

		info->finger[i].state = EVENTID_LEAVE_POINTER;
		info->finger[i].mcount = 0;
	}

	input_report_key(info->input_dev, BTN_TOUCH, 0);
	input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

	input_sync(info->input_dev);
}

static int fts_stop_device(struct fts_ts_info *info)
{
	tsp_debug_err(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->device_mutex);

	if (info->power_state == FTS_POWER_STATE_POWERDOWN) {
		tsp_debug_err(&info->client->dev, "%s already powered off\n", __func__);
		goto out;
	}

	fts_interrupt_set(info, INT_DISABLE);
	fts_irq_enable(info, false);

	fts_command(info, FTS_CMD_SENSEOFF);
	fts_command(info, FTS_CMD_FLUSHBUFFER);
	fts_release_all_finger(info);

	info->power_state = FTS_POWER_STATE_POWERDOWN;

	if (info->board->power)
		info->board->power(info, false);

 out:
	mutex_unlock(&info->device_mutex);

	return 0;
}

static int fts_start_device(struct fts_ts_info *info)
{
	tsp_debug_err(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->device_mutex);

	if (info->power_state == FTS_POWER_STATE_ACTIVE) {
		tsp_debug_err(&info->client->dev, "%s already powered on\n", __func__);
		goto out;
	}

	fts_release_all_finger(info);

	if (info->board->power)
		info->board->power(info, true);

	info->power_state = FTS_POWER_STATE_ACTIVE;

	fts_systemreset(info);
	fts_wait_for_ready(info);

	info->touch_count = 0;
	info->palm_pressed = false;

	/* Calibrate touch panel */
	if (!info->board->disable_tuning)
		fts_auto_calibration(info);

	/* Enable finger sense mode. */
	fts_switch_sense_mode(info, FTS_FINGER_MODE);

	fts_command(info, FTS_CMD_SENSEON);
	fts_command(info, FTS_CMD_FLUSHBUFFER);

	fts_interrupt_set(info, INT_ENABLE);
	fts_irq_enable(info, true);

out:
	mutex_unlock(&info->device_mutex);

	return 0;
}

static void fts_input_open_work(struct work_struct *work)
{
	struct fts_ts_info *info = container_of(work, struct fts_ts_info, open_work.work);

	tsp_debug_dbg(&info->client->dev, "%s\n", __func__);

	if (fts_start_device(info) < 0)
		tsp_debug_err(&info->client->dev, "%s: Failed to start device\n", __func__);
}

static int fts_input_open(struct input_dev *dev)
{
	struct fts_ts_info *info = input_get_drvdata(dev);

	tsp_debug_dbg(&info->client->dev, "%s\n", __func__);

	if (info->board->delayed_open)
		schedule_delayed_work(&info->open_work, msecs_to_jiffies(info->board->delayed_open_time));
	else {
		if (fts_start_device(info) < 0)
			tsp_debug_err(&info->client->dev, "%s: Failed to start device\n", __func__);
	}

	return 0;
}

static void fts_input_close(struct input_dev *dev)
{
	struct fts_ts_info *info = input_get_drvdata(dev);

	tsp_debug_dbg(&info->client->dev, "%s\n", __func__);

	if (info->board->delayed_open)
		cancel_delayed_work(&info->open_work);

	fts_stop_device(info);
}

static void fts_shutdown(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	tsp_debug_info(&info->client->dev, "FTS %s called!\n", __func__);

	if (info->board->delayed_open)
		cancel_delayed_work(&info->open_work);

	fts_stop_device(info);
}

#ifdef CONFIG_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users)
		fts_stop_device(info);

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	tsp_debug_info(&info->client->dev, "%s\n", __func__);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		if (info->board->delayed_open)
			schedule_delayed_work(&info->open_work, msecs_to_jiffies(info->board->delayed_open_time));
		else
			fts_start_device(info);
	}

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}
#endif

static const struct i2c_device_id fts_device_id[] = {
	{FTS_TS_DRV_NAME, 0},
	{}
};

#ifdef CONFIG_PM
static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id fts_match_table[] = {
	{ .compatible = "stm,ftm4_fts",},
	{ },
};
#else
#define fts_match_table NULL
#endif

static struct i2c_driver fts_i2c_driver = {
	.driver = {
		   .name = FTS_TS_DRV_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = fts_match_table,
#endif
#ifdef CONFIG_PM
		   .pm = &fts_dev_pm_ops,
#endif
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   },
	.probe = fts_probe,
	.remove = fts_remove,
	.shutdown = fts_shutdown,
	.id_table = fts_device_id,
};

static int __init fts_driver_init(void)
{
	return i2c_add_driver(&fts_i2c_driver);
}

static void __exit fts_driver_exit(void)
{
	i2c_del_driver(&fts_i2c_driver);
}

MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("STMicroelectronics, Inc.");
MODULE_LICENSE("GPL v2");

module_init(fts_driver_init);
module_exit(fts_driver_exit);
