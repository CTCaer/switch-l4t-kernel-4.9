// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch Joy-Cons and Pro Controllers
 *
 * Copyright (c) 2019 Daniel J. Ogorchock <djogorchock@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * The following resources/projects were referenced for this driver:
 *   https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 *   https://gitlab.com/pjranki/joycon-linux-kernel (Peter Rankin)
 *   https://github.com/FrotBot/SwitchProConLinuxUSB
 *   https://github.com/MTCKC/ProconXInput
 *   hid-wiimote kernel hid driver
 *
 *   TODO: add a real description
 */

#include "hid-ids.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/spinlock.h>

/* Reference the url below for the following HID report defines:
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 */

/* Output Reports */
#define SC_OUTPUT_RUMBLE_AND_SUBCMD	0x01
#define SC_OUTPUT_FW_UPDATE_PKT		0x03
#define	SC_OUTPUT_RUMBLE_ONLY		0x10
#define SC_OUTPUT_MCU_DATA		0x11
#define SC_OUTPUT_USB_CMD		0x80

/* Subcommand IDs */
#define SC_SUBCMD_STATE			0x00
#define SC_SUBCMD_MANUAL_BT_PAIRING	0x01
#define SC_SUBCMD_REQ_DEV_INFO		0x02
#define SC_SUBCMD_SET_REPORT_MODE	0x03
#define SC_SUBCMD_TRIGGERS_ELAPSED	0x04
#define SC_SUBCMD_GET_PAGE_LIST_STATE	0x05
#define SC_SUBCMD_SET_HCI_STATE		0x06
#define SC_SUBCMD_RESET_PAIRING_INFO	0x07
#define SC_SUBCMD_LOW_POWER_MODE	0x08
#define SC_SUBCMD_SPI_FLASH_READ	0x10
#define SC_SUBCMD_SPI_FLASH_WRITE	0x11
#define SC_SUBCMD_RESET_MCU		0x20
#define SC_SUBCMD_SET_MCU_CONFIG	0x21
#define SC_SUBCMD_SET_MCU_STATE		0x22
#define SC_SUBCMD_SET_PLAYER_LIGHTS	0x30
#define SC_SUBCMD_GET_PLAYER_LIGHTS	0x31
#define SC_SUBCMD_SET_HOME_LIGHT	0x38
#define SC_SUBCMD_ENABLE_IMU		0x40
#define SC_SUBCMD_SET_IMU_SENSITIVITY	0x41
#define SC_SUBCMD_WRITE_IMU_REG		0x42
#define SC_SUBCMD_READ_IMU_REG		0x43
#define SC_SUBCMD_ENABLE_VIBRATION	0x48
#define SC_SUBCMD_GET_REGULATED_VOLTAGE	0x50

/* Input Reports */
#define SC_INPUT_BUTTON_EVENT		0x3F
#define SC_INPUT_SUBCMD_REPLY		0x21
#define SC_INPUT_IMU_DATA		0x30
#define SC_INPUT_MCU_DATA		0x31
#define SC_INPUT_USB_RESPONSE		0x81

/* Feature Reports */
#define SC_FEATURE_LAST_SUBCMD		0x02
#define SC_FEATURE_OTA_FW_UPGRADE	0x70
#define SC_FEATURE_SETUP_MEM_READ	0x71
#define SC_FEATURE_MEM_READ		0x72
#define SC_FEATURE_ERASE_MEM_SECTOR	0x73
#define SC_FEATURE_MEM_WRITE		0x74
#define SC_FEATURE_LAUNCH		0x75

/* USB Commands */
#define SC_USB_CMD_CONN_STATUS		0x01
#define SC_USB_CMD_HANDSHAKE		0x02
#define SC_USB_CMD_BAUDRATE_3M		0x03
#define SC_USB_CMD_NO_TIMEOUT		0x04
#define SC_USB_CMD_EN_TIMEOUT		0x05
#define SC_USB_RESET			0x06
#define SC_USB_PRE_HANDSHAKE		0x91
#define SC_USB_SEND_UART		0x92

/* SPI storage addresses of factory calibration data */
#define	SC_CAL_DATA_START		0x603d
#define	SC_CAL_DATA_END			0x604e
#define SC_CAL_DATA_SIZE		(SC_CAL_DATA_END - SC_CAL_DATA_START + 1)

struct switchcon_ctlr;
struct switchcon_input;

/* States for controller state machine */
enum switchcon_ctlr_state {
	SWITCHCON_CTLR_STATE_INIT,
	SWITCHCON_CTLR_STATE_USB_SET_BAUD,
	SWITCHCON_CTLR_STATE_USB_HANDSHAKE,
	SWITCHCON_CTLR_STATE_CALIBRATION,
	SWITCHCON_CTLR_STATE_POST_CALIBRATION,
	SWITCHCON_CTLR_STATE_SEARCH,
	SWITCHCON_CTLR_STATE_READ,
};

/* Function pointers will differ based on controller type */
struct switchcon_impl {
	int (*init)		(struct switchcon_ctlr *);
	void (*deinit)		(struct switchcon_ctlr *);
	int (*handle_event)	(struct switchcon_ctlr *, u8 *, int);
};

struct switchcon_output {
	u8	*data;
	u8	size;
};

struct switchcon_stick_cal {
	u16	x_max;
	u16	x_min;
	u16	x_center;
	u16	y_max;
	u16	y_min;
	u16	y_center;
};

#define SC_OUTPUT_BUF_SIZE	50
/* Each physical controller is associated with a switchcon_ctlr struct */
struct switchcon_ctlr {
	struct hid_device		*hdev;
	struct switchcon_input		*switchcon_in;
	const struct switchcon_impl	*impl;
	bool				is_right_joycon;
	bool				searching;
	enum switchcon_ctlr_state	ctlr_state;
	u8				subcmd_num;
	struct switchcon_output		output_buf[SC_OUTPUT_BUF_SIZE];
	u8				output_head;
	u8				output_tail;
	spinlock_t			output_lock;
	struct work_struct		output_worker;
	struct work_struct		create_input_worker;
	struct work_struct		detect_pair_worker;
	struct work_struct		create_horiz_worker;
	struct mutex			mutex;

	/* factory calibration data */
	struct switchcon_stick_cal	left_stick_cal;
	struct switchcon_stick_cal	right_stick_cal;

	/* buttons/sticks */
	u16	left_stick_x;
	u16	left_stick_y;
	u16	right_stick_x;
	u16	right_stick_y;
	bool	but_y;
	bool	but_x;
	bool	but_b;
	bool	but_a;
	bool	but_sr_right_jc;
	bool	but_sl_right_jc;
	bool	but_r;
	bool	but_zr;
	bool	but_l;
	bool	but_zl;
	bool	but_minus;
	bool	but_plus;
	bool	but_rstick;
	bool	but_lstick;
	bool	but_home;
	bool	but_capture;
	bool	but_down;
	bool	but_up;
	bool	but_right;
	bool	but_left;
	bool	but_sr_left_jc;
	bool	but_sl_left_jc;

};

enum switchcon_input_type {
	SWITCHCON_INPUT_TYPE_PROCON,
	SWITCHCON_INPUT_TYPE_JOYCONS,
	SWITCHCON_INPUT_TYPE_JOYCON_H,
};

static const char *switchcon_input_names[] = {
	"Nintendo Switch Pro Controller",
	"Nintendo Switch Joy-Cons",
	"Nintendo Switch Joy-Con Horizontal",
};

/* Each linux input device is associated with a switchcon_input */
struct switchcon_input {
	enum switchcon_input_type	type;
	struct input_dev	*input;
	struct switchcon_ctlr	*ctlr_left; /* left joycon or pro controller */
	struct switchcon_ctlr	*ctlr_right; /* only used for joycons */
	struct mutex		mutex;
	struct work_struct	input_worker;
};

static int switchcon_ctlr_init(struct switchcon_ctlr *ctlr)
{
	if (ctlr->impl && ctlr->impl->init)
		return ctlr->impl->init(ctlr);
	else
		return -EINVAL;
}

static void switchcon_ctlr_deinit(struct switchcon_ctlr *ctlr)
{
	if (ctlr->impl && ctlr->impl->deinit)
		ctlr->impl->deinit(ctlr);
}

static int switchcon_ctlr_handle_event(struct switchcon_ctlr *ctlr, u8 *data,
								    int size)
{
	if (ctlr->impl && ctlr->impl->handle_event)
		return ctlr->impl->handle_event(ctlr, data, size);
	else
		return -EINVAL;
}

static int switchcon_hid_queue_send(struct switchcon_ctlr *ctlr,
				    const u8 *buffer, size_t count)
{
	struct switchcon_output output;

	output.size = count;
	output.data = kmemdup(buffer, count, GFP_ATOMIC);
	if (!output.data)
		return -ENOMEM;

	spin_lock(&ctlr->output_lock);

	ctlr->output_buf[ctlr->output_head++] = output;
	if (ctlr->output_head == SC_OUTPUT_BUF_SIZE)
		ctlr->output_head = 0;
	schedule_work(&ctlr->output_worker);

	spin_unlock(&ctlr->output_lock);
	return 0;
}

static int switchcon_queue_subcommand(struct switchcon_ctlr *ctlr,
				      const u8 *buffer, size_t count)
{
	/* header including packet number and empty rumble data */
	u8 header[10] = {SC_OUTPUT_RUMBLE_AND_SUBCMD, ctlr->subcmd_num};
	struct switchcon_output output;
	u8 *buf;

	/* the packet number loops after 0xF */
	ctlr->subcmd_num++;
	if (ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;

	buf = kzalloc(sizeof(header) + count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, header, sizeof(header));
	memcpy(buf + sizeof(header), buffer, count);
	output.data = buf;
	output.size = count + sizeof(header);

	spin_lock(&ctlr->output_lock);

	ctlr->output_buf[ctlr->output_head++] = output;
	if (ctlr->output_head == SC_OUTPUT_BUF_SIZE)
		ctlr->output_head = 0;
	schedule_work(&ctlr->output_worker);

	spin_unlock(&ctlr->output_lock);
	return 0;
}

static void switchcon_set_player_leds(struct switchcon_ctlr *ctlr,
				      u8 flash, u8 on)
{
	u8 buffer[] = {SC_SUBCMD_SET_PLAYER_LIGHTS, (flash << 4) | on};

	switchcon_queue_subcommand(ctlr, buffer, sizeof(buffer));
}

static void switchcon_request_calibration(struct switchcon_ctlr *ctlr)
{
	u8 cal_subcmd[]		= {SC_SUBCMD_SPI_FLASH_READ,
				   0xFF & SC_CAL_DATA_START,
				   0xFF & (SC_CAL_DATA_START >> 8),
				   0, 0, SC_CAL_DATA_SIZE};

	switchcon_queue_subcommand(ctlr, cal_subcmd, sizeof(cal_subcmd));
	hid_dbg(ctlr->hdev, "requested cal data\n");
}

static void switchcon_update_input_handler(struct work_struct *ws)
{
	struct switchcon_input *sc_input = container_of(ws,
					struct switchcon_input, input_worker);
	struct input_dev *dev = sc_input->input;
	struct switchcon_ctlr *ctlr_l = sc_input->ctlr_left;
	struct switchcon_ctlr *ctlr_r = sc_input->ctlr_right;
	s32 conversion;

	if (!ctlr_r)
		ctlr_r = ctlr_l;

	mutex_lock(&sc_input->mutex);
	mutex_lock(&ctlr_l->mutex);
	if (sc_input->ctlr_right)
		mutex_lock(&sc_input->ctlr_right->mutex);

	if (sc_input->type == SWITCHCON_INPUT_TYPE_JOYCON_H) {
		if (ctlr_l->is_right_joycon) {
			/* report stick */
			conversion = ctlr_r->right_stick_x;
			conversion -= ctlr_r->right_stick_cal.x_center;
			conversion *= -1;
			conversion += ctlr_r->right_stick_cal.x_center;
			input_report_abs(dev, ABS_X, ctlr_r->right_stick_y);
			input_report_abs(dev, ABS_Y, conversion);

			/* report buttons */
			input_report_key(dev, BTN_WEST,	  ctlr_r->but_b);
			input_report_key(dev, BTN_NORTH,  ctlr_r->but_y);
			input_report_key(dev, BTN_EAST,	  ctlr_r->but_x);
			input_report_key(dev, BTN_SOUTH,  ctlr_r->but_a);
			input_report_key(dev, BTN_THUMBR, ctlr_r->but_rstick);
			input_report_key(dev, BTN_START,  ctlr_r->but_minus);
			input_report_key(dev, BTN_MODE,	  ctlr_r->but_capture);
			input_report_key(dev, BTN_TR,
					 ctlr_r->but_sr_right_jc);
			input_report_key(dev, BTN_TL,
					 ctlr_r->but_sl_right_jc);
		} else {
			/* report stick */
			conversion = ctlr_l->left_stick_y;
			conversion -= ctlr_l->left_stick_cal.y_center;
			conversion *= -1;
			conversion += ctlr_l->left_stick_cal.y_center;
			input_report_abs(dev, ABS_X, conversion);
			input_report_abs(dev, ABS_Y, ctlr_l->left_stick_x);

			/* report buttons */
			input_report_key(dev, BTN_WEST,	  ctlr_l->but_up);
			input_report_key(dev, BTN_NORTH,  ctlr_l->but_right);
			input_report_key(dev, BTN_EAST,	  ctlr_l->but_down);
			input_report_key(dev, BTN_SOUTH,  ctlr_l->but_left);
			input_report_key(dev, BTN_THUMBL, ctlr_l->but_lstick);
			input_report_key(dev, BTN_START,  ctlr_l->but_plus);
			input_report_key(dev, BTN_MODE,	  ctlr_l->but_home);
			input_report_key(dev, BTN_TR,
					 ctlr_l->but_sr_left_jc);
			input_report_key(dev, BTN_TL,
					 ctlr_l->but_sl_left_jc);
		}
	} else {
		/* report sticks */
		input_report_abs(dev, ABS_X, ctlr_l->left_stick_x);
		input_report_abs(dev, ABS_Y, ctlr_l->left_stick_y);
		input_report_abs(dev, ABS_RX, ctlr_r->right_stick_x);
		input_report_abs(dev, ABS_RY, ctlr_r->right_stick_y);

		/* report buttons */
		input_report_key(dev, BTN_WEST,		ctlr_r->but_y);
		input_report_key(dev, BTN_NORTH,	ctlr_r->but_x);
		input_report_key(dev, BTN_EAST,		ctlr_r->but_a);
		input_report_key(dev, BTN_SOUTH,	ctlr_r->but_b);
		input_report_key(dev, BTN_TR,		ctlr_r->but_r);
		input_report_key(dev, BTN_TR2,		ctlr_r->but_zr);
		input_report_key(dev, BTN_TL,		ctlr_l->but_l);
		input_report_key(dev, BTN_TL2,		ctlr_l->but_zl);
		input_report_key(dev, BTN_SELECT,	ctlr_l->but_minus);
		input_report_key(dev, BTN_START,	ctlr_r->but_plus);
		input_report_key(dev, BTN_THUMBL,	ctlr_l->but_lstick);
		input_report_key(dev, BTN_THUMBR,	ctlr_r->but_rstick);
		input_report_key(dev, BTN_MODE,		ctlr_r->but_home);
		input_report_key(dev, BTN_Z,		ctlr_l->but_capture);
		input_report_key(dev, BTN_DPAD_DOWN,	ctlr_l->but_down);
		input_report_key(dev, BTN_DPAD_UP,	ctlr_l->but_up);
		input_report_key(dev, BTN_DPAD_RIGHT,	ctlr_l->but_right);
		input_report_key(dev, BTN_DPAD_LEFT,	ctlr_l->but_left);
	}

	input_sync(dev);

	mutex_unlock(&ctlr_l->mutex);
	if (sc_input->ctlr_right)
		mutex_unlock(&sc_input->ctlr_right->mutex);
	mutex_unlock(&sc_input->mutex);
}

static void switchcon_input_destroy(struct switchcon_input *sc_input)
{
	hid_dbg(sc_input->ctlr_left->hdev, "destroying input\n");
	mutex_lock(&sc_input->mutex);

	sc_input->ctlr_left->switchcon_in = NULL;
	mutex_lock(&sc_input->ctlr_left->mutex);
	if (sc_input->ctlr_right) {
		mutex_lock(&sc_input->ctlr_right->mutex);
		sc_input->ctlr_right->switchcon_in = NULL;
	}
	/* set the joycon states to searching */
	sc_input->ctlr_left->ctlr_state = SWITCHCON_CTLR_STATE_POST_CALIBRATION;
	if (sc_input->ctlr_right)
		sc_input->ctlr_right->ctlr_state =
					  SWITCHCON_CTLR_STATE_POST_CALIBRATION;

	input_unregister_device(sc_input->input);

	mutex_unlock(&sc_input->ctlr_left->mutex);
	if (sc_input->ctlr_right)
		mutex_unlock(&sc_input->ctlr_right->mutex);
	mutex_unlock(&sc_input->mutex);
	mutex_destroy(&sc_input->mutex);
	kfree(sc_input);
}

static const unsigned int switchcon_button_inputs[] = {
	BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST, /* b, a, x, y */
	BTN_TL, BTN_TR, BTN_TL2, BTN_TR2,	  /* L, R, ZL, ZR */
	BTN_SELECT, BTN_START, BTN_MODE, BTN_Z,	  /* -, +, home, media */
	BTN_THUMBL, BTN_THUMBR,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
	0
};

DEFINE_MUTEX(switchcon_input_num_mutex);
static struct switchcon_input *switchcon_input_create(
					enum switchcon_input_type type,
					struct switchcon_ctlr *ctlr_l,
					struct switchcon_ctlr *ctlr_r)
{
	struct switchcon_input *sc_input;
	struct hid_device *hdev;
	static int input_num = 1;
	int i;
	int ret;

	if (!ctlr_l)
		return ERR_PTR(-EINVAL);
	hdev = ctlr_l->hdev;
	sc_input = kzalloc(sizeof(*sc_input), GFP_KERNEL);
	if (!sc_input)
		return ERR_PTR(-ENOMEM);

	sc_input->type		= type;
	sc_input->ctlr_left	= ctlr_l;
	sc_input->ctlr_right	= ctlr_r;
	if (!ctlr_r)
		ctlr_r = ctlr_l;

	sc_input->input = input_allocate_device();
	if (!sc_input->input) {
		ret = -ENOMEM;
		goto err;
	}
	sc_input->input->dev.parent	= &hdev->dev;
	sc_input->input->id.bustype	= hdev->bus;
	sc_input->input->id.vendor	= hdev->vendor;
	sc_input->input->id.product	= hdev->product;
	sc_input->input->id.version	= hdev->version;
	sc_input->input->name = switchcon_input_names[sc_input->type];
	input_set_drvdata(sc_input->input, sc_input);

	/* set up button inputs */
	for (i = 0; switchcon_button_inputs[i] > 0; i++)
		input_set_capability(sc_input->input, EV_KEY,
				     switchcon_button_inputs[i]);

	/* set up sticks */
	if (sc_input->type == SWITCHCON_INPUT_TYPE_JOYCON_H) {
		if (ctlr_l->is_right_joycon) {
			input_set_abs_params(sc_input->input, ABS_X,
					     ctlr_l->right_stick_cal.y_min,
					     ctlr_l->right_stick_cal.y_max,
					     50, 20);
			input_set_abs_params(sc_input->input, ABS_Y,
					     ctlr_l->right_stick_cal.x_min,
					     ctlr_l->right_stick_cal.x_max,
					     50, 20);
		} else {
			input_set_abs_params(sc_input->input, ABS_X,
					     ctlr_l->left_stick_cal.y_min,
					     ctlr_l->left_stick_cal.y_max,
					     50, 20);
			input_set_abs_params(sc_input->input, ABS_Y,
					     ctlr_l->left_stick_cal.x_min,
					     ctlr_l->left_stick_cal.x_max,
					     50, 20);
		}
	} else {
		input_set_abs_params(sc_input->input, ABS_X,
				     ctlr_l->left_stick_cal.x_min,
				     ctlr_l->left_stick_cal.x_max,
				     50, 20);
		input_set_abs_params(sc_input->input, ABS_Y,
				     ctlr_l->left_stick_cal.y_min,
				     ctlr_l->left_stick_cal.y_max,
				     50, 20);
		input_set_abs_params(sc_input->input, ABS_RX,
				     ctlr_r->right_stick_cal.x_min,
				     ctlr_r->right_stick_cal.x_max,
				     50, 20);
		input_set_abs_params(sc_input->input, ABS_RY,
				     ctlr_r->right_stick_cal.y_min,
				     ctlr_r->right_stick_cal.y_max,
				     50, 20);
	}

	ret = input_register_device(sc_input->input);
	if (ret)
		goto err_input;

	mutex_init(&sc_input->mutex);
	INIT_WORK(&sc_input->input_worker, switchcon_update_input_handler);

	/* Set the controller player leds based on input number */
	mutex_lock(&switchcon_input_num_mutex);
	/* Need to send multiple times to make sure it's not ignored */
	for (i = 0; i < 5; i++) {
		switchcon_set_player_leds(ctlr_l, 0, 0xF >> (4 - input_num));
		switchcon_set_player_leds(ctlr_r, 0, 0xF >> (4 - input_num));
	}
	if (++input_num > 4)
		input_num = 1;
	mutex_unlock(&switchcon_input_num_mutex);

	return sc_input;
err_input:
	input_free_device(sc_input->input);
err:
	kfree(sc_input);
	return ERR_PTR(ret);
}

static void switchcon_create_procon_input_handler(struct work_struct *ws)
{
	struct switchcon_ctlr *ctlr = container_of(ws, struct switchcon_ctlr,
							create_input_worker);
	hid_dbg(ctlr->hdev, "create_procon_input_handler\n");

	mutex_lock(&ctlr->mutex);

	ctlr->switchcon_in = switchcon_input_create(SWITCHCON_INPUT_TYPE_PROCON,
								    ctlr, NULL);
	if (IS_ERR(ctlr->switchcon_in)) {
		hid_err(ctlr->hdev, "failed to create input\n");
		ctlr->switchcon_in = NULL;
	}

	mutex_unlock(&ctlr->mutex);
}

DEFINE_MUTEX(switchcon_detect_pair_mutex);
static void switchcon_detect_pair_handler(struct work_struct *ws)
{
	struct switchcon_ctlr *ctlr = container_of(ws, struct switchcon_ctlr,
							detect_pair_worker);
	static struct switchcon_ctlr *ctlr_l = NULL;
	static struct switchcon_ctlr *ctlr_r = NULL;

	mutex_lock(&switchcon_detect_pair_mutex);
	mutex_lock(&ctlr->mutex);
	/* Check if this is a controller no longer searching for partner */
	if (!ctlr->searching) {
		if (ctlr == ctlr_l)
			ctlr_l = NULL;
		if (ctlr == ctlr_r)
			ctlr_r = NULL;
	} else {
		if (!ctlr->is_right_joycon && ctlr_l == NULL)
			ctlr_l = ctlr;
		if (ctlr->is_right_joycon && ctlr_r == NULL)
			ctlr_r = ctlr;

		/* see if there's a pair */
		if (ctlr_l && ctlr_r) {
			if (ctlr == ctlr_l)
				mutex_lock(&ctlr_r->mutex);
			else
				mutex_lock(&ctlr_l->mutex);
			hid_info(ctlr->hdev, "Joy-Con pair found\n");

			ctlr_l->switchcon_in = switchcon_input_create(
						   SWITCHCON_INPUT_TYPE_JOYCONS,
						   ctlr_l, ctlr_r);
			ctlr_r->switchcon_in = ctlr_l->switchcon_in;
			ctlr_l->ctlr_state = SWITCHCON_CTLR_STATE_READ;
			ctlr_r->ctlr_state = SWITCHCON_CTLR_STATE_READ;

			if (IS_ERR(ctlr_l->switchcon_in)) {
				hid_err(ctlr->hdev, "Failed creating input\n");
				ctlr_l->switchcon_in = NULL;
				ctlr_r->switchcon_in = NULL;
			}

			if (ctlr == ctlr_l)
				mutex_unlock(&ctlr_r->mutex);
			else
				mutex_unlock(&ctlr_l->mutex);
			ctlr_l = NULL;
			ctlr_r = NULL;
		}
	}

	mutex_unlock(&ctlr->mutex);
	mutex_unlock(&switchcon_detect_pair_mutex);
}

static void switchcon_create_horiz_handler(struct work_struct *ws)
{
	struct switchcon_ctlr *ctlr = container_of(ws, struct switchcon_ctlr,
							create_horiz_worker);

	mutex_lock(&ctlr->mutex);

	ctlr->switchcon_in = switchcon_input_create(
				SWITCHCON_INPUT_TYPE_JOYCON_H, ctlr, NULL);
	if (IS_ERR(ctlr->switchcon_in)) {
		hid_err(ctlr->hdev, "failed to create input\n");
		ctlr->switchcon_in = NULL;
	}

	mutex_unlock(&ctlr->mutex);
}

static void switchcon_send_work_handler(struct work_struct *ws)
{
	struct switchcon_ctlr *ctlr = container_of(ws, struct switchcon_ctlr,
								output_worker);
	struct switchcon_output output;
	struct hid_device *hdev = ctlr->hdev;
	int ret;

	spin_lock(&ctlr->output_lock);
	while (ctlr->output_tail != ctlr->output_head) {
		output = ctlr->output_buf[ctlr->output_tail];
		ctlr->output_buf[ctlr->output_tail].data = NULL;
		ctlr->output_tail++;
		if (ctlr->output_tail == SC_OUTPUT_BUF_SIZE)
			ctlr->output_tail = 0;
		spin_unlock(&ctlr->output_lock);
		if (!output.data) {
			hid_warn(ctlr->hdev, "invalid output in buffer\n");
		} else {
			ret = hid_hw_output_report(hdev, output.data,
							 output.size);
			if (ret < 0)
				hid_warn(hdev, "failed output report ret=%d",
									ret);
			kfree(output.data);

		}
		spin_lock(&ctlr->output_lock);
	}
	spin_unlock(&ctlr->output_lock);
}

static int switchcon_hid_event(struct hid_device *hdev,
			struct hid_report *report, u8 *raw_data, int size)
{
	struct switchcon_ctlr *ctlr = hid_get_drvdata(hdev);

	if (size < 1)
		return -EINVAL;

	return switchcon_ctlr_handle_event(ctlr, raw_data, size);
}

/* data input must have at least 9 bytes */
static void switchcon_parse_left_stick_calibration(u8 *data,
					struct switchcon_stick_cal *cal) {
	u16 x_max_above;
	u16 x_min_below;
	u16 y_max_above;
	u16 y_min_below;

	x_max_above	= ((data[1] << 8) & 0xF00) | data[0];
	y_max_above	= (data[2] << 4) | (data[1] >> 4);
	x_min_below	= ((data[7] << 8) & 0xF00) | data[6];
	y_min_below	= (data[8] << 4) | (data[7] >> 4);
	cal->x_center	= ((data[4] << 8) & 0xF00) | data[3];
	cal->y_center	= (data[5] << 4) | (data[4] >> 4);
	cal->x_max	= cal->x_center + x_max_above;
	cal->x_min	= cal->x_center - x_min_below;
	cal->y_max	= cal->y_center + y_max_above;
	cal->y_min	= cal->y_center - y_min_below;
}

/* data input must have at least 9 bytes */
static void switchcon_parse_right_stick_calibration(u8 *data,
					struct switchcon_stick_cal *cal) {
	u16 x_max_above;
	u16 x_min_below;
	u16 y_max_above;
	u16 y_min_below;

	cal->x_center = ((data[1] << 8) & 0xF00) | data[0];
	cal->y_center = (data[2] << 4) | (data[1] >> 4);
	x_min_below	= ((data[7] << 8) & 0xF00) | data[6];
	y_min_below	= (data[8] << 4) | (data[7] >> 4);
	x_max_above = ((data[4] << 8) & 0xF00) | data[3];
	y_max_above	= (data[5] << 4) | (data[4] >> 4);
	cal->x_max	= cal->x_center + x_max_above;
	cal->x_min	= cal->x_center - x_min_below;
	cal->y_max	= cal->y_center + y_max_above;
	cal->y_min	= cal->y_center - y_min_below;
}
/* Common handler for parsing inputs */
static int switchcon_ctlr_read_handler(struct switchcon_ctlr *ctlr,
						u8 *data, int size)
{
	int ret = 0;

	switch (data[0]) {
	case SC_INPUT_SUBCMD_REPLY:
	case SC_INPUT_IMU_DATA:
	case SC_INPUT_MCU_DATA:
		if (size < 12) /* make sure it contains the input report */
			break;

		/* Parse analog sticks */
		ctlr->left_stick_x = hid_field_extract(ctlr->hdev, (data + 6), 0, 12);
		ctlr->left_stick_y = ctlr->left_stick_cal.y_min + ctlr->left_stick_cal.y_max - hid_field_extract(ctlr->hdev, (data + 7), 4, 12);
		ctlr->right_stick_x = hid_field_extract(ctlr->hdev, (data + 9), 0, 12);
		ctlr->right_stick_y = ctlr->right_stick_cal.y_min + ctlr->right_stick_cal.y_max - hid_field_extract(ctlr->hdev, (data + 10), 4, 12);


		/* Parse digital buttons */
		ctlr->but_y		= 0x01 & data[3];
		ctlr->but_x		= 0x02 & data[3];
		ctlr->but_b		= 0x04 & data[3];
		ctlr->but_a		= 0x08 & data[3];
		ctlr->but_sr_right_jc	= 0x10 & data[3];
		ctlr->but_sl_right_jc	= 0x20 & data[3];
		ctlr->but_r		= 0x40 & data[3];
		ctlr->but_zr		= 0x80 & data[3];
		ctlr->but_l		= 0x40 & data[5];
		ctlr->but_zl		= 0x80 & data[5];
		ctlr->but_minus		= 0x01 & data[4];
		ctlr->but_plus		= 0x02 & data[4];
		ctlr->but_rstick	= 0x04 & data[4];
		ctlr->but_lstick	= 0x08 & data[4];
		ctlr->but_home		= 0x10 & data[4];
		ctlr->but_capture	= 0x20 & data[4];
		ctlr->but_down		= 0x01 & data[5];
		ctlr->but_up		= 0x02 & data[5];
		ctlr->but_right		= 0x04 & data[5];
		ctlr->but_left		= 0x08 & data[5];
		ctlr->but_sr_left_jc	= 0x10 & data[5];
		ctlr->but_sl_left_jc	= 0x20 & data[5];

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int switchcon_ctlr_procon_init(struct switchcon_ctlr *ctlr)
{
	int ret;
	u8 baud_cmd[] = {SC_OUTPUT_USB_CMD, SC_USB_CMD_BAUDRATE_3M};
	mutex_lock(&ctlr->mutex);
	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_USB_SET_BAUD;
	ret = switchcon_hid_queue_send(ctlr, baud_cmd, sizeof(baud_cmd));
	mutex_unlock(&ctlr->mutex);
	return ret;
}

static int switchcon_ctlr_joyconl_init(struct switchcon_ctlr *ctlr)
{
	mutex_lock(&ctlr->mutex);
	switchcon_request_calibration(ctlr);
	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_CALIBRATION;
	ctlr->is_right_joycon = false;
	mutex_unlock(&ctlr->mutex);
	return 0;
}

static int switchcon_ctlr_joyconr_init(struct switchcon_ctlr *ctlr)
{
	mutex_lock(&ctlr->mutex);
	switchcon_request_calibration(ctlr);
	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_CALIBRATION;
	ctlr->is_right_joycon = true;
	switchcon_request_calibration(ctlr);
	mutex_unlock(&ctlr->mutex);
	return 0;
}

void switchcon_ctlr_general_deinit(struct switchcon_ctlr *ctlr)
{
	if (ctlr->switchcon_in)
		switchcon_input_destroy(ctlr->switchcon_in);
	mutex_lock(&ctlr->mutex);
	flush_work(&ctlr->output_worker);
	mutex_unlock(&ctlr->mutex);
	mutex_destroy(&ctlr->mutex);
}

static int switchcon_ctlr_general_handle_event(struct switchcon_ctlr *ctlr,
							u8 *data, int size)
{
	int ret;
	u8 inputmode_subcmd[]	= {SC_SUBCMD_SET_REPORT_MODE, 0x30};

	mutex_lock(&ctlr->mutex);
	switch (ctlr->ctlr_state) {
	case SWITCHCON_CTLR_STATE_CALIBRATION:
		if (data[0] != SC_INPUT_SUBCMD_REPLY || size < 38
		 || data[13] != 0x90 || data[14] != 0x10)
			break;
		switchcon_parse_left_stick_calibration(data + 20,
						  &ctlr->left_stick_cal);
		switchcon_parse_right_stick_calibration(data + 29,
						  &ctlr->right_stick_cal);

		hid_dbg(ctlr->hdev, "l_x_c=%hu l_x_max=%hu l_x_min=%hu\n",
						ctlr->left_stick_cal.x_center,
						ctlr->left_stick_cal.x_max,
						ctlr->left_stick_cal.x_min);
		hid_dbg(ctlr->hdev, "l_y_c=%hu l_y_max=%hu l_y_min=%hu\n",
						ctlr->left_stick_cal.y_center,
						ctlr->left_stick_cal.y_max,
						ctlr->left_stick_cal.y_min);
		hid_dbg(ctlr->hdev, "r_x_c=%hu r_x_max=%hu r_x_min=%hu\n",
						ctlr->right_stick_cal.x_center,
						ctlr->right_stick_cal.x_max,
						ctlr->right_stick_cal.x_min);
		hid_dbg(ctlr->hdev, "r_y_c=%hu r_y_max=%hu r_y_min=%hu\n",
						ctlr->right_stick_cal.y_center,
						ctlr->right_stick_cal.y_max,
						ctlr->right_stick_cal.y_min);
		ctlr->ctlr_state = SWITCHCON_CTLR_STATE_POST_CALIBRATION;
		ret = switchcon_queue_subcommand(ctlr, inputmode_subcmd,
						 sizeof(inputmode_subcmd));
		hid_dbg(ctlr->hdev, "requested 0x30 reporting mode\n");
		break;
	case SWITCHCON_CTLR_STATE_READ:
		switchcon_ctlr_read_handler(ctlr, data, size);
		if (ctlr->switchcon_in) {
			schedule_work(&ctlr->switchcon_in->input_worker);
		}
		break;
	default:
		break;
	}
	mutex_unlock(&ctlr->mutex);

	return 0;
}

static int switchcon_ctlr_procon_handle_event(struct switchcon_ctlr *ctlr,
							u8 *data, int size)
{
	int ret;
	u8 handshake_cmd[]	= {SC_OUTPUT_USB_CMD, SC_USB_CMD_HANDSHAKE};
	u8 timeout_cmd[]	= {SC_OUTPUT_USB_CMD, SC_USB_CMD_NO_TIMEOUT};

	mutex_lock(&ctlr->mutex);
	switch (ctlr->ctlr_state) {
	case SWITCHCON_CTLR_STATE_USB_SET_BAUD:
		if (size < 2 || data[0] != SC_INPUT_USB_RESPONSE
		    || data[1] != SC_USB_CMD_BAUDRATE_3M ) {
			hid_dbg(ctlr->hdev, "No usb response, assume ble\n");
			ctlr->ctlr_state = SWITCHCON_CTLR_STATE_CALIBRATION;
			switchcon_request_calibration(ctlr);
			break;
		}

		ret = switchcon_hid_queue_send(ctlr, handshake_cmd,
						sizeof(handshake_cmd));
		if (!ret) {
			ctlr->ctlr_state = SWITCHCON_CTLR_STATE_USB_HANDSHAKE;
			hid_dbg(ctlr->hdev, "sent handshake\n");
		}
		break;
	case SWITCHCON_CTLR_STATE_USB_HANDSHAKE:
		if (size < 2 || data[0] != SC_INPUT_USB_RESPONSE
		    || data[1] != SC_USB_CMD_HANDSHAKE )
			break;
		ret = switchcon_hid_queue_send(ctlr, timeout_cmd,
						sizeof(timeout_cmd));
		if (!ret) {
			hid_dbg(ctlr->hdev, "sent timeout disable\n");
			ctlr->ctlr_state = SWITCHCON_CTLR_STATE_CALIBRATION;
			switchcon_request_calibration(ctlr);
		}
		break;
	case SWITCHCON_CTLR_STATE_POST_CALIBRATION:
		schedule_work(&ctlr->create_input_worker);
		ctlr->ctlr_state = SWITCHCON_CTLR_STATE_READ;
		break;
	default:
		mutex_unlock(&ctlr->mutex);
		return switchcon_ctlr_general_handle_event(ctlr, data, size);
		break;
	}
	mutex_unlock(&ctlr->mutex);
	return 0;
}

static int switchcon_ctlr_joycon_handle_event(struct switchcon_ctlr *ctlr,
							u8 *data, int size)
{
	bool last_searching;
	int i;

	mutex_lock(&ctlr->mutex);
	switch (ctlr->ctlr_state) {
	case SWITCHCON_CTLR_STATE_POST_CALIBRATION:
		ctlr->ctlr_state = SWITCHCON_CTLR_STATE_SEARCH;
		/* flashing leds to indicate searching */
		for (i = 0; i < 10; i++)
			/* command multiple times to ensure it works */
			switchcon_set_player_leds(ctlr, 0b1111, 0);
		/* intentional fall-through */
	case SWITCHCON_CTLR_STATE_SEARCH:
		last_searching = ctlr->searching;
		switchcon_ctlr_read_handler(ctlr, data, size);
		ctlr->searching =  ctlr->but_r || ctlr->but_zr
				|| ctlr->but_l || ctlr->but_zl;
		if (ctlr->searching != last_searching) {
			schedule_work(&ctlr->detect_pair_worker);
		} else if ((ctlr->but_sr_left_jc && ctlr->but_sl_left_jc)
		        || (ctlr->but_sr_right_jc && ctlr->but_sl_right_jc)) {
			schedule_work(&ctlr->create_horiz_worker);
			ctlr->ctlr_state = SWITCHCON_CTLR_STATE_READ;
		}
		break;
	default:
		mutex_unlock(&ctlr->mutex);
		return switchcon_ctlr_general_handle_event(ctlr, data, size);
		break;
	}
	mutex_unlock(&ctlr->mutex);
	return 0;
}

/* Implementations for each supported controller type */
static const struct switchcon_impl switchcon_impl_procon = {
	.init		= switchcon_ctlr_procon_init,
	.deinit		= switchcon_ctlr_general_deinit,
	.handle_event	= switchcon_ctlr_procon_handle_event,
};

static const struct switchcon_impl switchcon_impl_joycon_l = {
	.init		= switchcon_ctlr_joyconl_init,
	.deinit		= switchcon_ctlr_general_deinit,
	.handle_event	= switchcon_ctlr_joycon_handle_event,
};

static const struct switchcon_impl switchcon_impl_joycon_r = {
	.init		= switchcon_ctlr_joyconr_init,
	.deinit		= switchcon_ctlr_general_deinit,
	.handle_event	= switchcon_ctlr_joycon_handle_event,
};

static struct switchcon_ctlr *switchcon_ctlr_create(struct hid_device *hdev)
{
	struct switchcon_ctlr *ctlr;

	ctlr = devm_kzalloc(&hdev->dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr)
		return ERR_PTR(-ENOMEM);

	switch (hdev->product) {
	case USB_DEVICE_ID_NINTENDO_PROCON:
		hid_info(hdev, "detected pro controller\n");
		ctlr->impl = &switchcon_impl_procon;
		break;
	case USB_DEVICE_ID_NINTENDO_JOYCONL:
		hid_info(hdev, "detected left joy-con\n");
		ctlr->impl = &switchcon_impl_joycon_l;
		break;
	case USB_DEVICE_ID_NINTENDO_JOYCONR:
		hid_info(hdev, "detected right joy-con\n");
		ctlr->impl = &switchcon_impl_joycon_r;
		break;
	default:
		hid_err(hdev, "unknown product id\n");
		return ERR_PTR(-EINVAL);
	}
	ctlr->hdev = hdev;
	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_INIT;
	hid_set_drvdata(hdev, ctlr);
	spin_lock_init(&ctlr->output_lock);
	mutex_init(&ctlr->mutex);
	INIT_WORK(&ctlr->output_worker, switchcon_send_work_handler);
	INIT_WORK(&ctlr->create_input_worker,
		  switchcon_create_procon_input_handler);
	INIT_WORK(&ctlr->detect_pair_worker, switchcon_detect_pair_handler);
	INIT_WORK(&ctlr->create_horiz_worker, switchcon_create_horiz_handler);
	return ctlr;
}

static int switchcon_hid_probe(struct hid_device *hdev,
			       const struct hid_device_id *id)
{
	int ret;
	struct switchcon_ctlr *ctlr;

	hid_dbg(hdev, "probe - start\n");
	hdev->quirks |= HID_QUIRK_NO_INIT_REPORTS;

	ctlr = switchcon_ctlr_create(hdev);
	if (IS_ERR(ctlr)) {
		hid_err(hdev, "Failed to create new controller\n");
		ret = PTR_ERR(ctlr);
		goto err;
	}

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "HID parse failed\n");
		goto err;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "HW start failed\n");
		goto err;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "cannot start hardware I/O\n");
		goto err_stop;
	}

	ret = switchcon_ctlr_init(ctlr);
	if (ret) {
		hid_err(hdev, "failed to initialize ctlr\n");
		goto err_close;
	}

	hid_dbg(hdev, "probe - success\n");
	return 0;

err_close:
	switchcon_ctlr_deinit(ctlr);
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
err:
	hid_err(hdev, "probe - fail = %d\n", ret);
	return ret;
}

static void switchcon_hid_remove(struct hid_device *hdev)
{
	struct switchcon_ctlr *ctlr = hid_get_drvdata(hdev);

	hid_dbg(hdev, "remove\n");
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
	switchcon_ctlr_deinit(ctlr);
}

static const struct hid_device_id switchcon_hid_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONR) },
	{ }
};
MODULE_DEVICE_TABLE(hid, switchcon_hid_devices);

static struct hid_driver switchcon_hid_driver = {
	.name		= "switchcon",
	.id_table	= switchcon_hid_devices,
	.probe		= switchcon_hid_probe,
	.remove		= switchcon_hid_remove,
	.raw_event	= switchcon_hid_event,
};
module_hid_driver(switchcon_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel J. Ogorchock <djogorchock@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch Controllers");
