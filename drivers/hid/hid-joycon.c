/*
 * HID driver for Nintendo Joy-Con peripherals
 * Copyright (c) 2018 Peter Rankin <rankstar59@gmail.com>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include "hid-ids.h"

struct joycon_data {
	struct input_dev *input;
	struct hid_device *hdev;
};

static const u16 JC_MAX_STICK_MAG		= 32767;
static const u16 JC_STICK_FUZZ			= 250;
static const u16 JC_STICK_FLAT			= 500;

static void joycon_virtual_stick_clockwise(__u8 offset, __u8 value, int *x, int *y) {
	// change HAT byte into virtual analog axis values
	*x = 0;
	*y = 0;
	if (0 != (value & 0x08)) {
		return;
	}
	value -= offset;
	value &= 0x7;
	switch(value) {
		case 0x00:  // LEFT
			*x = -32767;
			*y = 0;
			break;
		case 0x01:  // TOP, LEFT
			*x = -23258;
			*y = -23258;
			break;
		case 0x02:  // TOP
			*x = 0;
			*y = -32767;
			break;
		case 0x03:  // TOP, RIGHT
			*x = 23258;
			*y = -23258;
			break;
		case 0x04:  // RIGHT
			*x = 32767;
			*y = 0;
			break;
		case 0x05:  // BOTTOM, RIGHT
			*x = 23258;
			*y = 23258;
			break;
		case 0x06:  // BOTTOM
			*x = 0;
			*y = 32767;
			break;
		case 0x07:  // BOTTOM, LEFT
			*x = -23258;
			*y = 23258;
			break;
	}
}

static const unsigned int joycon_button_inputs[] = {
	BTN_SELECT, BTN_Z, BTN_THUMBL,
	BTN_START, BTN_MODE, BTN_THUMBR,
	BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
	BTN_TL, BTN_TR, BTN_TL2, BTN_TR2,
	0 /* 0 signals end of array */
};

static void joycon_init_input(struct joycon_data *jdata) {
	int i;
	/* set up sticks */
	input_set_abs_params(jdata->input, ABS_X,
				 -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				 JC_STICK_FUZZ, JC_STICK_FLAT);
	input_set_abs_params(jdata->input, ABS_Y,
				 -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				 JC_STICK_FUZZ, JC_STICK_FLAT);
	input_set_abs_params(jdata->input, ABS_RX,
				 -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				 JC_STICK_FUZZ, JC_STICK_FLAT);
	input_set_abs_params(jdata->input, ABS_RY,
				 -JC_MAX_STICK_MAG, JC_MAX_STICK_MAG,
				 JC_STICK_FUZZ, JC_STICK_FLAT);
	/* set up buttons */
	for (i = 0; joycon_button_inputs[i] > 0; i++)
		input_set_capability(jdata->input, EV_KEY,
					 joycon_button_inputs[i]);
}

static void joycon_handle_input_3f(struct joycon_data *jdata, const __u8 *payload)
{
	int stick_x = 0;
	int stick_y = 0;
	u32 id = jdata->hdev->product;
	if (id == USB_DEVICE_ID_NINTENDO_JOYCONL) {
		joycon_virtual_stick_clockwise(0x04, payload[3], &stick_x, &stick_y);
		input_report_abs(jdata->input, ABS_X, stick_x);
		input_report_abs(jdata->input, ABS_Y, stick_y);
		input_report_key(jdata->input, BTN_TL, !!(payload[2] & 0x40));  // L
		input_report_key(jdata->input, BTN_TL2, !!(payload[2] & 0x80));  // LZ
		input_report_key(jdata->input, BTN_TR, !!(payload[1] & 0x10));  // Report the S buttons as the non-existent triggers
		input_report_key(jdata->input, BTN_TR2, !!(payload[1] & 0x20));  // ^
		input_report_key(jdata->input, BTN_SELECT, !!(payload[2] & 0x01));  // MINUS
		input_report_key(jdata->input, BTN_THUMBL, !!(payload[2] & 0x04));  // MIDDLE JOYSTICK
		input_report_key(jdata->input, BTN_Z, !!(payload[2] & 0x20));  // MIDDLE JOYSTICK
		input_report_key(jdata->input, BTN_DPAD_LEFT, !!(payload[1] & 0x01));
		input_report_key(jdata->input, BTN_DPAD_DOWN, !!(payload[1] & 0x02));
		input_report_key(jdata->input, BTN_DPAD_UP, !!(payload[1] & 0x04));
		input_report_key(jdata->input, BTN_DPAD_RIGHT, !!(payload[1] & 0x08));
	} else if (id == USB_DEVICE_ID_NINTENDO_JOYCONR) {
		joycon_virtual_stick_clockwise(0x00, payload[3], &stick_x, &stick_y);
		input_report_abs(jdata->input, ABS_RX, stick_x);
		input_report_abs(jdata->input, ABS_RY, stick_y);
		input_report_key(jdata->input, BTN_TR, !!(payload[2] & 0x40));  // R
		input_report_key(jdata->input, BTN_TR2, !!(payload[2] & 0x80));  // RZ
		input_report_key(jdata->input, BTN_TL, !!(payload[1] & 0x10));  // Report the S buttons as the non-existent triggers
		input_report_key(jdata->input, BTN_TL2, !!(payload[1] & 0x20));  // ^
		input_report_key(jdata->input, BTN_START, !!(payload[2] & 0x02));  // MINUS
		input_report_key(jdata->input, BTN_THUMBR, !!(payload[2] & 0x08));  // MIDDLE JOYSTICK
		input_report_key(jdata->input, BTN_MODE, !!(payload[2] & 0x10));  // MODE
		input_report_key(jdata->input, BTN_NORTH, !!(payload[1] & 0x02));  // X (xbox Y)
		input_report_key(jdata->input, BTN_WEST, !!(payload[1] & 0x08));  // Y (xbox X)
		input_report_key(jdata->input, BTN_SOUTH, !!(payload[1] & 0x04));  // B (xbox A)
		input_report_key(jdata->input, BTN_EAST, !!(payload[1] & 0x01));  // A (xbox B)
	}
	input_sync(jdata->input);
}

static int joycon_hid_event(struct hid_device *hdev, struct hid_report *report,
							u8 *raw_data, int size)
{
	struct joycon_data *jdata = hid_get_drvdata(hdev);

	if (size < 1)
		return -EINVAL;

	switch(raw_data[0]) {
		case 0x3f:
			if (size >= 12) {
				joycon_handle_input_3f(jdata, raw_data);
			}
			break;
	}

	return 0;
}

static int joycon_hid_probe(struct hid_device *hdev,
				const struct hid_device_id *id)
{
	struct joycon_data *jdata;
	int ret;
	const char *name;

	switch (hdev->product) {
	case USB_DEVICE_ID_NINTENDO_JOYCONL:
		name = "Nintendo Switch Left Joy-Con";
		break;
	case USB_DEVICE_ID_NINTENDO_JOYCONR:
		name = "Nintendo Switch Right Joy-Con";
		break;
	default: /* Should be impossible */
		hid_err(hdev, "Invalid hid product\n");
		return -EINVAL;
	}

	jdata = kzalloc(sizeof(*jdata), GFP_KERNEL);
	if (!jdata) {
		hid_err(hdev, "Can't alloc device\n");
		return -ENOMEM;
	}

	jdata->hdev = hdev;
	hid_set_drvdata(hdev, jdata);

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

	hid_info(hdev, "New device registered\n");

	jdata->input = input_allocate_device();

	input_set_drvdata(jdata->input, jdata);
	jdata->input->dev.parent = &jdata->hdev->dev;
	jdata->input->id.bustype = jdata->hdev->bus;
	jdata->input->id.vendor = jdata->hdev->vendor;
	jdata->input->id.product = jdata->hdev->product;
	jdata->input->id.version = jdata->hdev->version;
	jdata->input->name = name;

	joycon_init_input(jdata);

	ret = input_register_device(jdata->input);
	if (ret) {
		input_free_device(jdata->input);
		jdata->input = NULL;
		goto err_stop;
	}

	return 0;

err_stop:
	hid_hw_stop(hdev);
err:
	kfree(jdata);
	return ret;
}

static void joycon_hid_remove(struct hid_device *hdev)
{
	struct joycon_data *jdata = hid_get_drvdata(hdev);

	hid_hw_close(jdata->hdev);
	hid_hw_stop(jdata->hdev);

	kfree(jdata);
}

static const struct hid_device_id joycon_hid_devices[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
				USB_DEVICE_ID_NINTENDO_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
				USB_DEVICE_ID_NINTENDO_JOYCONR) },
	{ }
};
MODULE_DEVICE_TABLE(hid, joycon_hid_devices);

static struct hid_driver joycon_hid_driver = {
	.name = "joycon",
	.id_table = joycon_hid_devices,
	.probe = joycon_hid_probe,
	.remove = joycon_hid_remove,
	.raw_event = joycon_hid_event,
};
module_hid_driver(joycon_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Rankin <rankstar59@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Joy-Con peripherals");

