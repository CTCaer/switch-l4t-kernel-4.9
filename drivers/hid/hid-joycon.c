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

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include "hid-ids.h"

#define JOYCON_BUFSIZE 0x40

enum joycon_hold {
    JOYCON_HOLD_INDIVIDUAL,
    JOYCON_HOLD_COMBINLE_INTO_L,
    JOYCON_HOLD_COMBINLE_INTO_R,
};

enum joycon_devtype {
    JOYCON_DEV_GENERIC,
    JOYCON_DEV_JOYCON_L,
    JOYCON_DEV_JOYCON_R,
    JOYCON_DEV_NUM,
};

struct joycon_global_data;

struct joycon_data {
    struct hid_device *hdev;
    struct input_dev *input;
    struct joycon_global_data *global;
    __u8 raw_data[JOYCON_BUFSIZE];
    int pkt_count;
};

struct joycon_global_data {
    struct joycon_data *left;
    struct joycon_data *right;
    __u8 hold;
};

#define dev_to_joy(pdev) hid_get_drvdata(container_of(pdev, struct hid_device, \
                                    dev))

static DEFINE_SPINLOCK(jdata_lock);
static __u8 jdata_hold = JOYCON_HOLD_COMBINLE_INTO_L;
static struct joycon_data *jdata_left = NULL;
static struct joycon_data *jdata_right = NULL;

static __u8 joycon_devtype(struct joycon_data *jdata) {
    // joycon HID device data to device enumerator
    __u16 vendor, product;

    vendor = jdata->hdev->vendor;
    product = jdata->hdev->product;

    if (vendor == USB_VENDOR_ID_NINTENDO) {
        if (product == USB_DEVICE_ID_NINTENDO_JOYCONL) {
            return JOYCON_DEV_JOYCON_L;
        } else if (product == USB_DEVICE_ID_NINTENDO_JOYCONR) {
            return JOYCON_DEV_JOYCON_R;
        }
    }
    return JOYCON_DEV_GENERIC;
}

static const char *joycon_name(struct joycon_data *jdata) {
    // friendly name from joycon device
    switch(joycon_devtype(jdata)) {
        case JOYCON_DEV_JOYCON_L:
            return "Nintendo JoyCon (L) Remote";
        case JOYCON_DEV_JOYCON_R:
            return "Nintendo JoyCon (R) Remote";
        default:
            break;
    }
    return "Nintendo JoyCon";
}

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
            *x = -510;
            *y = 0;
            break;
        case 0x01:  // TOP, LEFT
            *x = -362;
            *y = -362;
            break;
        case 0x02:  // TOP
            *x = 0;
            *y = -510;
            break;
        case 0x03:  // TOP, RIGHT
            *x = 362;
            *y = -362;
            break;
        case 0x04:  // RIGHT
            *x = 510;
            *y = 0;
            break;
        case 0x05:  // BOTTOM, RIGHT
            *x = 362;
            *y = 362;
            break;
        case 0x06:  // BOTTOM
            *x = 0;
            *y = 510;
            break;
        case 0x07:  // BOTTOM, LEFT
            *x = -362;
            *y = 362;
            break;
    }
}

static void joycon_init_input(struct joycon_data *jdata) {
    // setup input device for a controller
    // each controller has ALL the inputs as if they were combined
    // this makes for easier "combining" of controllers

    // controllers have buttons
    set_bit(EV_KEY, jdata->input->evbit);

    // controllers have analog axis
    set_bit(EV_ABS, jdata->input->evbit);

    // buttons for "left" side
    set_bit(KEY_LEFT, jdata->input->keybit);
    set_bit(KEY_RIGHT, jdata->input->keybit);
    set_bit(KEY_UP, jdata->input->keybit);
    set_bit(KEY_DOWN, jdata->input->keybit);
    // MODE button on left joycon is only used to change modes (combine/individual)
    set_bit(BTN_TL, jdata->input->keybit);
    set_bit(BTN_TL2, jdata->input->keybit);
    set_bit(BTN_SELECT, jdata->input->keybit);
    set_bit(BTN_THUMBL, jdata->input->keybit);

    // buttons for "right" side
    set_bit(BTN_Y, jdata->input->keybit);
    set_bit(BTN_X, jdata->input->keybit);
    set_bit(BTN_B, jdata->input->keybit);
    set_bit(BTN_A, jdata->input->keybit);
    set_bit(BTN_TR, jdata->input->keybit);
    set_bit(BTN_TR2, jdata->input->keybit);
    set_bit(BTN_START, jdata->input->keybit);
    set_bit(BTN_MODE, jdata->input->keybit);
    set_bit(BTN_THUMBR, jdata->input->keybit);

    // "left" stick
    input_set_abs_params(jdata->input, ABS_X, -512, 512, 0, 0);
    input_set_abs_params(jdata->input, ABS_Y, -512, 512, 0, 0);

    // "right" stick
    input_set_abs_params(jdata->input, ABS_Z, -512, 512, 0, 0);
    input_set_abs_params(jdata->input, ABS_RZ, -512, 512, 0, 0);
    
	input_set_abs_params(jdata->input, ABS_HAT0X, -1, 1, 0, 0);
    input_set_abs_params(jdata->input, ABS_HAT0Y, -1, 1, 0, 0);
	
	input_set_abs_params(jdata->input, ABS_HAT2X, 0, 512, 0, 0);
    input_set_abs_params(jdata->input, ABS_HAT2Y, 0, 512, 0, 0);
}


static void joycon_handle_input_3f_left(struct joycon_data *jdata, const __u8 *left_payload) {
    int stick_x = 0;
    int stick_y = 0;

    input_report_key(jdata->input, KEY_LEFT, 0);
    input_report_key(jdata->input, KEY_DOWN, 0);
    input_report_key(jdata->input, KEY_UP, 0);
    input_report_key(jdata->input, KEY_RIGHT, 0);
    input_report_key(jdata->input, BTN_TL, !!(left_payload[1] & 0x10));  // SL
    input_report_key(jdata->input, BTN_TL2, 0);
    input_report_key(jdata->input, BTN_SELECT, !!(left_payload[2] & 0x01));  // MINUS
    input_report_key(jdata->input, BTN_THUMBL, !!(left_payload[2] & 0x04));  // MIDDLE JOYSTICK
    joycon_virtual_stick_clockwise(0x04 + 0x2, left_payload[3], &stick_x, &stick_y);
    input_report_abs(jdata->input, ABS_X, stick_x);
    input_report_abs(jdata->input, ABS_Y, stick_y);

    input_report_key(jdata->input, BTN_Y, !!(left_payload[1] & 0x08));  // X (Right) (xbox Y)
    input_report_key(jdata->input, BTN_X, !!(left_payload[1] & 0x04));  // Y (Up) (xbox X)
    input_report_key(jdata->input, BTN_A, !!(left_payload[1] & 0x01));  // B (Left) (xbox A)
    input_report_key(jdata->input, BTN_B, !!(left_payload[1] & 0x02));  // A (Down) (xbox B)
    input_report_key(jdata->input, BTN_TR, !!(left_payload[1] & 0x20));  // SR
    input_report_key(jdata->input, BTN_TR2, 0);
    input_report_key(jdata->input, BTN_START, !!(left_payload[2] & 0x20));  // MODE
    input_report_key(jdata->input, BTN_MODE, 0);
    input_report_key(jdata->input, BTN_THUMBR, 0);
    input_report_abs(jdata->input, ABS_Z, 0);
    input_report_abs(jdata->input, ABS_RZ, 0);
	input_report_abs(jdata->input, ABS_HAT0X, 0);
	input_report_abs(jdata->input, ABS_HAT0Y, 0);
	input_report_abs(jdata->input, ABS_HAT2X, 0);
	input_report_abs(jdata->input, ABS_HAT2Y, 0);


    input_sync(jdata->input);
}

static void joycon_handle_input_3f_right(struct joycon_data *jdata, const __u8 *right_payload) {
    int stick_x = 0;
    int stick_y = 0;

    input_report_key(jdata->input, KEY_LEFT, 0);
    input_report_key(jdata->input, KEY_DOWN, 0);
    input_report_key(jdata->input, KEY_UP, 0);
    input_report_key(jdata->input, KEY_RIGHT, 0);
    input_report_key(jdata->input, BTN_TL, !!(right_payload[1] & 0x10));  // SL
    input_report_key(jdata->input, BTN_TL2, 0);
    input_report_key(jdata->input, BTN_SELECT, !!(right_payload[2] & 0x10));  // MINUS
    input_report_key(jdata->input, BTN_THUMBL, !!(right_payload[2] & 0x08));  // MIDDLE JOYSTICK
    joycon_virtual_stick_clockwise(0x06, right_payload[3], &stick_x, &stick_y);
    input_report_abs(jdata->input, ABS_X, stick_x);
    input_report_abs(jdata->input, ABS_Y, stick_y);

    input_report_key(jdata->input, BTN_Y, !!(right_payload[1] & 0x08));  // Y (UP) (xbox Y)
    input_report_key(jdata->input, BTN_X, !!(right_payload[1] & 0x04));  // B (LEFT) (xbox X)
    input_report_key(jdata->input, BTN_A, !!(right_payload[1] & 0x01));  // A (DOWN) (xbox A)
    input_report_key(jdata->input, BTN_B, !!(right_payload[1] & 0x02));  // X (RIGHT) (xbox B)
    input_report_key(jdata->input, BTN_TR, !!(right_payload[1] & 0x20));  // SR
    input_report_key(jdata->input, BTN_TR2, 0);
    input_report_key(jdata->input, BTN_START, !!(right_payload[2] & 0x02));  // MODE
    input_report_key(jdata->input, BTN_MODE, 0);
    input_report_key(jdata->input, BTN_THUMBR, 0);
    input_report_abs(jdata->input, ABS_Z, 0);
    input_report_abs(jdata->input, ABS_RZ, 0);
	input_report_abs(jdata->input, ABS_HAT0X, 0);
	input_report_abs(jdata->input, ABS_HAT0Y, 0);
	input_report_abs(jdata->input, ABS_HAT2X, 0);
	input_report_abs(jdata->input, ABS_HAT2Y, 0);

    input_sync(jdata->input);
}

static void joycon_handle_input_3f_combine(struct joycon_data *jdata, const __u8 *left_payload, const __u8 *right_payload) {
    int stick_x = 0;
    int stick_y = 0;

    input_report_key(jdata->input, BTN_TL, !!(left_payload[2] & 0x40));  // L
    input_report_key(jdata->input, BTN_TL2, !!(left_payload[2] & 0x80));  // LZ
    input_report_key(jdata->input, BTN_SELECT, !!(left_payload[2] & 0x01));  // MINUS
    input_report_key(jdata->input, BTN_THUMBL, !!(left_payload[2] & 0x04));  // MIDDLE JOYSTICK
    input_report_key(jdata->input, BTN_C, !!(right_payload[2] & 0x10));  // SCRSHOT
    joycon_virtual_stick_clockwise(0x04, left_payload[3], &stick_x, &stick_y);
    input_report_abs(jdata->input, ABS_X, stick_x);
    input_report_abs(jdata->input, ABS_Y, stick_y);

    input_report_key(jdata->input, BTN_Y, !!(right_payload[1] & 0x02));  // X (xbox Y)
    input_report_key(jdata->input, BTN_X, !!(right_payload[1] & 0x08));  // Y (xbox X)
    input_report_key(jdata->input, BTN_A, !!(right_payload[1] & 0x04));  // B (xbox A)
    input_report_key(jdata->input, BTN_B, !!(right_payload[1] & 0x01));  // A (xbox B)
    input_report_key(jdata->input, BTN_TR, !!(right_payload[2] & 0x40));  // R
    input_report_key(jdata->input, BTN_TR2, !!(right_payload[2] & 0x80));  // RZ
    input_report_key(jdata->input, BTN_START, !!(right_payload[2] & 0x02));  // PLUS
    input_report_key(jdata->input, BTN_MODE, !!(right_payload[2] & 0x10));  // MODE
    input_report_key(jdata->input, BTN_THUMBR, !!(right_payload[2] & 0x08));  // MIDDLE JOYSTICK
    joycon_virtual_stick_clockwise(0x00, right_payload[3], &stick_x, &stick_y);
    input_report_abs(jdata->input, ABS_Z, stick_x);
    input_report_abs(jdata->input, ABS_RZ, stick_y);
	input_report_abs(jdata->input, ABS_HAT0Y,
		(left_payload[1] & 0x02 ? 1 : 0) + (left_payload[1] & 0x04 ? -1 : 0));
	input_report_abs(jdata->input, ABS_HAT0X,
		(left_payload[1] & 0x01 ? -1 : 0) + (left_payload[1] & 0x08 ? 1 : 0));
	input_report_abs(jdata->input, ABS_HAT2X, (left_payload[2] & 0x80 ? 512 : -512));
	input_report_abs(jdata->input, ABS_HAT2Y, (right_payload[2] & 0x80 ? 512 : -512));
    
	input_sync(jdata->input);
}

static void joycon_load(struct joycon_data *jdata) {
    unsigned long flags;
    int ret = 0;

    jdata->input = input_allocate_device();
    if (!jdata->input)
        return;

    input_set_drvdata(jdata->input, jdata);
    jdata->input->dev.parent = &jdata->hdev->dev;
    jdata->input->id.bustype = jdata->hdev->bus;
    jdata->input->id.vendor = jdata->hdev->vendor;
    jdata->input->id.product = jdata->hdev->product;
    jdata->input->id.version = jdata->hdev->version;
    jdata->input->name = joycon_name(jdata);

    joycon_init_input(jdata);

    ret = input_register_device(jdata->input);
    if (ret)
        goto error;

    spin_lock_irqsave(&jdata_lock, flags);
    switch(joycon_devtype(jdata)) {
        case JOYCON_DEV_JOYCON_L:
            jdata_left = jdata;
            break;
        case JOYCON_DEV_JOYCON_R:
            jdata_right = jdata;
            break;
        default:
            break;
    }
    spin_unlock_irqrestore(&jdata_lock, flags);

    return;

error:

    if (jdata->input) {
        input_free_device(jdata->input);
        jdata->input = NULL;
    }
}

static void joycon_unload(struct joycon_data *jdata)
{
    unsigned long flags;
    spin_lock_irqsave(&jdata_lock, flags);
    switch(joycon_devtype(jdata)) {
        case JOYCON_DEV_JOYCON_L:
            jdata_left = NULL;
            break;
        case JOYCON_DEV_JOYCON_R:
            jdata_right = NULL;
            break;
        default:
            break;
    }
    if((NULL == jdata_left) && (NULL == jdata_right)) {
        printk("============================\n");
        printk("COMBINE INTO LEFT CONTROLLER\n");
        jdata_hold = JOYCON_HOLD_COMBINLE_INTO_L;
    }
    spin_unlock_irqrestore(&jdata_lock, flags);

    if (jdata->input) {
        input_get_device(jdata->input);
        input_unregister_device(jdata->input);
    }

    if (jdata->input) {
        input_put_device(jdata->input);
        jdata->input = NULL;
    }
}

static void joycon_handle_input_3f(struct joycon_data *jdata, const __u8 *payload)
{
    struct input_dev *input = NULL;
    const __u8 *left_payload = NULL;
    const __u8 *right_payload = NULL;
    const __u8 zero_payload[12] = {0x3f, 0x00, 0x00, 0x08, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80};

    input = jdata->input;
    left_payload = zero_payload;
    right_payload = zero_payload;

    memcpy(jdata->raw_data, payload, 12);

    if (JOYCON_HOLD_INDIVIDUAL != jdata_hold) {
        if (NULL != jdata_left) {
            left_payload = jdata_left->raw_data;
        }
        if (NULL != jdata_right) {
            right_payload = jdata_right->raw_data;
        }
    }

    if (JOYCON_HOLD_COMBINLE_INTO_L == jdata_hold) {
        // MODE on JOYCON (L)
        if ((left_payload[1] & 0x20 && left_payload[1] & 0x10) || (right_payload[1] & 0x20 && right_payload[1] & 0x10)) {
            printk("----------------------\n");
            printk("INDIVIDUAL CONTROLLERS\n");
            jdata_hold = JOYCON_HOLD_INDIVIDUAL;
        }
    }

    switch(joycon_devtype(jdata)) {
        case JOYCON_DEV_JOYCON_L:
            left_payload = payload;
            if (JOYCON_HOLD_COMBINLE_INTO_R == jdata_hold) {
                if (NULL != jdata_right) {
                    // redirected input to right controller
                    jdata = jdata_right;
                }
            }
            break;
        case JOYCON_DEV_JOYCON_R:
            right_payload = payload;
            if (JOYCON_HOLD_COMBINLE_INTO_L == jdata_hold) {
                if (NULL != jdata_left) {
                    // redirected input to left controller
                    jdata = jdata_left;
                }
            }
            break;
        default:
            break;
    }
    if (NULL == input) {
        return;
    }

    if (JOYCON_HOLD_COMBINLE_INTO_L == jdata_hold) {
        joycon_handle_input_3f_combine(jdata, left_payload, right_payload);
    } else if (JOYCON_DEV_JOYCON_L == joycon_devtype(jdata)) {
        joycon_handle_input_3f_left(jdata, left_payload);
    } else {
        joycon_handle_input_3f_right(jdata, right_payload);
    }
}

static int joycon_hid_event(struct hid_device *hdev, struct hid_report *report,
                            u8 *raw_data, int size)
{
    struct joycon_data *jdata = hid_get_drvdata(hdev);
    int i;
    unsigned long flags;

    if (size < 1)
        return -EINVAL;

    spin_lock_irqsave(&jdata_lock, flags);
    switch(raw_data[0]) {
        case 0x3f:
            if (size >= 12) {
                joycon_handle_input_3f(jdata, raw_data);
            }
            break;
    }
    spin_unlock_irqrestore(&jdata_lock, flags);

    return 0;
}

static ssize_t joycon_dev_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    struct joycon_data *jdata = dev_to_joy(dev);
    switch (joycon_devtype(jdata)) {
        case JOYCON_DEV_JOYCON_L:
            return sprintf(buf, "joyconL\n");
        case JOYCON_DEV_JOYCON_R:
            return sprintf(buf, "joyconR\n");
        default:
            break;
    }
    return sprintf(buf, "unknown\n");
}

static DEVICE_ATTR(devtype, S_IRUGO, joycon_dev_show, NULL);

static struct joycon_data *joycon_create(struct hid_device *hdev)
{
    __u8 normal[12] = {0x3f, 0x00, 0x00, 0x08, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80};
    struct joycon_data *jdata;

    jdata = kzalloc(sizeof(*jdata), GFP_KERNEL);
    if (!jdata)
        return NULL;

    memcpy(jdata->raw_data, normal, 12);
    jdata->hdev = hdev;
    hid_set_drvdata(hdev, jdata);

    return jdata;
}

static void joycon_destroy(struct joycon_data *jdata)
{
    device_remove_file(&jdata->hdev->dev, &dev_attr_devtype);

    joycon_unload(jdata);
    hid_hw_close(jdata->hdev);
    hid_hw_stop(jdata->hdev);

    kfree(jdata);
}

static int joycon_hid_probe(struct hid_device *hdev,
                const struct hid_device_id *id)
{
    struct joycon_data *jdata;
    int ret;

    hdev->quirks |= HID_QUIRK_NO_INIT_REPORTS;

    jdata = joycon_create(hdev);
    if (!jdata) {
        hid_err(hdev, "Can't alloc device\n");
        return -ENOMEM;
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

    ret = device_create_file(&hdev->dev, &dev_attr_devtype);
    if (ret) {
        hid_err(hdev, "cannot create sysfs attribute\n");
        goto err_stop;
    }

    hid_info(hdev, "New device registered\n");

    joycon_load(jdata);

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

    hid_info(hdev, "Device removed\n");
    joycon_destroy(jdata);
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
