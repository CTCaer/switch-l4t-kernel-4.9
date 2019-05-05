/*
 *  Copyright (c) 2018 Max Thomas
 */

/*
 * Nintendo Joy-Con serial gamepad driver for Linux
 */

/*
 * This program is free warftware; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *  Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/tty.h>
#include <linux/serdev.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/crc8.h>
#include <linux/power_supply.h>
#include <linux/gpio/consumer.h>

#define DRIVER_DESC	"Nintendo Joy-Con serial gamepad driver"

MODULE_AUTHOR("Max Thomas <mtinc2@gmail.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");


/*
 * Constants.
 */


static const u8 switch_baud[0x14] = {0x19, 0x01, 0x03, 0x0F, 0x00, 0x91, 0x20, 0x08, 0x00, 0x00, 0xBD, 0xB1, 0xC0, 0xC6, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u8 controller_status[0xD] = {0x19, 0x01, 0x03, 0x08, 0x00, 0x92, 0x00, 0x01, 0x00, 0x00, 0x69, 0x2D, 0x1F};
static const u8 unk_3[0x10] = {0x19, 0x01, 0x03, 0x0B, 0x00, 0x91, 0x12, 0x04, 0x00, 0x00, 0x12, 0xA6, 0x0F, 0x00, 0x00, 0x00};
 
#define JOYCON_COMMAND_EXTSEND   (0x91)
#define JOYCON_COMMAND_EXTRET    (0x92)
#define JOYCON_COMMAND_INITRET   (0x94)
#define JOYCON_COMMAND_HANDSHAKE (0xA5)

#define JOYCON_INIT_MAC      (0x1)
#define JOYCON_INIT_BAUDRATE (0x20)
#define JOYCON_INIT_UNK1     (0x11)
#define JOYCON_INIT_UNK2     (0x10)
#define JOYCON_INIT_UNK3     (0x12)

#define JOYCON_EXT_ACK        (0x1F)
#define JOYCON_EXT_INPUT      (0x30)
#define JOYCON_EXT_HIDCOMMAND (0x21)

#define JOYCON_HID_DEVICE_INFO (0x2)
#define JOYCON_HID_SPI_READ    (0x10)
#define JOYCON_HID_GET_REGVOLT (0x50)

#define JOYCON_SPI_FACTORY_CAL_2 (0x603D)
#define JOYCON_SPI_FACTORY_CAL_3 (0x6080)
#define JOYCON_SPI_STICK_USERCAL (0x8010)

#define JOYCON_BATT_MV_MIN (3300)
#define JOYCON_BATT_MV_MAX (4200)

#define JOYCON_BUTTONS_LEFT  (0xFFE900)
#define JOYCON_BUTTONS_RIGHT (0x76FF)
#define JOYCON_BUTTONS_ALL (JOYCON_BUTTONS_LEFT | JOYCON_BUTTONS_RIGHT)

#define JOYCON_BUTTONSET_LEFT  BIT(0)
#define JOYCON_BUTTONSET_RIGHT BIT(1)

#define JOYCON_CRC8_POLY (0x8D)
#define JOYCON_CRC8_INIT (0x00)

typedef struct joycon_uart_initial
{
	u8 magic[3];
	u8 total_size;
	u8 pad;
} joycon_uart_initial;

typedef struct joycon_uart_header
{
	joycon_uart_initial initial;
	u8 command;
	u8 data[5];
	u8 crc;
} joycon_uart_header;

typedef struct joycon_subcmd_packet
{
	joycon_uart_header pre;
	u8 command;
	u8 data[38];
} joycon_subcmd_packet;

struct joycon_stick_calib
{
	u16 x_max;
	u16 y_max;
	u16 x_cent;
	u16 y_cent;
	u16 x_min; 
	u16 y_min;	
};

/* Joy-Con button mappings */
static const signed short joycon_common_btn[] = {
	BTN_Y, BTN_X, BTN_B, BTN_A,
	BTN_0, BTN_1, BTN_TR, BTN_TR2,
	BTN_SELECT, BTN_START, 
	BTN_THUMBR, BTN_THUMBL,
	BTN_MODE, BTN_Z, 
	BTN_4, BTN_5, /* These two buttons do not exist on retail controllers. */
	BTN_DPAD_DOWN, BTN_DPAD_UP, BTN_DPAD_RIGHT, BTN_DPAD_LEFT, 
	BTN_2, BTN_3, BTN_TL, BTN_TL2,
	-1 /* terminating entry */
};

/* Driver data */
struct joycon_device {
	struct list_head	list;

	struct device		*dev;

	const char		*name;
	int			irq;
};

/*
 * Per-Joy-Con data.
 */

struct joycon {
	struct serdev_device *serdev;
	struct input_dev *input_dev;
	struct power_supply *batt_dev;
	struct device *dev;
	struct power_supply_desc batt_desc;
	struct gpio_desc *charge_gpio;
	
	bool handshaken;
	bool initialized;
	bool factory_cal_2_parsed;
	bool factory_cal_3_parsed;
	bool usercal_parsed;
	
	int timeout_samples;
	int num_samples;
	int reconnect_threshold;
	
	u8 button_set;
	struct joycon_stick_calib l_calib;
	struct joycon_stick_calib r_calib;
	
	struct workqueue_struct *wq;
	
	int idx;
	u16 regvolt;
	u32 buttons;
	u8 mac[6];
};

/*
 * Globals
 */
static DEFINE_MUTEX(joycon_device_lock);
static DEFINE_MUTEX(joycon_input_lock);
static LIST_HEAD(joycon_device_list);

static u8 crc_table[CRC8_TABLE_SIZE];

static struct serdev_device_ops joycon_ops;
static struct input_dev *input_dev;
static u32 buttons;
static int stick_lx, stick_ly, stick_rx, stick_ry;
static int joycon_increment;


struct work_timeout_poll {
	struct delayed_work work;
	struct joycon* joycon;
};

struct work_input_poll {
	struct delayed_work work;
	struct joycon* joycon;
};

struct work_sync_poll {
	struct delayed_work work;
	struct joycon* joycon;
};

u8 joycon_crc8(u8* data, size_t len)
{
	return crc8(crc_table, data, len, JOYCON_CRC8_INIT);
}

/*
 * Battery stuff
 */

static enum power_supply_property joycon_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static int joycon_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct joycon *joycon = power_supply_get_drvdata(psy);
	u32 regvolt_scaled, total_scaled;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		regvolt_scaled = ((joycon->regvolt - JOYCON_BATT_MV_MIN) * 100);
		total_scaled = (JOYCON_BATT_MV_MAX - JOYCON_BATT_MV_MIN);
		val->intval = regvolt_scaled / total_scaled;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = joycon->regvolt * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = JOYCON_BATT_MV_MAX * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = JOYCON_BATT_MV_MIN * 1000;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}


/*
 * Packet crafting
 */

u16 joycon_packet_craft(u8 *out, u8 command, u8 *data, u16 size)
{
	u16 packet_size = sizeof(joycon_uart_header);
	joycon_uart_header *pre = (joycon_uart_header*)out;

	pre->initial.magic[0] = 0x19;
	pre->initial.magic[1] = 0x01;
	pre->initial.magic[2] = 0x3;

	pre->initial.total_size = 7;
	pre->command = command;

	if (data)
		memcpy(pre->data, data, size);

	pre->crc = joycon_crc8((void*)pre + 0x4, 0x7);

	return packet_size;
}

u16 joycon_extcommand_craft(u8 *out, u8 command, u8 *data, u16 size)
{
	u16 packet_size;
	struct
	{
		u8 size[2];
	} subcmd_data;

	joycon_subcmd_packet* subcmd_packet = (joycon_subcmd_packet*)out;

	subcmd_data.size[0] = (size + sizeof(u8)) >> 8;
	subcmd_data.size[1] = (size + sizeof(u8)) & 0xFF;
	packet_size = joycon_packet_craft(out, 0x92, (u8*)&subcmd_data, sizeof(subcmd_data));

	subcmd_packet->pre.initial.total_size += size;
	subcmd_packet->command = command;
	if (data)
		memcpy(subcmd_packet->data, data, size);

	packet_size += size + sizeof(u8);
	return packet_size;
}

void joycon_send_command(struct serdev_device *serdev, u8 command, u8 *data, u16 size)
{
	//TODO: memory corruption somewhere
	u8 temp[0x100]; //TODO: why did I make this static again
	memset(temp, 0, sizeof(temp));

	size = joycon_packet_craft(temp, command, data, size);

	//print_bytes(temp, size);
	serdev_device_write(serdev, temp, size, msecs_to_jiffies(200));
}

void joycon_send_extcommand(struct serdev_device *serdev, u8 command, u8 *data, u16 size)
{
	//u8 *temp = devm_kzalloc(&serdev->dev, size+0x1000, GFP_KERNEL);
	u8 temp[0x100];
	memset(temp, 0, sizeof(temp));

	//TODO: memory corruption somewhere
	size = joycon_extcommand_craft(temp, command, data, size);

	//print_bytes(temp, size);
	serdev_device_write(serdev, temp, size, msecs_to_jiffies(200));
	//kfree(temp);
}

void joycon_send_hidcommand(struct serdev_device *serdev, u8 command, u8 *data, u16 size)
{
	u8 data_gen[0x100];
	u8 rumble[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};

	memset(data_gen, 0, sizeof(data_gen));

	//TODO: memory corruption somewhere
	//u8 *data_gen = devm_kzalloc(&serdev->dev, size+10+0x1000, GFP_KERNEL);
	data_gen[0] = 0; //inc
	memcpy(&data_gen[1], rumble, sizeof(rumble));
	data_gen[9] = command;
	
	if (data)
		memcpy(&data_gen[10], data, size);

	joycon_send_extcommand(serdev, 1, data_gen, 0x30);
	//kfree(data_gen);
}

/*
 * Handlers
 */
 
static void timeout_handler(struct work_struct *work)
{
	struct work_timeout_poll * data = (struct work_timeout_poll *)work;
	struct joycon *joycon = data->joycon;
	struct serdev_device *serdev = joycon->serdev;
	int err;
	
	// Inactive for 600ms, try handshaking again
	if (joycon->num_samples - joycon->timeout_samples == 0)
		joycon->reconnect_threshold++;
	else
		joycon->reconnect_threshold = 0;
	
	// We haven't gotten any samples within 600ms, we've probably been disconnected.
	if (joycon->reconnect_threshold > 3)
	{
		joycon->handshaken = false;
		joycon->initialized = false;
		joycon->regvolt = 0;
		gpiod_set_value(joycon->charge_gpio, 0);

		err = serdev_device_write(serdev, (u8[]){0xA1, 0xA2, 0xA3, 0xA4}, 4, msecs_to_jiffies(200));
		if (err < 0) goto done;

		joycon_send_command(serdev, JOYCON_COMMAND_HANDSHAKE, (u8[]){0x02, 0x01, 0x7E}, 3); //TODO: error passing
		
		serdev_device_write_flush(serdev);

		joycon->reconnect_threshold = 0;
		printk("Sent handshake\n");
	}
	
	if (!joycon->initialized) goto done;
	
	joycon_send_hidcommand(serdev, JOYCON_HID_GET_REGVOLT, NULL, 0);

done:
	joycon->timeout_samples = joycon->num_samples;
	INIT_DELAYED_WORK(&data->work,timeout_handler);
	queue_delayed_work(joycon->wq, &data->work, msecs_to_jiffies(200));
}

static void input_handler(struct work_struct *work)
{
	struct work_input_poll * data = (struct work_input_poll *)work;
	struct joycon *joycon = data->joycon;
	
	if (!joycon->initialized) goto done;
	serdev_device_write(joycon->serdev, controller_status, sizeof(controller_status), msecs_to_jiffies(200));

	if (!joycon->button_set)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_DEVICE_INFO, NULL, 0);

	if (joycon->button_set && !joycon->factory_cal_2_parsed)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SPI_READ, (u8[]){0x3D, 0x60, 0x00, 0x00, 0x1E}, 5); //TODO nicer

	if (joycon->factory_cal_2_parsed && !joycon->usercal_parsed)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SPI_READ, (u8[]){0x10, 0x80, 0x00, 0x00, 0x2F}, 5); //TODO nicer

	if (joycon->factory_cal_2_parsed && !joycon->factory_cal_3_parsed)
		joycon_send_hidcommand(joycon->serdev, JOYCON_HID_SPI_READ, (u8[]){0x80, 0x60, 0x00, 0x00, 0x17}, 5); //TODO nicer

done:
	INIT_DELAYED_WORK(&data->work, input_handler);
	queue_delayed_work(joycon->wq, &data->work, msecs_to_jiffies(16));
}

static void sync_handler(struct work_struct *work)
{
	struct work_sync_poll * data = (struct work_sync_poll *)work;
	struct joycon *joycon = data->joycon;
	int i;
	
	mutex_lock(&joycon_input_lock);
	for (i = 0; joycon_common_btn[i] >= 0; i++)
	{
		input_report_key(input_dev, joycon_common_btn[i], buttons & BIT(i));
	}
	
	input_report_abs(input_dev, ABS_X, stick_lx);
	input_report_abs(input_dev, ABS_Y, stick_ly);
	input_report_abs(input_dev, ABS_RX, stick_rx);
	input_report_abs(input_dev, ABS_RY, stick_ry);
	input_sync(input_dev);
	mutex_unlock(&joycon_input_lock);
	
	INIT_DELAYED_WORK(&data->work, sync_handler);
	queue_delayed_work(joycon->wq, &data->work, msecs_to_jiffies(10));
}

/*
 * Packet parsing
 */

static void joycon_parse_stick_cal(struct joycon_stick_calib *calib, const u8* packed)
{
	//min->max cent->min max->cent

	calib->x_cent = ((packed[1] << 8) & 0xF00) | packed[0];
	calib->y_cent = (packed[2] << 4) | (packed[1] >> 4);
	calib->x_min = ((packed[4] << 8) & 0xF00) | packed[3];
	calib->y_min = (packed[5] << 4) | (packed[4] >> 4);
	calib->x_max = ((packed[7] << 8) & 0xF00) | packed[6];
	calib->y_max = (packed[8] << 4) | (packed[7] >> 4);
}

static void joycon_calib_absinfo(struct input_absinfo *absinfo, unsigned int ax, 
				 unsigned int ay, struct joycon_stick_calib *calib)
{
	absinfo[ax].minimum = calib->x_cent - calib->x_min;
	absinfo[ax].maximum = calib->x_cent + calib->x_max;
	absinfo[ay].minimum = calib->y_cent - calib->y_min;
	absinfo[ay].maximum = calib->y_cent + calib->y_max;
}

static void joycon_parse_spi_read(struct serdev_device *serdev, const u8* packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	u32 addr = (packet[4] << 24) | (packet[3] << 16) | (packet[2] << 8)
	           | packet[1];
	u8 bytes_read = packet[5];
	
	if (addr == JOYCON_SPI_FACTORY_CAL_2 && bytes_read >= 0x1E)
	{
		joycon_parse_stick_cal(&joycon->l_calib, packet + 5);
		joycon_parse_stick_cal(&joycon->r_calib, packet + 14);
		
		//TODO: Colors
		
		if (joycon->button_set & JOYCON_BUTTONSET_LEFT)
		{
			joycon_calib_absinfo(input_dev->absinfo, ABS_X, ABS_Y, 
					     &joycon->l_calib);
			printk("Parsed left joystick configs\n");
		}
		
		if (joycon->button_set & JOYCON_BUTTONSET_RIGHT)
		{
			joycon_calib_absinfo(input_dev->absinfo, ABS_RX, ABS_RY, 
					     &joycon->r_calib);
			printk("Parsed right joystick configs\n");
		}
		
		joycon->factory_cal_2_parsed = true;
	}
	else if (addr == JOYCON_SPI_STICK_USERCAL)
	{
		if (packet[6] == 0xB2 && packet[7] == 0xA1 
		    && joycon->button_set & JOYCON_BUTTONSET_LEFT)
		{
			joycon_parse_stick_cal(&joycon->l_calib, packet + 7);
			joycon_calib_absinfo(input_dev->absinfo, ABS_X, ABS_Y, 
					     &joycon->l_calib);
			printk("Parsed left joystick user configs\n");
		}
			
		if (packet[16] == 0xB2 && packet[17] == 0xA1
		    && joycon->button_set & JOYCON_BUTTONSET_RIGHT)
		{
			joycon_parse_stick_cal(&joycon->r_calib, packet + 17);
			joycon_calib_absinfo(input_dev->absinfo, ABS_RX, ABS_RY, 
					     &joycon->r_calib);
			printk("Parsed right joystick user configs\n");
		}

		//TODO: six-axis

		joycon->usercal_parsed = true;	
	}
	else if (addr == JOYCON_SPI_FACTORY_CAL_3)
	{
		u16 dead_zone = packet[6+4] | (packet[6+5] << 8);
		printk("Dead zone %x\n", dead_zone);
		
		/*if (joycon->button_set & JOYCON_BUTTONSET_LEFT)
		{
			input_dev->absinfo[ABS_X].flat = dead_zone;
			input_dev->absinfo[ABS_Y].flat = dead_zone;
		}
		
		if (joycon->button_set & JOYCON_BUTTONSET_RIGHT)
		{
			input_dev->absinfo[ABS_RX].flat = dead_zone;
			input_dev->absinfo[ABS_RY].flat = dead_zone;
		}*/
		
		joycon->factory_cal_3_parsed = true;
	}
}

static void joycon_hidret_parse(struct serdev_device *serdev, const u8* packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	switch (packet[0])
	{
	case JOYCON_HID_DEVICE_INFO:
		printk("Joy-Con firmware version %X.%02X\n", packet[1], packet[2]);
		joycon->button_set = packet[3];
		if (packet[3] == 1)
			printk("Joy-Con is a left controller\n");
		else if (packet[3] == 2)
			printk("Joy-Con is a right controller\n");
		else if (packet[3] == 3)
			printk("Joy-Con is a pro controller\n");
		else
			printk("Joy-Con is unknown\n");
		break;
	case JOYCON_HID_SPI_READ:
		joycon_parse_spi_read(serdev, packet, size);
		break;
	case JOYCON_HID_GET_REGVOLT:
		// The actual regulated battery voltage is the raw * 2.5
		joycon->regvolt = packet[1] | packet[2] << 8;
		joycon->regvolt *= 5;
		joycon->regvolt /= 2;
		break;
	default:
		printk("Unknown hidret %x\n", packet[0]);
		break;
	}
	
	//print_bytes(packet, size);
}

static void joycon_extret_parse(struct serdev_device *serdev, const u8* packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	switch (packet[0])
	{
	case JOYCON_EXT_INPUT:
		joycon->buttons = packet[3] | packet[4] << 8 | packet[5] << 16;
		
		mutex_lock(&joycon_input_lock);
		if (joycon->button_set & JOYCON_BUTTONSET_LEFT)
		{
			buttons &= ~JOYCON_BUTTONS_LEFT;;
			buttons |= (joycon->buttons & JOYCON_BUTTONS_LEFT);
		}
		if (joycon->button_set & JOYCON_BUTTONSET_RIGHT)
		{
			buttons &= ~JOYCON_BUTTONS_RIGHT;
			buttons |= (joycon->buttons & JOYCON_BUTTONS_RIGHT);
		}

		if (joycon->button_set & JOYCON_BUTTONSET_LEFT)
		{
			stick_lx = packet[6+0] | ((packet[6+1] & 0xF) << 8);
			stick_ly = (packet[6+1] >> 4) | (packet[6+2] << 4);
		}
		
		if (joycon->button_set & JOYCON_BUTTONSET_RIGHT)
		{
			stick_rx = packet[9+0] | ((packet[9+1] & 0xF) << 8);
			stick_ry = (packet[9+1] >> 4) | (packet[9+2] << 4);
		}
		mutex_unlock(&joycon_input_lock);
			

		joycon->num_samples++;
		break;
	case JOYCON_EXT_HIDCOMMAND:
		joycon_hidret_parse(serdev, packet+14, size-14);
		break;
	default:
		printk("Unknown extret %x\n", packet[0]);
		break;
	}
}

static void joycon_initret_parse(struct serdev_device *serdev, const u8* packet, u32 size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	int i, j;

	switch (packet[0])
	{
	case JOYCON_INIT_MAC:
		for (i = 0xC, j = 0; i >= 0x6; i--)
			joycon->mac[j++] = packet[i];
		printk("Joy-Con with MAC %02X::%02X::%02X::%02X::%02X::%02X\n", 
			joycon->mac[0], joycon->mac[1], joycon->mac[2], 
			joycon->mac[3], joycon->mac[4], joycon->mac[5]);
			
		// Next init command
		//serdev_device_write(serdev, switch_baud, sizeof(switch_baud), msecs_to_jiffies(200));
		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_UNK1}, 1);
		break;
	case JOYCON_INIT_UNK1:
		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_UNK2}, 1);
		break;
	case JOYCON_INIT_UNK2:
		serdev_device_write(serdev, unk_3, sizeof(unk_3), msecs_to_jiffies(200));
		break;
	case JOYCON_INIT_UNK3:
		joycon->button_set = 0;
		joycon->num_samples = 0;
		joycon->factory_cal_2_parsed = false;
		joycon->factory_cal_3_parsed = false;
		joycon->usercal_parsed = false;
		joycon->initialized = true;
		
		gpiod_set_value(joycon->charge_gpio, 1);
		break;
	case JOYCON_INIT_BAUDRATE:
		serdev_device_set_baudrate(serdev, 3000000);
		printk("High-speed baudrate shifted...\n");
		break;
	default:
		printk("Unknown initret %x\n", packet[0]);
		break;
	}
}

static void joycon_packet_parse(struct serdev_device *serdev, const u8* packet, size_t size)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	joycon_uart_header* header = (joycon_uart_header*) packet;

	switch (header->command)
	{
	case JOYCON_COMMAND_EXTRET:
		joycon_extret_parse(serdev, packet + sizeof(joycon_uart_header), (header->data[0] << 8) | header->data[1]);
		break;
	case JOYCON_COMMAND_INITRET:
		joycon_initret_parse(serdev, packet + sizeof(joycon_uart_initial) + 1, size - sizeof(joycon_uart_initial) - 1);
		break;
	case JOYCON_COMMAND_HANDSHAKE:
		printk("Got handshake response\n");
		joycon->handshaken = true;
		
		// Get MAC
		joycon_send_command(serdev, JOYCON_COMMAND_EXTSEND, (u8[]){JOYCON_INIT_MAC}, 1);
		break;
	default:
		printk("Unknown command %x\n", header->command);
		break;
	}
}

static int joycon_serdev_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t len)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);

	if (!joycon || serdev != joycon->serdev) {
		WARN_ON(1);
		return 0;
	}
	
	joycon_packet_parse(serdev, buf, len);

	return len;
}

static void joycon_serdev_write_wakeup(struct serdev_device *serdev)
{
	//printk("Wakeup\n");
}

/*
 * Serdev driver
 */

static int joycon_serdev_probe(struct serdev_device *serdev)
{
	struct joycon *joycon;
	
	struct work_timeout_poll *work_timeout;
	struct work_input_poll *work_input;
	struct work_sync_poll *work_sync;
	struct power_supply_config psy_cfg = {};
	int i;
	int err = -ENOMEM;

	printk("Joy-Con probe!\n");

	joycon = devm_kzalloc(&serdev->dev, sizeof(*joycon), GFP_KERNEL);
	if (!joycon)
		goto fail2;

	joycon->wq = create_workqueue("joycon_wq");

	if (input_dev) goto input_exists;

	// TODO: For USB/BT this should be an array w/ Joy-Con 
	//       being matched to input devices by halves
	input_dev = input_allocate_device();
	if (!input_dev) goto fail2;
	
	input_dev->name = "Joy-Con Rails";
	input_dev->id.bustype = BUS_VIRTUAL;
	input_dev->id.vendor = 0x057E;
	input_dev->id.product = 0x2008;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	
	/* set up standard buttons */
	for (i = 0; joycon_common_btn[i] >= 0; i++)
		__set_bit(joycon_common_btn[i], input_dev->keybit);

	//TODO: dead zones
	input_set_abs_params(input_dev, ABS_X, 0x400, 0xC00, 0, 180);
	input_set_abs_params(input_dev, ABS_Y, 0x400, 0xC00, 0, 180);
	input_set_abs_params(input_dev, ABS_RX, 0x400, 0xC00, 0, 180);
	input_set_abs_params(input_dev, ABS_RY, 0x400, 0xC00, 0, 180);
	
	err = input_register_device(input_dev);
	if (err) goto fail1;
	
	// Start input sync polling
	work_sync = kmalloc(sizeof(struct work_sync_poll), GFP_KERNEL);
	INIT_DELAYED_WORK(&work_sync->work, sync_handler);
	work_sync->joycon = joycon;
	queue_delayed_work(joycon->wq, &work_sync->work, msecs_to_jiffies(10));

input_exists:

	psy_cfg.of_node = serdev->dev.of_node;
	psy_cfg.drv_data = joycon;
	
	joycon->batt_desc.name = kasprintf(GFP_KERNEL, "joycon_battery_%u", joycon_increment++);
	joycon->batt_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	joycon->batt_desc.properties = joycon_battery_props;
	joycon->batt_desc.num_properties = ARRAY_SIZE(joycon_battery_props);
	joycon->batt_desc.get_property = joycon_battery_get_property;
	
	joycon->batt_dev = devm_power_supply_register(&serdev->dev, &joycon->batt_desc,
					              &psy_cfg);

	err = PTR_ERR_OR_ZERO(joycon->batt_dev);
	if (err) {
		dev_err(&serdev->dev, "failed to register power supply\n");
		goto fail1;
	}

	joycon->serdev = serdev;
	joycon->dev = &serdev->dev;
	joycon->input_dev = input_dev;

	serdev_device_set_drvdata(serdev, joycon);

	//TODO: error checking	
	serdev_device_open(serdev);
	serdev_device_set_flow_control(serdev, true);
	serdev_device_set_baudrate(serdev, 1000000);
	
	serdev_device_set_client_ops(serdev, &joycon_ops);
	
	// Get charging GPIO
	joycon->charge_gpio = devm_gpiod_get(&serdev->dev, "charge", GPIOD_OUT_LOW);
	if (IS_ERR(joycon->charge_gpio)) {
		err = PTR_ERR(joycon->charge_gpio);
		dev_err(&serdev->dev, "cannot get charge-gpio %d\n", err);
		goto fail3;
	}
	
	// Set up work polling
	work_timeout = kmalloc(sizeof(struct work_timeout_poll), GFP_KERNEL);
	INIT_DELAYED_WORK(&work_timeout->work, timeout_handler);
	work_timeout->joycon = joycon;
	queue_delayed_work(joycon->wq, &work_timeout->work, msecs_to_jiffies(200));
	
	work_input = kmalloc(sizeof(struct work_input_poll), GFP_KERNEL);
	work_input->joycon = joycon;
	INIT_DELAYED_WORK(&work_input->work, input_handler);
	queue_delayed_work(joycon->wq, &work_input->work, msecs_to_jiffies(200));

	return 0;

 fail3: serdev_device_set_drvdata(serdev, NULL);
 fail1:	input_free_device(input_dev);
 fail2:
	kfree(joycon);
	return err;
}

static void joycon_serdev_remove(struct serdev_device *serdev)
{
	struct joycon *joycon = serdev_device_get_drvdata(serdev);
	printk("Joy-Con remove!\n");
	
	input_unregister_device(joycon->input_dev);
	
	flush_workqueue(joycon->wq);
	destroy_workqueue(joycon->wq);
	
	kfree(joycon->batt_desc.name);
	kfree(joycon);
}

/*
 * The serdev driver structure.
 */
#ifdef CONFIG_OF
static const struct of_device_id joycon_uart_of_match[] = {
	{ .compatible = "nintendo,joycon-uart" },
	{ },
};
MODULE_DEVICE_TABLE(of, joycon_uart_of_match);
#endif

static struct serdev_device_driver joycon_serdev_driver = {
	.probe = joycon_serdev_probe,
	.remove = joycon_serdev_remove,
	.driver = {
		.name = "joycon-uart",
		.of_match_table = of_match_ptr(joycon_uart_of_match),
	},
};

static struct serdev_device_ops joycon_ops = {
	.receive_buf = joycon_serdev_receive_buf,
	.write_wakeup = joycon_serdev_write_wakeup,
};

/*
 * Joy-Con base driver
 */

static int joycon_probe(struct platform_device *pdev)
{
	struct joycon_device *dev;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	dev->irq = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, dev);

	dev_info(&pdev->dev, "%s device registered.\n", dev->name);

	/* Place this instance on the device list */
	mutex_lock(&joycon_device_lock);
	list_add_tail(&dev->list, &joycon_device_list);
	mutex_unlock(&joycon_device_lock);

	return 0;
}

static int joycon_remove(struct platform_device *pdev)
{
	struct joycon_device *dev = platform_get_drvdata(pdev);

	mutex_lock(&joycon_device_lock);
	list_del(&dev->list);
	mutex_unlock(&joycon_device_lock);

	dev_info(&pdev->dev, "%s device unregistered.\n", dev->name);

	return 0;
}

static struct platform_driver joycon_driver = {
	.probe = joycon_probe,
	.remove = joycon_remove,
	.driver = {
		.name = "joycon",
	},
};

int __init joycon_init(void)
{
	platform_driver_register(&joycon_driver);
	serdev_device_driver_register(&joycon_serdev_driver);
	
	joycon_increment = 0;
	crc8_populate_lsb(crc_table, JOYCON_CRC8_POLY);

	return 0;
}

void __exit joycon_exit(void)
{
	platform_driver_unregister(&joycon_driver);
	serdev_device_driver_unregister(&joycon_serdev_driver);
}

module_init(joycon_init);
module_exit(joycon_exit);
