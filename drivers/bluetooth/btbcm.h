/*
 *
 *  Bluetooth support for Broadcom devices
 *
 *  Copyright (C) 2015  Intel Corporation
 *  Copyright (C) 2022  CTCaer
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define BCM_SCO_PCM_ROUTING_PCM		0x00
#define BCM_SCO_PCM_ROUTING_TRS		0x01 /* Transport */
#define BCM_SCO_PCM_ROUTING_CDC		0x02 /* Codec */
#define BCM_SCO_PCM_ROUTING_I2S		0x03

#define BCM_SCO_PCM_RATE_128_KBPS	0x00
#define BCM_SCO_PCM_RATE_256_KBPS	0x01
#define BCM_SCO_PCM_RATE_512_KBPS	0x02
#define BCM_SCO_PCM_RATE_1024_KBPS	0x03
#define BCM_SCO_PCM_RATE_2048_KBPS	0x04

#define BCM_UART_CLOCK_48MHZ		0x01
#define BCM_UART_CLOCK_24MHZ		0x02

struct bcm_update_uart_sco_pcm {
	__u8 sco_routing;
	__u8 pcm_interface_rate;
	__u8 frame_type;
	__u8 sync_mode;
	__u8 clock_mode;
} __packed;

struct bcm_update_uart_sco_pcm_fmt {
	__u8 lsb_first;
	__u8 fill_bits;
	__u8 fill_method;
	__u8 fill_num;
	__u8 right_justify;
} __packed;

struct bcm_update_uart_baud_rate {
	__le16 zero;
	__le32 baud_rate;
} __packed;

struct bcm_write_uart_clock_setting {
	__u8 type;
} __packed;

struct bcm_set_sleep_mode {
	__u8 sleep_mode;
	__u8 idle_host;
	__u8 idle_dev;
	__u8 bt_wake_active;
	__u8 host_wake_active;
	__u8 allow_host_sleep;
	__u8 combine_modes;
	__u8 tristate_control;
	__u8 usb_auto_sleep;
	__u8 usb_resume_timeout;
	__u8 break_to_host;
	__u8 pulsed_host_wake;
} __packed;

struct bcm_set_pcm_int_params {
	__u8 routing;
	__u8 rate;
	__u8 frame_sync;
	__u8 sync_mode;
	__u8 clock_mode;
} __packed;

struct bcm_set_pcm_format_params {
	__u8 lsb_first;
	__u8 fill_value;
	__u8 fill_method;
	__u8 fill_num;
	__u8 right_justify;
} __packed;

#if IS_ENABLED(CONFIG_BT_BCM)

int btbcm_check_bdaddr(struct hci_dev *hdev);
int btbcm_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr);
int btbcm_patchram(struct hci_dev *hdev, const struct firmware *fw);

int btbcm_setup_patchram(struct hci_dev *hdev);
int btbcm_setup_apple(struct hci_dev *hdev);

int btbcm_initialize(struct hci_dev *hdev, char *fw_name, size_t len,
		     bool reinit);
int btbcm_finalize(struct hci_dev *hdev);

#else

static inline int btbcm_check_bdaddr(struct hci_dev *hdev)
{
	return -EOPNOTSUPP;
}

static inline int btbcm_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	return -EOPNOTSUPP;
}

static inline int btbcm_patchram(struct hci_dev *hdev, const struct firmware *fw)
{
	return -EOPNOTSUPP;
}

static inline int btbcm_setup_patchram(struct hci_dev *hdev)
{
	return 0;
}

static inline int btbcm_setup_apple(struct hci_dev *hdev)
{
	return 0;
}

static inline int btbcm_initialize(struct hci_dev *hdev, char *fw_name,
				   size_t len, bool reinit)
{
	return 0;
}

static inline int btbcm_finalize(struct hci_dev *hdev)
{
	return 0;
}

#endif
