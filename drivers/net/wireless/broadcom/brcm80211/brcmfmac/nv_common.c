/*
 * drivers/net/wireless/bcmdhd_pcie/dhd_custom_tegra.c
 *
 * Copyright (C) 2014-2018 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifdef CONFIG_BRCMFMAC_NV_CUSTOM_FILES
#include <linux/types.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/ieee80211.h>

#include "debug.h"
#include "core.h"
#include "common.h"

/* DT node used by all the features */
struct device_node *node;

#ifdef CONFIG_BRCMFMAC_NV_PRIV_CMD
#include <brcmu_wifi.h>
#include "fwil.h"
extern bool builtin_roam_disabled;
#endif /* CONFIG_BRCMFMAC_NV_PRIV_CMD */

#ifdef CONFIG_BRCMFMAC_NV_GPIO
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

void setup_gpio(struct platform_device *pdev, bool on) {
	struct device_node *dt_node;
	int ret;

	brcmf_dbg(INFO, "Enter\n");
	if (on) {
		dt_node = of_find_compatible_node(NULL, NULL, "brcm,android-fmac");
		if (dt_node)
			brcmf_dbg(INFO, "DT entry found\n");

		if (pdev->dev.of_node) {
			node = pdev->dev.of_node;
			brcmf_mp_global.wlan_pwr = of_get_named_gpio(node, "wlan-pwr-gpio", 0);
			brcmf_mp_global.wlan_rst = of_get_named_gpio(node, "wlan-rst-gpio", 0);

			if (gpio_is_valid(brcmf_mp_global.wlan_pwr)) {
			       ret = devm_gpio_request(&pdev->dev, brcmf_mp_global.wlan_pwr,
							"wlan_pwr");
				if(ret)
					brcmf_err("Failed to request wlan_pwr gpio\n");
			}

			if (gpio_is_valid(brcmf_mp_global.wlan_rst)) {
				ret = devm_gpio_request(&pdev->dev, brcmf_mp_global.wlan_rst,
						"wlan_rst");
				if(ret)
					brcmf_err("Failed to request wlan_rst gpio\n");
			}
		}
	}

#ifdef CONFIG_BRCMFMAC_NV_PRIV_CMD
	builtin_roam_disabled = device_property_read_bool(&pdev->dev, "builtin-roam-disabled");
#endif /* CONFIG_BRCMFMAC_NV_PRIV_CMD */

}

void toggle_gpio(bool on, unsigned long msec) {

	if (gpio_is_valid(brcmf_mp_global.wlan_pwr))
		gpio_direction_output(brcmf_mp_global.wlan_pwr, on);
	if (gpio_is_valid(brcmf_mp_global.wlan_rst))
		gpio_direction_output(brcmf_mp_global.wlan_rst, on);

	if (msec && on)
		msleep(msec);
}
#endif /* CONFIG_BRCMFMAC_NV_GPIO */

#ifdef CONFIG_BRCMFMAC_NV_CUSTOM_MAC
#define WIFI_MAC_ADDR_FILE "/mnt/factory/wifi/wifi_mac.txt"

static int wifi_get_mac_addr_file(unsigned char *buf)
{
	struct file *fp;
	int rdlen;
	char str[32];
	int mac[6];
	int ret = 0;

	/* open wifi mac address file */
	fp = filp_open(WIFI_MAC_ADDR_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		brcmf_err("%s: cannot open %s\n",
			__func__, WIFI_MAC_ADDR_FILE);
		return -ENOENT;
	}

	/* read wifi mac address file */
	memset(str, 0, sizeof(str));
	rdlen = kernel_read(fp, fp->f_pos, str, 17);
	if (rdlen > 0)
		fp->f_pos += rdlen;
	if (rdlen != 17) {
		brcmf_err("%s: bad mac address file - len %d != 17",
						__func__, rdlen);
		ret = -ENOENT;
	} else if (sscanf(str, "%x:%x:%x:%x:%x:%x",
		&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
		brcmf_err("%s: bad mac address file"
			" - must contain xx:xx:xx:xx:xx:xx\n",
			__func__);
		ret = -ENOENT;
	} else {
		brcmf_err("%s: using wifi mac %02x:%02x:%02x:%02x:%02x:%02x\n",
			__func__,
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		buf[0] = (unsigned char) mac[0];
		buf[1] = (unsigned char) mac[1];
		buf[2] = (unsigned char) mac[2];
		buf[3] = (unsigned char) mac[3];
		buf[4] = (unsigned char) mac[4];
		buf[5] = (unsigned char) mac[5];

	}

	if (!is_valid_ether_addr(buf)) {
		brcmf_err("%s: invalid mac %02x:%02x:%02x:%02x:%02x:%02x\n",
			__FUNCTION__,
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		ret = -EINVAL;
	}

	/* close wifi mac address file */
	filp_close(fp, NULL);

	return ret;
}

/* Get MAC address from the specified DTB path */
static int wifi_get_mac_address_dtb(const char *node_name,
					const char *property_name,
					unsigned char *mac_addr)
{
	struct device_node *np = of_find_node_by_path(node_name);
	const char *mac_str = NULL;
	int values[6] = {0};
	unsigned char mac_temp[6] = {0};
	int i, ret = 0;

	if (!np)
		return -EADDRNOTAVAIL;

	/* If the property is present but contains an invalid value,
	 * then something is wrong. Log the error in that case.
	 */
	if (of_property_read_string(np, property_name, &mac_str)) {
		ret = -EADDRNOTAVAIL;
		goto err_out;
	}

	/* The DTB property is a string of the form xx:xx:xx:xx:xx:xx
	 * Convert to an array of bytes.
	 */
	if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
		&values[0], &values[1], &values[2],
		&values[3], &values[4], &values[5]) != 6) {
		ret = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < 6; ++i)
		mac_temp[i] = (unsigned char)values[i];

	if (!is_valid_ether_addr(mac_temp)) {
		ret = -EINVAL;
		goto err_out;
	}

	memcpy(mac_addr, mac_temp, 6);

	of_node_put(np);

	return ret;

err_out:
	brcmf_err("%s: bad mac address at %s/%s: %s.\n",
		__func__, node_name, property_name,
		(mac_str) ? mac_str : "null");

	of_node_put(np);

	return ret;
}

static int wifi_get_mac_addr(unsigned char *buf)
{
	int ret = -ENODATA;

	/* The MAC address search order is:
	 * DTB (from NCT/EEPROM)
	 * NCT
	 * File (FCT/rootfs)
	*/
	ret = wifi_get_mac_address_dtb("/chosen", "nvidia,wifi-mac", buf);
	if (ret)
		ret = wifi_get_mac_addr_file(buf);

	return ret;
}

int nv_set_mac_address(struct net_device *ndev) {

	struct brcmf_if *ifp =  netdev_priv(ndev);
	int err = 0;

	/* retrieve mac addresses */
	err = wifi_get_mac_addr(ifp->mac_addr);
	if (err < 0) {
		brcmf_err("No custom MAC address found, %d\n", err);
		return err;
	} else {

		brcmf_err("%s: setting mac %02x:%02x:%02x:%02x:%02x:%02x\n",
			__FUNCTION__,
			ifp->mac_addr[0],
			ifp->mac_addr[1],
			ifp->mac_addr[2],
			ifp->mac_addr[3],
			ifp->mac_addr[4],
			ifp->mac_addr[5]);

		err = brcmf_fil_iovar_data_set(ifp,
			"cur_etheraddr", ifp->mac_addr,
				ETH_ALEN);
		if (err < 0) {
			brcmf_err("Setting custom MAC address failed, %d\n", err);
			return err;
		} else {
			brcmf_dbg(TRACE, "updated to %pM\n", ifp->mac_addr);
			memcpy(ifp->ndev->dev_addr, ifp->mac_addr, ETH_ALEN);
		}
	}

	return err;
}

#endif /* CONFIG_BRCMFMAC_NV_CUSTOM_MAC */

#ifdef CONFIG_BRCMFMAC_NV_COUNTRY_CODE
int wifi_platform_get_country_code_map(void)
{
	struct device_node *np_country;
	struct device_node *child;
	struct brcmf_fil_country_le *country;
	int n_country;
	int ret;
	int i;
	const char *strptr;

	np_country = of_get_child_by_name(node, "country_code_map");
	if (!np_country) {
		brcmf_err("could not get country_code_map\n");
		return -1;
	}

	n_country = of_get_child_count(np_country);
	if (!n_country) {
		brcmf_err("n_country\n");
		return -1;
	}

	country = kmalloc(n_country * sizeof(struct brcmf_fil_country_le), GFP_KERNEL);
	if (!country) {
		brcmf_err("fail to allocate memory\n");
	       return -1;
	}
	memset(country, 0, n_country * sizeof(struct brcmf_fil_country_le));

	i = 0;
	for_each_child_of_node(np_country, child) {
		ret = of_property_read_string(child, "iso_abbrev", &strptr);
		if (ret) {
			brcmf_err("read error iso_abbrev %s\n", child->name);
			goto fail;
		} else {
			strncpy(country[i].country_abbrev, strptr, 3);
		}

		ret = of_property_read_string(child, "custom_locale", &strptr);
		if (ret) {
		brcmf_err("read error iso_abbrev %s\n", child->name);
			goto fail;
		} else {
			strncpy(country[i].country_abbrev, strptr, 3);
		}

		ret = of_property_read_string(child, "custom_locale", &strptr);
		if (ret) {
			brcmf_err("read error custom_locale %s\n", child->name);
			goto fail;
		} else {
			strncpy(country[i].ccode, strptr, 3);
		}

		ret = of_property_read_u32(child, "custom_locale_rev", &country[i].rev);
		if (ret) {
			brcmf_err("read error custom_locale_rev %s\n", child->name);
			goto fail;
		}
		i++;
	}

	brcmf_mp_global.country_code_map = country;
	brcmf_mp_global.n_country = n_country;
	return 0;
fail:
	kfree(country);
	brcmf_mp_global.country_code_map = NULL;
	brcmf_mp_global.n_country = 0;
	return -1;
}

void wifi_platform_free_country_code_map(void)
{
	if (brcmf_mp_global.country_code_map) {
		kfree(brcmf_mp_global.country_code_map);
		brcmf_mp_global.country_code_map = NULL;
	}
	brcmf_mp_global.n_country = 0;
}
#endif /* CONFIG_BRCMFMAC_NV_COUNTRY_CODE */

#ifdef CONFIG_BRCMFMAC_NV_PRIV_CMD
int nv_brcmf_android_set_im_mode(struct brcmf_pub *drvr,
		struct net_device *ndev, char *command, u32 cmd_len)
{
	int mode = 0;
	int error = 0;
	struct brcmf_if *ifp =	netdev_priv(ndev);
	int ampdu_mpdu;
	int ampdu_rx_tid = -1;
	int i;
#ifdef VSDB_BW_ALLOCATE_ENABLE
	int mchan_algo;
	int mchan_bw;
#endif /* VSDB_BW_ALLOCATE_ENABLE */

	if (sscanf(command, "%*s %d", &mode) != 1) {
		brcmf_err("Cannot fetch th mode\n");
		return -1;
	}
set_mode:
	if (mode == 0) {
		/* Normal mode: restore everything to default */
#ifdef CUSTOM_AMPDU_MPDU
		ampdu_mpdu = CUSTOM_AMPDU_MPDU;
#else
		ampdu_mpdu = -1;	/* FW default */
#endif
#ifdef VSDB_BW_ALLOCATE_ENABLE
		mchan_algo = 0; /* Default */
		mchan_bw = 50;	/* 50:50 */
#endif /* VSDB_BW_ALLOCATE_ENABLE */
	} else if (mode == 1) {
		/* Miracast source mode */
		ampdu_mpdu = 8; /* for tx latency */
#ifdef VSDB_BW_ALLOCATE_ENABLE
		mchan_algo = 1; /* BW based */
		mchan_bw = 25;	/* 25:75 */
#endif /* VSDB_BW_ALLOCATE_ENABLE */
	}
	else if (mode == 2) {
		/* Miracast sink/PC Gaming mode */
		ampdu_mpdu = 8; /* FW default */
#ifdef VSDB_BW_ALLOCATE_ENABLE
		mchan_algo = 0; /* Default */
		mchan_bw = 50;	/* 50:50 */
#endif /* VSDB_BW_ALLOCATE_ENABLE */
	} else if (mode == 3) {
		ampdu_rx_tid = 0;
		mode = 2;
		goto set_mode;
	} else if (mode == 4) {
		ampdu_rx_tid = 0x7f;
		mode = 0;
		goto set_mode;
	} else if (mode == 5) {
		/* no interference mitigation 0 */
		error = brcmf_fil_cmd_int_set(ifp, BRCMF_C_SET_IM_MODE, 0);
		if (error) {
			brcmf_err("Failed to set interference_override: mode:%d, error:%d\n",
					mode, error);
			return -1;
		}
	} else if (mode == 6) {
		/* auto interference mitigation -1 */
		error = brcmf_fil_cmd_int_set(ifp, BRCMF_C_SET_IM_MODE, -1);

		if (error) {
			brcmf_err("Failed to set interference_override: mode:%d, error:%d\n",
					mode, error);
			return -1;
		}
	} else {

	}

	error =  brcmf_fil_iovar_int_set(ifp, "ampdu_mpdu", ampdu_mpdu);
	if (error) {
		 brcmf_err("Failed to set ampdu_mpdu: mode:%d, error:%d\n",
			mode, error);
		return -1;
	}

	if (ampdu_rx_tid != -1) {
		/* structure for per-tid ampdu control */
		struct ampdu_tid_control {
			u8 tid;			     /* tid */
			u8 enable;		     /* enable/disable */
		};
		struct ampdu_tid_control atc;
		for (i = 0; i < 8; i++) { /* One bit each for traffic class CS7 - CS0 */
			atc.tid = i;
			atc.enable = (ampdu_rx_tid >> i) & 1;
		}
		brcmf_fil_iovar_data_set(ifp, "ampdu_rx_tid", &atc,
				       sizeof(atc));
	}

#ifdef VSDB_BW_ALLOCATE_ENABLE
	if (bcmdhd_vsdb_bw_allocate_enable) {
		error = brcmf_fil_iovar_int_set(ifp, "mchan_algo", mchan_algo);
		if (error) {
			brcmf_error("Failed to set mchan_algo: mode:%d, error:%d\n",
			mode, error);
			return -1;
		}

		error = brcmf_fil_iovar_int_set(ifp, "mchan_bw", mchan_bw);
		if (error) {
			brcmf_error("Failed to set mchan_bw: mode:%d, error:%d\n",
			mode, error);
			return -1;
		}
	}
#endif /* VSDB_BW_ALLOCATE_ENABLE */

	return error;
}

int nv_set_roam_mode(struct net_device *dev, char *command, int total_len)
{
	int error = 0;
	int mode = 0;
	struct brcmf_if *ifp =	netdev_priv(dev);

	if (sscanf(command, "%*s %d", &mode) != 1) {
		brcmf_err("Failed to get Parameter\n");
		return -1;
	}

	error = brcmf_fil_iovar_int_set(ifp, "roam_off", mode);
	if (error) {
		brcmf_err("Failed to set roaming Mode %d, error = %d\n",
			mode, error);
		return -1;
	}
	else {
		/* Log in IDS */
		brcmf_err("succeeded to set roaming Mode %d, error = %d\n",
				mode, error);
	}
	return 0;
}

int nv_btcoex_set_btcparams(struct net_device *dev, char *command, int total_len)
{
	int bytes_written = 0, ret = -1;
	u32 param = -1, value = -1;
	struct brcmf_if *ifp =	netdev_priv(dev);

	struct {
		__le32 addr;
		__le32 data;
	} reg_write;

	if (sscanf(command, "%*s %d %d", &param, &value) != 2) {
		brcmf_err("%s: command error", __func__);
		return -2;
	}
	brcmf_dbg(TRACE, "%s:btcparams param %d, value %d\n", __func__, param, value);
	memset(command, 0, total_len);
	/* copy of function brcmf_btcoex_params_write */

	reg_write.addr = cpu_to_le32(param);
	reg_write.data = cpu_to_le32(value);
	ret = brcmf_fil_iovar_data_set(ifp, "btc_params",
					&reg_write, sizeof(reg_write));

	if (ret != 0) {
		brcmf_err("%s: failed %d\n", __func__, ret);
		return ret;
	}
	bytes_written = snprintf(command, total_len, "OK");
	return bytes_written;
}

int nv_btcoex_get_btcparams(struct net_device *dev, char *command, int total_len)
{
	int bytes_written = 0, ret = -1;
	u32 param = 0, value = -1;
	struct brcmf_if *ifp =	netdev_priv(dev);

	if (sscanf(command, "%*s %d", &param) != 1) {
		brcmf_err("%s: command error", __func__);
		return -2;
	}
	brcmf_dbg(TRACE, "%s: btcparams value %d\n", __func__, param);
	/*copy of function brcmf_btcoex_params_read */
	value = param;
	ret = brcmf_fil_iovar_int_get(ifp, "btc_params", &value);
	if (ret != 0) {
		brcmf_err("%s: failed %d\n", __func__, ret);
		return ret;
	}
	memset(command, 0, total_len);
	bytes_written = snprintf(command, total_len, "%d", value);
	return bytes_written;
}
#endif /* CONFIG_BRCMFMAC_NV_PRIV_CMD */
#endif /* CONFIG_BRCMFMAC_NV_CUSTOM_FILES */
