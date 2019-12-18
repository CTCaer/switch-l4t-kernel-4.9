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
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_CUSTOM_FILES
#include <linux/platform_device.h>
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_PRIV_CMD
int brcmf_get_max_linkspeed(struct net_device *dev,
		char *command, int total_len);
int nv_brcmf_android_set_im_mode(struct brcmf_pub *drvr,
		struct net_device *ndev, char *command, u32 cmd_len);
int nv_set_roam_mode(struct net_device *dev, char *command, int total_len);
int nv_btcoex_get_btcparams(struct net_device *dev, char *command, int total_len);
int nv_btcoex_set_btcparams(struct net_device *dev, char *command, int total_len);
#endif /*CONFIG_BACKPORT_BRCMFMAC_NV_PRIV_CMD */
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_GPIO
void setup_gpio(struct platform_device *pdev, bool on);
void toggle_gpio(bool on, unsigned long msec);
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_GPIO */
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_COUNTRY_CODE
int wifi_platform_get_country_code_map(void);
void wifi_platform_free_country_code_map(void);
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_COUNTRY_CODE */
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_CUSTOM_MAC
int nv_set_mac_address(struct net_device *ndev);
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_CUSTOM_MAC */
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_CUSTOM_FILES */
