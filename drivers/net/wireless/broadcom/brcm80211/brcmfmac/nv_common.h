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
#ifdef CPTCFG_BRCMFMAC_NV_CUSTOM_FILES
#ifdef CPTCFG_BRCMFMAC_NV_PRIV_CMD
int brcmf_get_max_linkspeed(struct net_device *dev,
		char *command, int total_len);
int brcmf_set_band(struct net_device *ndev, uint band);
int nv_brcmf_android_set_im_mode(struct brcmf_pub *drvr,
		struct net_device *ndev, char *command, u32 cmd_len);
int nv_set_roam_mode(struct net_device *dev, char *command, int total_len);
#endif /*CPTCFG_BRCMFMAC_NV_PRIV_CMD */
#ifdef CPTCFG_BRCMFMAC_NV_GPIO
void setup_gpio(struct platform_device *pdev, bool on);
void toggle_gpio(bool on, unsigned long msec);
#endif /* CPTCFG_BRCMFMAC_NV_GPIO */
#ifdef CPTCFG_BRCMFMAC_NV_COUNTRY_CODE
int wifi_platform_get_country_code_map(void);
void wifi_platform_free_country_code_map(void);
#endif /* CPTCFG_BRCMFMAC_NV_COUNTRY_CODE */
#ifdef CPTCFG_BRCMFMAC_NV_CUSTOM_MAC
int wifi_get_mac_addr(unsigned char *buf);
#endif /* CPTCFG_BRCMFMAC_NV_CUSTOM_MAC */
#endif /* CPTCFG_BRCMFMAC_NV_CUSTOM_FILES */
