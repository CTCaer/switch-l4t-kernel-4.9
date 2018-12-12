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
#ifdef CPTCFG_BRCMFMAC_NV_GPIO
void setup_gpio(struct platform_device *pdev, bool on);
void toggle_gpio(bool on, unsigned long msec);
#endif /* CPTCFG_BRCMFMAC_NV_GPIO */
#ifdef CPTCFG_BRCMFMAC_NV_CUSTOM_MAC
int wifi_get_mac_addr(unsigned char *buf);
#endif /* CPTCFG_BRCMFMAC_NV_CUSTOM_MAC */
#endif /* CPTCFG_BRCMFMAC_NV_CUSTOM_FILES */
