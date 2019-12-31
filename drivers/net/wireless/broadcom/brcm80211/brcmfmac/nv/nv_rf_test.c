/*
 * nv_rf_test.c
 *
 * NVIDIA Tegra Rf Test for brcmfmac driver
 *
 * Copyright (C) 2015-2019 NVIDIA Corporation. All rights reserved.
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

#include "nv_custom_sysfs_tegra.h"

atomic_t rf_test = ATOMIC_INIT(0);
atomic_t cur_power_mode = ATOMIC_INIT(0);
extern int tegra_sysfs_wifi_on;

//NUM_RF_TEST_PARAMS needs to be updated when modifying this array
char *rf_test_vars[] = {"roam_off", "txchain", "rxchain"};

rf_test_params_t rf_test_params[NUM_RF_TEST_PARAMS];

void rf_test_params_init() {
	int i;
	for (i = 0; i < NUM_RF_TEST_PARAMS; i++) {
		rf_test_params[i].var = rf_test_vars[i];
		atomic_set(&rf_test_params[i].cur_val, 0);
	}
}

void
tegra_sysfs_rf_test_enable()
{
	extern struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct brcmf_if *ifp = netdev_priv(net);
	u32 power_mode_off = 0;
	int i;

	pr_info("%s\n", __func__);

	/* Get current roam_off, txchain, rxchain and power mode state */
	for (i = 0; i < NUM_RF_TEST_PARAMS; i++) {
		if (brcmf_fil_iovar_int_get(ifp, rf_test_params[i].var,
		    (s32 *)&rf_test_params[i].cur_val) != 0) {
			pr_err("%s: Failed to get current %s val\n", __func__, rf_test_params[i].var);
			return;
		}
	}
	if(brcmf_fil_cmd_int_get(ifp, BRCMF_C_GET_PM, &i) < 0) {
		pr_err("%s: Failed to get current power mode state\n", __func__);
	}
	atomic_set(&cur_power_mode, i);

	/* set roam_off/power mode and block setting it */
	if (brcmf_fil_iovar_int_set(ifp, "roam_off", 1) != 0) {
		pr_err("%s: Failed to set roam off state\n", __func__);
		return;
	}
	if(brcmf_fil_cmd_int_set(ifp, BRCMF_C_SET_PM, power_mode_off) < 0) {
		pr_err("%s: Failed to set power mode off state\n", __func__);
	}
	atomic_set(&rf_test, 1);
}


void
tegra_sysfs_rf_test_disable()
{
	extern struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct brcmf_if *ifp = netdev_priv(net);
	int i;
	u32 arg;

	pr_info("%s\n", __func__);

	if (atomic_read(&rf_test)) {
		atomic_set(&rf_test, 0);
		/* Restore saved roam_off and power mode state */
		for (i = 0; i < NUM_RF_TEST_PARAMS; i++) {
			if (brcmf_fil_iovar_int_set(ifp, rf_test_params[i].var,
			atomic_read(&rf_test_params[i].cur_val)) != 0) {
				pr_err("%s: Failed to restore %s val\n", __func__, rf_test_params[i].var);
			}
		}
		arg = atomic_read(&cur_power_mode);
		if (brcmf_fil_cmd_int_set(ifp, BRCMF_C_SET_PM, arg) < 0) {
			pr_err("%s: Failed to restore power mode state\n", __func__);
		}
	}
}

ssize_t
tegra_sysfs_rf_test_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	if (atomic_read(&rf_test)) {
		strcpy(buf, "enabled\n");
		return strlen(buf);
	} else {
		strcpy(buf, "disabled\n");
		return strlen(buf);
	}
}

ssize_t
tegra_sysfs_rf_test_state_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (strncmp(buf, "enable", 6) == 0) {
		if (!atomic_read(&rf_test) && tegra_sysfs_wifi_on) {
			tegra_sysfs_rf_test_enable();
		} else {
			pr_info("%s: operation not allowed\n", __func__);
		}
	} else if (strncmp(buf, "disable", 7) == 0) {
		if (atomic_read(&rf_test) && tegra_sysfs_wifi_on) {
			tegra_sysfs_rf_test_disable();
		} else {
			pr_info("%s: operation not allowed\n", __func__);
		}
	} else {
		pr_err("%s: unknown command\n", __func__);
	}

	return count;
}
