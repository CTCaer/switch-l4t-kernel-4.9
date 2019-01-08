/*
 * nv_custom_sysfs_tegra.c
 *
 * NVIDIA Tegra Sysfs for brcmfmac driver
 *
 * Copyright (C) 2014-2019 NVIDIA Corporation. All rights reserved.
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

struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
int lp0_logs_enable = 1;
const char dummy_inf[] = "dummy:";
int bcmdhd_irq_number;

extern int bcmdhd_resume_trigger;

static struct attribute *tegra_sysfs_entries_histogram[] = {
	NULL,
};

static struct attribute_group tegra_sysfs_group_histogram = {
	.name = "histogram",
	.attrs = tegra_sysfs_entries_histogram,
};

static struct dentry *tegra_debugfs_root;

int
tegra_sysfs_register(struct device *dev)
{
	int err = 0;

	pr_info("%s\n", __func__);

	/* create sysfs */
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_histogram);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_histogram.name);
		goto exit;
	}

	/* create debugfs */
	tegra_debugfs_root = debugfs_create_dir("bcmdhd_histogram", NULL);
	if (tegra_debugfs_root) {
	}

	/* start sysfs work */

	goto exit;

exit:
	return err;
}

void
tegra_sysfs_unregister(struct device *dev)
{
	pr_info("%s\n", __func__);

	/* stop sysfs work */

	/* remove debugfs */
	if (tegra_debugfs_root) {
		debugfs_remove_recursive(tegra_debugfs_root);
		tegra_debugfs_root = NULL;
	}

	/* remove sysfs */
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);

}

int tegra_sysfs_wifi_on;

void
tegra_sysfs_on(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_wifi_on = 1;

	/* resume (start) sysfs work */

}

void
tegra_sysfs_off(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_wifi_on = 0;

	/* suspend (stop) sysfs work */

}

void
tegra_sysfs_suspend(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* suspend (stop) sysfs work */

}

void
tegra_sysfs_resume(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* resume (start) sysfs work */

}

int
tegra_sysfs_bus_register(struct device *dev)
{
	int err = -1;
	/* create bus sysfs here */

	return err;
}

void
tegra_sysfs_bus_unregister(struct device *dev)
{
	/* remove bus sysfs here */

}
