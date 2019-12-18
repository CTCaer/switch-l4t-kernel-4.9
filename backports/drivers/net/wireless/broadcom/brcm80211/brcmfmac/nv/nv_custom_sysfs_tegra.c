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

#include "debug.h"
#include "nv_custom_sysfs_tegra.h"

#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_BW_EST_TEGRA
#include "dhd_custom_net_bw_est_tegra.h"
#endif

#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_DIAG_TEGRA
#include "dhd_custom_net_diag_tegra.h"
#endif

struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
int lp0_logs_enable = 1;
const char dummy_inf[] = "dummy:";
int bcmdhd_irq_number;

extern int bcmdhd_resume_trigger;

#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
static DEVICE_ATTR(ping, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_ping_show,
	tegra_sysfs_histogram_ping_store);

static DEVICE_ATTR(rssi, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_rssi_show,
	tegra_sysfs_histogram_rssi_store);

static DEVICE_ATTR(tcpdump, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_tcpdump_show,
	tegra_sysfs_histogram_tcpdump_store);

static struct file_operations tegra_debugfs_histogram_tcpdump_fops = {
	.read = tegra_debugfs_histogram_tcpdump_read,
	.write = tegra_debugfs_histogram_tcpdump_write,
};
#endif

#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
static DEVICE_ATTR(scan, S_IRUGO | S_IWUSR,
	tegra_sysfs_histogram_scan_show,
	tegra_sysfs_histogram_scan_store);
#endif

static struct attribute *tegra_sysfs_entries_histogram[] = {
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	&dev_attr_ping.attr,
	&dev_attr_rssi.attr,
	&dev_attr_tcpdump.attr,
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	&dev_attr_scan.attr,
#endif
#endif
	NULL,
};

static struct attribute_group tegra_sysfs_group_histogram = {
	.name = "histogram",
	.attrs = tegra_sysfs_entries_histogram,
};

/* RF test attributes */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_RF_TEST
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR,
	tegra_sysfs_rf_test_state_show,
	tegra_sysfs_rf_test_state_store);

static struct attribute *tegra_sysfs_entries_rf_test[] = {
	&dev_attr_state.attr,
	NULL,
};

static struct attribute_group tegra_sysfs_group_rf_test = {
	.name = "rf_test",
	.attrs = tegra_sysfs_entries_rf_test,
};
#endif
/* End RF test attributes */

static struct dentry *tegra_debugfs_root;

#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
static struct file_operations tegra_debugfs_histogram_scan_fops = {
	.read = tegra_debugfs_histogram_scan_read,
	.write = tegra_debugfs_histogram_scan_write,
};
#endif

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
#ifdef CONFIG_BACKPORT_NV_CUSTOM_RF_TEST
	rf_test_params_init();
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_rf_test);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_rf_test.name);
		goto cleanup;
	}
#endif

	/* create debugfs */
	tegra_debugfs_root = debugfs_create_dir("bcmdhd_histogram", NULL);
	if (tegra_debugfs_root) {
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
		debugfs_create_file("tcpdump", S_IRUGO,
			tegra_debugfs_root, (void *) 0,
			&tegra_debugfs_histogram_tcpdump_fops);
#endif
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
		debugfs_create_file("scan", S_IRUGO,
			tegra_debugfs_root, (void *) 0,
			&tegra_debugfs_histogram_scan_fops);
#endif
	}
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_IDS
	dhdlog_sysfs_init(dev);
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_IDS */

	/* start sysfs work */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_ping_work_start();
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_start();
#endif
#endif
	goto exit;

#ifdef CONFIG_BACKPORT_NV_CUSTOM_RF_TEST
cleanup:
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);
#endif
exit:
	return err;
}

void
tegra_sysfs_unregister(struct device *dev)
{
	pr_info("%s\n", __func__);

	/* stop sysfs work */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_stop();
#endif
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();
	tegra_sysfs_histogram_ping_work_stop();
#endif

	/* remove debugfs */
	if (tegra_debugfs_root) {
		debugfs_remove_recursive(tegra_debugfs_root);
		tegra_debugfs_root = NULL;
	}

	/* remove sysfs */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_RF_TEST
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_rf_test);
#endif
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);

#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_IDS
	dhdlog_sysfs_deinit(dev);
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_IDS */
}

int tegra_sysfs_wifi_on;

void
tegra_sysfs_on(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_wifi_on = 1;

	/* resume (start) sysfs work */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();
#endif
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_start();
#endif

#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_CUSTOM_MAC
	if (dhd_custom_sysfs_tegra_histogram_stat_netdev != NULL) {
		brcmf_err("%s: MAC seting\n", __func__);
		nv_set_mac_address(
			dhd_custom_sysfs_tegra_histogram_stat_netdev);
	}
#endif
}

void
tegra_sysfs_off(void)
{
	pr_info("%s\n", __func__);

	tegra_sysfs_wifi_on = 0;

	/* suspend (stop) sysfs work */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();
#endif
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_stop();
#endif

}

void
tegra_sysfs_suspend(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* suspend (stop) sysfs work */

#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_stop();
#endif
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();
	tegra_sysfs_histogram_ping_work_stop();
#endif
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_IDS
	nvlogger_suspend_work();
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_IDS */
}

void
tegra_sysfs_resume(void)
{
	pr_info("%s\n", __func__);

	/* do nothing if wifi off */
	if (!tegra_sysfs_wifi_on)
		return;

	/* resume (start) sysfs work */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_CAP
	tegra_sysfs_histogram_tcpdump_work_start();
	tegra_sysfs_histogram_ping_work_start();
	tegra_sysfs_histogram_rssi_work_start();
#endif
#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_IDS
        nvlogger_resume_work();
#endif /*CONFIG_BACKPORT_BRCMFMAC_NV_IDS */
#ifdef CONFIG_BACKPORT_NV_CUSTOM_SCAN
	tegra_sysfs_histogram_scan_work_start();
#endif

}

int
tegra_sysfs_bus_register(struct device *dev)
{
	int err = -1;
#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_BW_EST_TEGRA
	err = tegra_net_bw_est_register(dev);
	if (err < 0) {
		brcmf_err("%s: tegra_net_bw_est_register() failed\n", __func__);
	}
#endif

#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_DIAG_TEGRA
	err = tegra_net_diag_register(dev);
	if (err < 0) {
		brcmf_err("%s: tegra_net_diag_register() failed\n", __func__);
#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_BW_EST_TEGRA
		tegra_net_bw_est_unregister(dev);
#endif
	}
#endif
	/* create bus sysfs here */

	return err;
}

void
tegra_sysfs_bus_unregister(struct device *dev)
{
	/* remove bus sysfs here */
#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_DIAG_TEGRA
	tegra_net_diag_unregister(dev);
#endif
#ifdef  CONFIG_BACKPORT_BRCMFMAC_NV_NET_BW_EST_TEGRA
	tegra_net_bw_est_unregister(dev);
#endif
}
