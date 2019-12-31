/*
 * nv_custom_sysfs_tegra_rssi.c
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
#include <linux/ieee80211.h>

#include "nv_custom_sysfs_tegra.h"

extern struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;

static void
rssi_work_func(struct work_struct *work);

static unsigned int rssi_rate_ms = 10 * 1000;
static DECLARE_DELAYED_WORK(rssi_work, rssi_work_func);

static void
rssi_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	char *netif = net ? net->name : "";
	struct brcmf_if *ifp = netdev_priv(net);
	struct brcmf_scb_val_le scb_val;
	int err;
	int i;

	UNUSED_PARAMETER(dwork);


	/* create rssi request */
	memset(&scb_val, 0, sizeof(scb_val));
	scb_val.val = 0;

	/* send rssi request */
	err = brcmf_fil_cmd_data_get(ifp, BRCMF_C_GET_RSSI, &scb_val,
				     sizeof(scb_val));
	if (err) {
		pr_err("%s: brcmf_fil_cmd_data_get(BRCMF_C_GET_RSSI,) "
			"failed\n", __func__);
		goto fail;
	}

	/* log rssi request */
	for (i = 0; i < sizeof(scb_val.val); i += 64) {
		tcpdump_pkt_save('r' + i / 64,
			netif,
			__func__,
			__LINE__,
			((unsigned char *) &scb_val.val) + i,
			(i + 64) <= sizeof(scb_val.val)
				? 64 : sizeof(scb_val.val) - i,
			0);
	}

#ifdef CPTCFG_NV_CUSTOM_STATS
	/* save rssi value for statistics */
	if (((int) scb_val.val) < 0) {
		TEGRA_SYSFS_HISTOGRAM_STAT_SET(rssi, scb_val.val);
		if (bcmdhd_stat.gen_stat.rssi < -67) {
			if (bcmdhd_stat.gen_stat.channel_stat)
				TEGRA_SYSFS_HISTOGRAM_STAT_INC
					(channel_stat->rssi_low);
			TEGRA_SYSFS_HISTOGRAM_STAT_INC(rssi_low);
		} else {
			if (bcmdhd_stat.gen_stat.channel_stat)
				TEGRA_SYSFS_HISTOGRAM_STAT_INC
					(channel_stat->rssi_high);
			TEGRA_SYSFS_HISTOGRAM_STAT_INC(rssi_high);
		}
	}
#endif

	/* schedule next rssi */
fail:
	schedule_delayed_work(&rssi_work,
		msecs_to_jiffies(rssi_rate_ms));

}

void
tegra_sysfs_histogram_rssi_work_start(void)
{
	if (rssi_rate_ms > 0)
		schedule_delayed_work(&rssi_work,
			msecs_to_jiffies(rssi_rate_ms));
}

void
tegra_sysfs_histogram_rssi_work_stop(void)
{
	cancel_delayed_work_sync(&rssi_work);
}

ssize_t
tegra_sysfs_histogram_rssi_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	static int i;


	if (!i) {
		i++;
		strcpy(buf, "dummy rssi!");
		return strlen(buf);
	} else {
		i = 0;
		return 0;
	}
}

ssize_t
tegra_sysfs_histogram_rssi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int uint;


	if (strncmp(buf, "enable", 6) == 0) {
		pr_info("%s: starting rssi delayed work...\n", __func__);
		tegra_sysfs_histogram_rssi_work_start();
	} else if (strncmp(buf, "disable", 7) == 0) {
		pr_info("%s: stopping rssi delayed work...\n", __func__);
		tegra_sysfs_histogram_rssi_work_stop();
	} else if (strncmp(buf, "rate ", 5) == 0) {
		err = kstrtouint(buf + 5, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid rssi rate (ms)\n", __func__);
			return count;
		}
		pr_info("%s: set rssi rate (ms) %u\n", __func__, uint);
		rssi_rate_ms = uint;
	} else {
		pr_err("%s: unknown command\n", __func__);
	}

	return count;
}
