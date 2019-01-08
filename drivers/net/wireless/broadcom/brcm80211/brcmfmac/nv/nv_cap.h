/*
 * nv_cap.h
 *
 * NVIDIA Tegra NvCap for brcmfmac driver
 *
 * Copyright (C) 2019 NVIDIA Corporation. All rights reserved.
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

#ifndef _nv_cap_h_
#define _nv_cap_h_

#define TCPDUMP_TAG_FREE	'?'
#define TCPDUMP_TAG_RX  	'<'
#define TCPDUMP_TAG_TX  	'>'
#define TCPDUMP_TAG_TIME	'@'
#define TCPDUMP_TAG_STAT	'&'
#define TCPDUMP_TAG_PWR 	'P'

/* ping histogram */
void
tegra_sysfs_histogram_ping_request(struct sk_buff *skb);

void
tegra_sysfs_histogram_ping_reply(struct sk_buff *skb);

void
tegra_sysfs_histogram_ping_work_start(void);

void
tegra_sysfs_histogram_ping_work_stop(void);

ssize_t
tegra_sysfs_histogram_ping_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_ping_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

/* rssi histogram */
void
tegra_sysfs_histogram_rssi_work_start(void);

void
tegra_sysfs_histogram_rssi_work_stop(void);

ssize_t
tegra_sysfs_histogram_rssi_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_rssi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

/* tcpdump histogram */
void
tcpdump_pkt_save(char tag, const char *netif, const char *func, int line,
	const unsigned char *data,
	unsigned int data_nonpaged_len,
	unsigned int data_paged_len);

void
tegra_sysfs_histogram_tcpdump_rx(struct sk_buff *skb,
	const char *func, int line);

void
tegra_sysfs_histogram_tcpdump_tx(struct sk_buff *skb,
	const char *func, int line);

void
tegra_sysfs_histogram_tcpdump_work_start(void);

void
tegra_sysfs_histogram_tcpdump_work_stop(void);

ssize_t
tegra_sysfs_histogram_tcpdump_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_histogram_tcpdump_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

ssize_t
tegra_debugfs_histogram_tcpdump_read(struct file *filp,
	char __user *buff, size_t count, loff_t *offp);

ssize_t
tegra_debugfs_histogram_tcpdump_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *offp);

#endif  /* _nv_cap_h_ */
