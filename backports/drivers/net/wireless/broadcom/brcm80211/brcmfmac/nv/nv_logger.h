/*
 * drivers/net/wireless/bcmdhd/nv_logger.h
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2016-2019 NVIDIA Corporation. All rights reserved.
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

#ifdef CONFIG_BACKPORT_BRCMFMAC_NV_IDS

#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <net/netlink.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#define MAX_LOGLIMIT 1024
#define TIMESTAMPSIZE 40
#define MAX_ERROR_SIZE 512
#define MAX_LOG_NUM 20
#define nv_sprintf(fmt, args...) \
do { \
	snprintf(nv_error_buffer, MAX_ERROR_SIZE-1, fmt , ## args); \
	write_log(0, nv_error_buffer, NULL ); \
} while(0)

extern struct workqueue_struct  *logger_wqueue;
extern bool enable_file_logging;
int write_log(int, const char *, u8 *);
void write_log_init(void);
void write_log_uninit(void);
void write_log_file(const char *);
void write_queue_work(struct work_struct *);
void nvlogger_suspend_work(void);
void nvlogger_resume_work(void);
s32 dhd_log_netlink_send_msg(int pid, int type, int seq,
			void *data, size_t size);
int dhdlog_sysfs_init(struct device *dev);
int dhdlog_sysfs_deinit(struct device *dev);
extern char nv_error_buffer[MAX_ERROR_SIZE];
#endif /* CONFIG_BACKPORT_BRCMFMAC_NV_IDS */
