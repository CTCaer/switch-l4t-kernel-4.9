/*
 * nv_rf_test.h
 *
 * NVIDIA Tegra Rf Test for brcmfmac driver
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

#ifndef _nv_rf_test_h_
#define _nv_rf_test_h_

/* RF testing */
#define NUM_RF_TEST_PARAMS 3
typedef struct {
	char *var;
	atomic_t cur_val;
} rf_test_params_t;

ssize_t
tegra_sysfs_rf_test_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf);

ssize_t
tegra_sysfs_rf_test_state_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);

void tegra_sysfs_rf_test_enable(void);
void tegra_sysfs_rf_test_disable(void);
void rf_test_params_init(void);

#endif  /* _nv_rf_test_h_ */
