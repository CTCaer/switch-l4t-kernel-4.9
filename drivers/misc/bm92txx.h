/*
 * bm92txx.h
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION, All Rights Reserved.
 *
 * Authors:
 *     Adam Jiang chaoj@nvidia.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __BM92TXX_H__
#define __BM92TXX_H__

struct bm92t_platform_data {
	int irq_gpio;
	bool disable_power_nego;
};

#endif /* __BM92TXX_H__ */
