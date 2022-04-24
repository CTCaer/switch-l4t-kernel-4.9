/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/of.h>
#include <soc/tegra/chip-id.h>

bool is_tegra_safety_build(void)
{
#ifdef CONFIG_OF
	return of_property_read_bool(of_chosen,
		"nvidia,tegra-safety-build");
#else
	return false;
#endif
}
EXPORT_SYMBOL(is_tegra_safety_build);
