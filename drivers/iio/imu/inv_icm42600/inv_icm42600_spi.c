// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 InvenSense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm42600.h"

static int inv_icm42600_spi_bus_setup(struct inv_icm42600_state *st)
{
	unsigned int mask, val;
	int ret;

	/* setup interface registers */
	val = INV_ICM42600_INTF_CONFIG6_I3C_EN |
	      INV_ICM42600_INTF_CONFIG6_I3C_SDR_EN |
	      INV_ICM42600_INTF_CONFIG6_I3C_DDR_EN;
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG6,
				 INV_ICM42600_INTF_CONFIG6_MASK, val);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG4,
				 INV_ICM42600_INTF_CONFIG4_I3C_BUS_ONLY, 0);
	if (ret)
		return ret;

	/* set slew rates for I2C and SPI */
	mask = INV_ICM42600_DRIVE_CONFIG_I2C_MASK |
	       INV_ICM42600_DRIVE_CONFIG_SPI_MASK;
	val = INV_ICM42600_DRIVE_CONFIG_I2C(INV_ICM42600_SLEW_RATE_20_60NS) |
	      INV_ICM42600_DRIVE_CONFIG_SPI(INV_ICM42600_SLEW_RATE_INF_2NS);
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_DRIVE_CONFIG,
				 mask, val);
	if (ret)
		return ret;

	/* disable i2c bus */
	return regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG0,
				  INV_ICM42600_INTF_CONFIG0_UI_SIFS_CFG_MASK,
				  INV_ICM42600_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS);
}

static int inv_icm42600_find_compat(struct spi_device *spi,
	const struct of_device_id **of_id)
{
	const struct of_device_id *id;
	int hw_id;

	while (true) {
		id = *of_id;
		if (!id->compatible || !id->compatible[0]) {
			dev_err(&spi->dev, "Failed to probe compatible device\n");
			return -ENODEV;
		}
		if (of_device_is_compatible(spi->dev.of_node, id->compatible)) {
			dev_info(&spi->dev, "Probing %s\n", id->compatible);
			hw_id = (int)(uintptr_t)id->data;
			break;
		}
		(*of_id)++;
	}
	(*of_id)++;

	return hw_id;
}

static int inv_icm42600_probe(struct spi_device *spi)
{
	const struct spi_driver *sdrv = to_spi_driver(spi->dev.driver);
	const struct of_device_id *of_id = sdrv->driver.of_match_table;
	const struct spi_device_id *id = spi_get_device_id(spi);
	bool multi_driver = false;
	struct regmap *regmap;
	int res, hw_id;

	if (!id) {
		if (!strcmp(spi->modalias, "multi-driver")) {
			multi_driver = true;
		} else {
			dev_err(&spi->dev, "Failed to get spi id: %s\n",
				spi->modalias);
			return -ENODEV;
		}
	} else
		hw_id = id->driver_data;

	regmap = devm_regmap_init_spi(spi, &inv_icm42600_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

try_next:
	if (multi_driver) {
		hw_id = inv_icm42600_find_compat(spi, &of_id);
		if (hw_id < 0)
			return hw_id;
	}

	res = inv_icm42600_core_probe(regmap, hw_id, spi->irq,
				      inv_icm42600_spi_bus_setup);

	if (multi_driver) {
		if (res)
			goto try_next;
		dev_info(&spi->dev, "Probed\n");
	}

	return res;
}

static const struct of_device_id inv_icm42600_of_matches[] = {
	{
		.compatible = "invensense,icm42600",
		.data = (void *)INV_CHIP_ICM42600,
	}, {
		.compatible = "invensense,icm40607",
		.data = (void *)INV_CHIP_ICM40607,
	}, {
		.compatible = "invensense,icm42602",
		.data = (void *)INV_CHIP_ICM42602,
	}, {
		.compatible = "invensense,icm42605",
		.data = (void *)INV_CHIP_ICM42605,
	}, {
		.compatible = "invensense,icm42622",
		.data = (void *)INV_CHIP_ICM42622,
	},
	{}
};
MODULE_DEVICE_TABLE(of, inv_icm42600_of_matches);

static const struct spi_device_id inv_icm42600_spi_id_table[] = {
	{ "icm42600", INV_CHIP_ICM42600 },
	{ "icm40607", INV_CHIP_ICM40607 },
	{ "icm42602", INV_CHIP_ICM42602 },
	{ "icm42605", INV_CHIP_ICM42605 },
	{ "icm42622", INV_CHIP_ICM42622 },
	{},
};
MODULE_DEVICE_TABLE(spi, inv_icm42600_spi_id_table);

static struct spi_driver inv_icm42600_driver = {
	.driver = {
		.name = "inv-icm42600-spi",
		.of_match_table = of_match_ptr(inv_icm42600_of_matches),
		.pm = &inv_icm42600_pm_ops,
	},
	.probe = inv_icm42600_probe,
	.id_table = inv_icm42600_spi_id_table,
};
module_spi_driver(inv_icm42600_driver);

MODULE_AUTHOR("InvenSense, Inc.");
MODULE_DESCRIPTION("InvenSense ICM-426xx SPI driver");
MODULE_LICENSE("GPL");
