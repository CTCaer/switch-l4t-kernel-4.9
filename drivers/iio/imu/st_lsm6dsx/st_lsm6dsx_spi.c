/*
 * STMicroelectronics st_lsm6dsx spi driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "st_lsm6dsx.h"

static const struct regmap_config st_lsm6dsx_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lsm6dsx_spi_compat(struct spi_device *spi,
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

static int st_lsm6dsx_spi_probe(struct spi_device *spi)
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

	regmap = devm_regmap_init_spi(spi, &st_lsm6dsx_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

try_next:
	if (multi_driver) {
		hw_id = st_lsm6dsx_spi_compat(spi, &of_id);
		if (hw_id < 0)
			return hw_id;
	}

	res = st_lsm6dsx_probe(&spi->dev, spi->irq, hw_id, id->name, regmap);

	if (multi_driver) {
		if (res)
			goto try_next;
		dev_info(&spi->dev, "Probed\n");
	}

	return res;
}

static void st_lsm6dsx_spi_shutdown(struct spi_device *spi)
{
	st_lsm6dsx_shutdown(&spi->dev);
}

static const struct of_device_id st_lsm6dsx_spi_of_match[] = {
	{
		.compatible = "st,lsm6ds3",
		.data = (void *)ST_LSM6DS3_ID,
	},
	{
		.compatible = "st,lsm6ds3h",
		.data = (void *)ST_LSM6DS3H_ID,
	},
	{
		.compatible = "st,lsm6dse",
		.data = (void *)ST_LSM6DSE_ID,
	},
	{
		.compatible = "st,lsm6dsl",
		.data = (void *)ST_LSM6DSL_ID,
	},
	{
		.compatible = "st,lsm6dsm",
		.data = (void *)ST_LSM6DSM_ID,
	},
	{
		.compatible = "st,lsm6dso",
		.data = (void *)ST_LSM6DSO_ID,
	},
	{
		.compatible = "st,lsm6dsox",
		.data = (void *)ST_LSM6DSOX_ID,
	},
	{
		.compatible = "st,lsm6dsop",
		.data = (void *)ST_LSM6DSOP_ID,
	},
	{
		.compatible = "st,ism330dlc",
		.data = (void *)ST_ISM330DLC_ID,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_lsm6dsx_spi_of_match);

static const struct spi_device_id st_lsm6dsx_spi_id_table[] = {
	{ ST_LSM6DS3_DEV_NAME, ST_LSM6DS3_ID },
	{ ST_LSM6DS3H_DEV_NAME, ST_LSM6DS3H_ID },
	{ ST_LSM6DSE_DEV_NAME, ST_LSM6DSE_ID },
	{ ST_LSM6DSL_DEV_NAME, ST_LSM6DSL_ID },
	{ ST_LSM6DSM_DEV_NAME, ST_LSM6DSM_ID },
	{ ST_LSM6DSO_DEV_NAME, ST_LSM6DSO_ID },
	{ ST_LSM6DSOX_DEV_NAME, ST_LSM6DSOX_ID },
	{ ST_LSM6DSOP_DEV_NAME, ST_LSM6DSOP_ID },
	{ ST_ISM330DLC_DEV_NAME, ST_ISM330DLC_ID },
	{},
};
MODULE_DEVICE_TABLE(spi, st_lsm6dsx_spi_id_table);

static struct spi_driver st_lsm6dsx_driver = {
	.driver = {
		.name = "st_lsm6dsx_spi",
		.pm = &st_lsm6dsx_pm_ops,
		.of_match_table = of_match_ptr(st_lsm6dsx_spi_of_match),
	},
	.probe = st_lsm6dsx_spi_probe,
	.shutdown = st_lsm6dsx_spi_shutdown,
	.id_table = st_lsm6dsx_spi_id_table,
};
module_spi_driver(st_lsm6dsx_driver);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsx spi driver");
MODULE_LICENSE("GPL v2");
