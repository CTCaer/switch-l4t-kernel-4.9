/*
 * max77801.c - Regulator driver for the Maxim 77801
 *
 * Copyright (c) 2022, CTCaer.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>

#define MAX77801_REG_CHIPID		0x00
#define MAX77801_REG_STATUS		0x01
#define MAX77801_REG_CONFIG1		0x02
#define MAX77801_REG_CONFIG2		0x03
#define MAX77801_REG_VOUT_DVS_L		0x04 /* EWP/ETP 3.3V */
#define MAX77801_REG_VOUT_DVS_H		0x05 /* EWP: 3.4V, ETP: 3.75V */

#define  MAX77801_STATUS_OCP		BIT(0)
#define  MAX77801_STATUS_OVP		BIT(1)
#define  MAX77801_STATUS_POKN		BIT(2)
#define  MAX77801_STATUS_TSHDN		BIT(3)

#define  MAX77801_CFG1_FPWM		BIT(0)
#define  MAX77801_CFG1_AD		BIT(1)
#define  MAX77801_CFG1_OVP_TH_MASK	(3 << 2)
#define  MAX77801_CFG1_OVP_NONE		(0 << 2)
#define  MAX77801_CFG1_OVP_110_PCT	(1 << 2)
#define  MAX77801_CFG1_OVP_115_PCT	(2 << 2)
#define  MAX77801_CFG1_OVP_120_PCT	(3 << 2)
#define  MAX77801_CFG1_RD_SR		BIT(4)
#define  MAX77801_CFG1_RU_SR		BIT(5)

#define  MAX77801_CFG2_POK_POL		BIT(4)
#define  MAX77801_CFG2_PD		BIT(5)
#define  MAX77801_CFG2_EN		BIT(6)

#define  MAX77801_VOUT_UV_MASK		0x7F
#define  MAX77801_VOUT_UV_MIN		2600000
#define  MAX77801_VOUT_UV_STEP		12500
#define  MAX77801_VOUT_UV_MAX		4187500

struct max77801_regulator {
	struct device *dev;
	struct regmap *rmap;
	bool forced_pwm;
	int  dvs_high_voltage;
};

static int max77801_vsel(struct max77801_regulator *max77801,
				    int dvs_uv)
{
	if (!dvs_uv)
		return -EINVAL;

	if (dvs_uv > MAX77801_VOUT_UV_MAX || dvs_uv < MAX77801_VOUT_UV_MIN) {
		dev_err(max77801->dev, "voltage out of range. skipping.\n");
		return -EINVAL;
	}

	dvs_uv -= MAX77801_VOUT_UV_MIN;
	dvs_uv /= MAX77801_VOUT_UV_STEP;

	return dvs_uv;
}

int max77801_set_voltage(struct regulator_dev *rdev, unsigned sel)
{
	struct max77801_regulator *max77801 = rdev_get_drvdata(rdev);
	int vsel = -EINVAL;
	int ret;

	ret = regmap_update_bits(rdev->regmap, MAX77801_REG_VOUT_DVS_L,
				 MAX77801_VOUT_UV_MASK, sel);
	if (ret)
		return ret;

	if (max77801->dvs_high_voltage)
		vsel = max77801_vsel(max77801, max77801->dvs_high_voltage);

	if (vsel < 0)
		vsel = sel;

	ret = regmap_update_bits(rdev->regmap, MAX77801_REG_VOUT_DVS_H,
				 MAX77801_VOUT_UV_MASK, vsel);

	return ret;
}

static int max77801_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct max77801_regulator *max77801 = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		regmap_update_bits(rdev->regmap, MAX77801_REG_CONFIG1,
				   MAX77801_CFG1_FPWM, MAX77801_CFG1_FPWM);
		max77801->forced_pwm = true;
		break;
	case REGULATOR_MODE_NORMAL:
		regmap_update_bits(rdev->regmap, rdev->desc->vsel_reg,
				   MAX77801_CFG1_FPWM, 0);
		max77801->forced_pwm = false;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int max77801_get_mode(struct regulator_dev *rdev)
{
	struct max77801_regulator *max77801 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, MAX77801_REG_CONFIG1, &val);
	if (ret != 0)
		return ret;
	if (val & MAX77801_CFG1_FPWM) {
		max77801->forced_pwm = true;
		return REGULATOR_MODE_FAST;
	}
	max77801->forced_pwm = false;
	return REGULATOR_MODE_NORMAL;
}

static int max77801_set_pd(struct regulator_dev *rdev)
{
	return regmap_update_bits(rdev->regmap, MAX77801_REG_CONFIG2,
				  MAX77801_CFG2_PD, MAX77801_CFG2_PD);
}

static unsigned int max77801_map_mode(unsigned int mode)
{
	return mode;
}

static const struct regulator_ops max77801_reg_ops = {
	.set_voltage_sel = max77801_set_voltage,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.list_voltage	= regulator_list_voltage_linear,
	.map_voltage	= regulator_map_voltage_linear,
	.enable		= regulator_enable_regmap,
	.disable	= regulator_disable_regmap,
	.is_enabled	= regulator_is_enabled_regmap,
	.set_mode	= max77801_set_mode,
	.get_mode	= max77801_get_mode,
	.set_active_discharge = regulator_set_active_discharge_regmap,
	.set_pull_down	= max77801_set_pd,

};

static struct regulator_desc max77801_reg_desc = {
	.name		= "max77801",
	.ops		= &max77801_reg_ops,
	.type		= REGULATOR_VOLTAGE,
	.n_voltages	= 1 << 7,
	.owner		= THIS_MODULE,
	.vsel_reg	= MAX77801_REG_VOUT_DVS_L,
	.vsel_mask	= MAX77801_VOUT_UV_MASK,
	.min_uV		= MAX77801_VOUT_UV_MIN,
	.uV_step	= MAX77801_VOUT_UV_STEP,
	.enable_reg	= MAX77801_REG_CONFIG2,
	.enable_mask	= MAX77801_CFG2_EN,
	.active_discharge_off = 0,
	.active_discharge_on = MAX77801_CFG1_AD,
	.active_discharge_mask = MAX77801_CFG1_AD,
	.active_discharge_reg = MAX77801_REG_CONFIG1,
	.of_map_mode = max77801_map_mode
};

static const struct regmap_config max77801_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= MAX77801_REG_VOUT_DVS_H,
	.cache_type		= REGCACHE_NONE,
};

struct regulator_init_data *max77801_parse_dt(struct device *dev,
					struct device_node *np,
					struct max77801_regulator *max77801)
{
	struct regulator_init_data *regulator;
	u32 pval;
	int ret;

	regulator = of_get_regulator_init_data(dev, np, &max77801_reg_desc);
	ret = of_property_read_u32(np, "maxim,dvs-high-microvolt", &pval);
	if (!ret)
		max77801->dvs_high_voltage = pval;
	else
		max77801->dvs_high_voltage = 0;

	return regulator;
}

static int max77801_probe(struct i2c_client *client,
			  const struct i2c_device_id *client_id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct max77801_regulator *max77801;
	struct regulator_dev *regulator;
	struct regulator_config config = { };
	struct regulator_init_data *regulator_data;
	unsigned int id;
	int ret;

	max77801 = devm_kzalloc(dev, sizeof(*max77801), GFP_KERNEL);
	if (!max77801)
		return -ENOMEM;

	regulator_data = max77801_parse_dt(dev, np, max77801);
	if (!regulator_data) {
		dev_err(dev, "Reading regulator data from DT failed\n");
		return -EINVAL;
	}

	max77801->rmap = devm_regmap_init_i2c(client, &max77801_regmap_config);
	if (IS_ERR(max77801->rmap)) {
		ret = PTR_ERR(max77801->rmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, max77801);
	max77801->dev = dev;

	ret = regmap_read(max77801->rmap, MAX77801_REG_CHIPID, &id);
	if (ret < 0) {
		dev_err(dev, "id reg read failed:%d\n", ret);
		return ret;
	}

	dev_info(dev, "MAX77801 ID: 0x%02X\n", id);

	config.dev = &client->dev;
	config.init_data = regulator_data;
	config.driver_data = max77801;
	config.regmap = max77801->rmap;
	config.of_node = np;

	regulator = devm_regulator_register(&client->dev, &max77801_reg_desc,
					    &config);
	if (IS_ERR(regulator)) {
		dev_err(dev, "failed to register regulator %s\n",
			max77801_reg_desc.name);
		return PTR_ERR(regulator);
	}

	return 0;
}

#ifdef CONFIG_PM
static int max77801_pm_suspend(struct device *dev)
{
	struct max77801_regulator *max77801 = dev_get_drvdata(dev);
	int ret;

	if (max77801->forced_pwm) {
		ret = regmap_update_bits(max77801->rmap, MAX77801_REG_CONFIG1,
					 MAX77801_CFG1_FPWM, 0);
		if (ret < 0) {
			dev_err(max77801->dev,
				"reg config update failed %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int max77801_pm_resume(struct device *dev)
{
	struct max77801_regulator *max77801 = dev_get_drvdata(dev);
	int ret;

	if (max77801->forced_pwm) {
		ret = regmap_update_bits(max77801->rmap, MAX77801_REG_CONFIG1,
					 MAX77801_CFG1_FPWM, MAX77801_CFG1_FPWM);
		if (ret < 0) {
			dev_err(max77801->dev,
				"reg config update failed %d\n", ret);
			return ret;
		}
	}


	return 0;
}

static const struct dev_pm_ops max77801_pm_ops = {
	.suspend = max77801_pm_suspend,
	.resume = max77801_pm_resume,
};
#endif

static void max77801_shutdown(struct i2c_client *client)
{
	struct max77801_regulator *max77801 = i2c_get_clientdata(client);
	int ret;

	if (max77801->forced_pwm) {
		ret = regmap_update_bits(max77801->rmap, MAX77801_REG_CONFIG1,
					 MAX77801_CFG1_FPWM, 0);
		if (ret < 0)
			dev_err(max77801->dev,
				"reg config update failed %d\n", ret);
	}
}

static const struct of_device_id max77801_of_match[] = {
	{ .compatible = "maxim,max77801-regulator", },
	{},
};
MODULE_DEVICE_TABLE(of, max77801_of_match);

static const struct i2c_device_id max77801_id[] = {
	{.name = "max77801",},
	{},
};

static struct i2c_driver max77801_i2c_driver = {
	.driver = {
		.name = "max77801",
		.owner = THIS_MODULE,
		.of_match_table = max77801_of_match,
		.pm = &max77801_pm_ops,
	},
	.probe = max77801_probe,
	.id_table = max77801_id,
	.shutdown = max77801_shutdown,
};
static int __init max77801_init(void)
{
	return i2c_add_driver(&max77801_i2c_driver);
}
subsys_initcall(max77801_init);

static void __exit max77801_exit(void)
{
	i2c_del_driver(&max77801_i2c_driver);
}
module_exit(max77801_exit);

MODULE_DESCRIPTION("max77801 regulator driver");
MODULE_AUTHOR("CTCaer <ctcaer@gmail.com>");
MODULE_LICENSE("GPL v2");
