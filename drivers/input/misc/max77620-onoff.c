// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 Billy Laws
// Author: Billy Laws <blaws05@gmail.com>
//
// ONOFF driver for MAXIM 77620 charger/power-supply.

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77620.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct max77620_onoff {
	struct input_dev *input;
	int irq_f, irq_r;
	unsigned int code;
};

static const struct regmap_irq max77620_onoff_irqs[] = {
	[0] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_MRWRN,
		.reg_offset = 0,
	},
	[1] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_EN0_1SEC,
		.reg_offset = 0,
	},
	[2] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_EN0_F,
		.reg_offset = 0,
	},
	[3] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_EN0_R,
		.reg_offset = 0,
	},	
	[4] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_ACOK_F,
		.reg_offset = 0,
	},
	[5] = {
		.mask = MAX77620_IRQ_LVL2_ONOFF_ACOK_R,
		.reg_offset = 0,
	},
};

static struct regmap_irq_chip max77620_onoff_irq_chip = {
	.name = "max77620-onoff",
	.irqs = max77620_onoff_irqs,
	.num_irqs = ARRAY_SIZE(max77620_onoff_irqs),
	.num_regs = 1,
	.irq_reg_stride = 1,
	.status_base = MAX77620_REG_ONOFFIRQ,
	.mask_base = MAX77620_REG_ONOFFIRQM,
};

static irqreturn_t max77620_onoff_falling(int irq, void *data)
{
	struct max77620_onoff *onoff = data;

	input_report_key(onoff->input, onoff->code, 0);
	input_sync(onoff->input);

	return IRQ_HANDLED;
}

static irqreturn_t max77620_onoff_rising(int irq, void *data)
{
	struct max77620_onoff *onoff = data;

	input_report_key(onoff->input, onoff->code, 1);
	input_sync(onoff->input);

	return IRQ_HANDLED;
}

static int max77620_onoff_probe(struct platform_device *pdev)
{
	struct max77620_onoff *onoff;
	struct device *dev, *parent;
	struct max77620_chip *chip =  dev_get_drvdata(pdev->dev.parent);
	struct regmap *map;
	int irq, ret;

	dev = &pdev->dev;
	parent = dev->parent;

	map = dev_get_regmap(parent, NULL);
	if (!map)
		return -ENODEV;

	onoff = devm_kzalloc(dev, sizeof(*onoff), GFP_KERNEL);
	if (!onoff)
		return -ENOMEM;

	onoff->code = KEY_SLEEP;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	onoff->input = devm_input_allocate_device(dev);
	if (!onoff->input)
		return -ENOMEM;

	onoff->input->name = "max77620_onoff";
	onoff->input->phys = "max77620_onoff/input0";
	onoff->input->id.bustype = BUS_I2C;
	input_set_capability(onoff->input, EV_KEY, onoff->code);

	ret = devm_regmap_add_irq_chip(&pdev->dev, map, irq,
				       IRQF_ONESHOT, -1,
				       &max77620_onoff_irq_chip,
				       &chip->onoff_irq_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add onoff irq_chip %d\n", ret);
		return ret;
	}
	
	onoff->irq_f = regmap_irq_get_virq(chip->onoff_irq_data, 2);
	onoff->irq_r = regmap_irq_get_virq(chip->onoff_irq_data, 3);


	ret = devm_request_any_context_irq(dev, onoff->irq_f, max77620_onoff_falling,
					     IRQF_ONESHOT, "en0-down", onoff);
	if (ret < 0)
		return ret;

	ret = devm_request_any_context_irq(dev, onoff->irq_r, max77620_onoff_rising,
					     IRQF_ONESHOT, "en0-up", onoff);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, onoff);
	input_set_drvdata(onoff->input, onoff);
	device_init_wakeup(dev, 1);
	return input_register_device(onoff->input);
}

static int __maybe_unused max77620_onoff_suspend(struct device *dev)
{
	struct max77620_onoff *onoff = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		return enable_irq_wake(onoff->irq_r);
	}

	return 0;
}

static int __maybe_unused max77620_onoff_resume(struct device *dev)
{	
	struct max77620_onoff *onoff = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		return disable_irq_wake(onoff->irq_r);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(max77620_onoff_pm_ops, max77620_onoff_suspend, max77620_onoff_resume);

static struct platform_driver max77620_onoff_driver = {
	.driver = {
		.name = "max77620-onoff",
	},
	.probe = max77620_onoff_probe,
};
module_platform_driver(max77620_onoff_driver);

MODULE_DESCRIPTION("MAXIM 77620 ONOFF driver");
MODULE_AUTHOR("Billy Laws <blaws05@gmail.com>");
MODULE_LICENSE("GPL v2");
