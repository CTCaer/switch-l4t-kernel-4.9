/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Billy Laws
 * Copyright (C) 2020 CTCaer
 *
 * Author: 
 *  Billy Laws <blaws05@gmail.com>
 *  CTCaer <ctcaer@gmail.com>
 *
 * Description:
 *  ONOFF driver for MAXIM 77620 charger/power-supply.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77620.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define ONOFF_IRQF 2
#define ONOFF_IRQR 3

/* MAX77620 IRQ ONOFF interrupts */
#define MAX77620_IRQ_ONOFF_MRWRN	0
#define MAX77620_IRQ_ONOFF_EN0_1SEC	1
#define MAX77620_IRQ_ONOFF_EN0_F	2
#define MAX77620_IRQ_ONOFF_EN0_R	3
#define MAX77620_IRQ_ONOFF_LID_F	4
#define MAX77620_IRQ_ONOFF_LID_R	5
#define MAX77620_IRQ_ONOFF_ACOK_F	6
#define MAX77620_IRQ_ONOFF_ACOK_R	7

struct max77620_onoff {
	struct regmap *rmap;
	struct input_dev *input;
	int irq_f, irq_r;
	unsigned int code;
	bool suspended;
};

static const struct regmap_irq max77620_onoff_irqs[] = {
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_MRWRN, 0, MAX77620_IRQ_ONOFF_MRWRN_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_EN0_1SEC, 0, MAX77620_IRQ_ONOFF_EN0_1SEC_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_EN0_F, 0, MAX77620_IRQ_ONOFF_EN0_F_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_EN0_R, 0, MAX77620_IRQ_ONOFF_EN0_R_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_LID_F, 0, MAX77620_IRQ_ONOFF_LID_F_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_LID_R, 0, MAX77620_IRQ_ONOFF_LID_R_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_ACOK_F, 0, MAX77620_IRQ_ONOFF_ACOK_F_MASK),
	REGMAP_IRQ_REG(MAX77620_IRQ_ONOFF_ACOK_R, 0, MAX77620_IRQ_ONOFF_ACOK_R_MASK),
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

static irqreturn_t max77620_onoff_handler(int irq, void *data)
{
	struct max77620_onoff *onoff = data;
	bool pressed = irq == onoff->irq_r;

	/* Skip release report in suspended state */
	if (onoff->suspended && !pressed)
		return IRQ_HANDLED;

	input_report_key(onoff->input, onoff->code, pressed ? 1 : 0);
	input_sync(onoff->input);

	return IRQ_HANDLED;
}

static int max77620_onoff_probe(struct platform_device *pdev)
{
	struct max77620_onoff *onoff;
	struct device *dev, *parent;
	struct max77620_chip *chip =  dev_get_drvdata(pdev->dev.parent);
	struct device_node *np;
	unsigned int code;
	int irq, ret;

	dev = &pdev->dev;
	parent = dev->parent;

	onoff = devm_kzalloc(dev, sizeof(*onoff), GFP_KERNEL);
	if (!onoff)
		return -ENOMEM;

	onoff->rmap = chip->rmap;

	np = of_get_child_by_name(pdev->dev.parent->of_node, "onoff");
	if (np && !of_device_is_available(np))
		np = NULL;

	if (np)
		ret = of_property_read_u32(np, "maxim,onoff-keycode",
				   &code);
	if (!np || ret < 0)
		onoff->code = KEY_POWER;
	else
		onoff->code = code;

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

	ret = devm_regmap_add_irq_chip(&pdev->dev, onoff->rmap, irq,
				       IRQF_ONESHOT, -1,
				       &max77620_onoff_irq_chip,
				       &chip->onoff_irq_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add onoff irq_chip %d\n", ret);
		goto fail;
	}

	onoff->irq_f = regmap_irq_get_virq(chip->onoff_irq_data,
					   MAX77620_IRQ_ONOFF_EN0_F);
	onoff->irq_r = regmap_irq_get_virq(chip->onoff_irq_data,
					   MAX77620_IRQ_ONOFF_EN0_R);

	ret = devm_request_any_context_irq(dev, onoff->irq_f,
					   max77620_onoff_handler,
					   IRQF_ONESHOT, "en0-down", onoff);
	if (ret < 0)
		goto fail;

	ret = devm_request_any_context_irq(dev, onoff->irq_r,
					   max77620_onoff_handler,
					   IRQF_ONESHOT, "en0-up", onoff);
	if (ret < 0)
		goto fail;

	platform_set_drvdata(pdev, onoff);
	input_set_drvdata(onoff->input, onoff);
	device_init_wakeup(dev, 1);

	ret = input_register_device(onoff->input);

fail:
	return ret;
}

#ifdef CONFIG_PM_SLEEP

static int max77620_onoff_suspend(struct device *dev)
{
	struct max77620_onoff *onoff = dev_get_drvdata(dev);

	onoff->suspended = true;

	if (device_may_wakeup(dev))
		return enable_irq_wake(onoff->irq_f);

	return 0;
}

static int max77620_onoff_resume(struct device *dev)
{
	struct max77620_onoff *onoff = dev_get_drvdata(dev);
	unsigned int val, ret;

	/* Clear ON/OFF interrupts */
	ret = regmap_read(onoff->rmap, MAX77620_REG_ONOFFIRQ, &val);
	if (ret < 0)
		dev_err(dev, "Failed to clear onoff irq: %d\n", ret);

	onoff->suspended = false;

	/* If ON/OFF was pressed, report it */
	if (val & MAX77620_IRQ_ONOFF_EN0_R_MASK) {
		input_report_key(onoff->input, onoff->code, 1);
		input_sync(onoff->input);
	}

	/* Unconditionally release after resume */
	input_report_key(onoff->input, onoff->code, 0);
	input_sync(onoff->input);

	if (device_may_wakeup(dev))
		return disable_irq_wake(onoff->irq_f);

	return 0;
}

static const struct dev_pm_ops max77620_onoff_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77620_onoff_suspend,
				max77620_onoff_resume)
};

#endif /* CONFIG_PM_SLEEP */

static struct platform_driver max77620_onoff_driver = {
	.driver = {
		.name = "max77620-onoff",
#ifdef CONFIG_PM_SLEEP
		.pm = &max77620_onoff_pm_ops,
#endif
	},
	.probe = max77620_onoff_probe,
};

static int __init max77620_onoff_init(void)
{
	return platform_driver_register(&max77620_onoff_driver);
}
subsys_initcall(max77620_onoff_init);

static void __exit max77620_onoff_exit(void)
{
	platform_driver_unregister(&max77620_onoff_driver);
}
module_exit(max77620_onoff_exit);

MODULE_DESCRIPTION("MAXIM 77620 ONOFF driver");
MODULE_AUTHOR("Billy Laws <blaws05@gmail.com>");
MODULE_LICENSE("GPL v2");
