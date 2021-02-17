/*
 * led driver for RT5033
 *
 * Copyright (C) 2015 Samsung Electronics, Co., Ltd.
 * Ingi Kim <ingi2.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/mfd/rt5033.h>
#include <linux/mfd/rt5033-private.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define RT5033_LED_FLASH_TIMEOUT_MIN		64000
#define RT5033_LED_FLASH_TIMEOUT_STEPS		32000
#define RT5033_LED_TORCH_CURRENT_LEVEL_MAX	16

/* Macro for getting offset of flash timeout */
#define GET_TIMEOUT_OFFSET(tm, step)	((tm) / (step) - 2)

static struct rt5033_led *flcdev_to_led(
				struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct rt5033_led, fled_cdev);
}

static int rt5033_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct rt5033_led *led = flcdev_to_led(fled_cdev);

	if (!brightness) {
		regmap_update_bits(led->regmap, RT5033_REG_FL_FUNCTION2,
				   RT5033_FL_CTRL2_MASK, 0x0);
	} else {
		regmap_update_bits(led->regmap, RT5033_REG_FL_FUNCTION1,
				   RT5033_FL_FUNC1_MASK, RT5033_FL_PINCTRL);
		regmap_update_bits(led->regmap,	RT5033_REG_FL_CTRL1,
				   RT5033_FL_CTRL1_MASK, (brightness-1) << 4);
		regmap_update_bits(led->regmap, RT5033_REG_FL_FUNCTION2,
				   RT5033_FL_CTRL2_MASK, RT5033_FL_ENFLED);
	}

	return 0;
}

static void rt5033_init_flash_timeout(struct rt5033_led *led)
{
	struct led_flash_setting *setting;

	setting = &led->fled_cdev.timeout;
	setting->min = RT5033_LED_FLASH_TIMEOUT_MIN;
	setting->max = led->data->flash_max_timeout;
	setting->step = RT5033_LED_FLASH_TIMEOUT_STEPS;
	setting->val = led->data->flash_max_timeout;
}

static int rt5033_led_parse_dt(struct rt5033_led *led, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *child_node;
	struct rt5033_led_config_data *data;
	int ret = 0;

	if (!np)
		return -ENXIO;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	child_node = of_get_next_available_child(np, NULL);
	if (!child_node) {
		dev_err(dev, "DT child node isn't found\n");
		return -EINVAL;
	}

	led->fled_cdev.led_cdev.name =
		of_get_property(child_node, "label", NULL) ? : child_node->name;

	ret = of_property_read_u32(child_node, "led-max-microamp",
				   &data->torch_max_microamp);
	if (ret) {
		dev_err(dev, "failed to parse led-max-microamp\n");
		return ret;
	}

	ret = of_property_read_u32(child_node, "flash-max-microamp",
				   &data->flash_max_microamp);
	if (ret) {
		dev_err(dev, "failed to parse flash-max-microamp\n");
		return ret;
	}

	ret = of_property_read_u32(child_node, "flash-max-timeout-us",
				   &data->flash_max_timeout);
	if (ret)
		dev_err(dev, "failed to parse flash-max-timeout-us\n");

	of_node_put(child_node);
	led->data = data;

	return ret;
}

static int rt5033_led_flash_timeout_set(struct led_classdev_flash *fled_cdev,
					 u32 timeout)
{
	return 0;
}

static int rt5033_led_flash_strobe_set(struct led_classdev_flash *fled_cdev,
					bool state)
{
	struct rt5033_led *led = flcdev_to_led(fled_cdev);
	u32 flash_tm_reg;

	regmap_update_bits(led->regmap,	RT5033_REG_FL_FUNCTION1,
			   RT5033_FL_FUNC1_MASK, RT5033_FL_RESET);
	fled_cdev->led_cdev.brightness = LED_OFF;

	if (state) {
		flash_tm_reg = GET_TIMEOUT_OFFSET(fled_cdev->timeout.val,
						  fled_cdev->timeout.step);
		regmap_update_bits(led->regmap,	RT5033_REG_FL_STROBE_CTRL2,
				   RT5033_FL_STRBCTRL2_MASK, flash_tm_reg);
		regmap_update_bits(led->regmap,	RT5033_REG_FL_FUNCTION1,
				   RT5033_FL_FUNC1_MASK, RT5033_FL_STRB_SEL
				   | RT5033_FL_PINCTRL);
		regmap_update_bits(led->regmap,	RT5033_REG_FL_FUNCTION2,
				   RT5033_FL_FUNC2_MASK, RT5033_FL_ENFLED
				   | RT5033_FL_SREG_STRB);
	}

	return 0;
}

static const struct led_flash_ops flash_ops = {
	.strobe_set = rt5033_led_flash_strobe_set,
	.timeout_set = rt5033_led_flash_timeout_set,
};

static int rt5033_led_probe(struct platform_device *pdev)
{
	struct rt5033_dev *rt5033 = dev_get_drvdata(pdev->dev.parent);
	struct rt5033_led *led;
	struct led_classdev *led_cdev;
	int ret;

	led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	platform_set_drvdata(pdev, led);
	led->dev = &pdev->dev;
	led->regmap = rt5033->regmap;

	ret = rt5033_led_parse_dt(led, &pdev->dev);
	if (ret)
		return ret;

	rt5033_init_flash_timeout(led);
	led->fled_cdev.ops = &flash_ops;
	led_cdev = &led->fled_cdev.led_cdev;

	led_cdev->max_brightness = RT5033_LED_TORCH_CURRENT_LEVEL_MAX;
	led_cdev->brightness_set_sync = rt5033_led_brightness_set;
	led_cdev->flags |= LED_CORE_SUSPENDRESUME | LED_DEV_CAP_FLASH;

	ret = led_classdev_flash_register(&pdev->dev, &led->fled_cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't register LED %s\n", led_cdev->name);
		return ret;
	}

	regmap_update_bits(led->regmap,	RT5033_REG_FL_FUNCTION1,
			   RT5033_FL_FUNC1_MASK, RT5033_FL_RESET);

	return 0;
}

static int rt5033_led_remove(struct platform_device *pdev)
{
	struct rt5033_led *led = platform_get_drvdata(pdev);

	led_classdev_flash_unregister(&led->fled_cdev);

	return 0;
}

static const struct platform_device_id rt5033_led_id[] = {
	{ "rt5033-led", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, rt5033_led_id);

static const struct of_device_id rt5033_led_match[] = {
	{ .compatible = "richtek,rt5033-led", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rt5033_led_match);

static struct platform_driver rt5033_led_driver = {
	.driver = {
		.name = "rt5033-led",
		.of_match_table = rt5033_led_match,
	},
	.probe		= rt5033_led_probe,
	.id_table	= rt5033_led_id,
	.remove		= rt5033_led_remove,
};
module_platform_driver(rt5033_led_driver);

MODULE_AUTHOR("Ingi Kim <ingi2.kim@samsung.com>");
MODULE_DESCRIPTION("Richtek RT5033 LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rt5033-led");
