/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MFD core driver for the RT5033
 *
 * Copyright (C) 2014 Samsung Electronics
 * Author: Beomho Seo <beomho.seo@samsung.com>
 */

#ifndef __RT5033_H__
#define __RT5033_H__

#include <linux/led-class-flash.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/* RT5033 regulator IDs */
enum rt5033_regulators {
	RT5033_BUCK = 0,
	RT5033_LDO,
	RT5033_SAFE_LDO,

	RT5033_REGULATOR_NUM,
};

struct rt5033_dev {
	struct device *dev;

	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;
	int irq;
	bool wakeup;
};

struct rt5033_battery {
	struct i2c_client	*client;
	struct rt5033_dev	*rt5033;
	struct regmap		*regmap;
	struct power_supply	*psy;
};

/* RT5033 charger platform data */
struct rt5033_charger_data {
	unsigned int pre_uamp;
	unsigned int pre_uvolt;
	unsigned int const_uvolt;
	unsigned int eoc_uamp;
	unsigned int fast_uamp;
};

struct rt5033_charger {
	struct device		*dev;
	struct rt5033_dev	*rt5033;
	struct power_supply	psy;

	struct rt5033_charger_data	*chg;
};

/* RT5033 led platform data */

struct rt5033_led_config_data {
	/* maximum LED current in movie mode */
	u32 torch_max_microamp;
	/* maximum LED current in flash mode */
	u32 flash_max_microamp;
	/* maximum flash timeout */
	u32 flash_max_timeout;
	/* max LED brightness level */
	enum led_brightness max_brightness;
};

struct rt5033_led {
	struct device		*dev;
	struct rt5033_dev	*rt5033;
	struct regmap		*regmap;

	/* Related LED class device */
	struct led_classdev	cdev;
};

#endif /* __RT5033_H__ */
