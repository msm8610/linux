/*
 * Fuel gauge driver for Richtek RT5033
 *
 * Copyright (C) 2014 Samsung Electronics, Co., Ltd.
 * Beomho Seo <beomho.seo@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published bythe Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/rt5033-private.h>
#include <linux/mfd/rt5033.h>

static int rt5033_battery_get_capacity(struct i2c_client *client)
{
	struct rt5033_battery *battery = i2c_get_clientdata(client);
	u32 msb;

	regmap_read(battery->regmap, RT5033_FUEL_REG_SOC_H, &msb);

	return msb;
}

static int rt5033_battery_get_present(struct i2c_client *client)
{
	struct rt5033_battery *battery = i2c_get_clientdata(client);
	u32 val;

	regmap_read(battery->regmap, RT5033_FUEL_REG_CONFIG_L, &val);

	return (val & RT5033_FUEL_BAT_PRESENT) ? true : false;
}

static int rt5033_battery_get_watt_prop(struct i2c_client *client,
		enum power_supply_property psp)
{
	struct rt5033_battery *battery = i2c_get_clientdata(client);
	unsigned int regh, regl;
	int ret;
	u32 msb, lsb;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		regh = RT5033_FUEL_REG_VBAT_H;
		regl = RT5033_FUEL_REG_VBAT_L;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		regh = RT5033_FUEL_REG_AVG_VOLT_H;
		regl = RT5033_FUEL_REG_AVG_VOLT_L;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		regh = RT5033_FUEL_REG_OCV_H;
		regl = RT5033_FUEL_REG_OCV_L;
		break;
	default:
		return -EINVAL;
	}

	regmap_read(battery->regmap, regh, &msb);
	regmap_read(battery->regmap, regl, &lsb);

	ret = ((msb << 4) + (lsb >> 4)) * 1250 / 1000;

	return ret;
}

static int rt5033_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct rt5033_battery *battery = container_of(psy,
				struct rt5033_battery, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = rt5033_battery_get_watt_prop(battery->client,
									psp);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = rt5033_battery_get_present(battery->client);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = rt5033_battery_get_capacity(battery->client);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property rt5033_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
};

static struct regmap_config rt5033_battery_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= RT5033_FUEL_REG_END,
};

static int rt5033_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rt5033_battery *battery;
	u32 ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -EINVAL;

	battery->client = client;
	battery->regmap = devm_regmap_init_i2c(client,
			&rt5033_battery_regmap_config);
	if (IS_ERR(battery->regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	i2c_set_clientdata(client, battery);

	battery->psy.name		= "rt5033-battery";
	battery->psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	battery->psy.get_property	= rt5033_battery_get_property;
	battery->psy.properties		= rt5033_battery_props;
	battery->psy.num_properties	= ARRAY_SIZE(rt5033_battery_props);

	ret = power_supply_register(&client->dev, &battery->psy);
	if (ret) {
		dev_err(&client->dev, "Failed to register power supply\n");
		return ret;
	}

	return 0;
}

static int rt5033_battery_remove(struct i2c_client *client)
{
	struct rt5033_battery *battery = i2c_get_clientdata(client);

	power_supply_unregister(&battery->psy);

	return 0;
}

static const struct i2c_device_id rt5033_battery_id[] = {
	{ "rt5033-battery", },
	{ }
};
MODULE_DEVICE_TABLE(platform, rt5033_battery_id);

static struct i2c_driver rt5033_battery_driver = {
	.driver = {
		.name = "rt5033-battery",
	},
	.probe = rt5033_battery_probe,
	.remove = rt5033_battery_remove,
	.id_table = rt5033_battery_id,
};
module_i2c_driver(rt5033_battery_driver);

MODULE_DESCRIPTION("Richtek RT5033 fuel gauge driver");
MODULE_AUTHOR("Beomho Seo <beomho.seo@samsung.com>");
MODULE_LICENSE("GPL");
