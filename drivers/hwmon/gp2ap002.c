/*
 *  gp2ap002.c - Proximity/Ambient light sensor
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *  Suchang Woo <suchang.woo@samsung.com>
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gp2ap002.h>

#define GP2AP002_PROX		0x00	/* Read Only */
#define GP2AP002_GAIN		0x01
#define GP2AP002_HYS		0x02
#define GP2AP002_CYCLE		0x03
#define GP2AP002_OPMOD		0x04
#define GP2AP002_CON		0x06

#define PROX_VO_NO_DETECT	(0 << 0)
#define PROX_VO_DETECT		(1 << 0)

#define GAIN_LED0_SMALL		(0 << 3)
#define GAIN_LED0_LARGE		(1 << 3)

#define HYS_HYSD		(1 << 7)
#define HYS_HYSC1		(1 << 6)
#define HYS_HYSC0		(1 << 5)
#define HYS_HYSF3		(1 << 3)
#define HYS_HYSF2		(1 << 2)
#define HYS_HYSF1		(1 << 1)
#define HYS_HYSF0		(1 << 0)

#define OPMOD_SSD_SHUTDOWN	(0 << 0)
#define OPMOD_SSD_OPERATING	(1 << 0)
#define OPMOD_VCON_NORMAL	(0 << 1)
#define OPMOD_VCON_IRQ		(1 << 1)

#define CON_OCON1		(1 << 4)
#define CON_OCON0		(1 << 3)

#define GP2AP002_MAX_LUX	10

static int lux_table[GP2AP002_MAX_LUX] = {
	1, 165, 288, 497, 869, 1532, 2692, 4692, 8280, 100000,
};

struct gp2ap002_chip {
	struct i2c_client	*client;
	struct device		*dev;
	struct input_dev	*idev;
	struct work_struct	work;
	struct mutex		lock;

	void (*power_enable)(int onoff);
	int (*get_adc)(void);
	int vo_gpio;

	/* Proximity */
	int enable;
	int mode;
	int vo;
	/* Ambient Light */
	int adc;
	int level;
};

static int gp2ap002_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int gp2ap002_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void gp2ap002_get_vo(struct gp2ap002_chip *chip)
{
	if (chip->mode == OPMOD_VCON_IRQ) {
		chip->vo = gp2ap002_read_reg(chip->client,
				GP2AP002_PROX) & PROX_VO_DETECT;
	} else {
		chip->vo = !gpio_get_value(chip->vo_gpio);
	}
}

static void gp2ap002_get_adc(struct gp2ap002_chip *chip)
{
	if (chip->get_adc)
		chip->adc = chip->get_adc();
}

static void gp2ap002_get_level(struct gp2ap002_chip *chip)
{
	int i;

	gp2ap002_get_adc(chip);

	for (i = 0; i < GP2AP002_MAX_LUX; i++) {
		if (lux_table[i] > chip->adc) {
			chip->level = i;
			break;
		}
	}
}

static void gp2ap002_set_mode(struct gp2ap002_chip *chip, int enable)
{
	if (enable == chip->enable)
		return;

	chip->enable = enable;
	chip->vo = 0;

	gp2ap002_write_reg(chip->client, GP2AP002_OPMOD, OPMOD_SSD_SHUTDOWN);

	if (enable) {
		gp2ap002_write_reg(chip->client, GP2AP002_OPMOD,
				OPMOD_SSD_OPERATING | chip->mode);
		gp2ap002_get_vo(chip);
	}
}

static void gp2ap002_set_enable(struct gp2ap002_chip *chip, const char *buf)
{
	if (!strncmp(buf, "1", 1))
		gp2ap002_set_mode(chip, 1);
	else if (!strncmp(buf, "0", 1))
		gp2ap002_set_mode(chip, 0);
}

static void gp2ap002_update_data(struct gp2ap002_chip *chip)
{
	gp2ap002_get_vo(chip);
	enable_irq(chip->client->irq);
}

static void gp2ap002_work(struct work_struct *work)
{
	struct gp2ap002_chip *chip = container_of(work,
			struct gp2ap002_chip, work);

	mutex_lock(&chip->lock);

	gp2ap002_update_data(chip);

	input_report_abs(chip->idev, ABS_DISTANCE, chip->vo);
	input_sync(chip->idev);

	mutex_unlock(&chip->lock);
}

static irqreturn_t gp2ap002_irq(int irq, void *data)
{
	struct gp2ap002_chip *chip = data;

	if (!work_pending(&chip->work)) {
		disable_irq_nosync(irq);
		schedule_work(&chip->work);
	} else {
		dev_err(&chip->client->dev, "work pending\n");
	}

	return IRQ_HANDLED;
}

#define GP2AP002_OUTPUT(name, field)					\
static ssize_t gp2ap002_show_##name(struct device *dev,			\
		struct device_attribute *attr, char *buf)		\
{									\
	struct gp2ap002_chip *chip = dev_get_drvdata(dev);		\
	gp2ap002_get_##name(chip);					\
	return sprintf(buf, "%d\n", chip->field);			\
}									\
static SENSOR_DEVICE_ATTR(name, S_IRUGO, gp2ap002_show_##name, NULL, 0);

#define GP2AP002_INPUT(name, field)					\
static ssize_t gp2ap002_store_##name(struct device *dev,		\
	struct device_attribute *attr, const char *buf, size_t count)	\
{									\
	struct gp2ap002_chip *chip = dev_get_drvdata(dev);		\
	if (!count)							\
		return -EINVAL;						\
	gp2ap002_set_##name(chip, buf);					\
	return count;							\
}									\
static ssize_t gp2ap002_show_##name(struct device *dev,			\
		struct device_attribute *attr, char *buf)		\
{									\
	struct gp2ap002_chip *chip = dev_get_drvdata(dev);		\
	return sprintf(buf, "%d\n", chip->field);			\
}									\
static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR,			\
		gp2ap002_show_##name, gp2ap002_store_##name, 0);

GP2AP002_OUTPUT(vo, vo);
GP2AP002_INPUT(enable, enable);
GP2AP002_OUTPUT(adc, adc);
GP2AP002_OUTPUT(level, level);

static struct attribute *proximity_attributes[] = {
	&sensor_dev_attr_vo.dev_attr.attr,
	&sensor_dev_attr_enable.dev_attr.attr,
	NULL
};

static struct attribute *ambient_attributes[] = {
	&sensor_dev_attr_adc.dev_attr.attr,
	&sensor_dev_attr_level.dev_attr.attr,
	NULL
};

static const struct attribute_group proximity_group = {
	.name = "proximity",
	.attrs = proximity_attributes,
};

static const struct attribute_group ambient_group = {
	.name = "ambient",
	.attrs = ambient_attributes,
};

static void gp2ap002_unregister_input_device(struct gp2ap002_chip *chip)
{
	struct i2c_client *client = chip->client;

	if (client->irq > 0)
		free_irq(client->irq, chip);

	input_unregister_device(chip->idev);
	input_free_device(chip->idev);
}

static int gp2ap002_register_input_device(struct gp2ap002_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct input_dev *idev;
	int ret;

	idev = chip->idev = input_allocate_device();
	if (!idev) {
		dev_err(&client->dev, "allocating input device is failed\n");
		ret = -ENOMEM;
		goto error_alloc;
	}

	idev->name = "GP2AP002 OpticalSensor";
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_DISTANCE, 0, 1, 0, 0);

	input_set_drvdata(idev, chip);

	ret = input_register_device(idev);
	if (ret) {
		dev_err(&client->dev, "registering input device is failed\n");
		goto error_reg;
	}

	if (client->irq > 0) {
		unsigned long irq_flag = IRQF_DISABLED;

		if (chip->mode == OPMOD_VCON_IRQ)
			irq_flag |= IRQF_TRIGGER_FALLING;
		else
			irq_flag |= IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;

		ret = request_irq(client->irq, gp2ap002_irq, irq_flag,
				"GP2AP002 OpticalSensor", chip);
		if (ret) {
			dev_err(&client->dev, "can't get IRQ %d, ret %d\n",
					client->irq, ret);
			goto error_irq;
		}
	}

	return 0;

error_irq:
	input_unregister_device(idev);
error_reg:
	input_free_device(idev);
error_alloc:
	return ret;
}


static int gp2ap002_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct gp2ap002_chip *chip;
	struct gp2ap002_platform_data *pdata;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(struct gp2ap002_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pdata = client->dev.platform_data;

	chip->client = client;
	i2c_set_clientdata(client, chip);

	chip->dev = hwmon_device_register(&client->dev);
	if (IS_ERR(chip->dev)) {
		dev_err(&client->dev,
				"Registering to hwmon device is failed\n");
		ret = PTR_ERR(chip->dev);
		goto error_hwmon;
	}

	ret = sysfs_create_group(&client->dev.kobj, &proximity_group);
	if (ret) {
		dev_err(&client->dev,
				"Creating proximity attribute group failed\n");
		goto error_sysfs1;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ambient_group);
	if (ret) {
		dev_err(&client->dev,
				"Creating light attribute group failed\n");
		goto error_sysfs2;
	}

	ret = gp2ap002_register_input_device(chip);
	if (ret) {
		dev_err(&client->dev, "Registering input device is failed\n");
		goto error_input;
	}

	chip->power_enable	= pdata->power_enable;
	chip->get_adc		= pdata->get_adc;
	chip->vo_gpio		= pdata->vo_gpio;
	chip->mode		= pdata->prox_mode;

	INIT_WORK(&chip->work, gp2ap002_work);
	mutex_init(&chip->lock);

	if (chip->power_enable)
		chip->power_enable(0);

	gp2ap002_set_mode(chip, pdata->enable);

	return 0;

error_input:
	sysfs_remove_group(&client->dev.kobj, &ambient_group);
error_sysfs2:
	sysfs_remove_group(&client->dev.kobj, &proximity_group);
error_sysfs1:
	hwmon_device_unregister(chip->dev);
error_hwmon:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return ret;
}

static int __exit gp2ap002_remove(struct i2c_client *client)
{
	struct gp2ap002_chip *chip = i2c_get_clientdata(client);

	gp2ap002_unregister_input_device(chip);
	hwmon_device_unregister(chip->dev);

	sysfs_remove_group(&client->dev.kobj, &proximity_group);
	sysfs_remove_group(&client->dev.kobj, &ambient_group);

	i2c_set_clientdata(client, NULL);

	if (chip->power_enable)
		chip->power_enable(0);

	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int gp2ap002_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct gp2ap002_chip *chip = i2c_get_clientdata(client);
	gp2ap002_set_mode(chip, 0);

	return 0;
}

static int gp2ap002_resume(struct i2c_client *client)
{
	struct gp2ap002_chip *chip = i2c_get_clientdata(client);
	gp2ap002_set_mode(chip, 1);

	return 0;
}

#else

#define gp2ap002_suspend NULL
#define gp2ap002_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id gp2ap002_id[] = {
	{ "GP2AP002", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gp2ap002_id);

static struct i2c_driver gp2ap002_i2c_driver = {
	.driver = {
		.name	= "GP2AP002",
	},
	.probe		= gp2ap002_probe,
	.remove		= __exit_p(gp2ap002_remove),
	.suspend	= gp2ap002_suspend,
	.resume		= gp2ap002_resume,
	.id_table	= gp2ap002_id,
};

static int __init gp2ap002_init(void)
{
	return i2c_add_driver(&gp2ap002_i2c_driver);
}
module_init(gp2ap002_init);

static void __exit gp2ap002_exit(void)
{
	i2c_del_driver(&gp2ap002_i2c_driver);
}
module_exit(gp2ap002_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("GP2AP002 Proximity/Ambient Light Sensor driver");
MODULE_LICENSE("GPL");
