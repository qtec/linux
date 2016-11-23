/*
 * Qtechnology apu_revb_pic 0.0.5
 * Voltage sensor
 *
 * Copyright (C) 2014 Qtechnology A/S, Denmark
 * Author: Ricardo Ribalda Delgado <ricardo.ribalda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/watchdog.h>
#include <linux/delay.h>

#define DRIVER_NAME "qtec-hwmon"

#define F_P(x) ((x*95)/100),((x*105)/100)

static struct volt_info {
	char label[16];
	int min;
	int max;
} volt_info[] = {
	{"VIN_SENSE",9000,28000},
	{"VIN",9000,28000},
	{"3V3_S5",F_P(3300)},
	{"1V1_S5",F_P(1100)},
	{"VDDIO_MEM",F_P(1500)},
	{"5V_S0",F_P(5000)},
	{"3V3_S0",F_P(3300)},
	{"1V8_S0",F_P(1800)},
	{"1V1_S0",F_P(1100)},
	{"1V05_S0",F_P(1050)},
	{"VDDCR_NB",0,1500},
	{"VDDCR_CPU",0,1500},
};

struct hwmon_data {
	struct i2c_client *client;
	struct watchdog_device wd;
	int voltages[ARRAY_SIZE(volt_info)];

	struct mutex update_lock;
	unsigned long last_updated;
	bool valid;
};

#define CMD_VOLT 5
static struct hwmon_data *hwmon_update(struct device *dev)
{
	struct hwmon_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const unsigned long interval = HZ;

	if (time_after(jiffies, data->last_updated + interval) ||
		!data->valid) {
		__u8 values[I2C_SMBUS_BLOCK_MAX];
		int i;
		int retries=5;

		while (retries--)
			if (i2c_smbus_read_block_data(client,CMD_VOLT,values)> 2*ARRAY_SIZE(volt_info))
				break;

		if (retries==0){
			mutex_unlock(&data->update_lock);
			return NULL;
		}

		for (i=0;i<ARRAY_SIZE(volt_info);i++)
			data->voltages[i] = values[2*i] | (values[2*i+1]<<8);

		data->last_updated = jiffies;
		data->valid = true;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

static ssize_t show_input(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;
	struct hwmon_data *data=hwmon_update(dev);

	if (!data)
		return -EIO;

	return sprintf(buf, "%d\n", data->voltages[nr]);
}

static ssize_t show_label(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;

	return sprintf(buf, "%s\n", volt_info[nr].label);
}

static ssize_t show_min(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;

	return sprintf(buf, "%d\n", volt_info[nr].min);
}

static ssize_t show_max(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;

	return sprintf(buf, "%d\n", volt_info[nr].max);
}

static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_label, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_label, NULL, 1);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO, show_label, NULL, 2);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO, show_label, NULL, 3);
static SENSOR_DEVICE_ATTR(in5_label, S_IRUGO, show_label, NULL, 4);
static SENSOR_DEVICE_ATTR(in6_label, S_IRUGO, show_label, NULL, 5);
static SENSOR_DEVICE_ATTR(in7_label, S_IRUGO, show_label, NULL, 6);
static SENSOR_DEVICE_ATTR(in8_label, S_IRUGO, show_label, NULL, 7);
static SENSOR_DEVICE_ATTR(in9_label, S_IRUGO, show_label, NULL, 8);
static SENSOR_DEVICE_ATTR(in10_label, S_IRUGO, show_label, NULL, 9);
static SENSOR_DEVICE_ATTR(in11_label, S_IRUGO, show_label, NULL, 10);
static SENSOR_DEVICE_ATTR(in12_label, S_IRUGO, show_label, NULL, 11);

static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_input, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_input, NULL, 1);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_input, NULL, 2);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_input, NULL, 3);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, show_input, NULL, 4);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, show_input, NULL, 5);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, show_input, NULL, 6);
static SENSOR_DEVICE_ATTR(in8_input, S_IRUGO, show_input, NULL, 7);
static SENSOR_DEVICE_ATTR(in9_input, S_IRUGO, show_input, NULL, 8);
static SENSOR_DEVICE_ATTR(in10_input, S_IRUGO, show_input, NULL, 9);
static SENSOR_DEVICE_ATTR(in11_input, S_IRUGO, show_input, NULL, 10);
static SENSOR_DEVICE_ATTR(in12_input, S_IRUGO, show_input, NULL, 11);

static SENSOR_DEVICE_ATTR(in1_min, S_IRUGO, show_min, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_min, S_IRUGO, show_min, NULL, 1);
static SENSOR_DEVICE_ATTR(in3_min, S_IRUGO, show_min, NULL, 2);
static SENSOR_DEVICE_ATTR(in4_min, S_IRUGO, show_min, NULL, 3);
static SENSOR_DEVICE_ATTR(in5_min, S_IRUGO, show_min, NULL, 4);
static SENSOR_DEVICE_ATTR(in6_min, S_IRUGO, show_min, NULL, 5);
static SENSOR_DEVICE_ATTR(in7_min, S_IRUGO, show_min, NULL, 6);
static SENSOR_DEVICE_ATTR(in8_min, S_IRUGO, show_min, NULL, 7);
static SENSOR_DEVICE_ATTR(in9_min, S_IRUGO, show_min, NULL, 8);
static SENSOR_DEVICE_ATTR(in10_min, S_IRUGO, show_min, NULL, 9);
static SENSOR_DEVICE_ATTR(in11_min, S_IRUGO, show_min, NULL, 10);
static SENSOR_DEVICE_ATTR(in12_min, S_IRUGO, show_min, NULL, 11);

static SENSOR_DEVICE_ATTR(in1_max, S_IRUGO, show_max, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_max, S_IRUGO, show_max, NULL, 1);
static SENSOR_DEVICE_ATTR(in3_max, S_IRUGO, show_max, NULL, 2);
static SENSOR_DEVICE_ATTR(in4_max, S_IRUGO, show_max, NULL, 3);
static SENSOR_DEVICE_ATTR(in5_max, S_IRUGO, show_max, NULL, 4);
static SENSOR_DEVICE_ATTR(in6_max, S_IRUGO, show_max, NULL, 5);
static SENSOR_DEVICE_ATTR(in7_max, S_IRUGO, show_max, NULL, 6);
static SENSOR_DEVICE_ATTR(in8_max, S_IRUGO, show_max, NULL, 7);
static SENSOR_DEVICE_ATTR(in9_max, S_IRUGO, show_max, NULL, 8);
static SENSOR_DEVICE_ATTR(in10_max, S_IRUGO, show_max, NULL, 9);
static SENSOR_DEVICE_ATTR(in11_max, S_IRUGO, show_max, NULL, 10);
static SENSOR_DEVICE_ATTR(in12_max, S_IRUGO, show_max, NULL, 11);

static struct attribute *hwmon_attrs[] = {
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in3_label.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
	&sensor_dev_attr_in5_label.dev_attr.attr,
	&sensor_dev_attr_in6_label.dev_attr.attr,
	&sensor_dev_attr_in7_label.dev_attr.attr,
	&sensor_dev_attr_in8_label.dev_attr.attr,
	&sensor_dev_attr_in9_label.dev_attr.attr,
	&sensor_dev_attr_in10_label.dev_attr.attr,
	&sensor_dev_attr_in11_label.dev_attr.attr,
	&sensor_dev_attr_in12_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	&sensor_dev_attr_in8_input.dev_attr.attr,
	&sensor_dev_attr_in9_input.dev_attr.attr,
	&sensor_dev_attr_in10_input.dev_attr.attr,
	&sensor_dev_attr_in11_input.dev_attr.attr,
	&sensor_dev_attr_in12_input.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	&sensor_dev_attr_in5_min.dev_attr.attr,
	&sensor_dev_attr_in6_min.dev_attr.attr,
	&sensor_dev_attr_in7_min.dev_attr.attr,
	&sensor_dev_attr_in8_min.dev_attr.attr,
	&sensor_dev_attr_in9_min.dev_attr.attr,
	&sensor_dev_attr_in10_min.dev_attr.attr,
	&sensor_dev_attr_in11_min.dev_attr.attr,
	&sensor_dev_attr_in12_min.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in5_max.dev_attr.attr,
	&sensor_dev_attr_in6_max.dev_attr.attr,
	&sensor_dev_attr_in7_max.dev_attr.attr,
	&sensor_dev_attr_in8_max.dev_attr.attr,
	&sensor_dev_attr_in9_max.dev_attr.attr,
	&sensor_dev_attr_in10_max.dev_attr.attr,
	&sensor_dev_attr_in11_max.dev_attr.attr,
	&sensor_dev_attr_in12_max.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(hwmon);

#define CMD_WD 6

static int wdt_set(struct watchdog_device *wdd, int val)
{
	struct hwmon_data *data = watchdog_get_drvdata(wdd);

	return i2c_smbus_write_word_data(data->client, CMD_WD, val);
}
static int wdt_start(struct watchdog_device *wdd)
{
	return wdt_set(wdd, wdd->timeout);
}

static int wdt_stop(struct watchdog_device *wdd)
{
	return wdt_set(wdd, 0);
}

static int wdt_set_timeout(struct watchdog_device *wdd, unsigned new_timeout)
{
	wdd->timeout = new_timeout;
	return wdt_start(wdd);
}

static const struct watchdog_info hwmon_wdt_ident = {
	.options = WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "QTEC HWMON Watchdog",
};

static const struct watchdog_ops hwmon_wdt_ops = {
	.owner = THIS_MODULE,
	.start = wdt_start,
	.stop = wdt_stop,
	.set_timeout = wdt_set_timeout,
};

static int hwmon_remove(struct i2c_client *client)
{
	struct hwmon_data *data = i2c_get_clientdata(client);
	watchdog_unregister_device(&data->wd);
	return 0;
}

static int hwmon_probe(struct i2c_client *client,
		      const struct i2c_device_id *id)
{
	struct hwmon_data *data;
	struct i2c_adapter *adap = client->adapter;
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	int ret;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_READ_WORD_DATA  )) {
		dev_err(dev, "Bus does not support required transaction types\n");
		return -ENODEV;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	data->client = client;
	data->valid = false;
	mutex_init(&data->update_lock);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							   client->name,
							   data,
							   hwmon_groups);
	if (IS_ERR(hwmon_dev)){
		dev_err(dev, "unable to register hwmon device\n");
		return -EIO;
	}

	data->wd.info = &hwmon_wdt_ident;
	data->wd.ops = &hwmon_wdt_ops;
	data->wd.info = &hwmon_wdt_ident;
	data->wd.timeout = 10;
	data->wd.min_timeout = 1;
	data->wd.max_timeout = 0xffff;
	data->wd.status = WATCHDOG_NOWAYOUT_INIT_STATUS;
	watchdog_set_drvdata(&data->wd, data);
	ret = watchdog_register_device(&data->wd);
	if (ret < 0){
		dev_dbg(dev, "unable to register watchdog device\n");
	}

	dev_info(hwmon_dev, "loading %s at %d, 0x%02x\n",
		client->name, i2c_adapter_id(client->adapter),
		client->addr);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* device ID table */
static const struct i2c_device_id hwmon_id[] = {
	{ "apurevb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hwmon_id);

static int hwmon_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	s32 id=0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_READ_WORD_DATA ))
		return -ENODEV;

	id = i2c_smbus_read_word_data(client,0);
	if (id != 0x0706){
		dev_warn(&adapter->dev, "Error loading %s at %d, 0x%02x: Invalid PIC version 0x%.2x, Retrying\n",
			client->name, i2c_adapter_id(client->adapter),
			client->addr,id);
		msleep(100);
		id = i2c_smbus_read_word_data(client,0);
	}
	if (id != 0x0706) {
		dev_warn(&adapter->dev, "Error loading %s at %d, 0x%02x: Invalid PIC version 0x%.2x\n",
			client->name, i2c_adapter_id(client->adapter),
			client->addr,id);
		return -ENODEV;
	}

	strlcpy(info->type, hwmon_id[0].name, I2C_NAME_SIZE);

	dev_info(&adapter->dev, "loading %s at %d, 0x%02x\n",
		client->name, i2c_adapter_id(client->adapter),
		client->addr);

	return 0;
}
static const unsigned short hwmon_addrs[] = { (0xec/2), I2C_CLIENT_END };

static struct i2c_driver hwmon_i2c_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver.name  = DRIVER_NAME,
	.probe        = hwmon_probe,
	.remove       = hwmon_remove,
	.id_table     = hwmon_id,
	.detect       = hwmon_detect,
	.address_list = hwmon_addrs,
};

module_i2c_driver(hwmon_i2c_driver);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("Qtechnology voltage sensor and watchdog");
MODULE_LICENSE("GPL");
