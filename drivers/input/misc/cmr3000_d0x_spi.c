/*
 * Implements SPI interface for VTI CMR300_D0x Accelerometer driver
 *
 * Copyright (C) 2011 Qtechnology
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com.com>
 * Based on:
 *	drivers/input/misc/cma3000_d0x_i2c.c by Hemanth V
 *	drivers/input/mis/adxl34x-spi.c	by Michael Hennerich
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/input/cmr3000.h>
#include "cmr3000_d0x.h"

enum { DO_READ = 0, DO_WRITE };

static int cmr3000_spi_cmd(struct spi_device *spi, u8 reg, u8 * val, int cmd,
			   char *msg)
{
	int ret;
	unsigned char tx_buf[2];
	unsigned char rx_buf[2];
	struct spi_transfer t = {
		.rx_buf = rx_buf,
		.tx_buf = tx_buf,
		.len = 2,
	};
	struct spi_message m;

	if (cmd == DO_WRITE) {
		tx_buf[0] = (reg << 2) | 2;
		tx_buf[1] = *val;
	} else {
		tx_buf[0] = reg << 2;
		tx_buf[1] = 0;
	}
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
	if (ret < 0) {
		dev_err(&spi->dev, "%s failed (%s, %d)\n", __func__, msg, ret);
		return ret;
	}
	if (cmd == DO_READ)
		*val = rx_buf[1];

	if (rx_buf[0] & 0x41)
		dev_err(&spi->dev,
			"%s Invalid Zero mask(0x%x)\n", __func__, rx_buf[0]);

	if ((rx_buf[0] & 0x2) != 0x2)
		dev_err(&spi->dev,
			"%s Invalid One mask (0x%x)\n", __func__, rx_buf[0]);

	return 0;
}

static int cmr3000_spi_write(struct device *dev, u8 reg, u8 val, char *msg)
{

	struct spi_device *spi = to_spi_device(dev);

	return cmr3000_spi_cmd(spi, reg, &val, DO_WRITE, msg);
}

static int cmr3000_spi_read(struct device *dev, u8 reg, char *msg)
{
	struct spi_device *spi = to_spi_device(dev);
	int ret;
	u8 val;

	ret = cmr3000_spi_cmd(spi, reg, &val, DO_READ, msg);
	if (ret)
		return ret;
	return val;
}

static const struct cmr3000_bus_ops cmr3000_spi_bops = {
	.bustype = BUS_SPI,
#define CMR3000_BUSSPI     (1 << 4)
	.ctrl_mod = CMR3000_BUSSPI,
	.read = cmr3000_spi_read,
	.write = cmr3000_spi_write,
};

static int cmr3000_spi_probe(struct spi_device *spi)
{
	struct cmr3000_gyro_data *data;

	data = cmr3000_init(&spi->dev, spi->irq, &cmr3000_spi_bops);
	if (IS_ERR(data))
		return PTR_ERR(data);

	spi_set_drvdata(spi, data);

	return 0;
}

static int cmr3000_spi_remove(struct spi_device *spi)
{
	struct cmr3000_gyro_data *data = dev_get_drvdata(&spi->dev);

	cmr3000_exit(data);

	return 0;
}

#ifdef CONFIG_PM
static int cmr3000_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct cmr3000_gyro_data *data = dev_get_drvdata(&spi->dev);

	cmr3000_suspend(data);

	return 0;
}

static int cmr3000_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct cmr3000_gyro_data *data = dev_get_drvdata(&spi->dev);

	cmr3000_resume(data);

	return 0;
}

static const struct dev_pm_ops cmr3000_spi_pm_ops = {
	.suspend = cmr3000_spi_suspend,
	.resume = cmr3000_spi_resume,
};
#endif

static SIMPLE_DEV_PM_OPS(cmr3000_spi_pm, cmr3000_spi_suspend,
			 cmr3000_spi_resume);

static struct spi_driver cmr3000_driver = {
	.driver = {
		   .name = "cmr3000_d01",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   .pm = &cmr3000_spi_pm,
		   },
	.probe = cmr3000_spi_probe,
	.remove = cmr3000_spi_remove,
};

static int __init cmr3000_spi_init(void)
{
	return spi_register_driver(&cmr3000_driver);
}

static void __exit cmr3000_spi_exit(void)
{
	spi_unregister_driver(&cmr3000_driver);
}

module_init(cmr3000_spi_init);
module_exit(cmr3000_spi_exit);

MODULE_DESCRIPTION("CMR3000-D0x Gyroscope SPI Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
