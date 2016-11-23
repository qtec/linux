/*
 * VTI CMR3000_D0x Gyroscope driver
 *
 * Copyright (C) 2011 Qtechnology
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>
 *
 * Based on:
 *	drivers/input/misc/cma3000_d0x.c by: Hemanth V
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

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/cmr3000.h>

#include "cmr3000_d0x.h"

#define CMR3000_REV         0x21

#define CMR3000_WHOAMI      0x00
#define CMR3000_REVID       0x01
#define CMR3000_CTRL        0x02
#define CMR3000_STATUS      0x03
#define CMR3000_X_LSB       0x0C
#define CMR3000_X_MSB       0x0D
#define CMR3000_Y_LSB       0x0E
#define CMR3000_Y_MSB       0x0F
#define CMR3000_Z_LSB       0x10
#define CMR3000_Z_MSB       0x11
#define CMR3000_I2C_ADDR    0x22
#define CMR3000_PDR         0x26

#define CMR3000_IRQDIS     (1 << 0)
#define CMR3000_MODEMASK   (3 << 1)
#define CMR3000_BUSI2C     (0 << 4)
#define CMR3000_BUSSPI     (1 << 4)
#define CMR3000_INTLOW     (1 << 6)
#define CMR3000_INTHIGH    (0 << 6)
#define CMR3000_RST        (1 << 7)

#define CMRMODE_SHIFT      1
#define CMRIRQLEVEL_SHIFT  6

#define CMR3000_STATUS_PERR    (1 << 0)
#define CMR3000_STATUS_PORST   (1 << 3)

/* Settling time delay in ms */
#define CMR3000_SETDELAY    30

/*
 * Bit weights mult/div in dps for bit 0, other bits need
 * multipy factor 2^n. 11th bit is the sign bit.
 */
#define BIT_TO_DPS_MUL  3
#define BIT_TO_DPS_DIV 32

static struct cmr3000_platform_data cmr3000_default_pdata = {
	.irq_level = CMR3000_INTHIGH,
	.mode = CMRMODE_MEAS80,
	.irqflags = 0,
	.fuzz_x = 1,
	.fuzz_y = 1,
	.fuzz_z = 1,
};

struct cmr3000_gyro_data {
	const struct cmr3000_bus_ops *bus_ops;
	const struct cmr3000_platform_data *pdata;

	struct device *dev;
	struct input_dev *input_dev;

	int irq_level;
	u8 mode;

	int bit_to_mg;
	int irq;

	struct mutex mutex;
	bool opened;
	bool suspended;
};

static void decode_dps(struct cmr3000_gyro_data *data, int *datax,
		       int *datay, int *dataz)
{
	/* Data in 2's complement, convert to dps */
	*datax = (((s16) ((*datax) << 2)) * BIT_TO_DPS_MUL) / BIT_TO_DPS_DIV;
	*datay = (((s16) ((*datay) << 2)) * BIT_TO_DPS_MUL) / BIT_TO_DPS_DIV;
	*dataz = (((s16) ((*dataz) << 2)) * BIT_TO_DPS_MUL) / BIT_TO_DPS_DIV;
}

static irqreturn_t cmr3000_thread_irq(int irq, void *dev_id)
{
	struct cmr3000_gyro_data *data = dev_id;
	int datax, datay, dataz;
	u8 mode, intr_status;

	intr_status = data->bus_ops->read(data->dev, CMR3000_STATUS,
							"irq status");
	intr_status = data->bus_ops->read(data->dev, CMR3000_CTRL,
							"control mode");
	if (intr_status < 0)
		return IRQ_NONE;

	/* Interrupt not for this device */
	if (intr_status & CMR3000_IRQDIS)
		return IRQ_NONE;

	mode = (intr_status & CMR3000_MODEMASK) >> CMRMODE_SHIFT;
	if ((mode != CMRMODE_MEAS80)
	    && (mode != CMRMODE_MEAS20))
		return IRQ_NONE;

	datax = (data->bus_ops->read(data->dev, CMR3000_X_MSB, "X_MSB")) << 8;
	datax |= data->bus_ops->read(data->dev, CMR3000_X_LSB, "X_LSB");
	datay = (data->bus_ops->read(data->dev, CMR3000_Y_MSB, "Y_MSB")) << 8;
	datay |= data->bus_ops->read(data->dev, CMR3000_Y_LSB, "Y_LSB");
	dataz = (data->bus_ops->read(data->dev, CMR3000_Z_MSB, "Z_MSB")) << 8;
	dataz |= data->bus_ops->read(data->dev, CMR3000_Z_LSB, "Z_LSB");

	/* Device closed */
	if ((data->mode != CMRMODE_MEAS80)
	    && (data->mode != CMRMODE_MEAS20))
		return IRQ_NONE;

	/* Decode register values to dps */
	decode_dps(data, &datax, &datay, &dataz);

	input_report_abs(data->input_dev, ABS_X, datax);
	input_report_abs(data->input_dev, ABS_Y, datay);
	input_report_abs(data->input_dev, ABS_Z, dataz);
	input_sync(data->input_dev);

	return IRQ_HANDLED;
}

static int cmr3000_poweron(struct cmr3000_gyro_data *data)
{
	const struct cmr3000_platform_data *pdata = data->pdata;
	u8 ctrl;
	int ret;

	ctrl = pdata->irq_level << CMRIRQLEVEL_SHIFT;
	ctrl |= data->mode << CMRMODE_SHIFT;
	ctrl |= data->bus_ops->ctrl_mod;
	ret = data->bus_ops->write(data->dev, CMR3000_CTRL, ctrl,
							"Mode setting");
	if (ret < 0)
		return -EIO;

	msleep(CMR3000_SETDELAY);

	return 0;
}

static int cmr3000_poweroff(struct cmr3000_gyro_data *data)
{
	int ret;
	u8 ctrl = CMRMODE_POFF;

	ctrl |= data->bus_ops->ctrl_mod;
	ctrl |= CMR3000_IRQDIS;

	ret = data->bus_ops->write(data->dev, CMR3000_CTRL, ctrl,
							"Mode setting");
	msleep(CMR3000_SETDELAY);

	return ret;
}

static int cmr3000_reset(struct cmr3000_gyro_data *data)
{
	int val;

	/* Reset chip */
	data->bus_ops->write(data->dev, CMR3000_CTRL, CMR3000_RST, "Reset");
	mdelay(2);

	/* Settling time delay */
	val = data->bus_ops->read(data->dev, CMR3000_STATUS, "Status");
	if (val < 0) {
		dev_err(data->dev, "Reset failed\n");
		return val;
	}

	if (val & CMR3000_STATUS_PERR) {
		dev_err(data->dev, "Parity Error\n");
		return -EIO;
	}

	return cmr3000_poweroff(data);
}

static int cmr3000_open(struct input_dev *input_dev)
{
	struct cmr3000_gyro_data *data = input_get_drvdata(input_dev);

	mutex_lock(&data->mutex);

	if (!data->suspended)
		cmr3000_poweron(data);

	data->opened = true;

	mutex_unlock(&data->mutex);

	return 0;
}

static void cmr3000_close(struct input_dev *input_dev)
{
	struct cmr3000_gyro_data *data = input_get_drvdata(input_dev);

	mutex_lock(&data->mutex);

	if (!data->suspended)
		cmr3000_poweroff(data);

	data->opened = false;

	mutex_unlock(&data->mutex);
}

void cmr3000_suspend(struct cmr3000_gyro_data *data)
{
	mutex_lock(&data->mutex);

	if (!data->suspended && data->opened)
		cmr3000_poweroff(data);

	data->suspended = true;

	mutex_unlock(&data->mutex);
}
EXPORT_SYMBOL(cmr3000_suspend);

void cmr3000_resume(struct cmr3000_gyro_data *data)
{
	mutex_lock(&data->mutex);

	if (data->suspended && data->opened)
		cmr3000_poweron(data);

	data->suspended = false;

	mutex_unlock(&data->mutex);
}
EXPORT_SYMBOL(cmr3000_resume);

struct cmr3000_gyro_data *cmr3000_init(struct device *dev, int irq,
				       const struct cmr3000_bus_ops *bops)
{
	const struct cmr3000_platform_data *pdata = dev->platform_data;
	struct cmr3000_gyro_data *data;
	struct input_dev *input_dev;
	int rev;
	int error;

	if (pdata == NULL) {
		dev_info(dev, "platform data not found, using default\n");
		pdata = &cmr3000_default_pdata;
	}

	/* if no IRQ return error */
	if (irq == 0) {
		error = -EINVAL;
		goto err_out;
	}

	data = kzalloc(sizeof(struct cmr3000_gyro_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	data->dev = dev;
	data->input_dev = input_dev;
	data->bus_ops = bops;
	data->pdata = pdata;
	data->irq = irq;
	mutex_init(&data->mutex);

	data->mode = pdata->mode;
	if ((data->mode != CMRMODE_MEAS80)
	    && (data->mode != CMRMODE_MEAS20)) {
		data->mode = CMRMODE_MEAS80;
		dev_warn(dev, "Invalid mode specified, assuming 80Hz\n");
	}

	data->irq_level = pdata->irq_level;
	if ((data->irq_level != CMR3000_INTLOW)
	    && (data->irq_level != CMR3000_INTHIGH)) {
		data->irq_level = CMR3000_INTHIGH;
		dev_warn(data->dev,
			 "Invalid int level specified, assuming high\n");
	}

	input_dev->name = "cmr3000-gyroscope";
	input_dev->id.bustype = bops->bustype;
	input_dev->open = cmr3000_open;
	input_dev->close = cmr3000_close;

	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X,
			     -CMRRANGE, CMRRANGE, pdata->fuzz_x, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     -CMRRANGE, CMRRANGE, pdata->fuzz_y, 0);
	input_set_abs_params(input_dev, ABS_Z,
			     -CMRRANGE, CMRRANGE, pdata->fuzz_z, 0);

	input_set_drvdata(input_dev, data);

	error = cmr3000_reset(data);
	if (error)
		goto err_free_mem;

	rev = data->bus_ops->read(data->dev, CMR3000_REVID, "Revid");
	if (rev < 0) {
		error = rev;
		goto err_free_mem;
	}
	if (rev != CMR3000_REV) {
		error = -EINVAL;
		pr_err("CMR3000 Gyroscope: Unknown Revision %x\n", rev);
		goto err_free_mem;
	}
	pr_info("CMR3000 Gyroscope: Revision %x\n", rev);

	error = request_threaded_irq(irq, NULL, cmr3000_thread_irq,
				     pdata->irqflags | IRQF_ONESHOT,
				     "cmr3000_d0x", data);
	if (error) {
		dev_err(dev, "request_threaded_irq failed\n");
		goto err_free_mem;
	}

	error = input_register_device(data->input_dev);
	if (error) {
		dev_err(dev, "Unable to register input device\n");
		goto err_free_irq;
	}

	return data;

err_free_irq:
	free_irq(irq, data);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
err_out:
	return ERR_PTR(error);
}
EXPORT_SYMBOL(cmr3000_init);

void cmr3000_exit(struct cmr3000_gyro_data *data)
{
	input_unregister_device(data->input_dev);
	free_irq(data->irq, data);
	kfree(data);
}
EXPORT_SYMBOL(cmr3000_exit);

MODULE_DESCRIPTION("CMR3000-D0x Gyroscope Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
