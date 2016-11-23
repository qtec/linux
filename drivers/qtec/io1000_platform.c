/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>

#define DRIVER_NAME "IO1000-platform"

static struct resource spi_resources[] = {
	[0] = {
		.start	= 0x043a,
		.end	= 0x043b,
		.flags	= IORESOURCE_IO,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_BUS,
	}
};

static struct platform_device_info spi_device_info = {
	.name		= "spi-qtec-lpc",
	.id		= PLATFORM_DEVID_AUTO,
	.res		= spi_resources,
	.num_res	= ARRAY_SIZE(spi_resources),
};

static struct platform_device *spi_device;

static struct mtd_partition io1000_spi_flash_partitions[] = {
	{
		.name = "Full Flash",
		.size = 0x84000,
		.offset = 0,
	},
	{
		.name = "User Flash",
		.size = 0x31800,
		.offset = 0,
	},
	{
		.name = "Golden Flash",
		.size = 0x31800,
		.offset = 0x31800,
	},
	{
		.name = "Production Data",
		.size = 0x10800,
		.offset = 0x63000,
	},
};

static struct flash_platform_data io1000_spi_flash_data = {
	.name = "m25p80",
	.parts =  io1000_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(io1000_spi_flash_partitions),
	.type = "3S200AN",
};

static struct spi_board_info spi_devices[] = {
	{
		.modalias	= "m25p80",
		.max_speed_hz	= 66 * 1000 * 1000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data = &io1000_spi_flash_data,
		.mode = SPI_MODE_3,
	},
};

static const char *tca6416_names[16]={
	"PROG_B",
	"M0",
	"M1",
	"M2",
	"DIN",
	"CCLK",
	NULL,
	NULL,
	"DONE",
	"INIT_B",
};

static struct pca953x_platform_data tca6416_pdata = {
	.irq_base =-1,
	.gpio_base = -1,
	.invert = 0,
	.names = tca6416_names,
};

static struct i2c_board_info i2c_device = {
	I2C_BOARD_INFO("tca6416", 0x21),
	.platform_data = &tca6416_pdata,
	.irq = -1,
};

static struct i2c_client *i2c_client=NULL;

static int __init io1000_init(void){
	struct i2c_adapter *i2c_adap;

	spi_device=platform_device_register_full(&spi_device_info);
	if (!spi_device){
		printk(KERN_ERR "Error registering platform device");
		return -EIO;
	}
	if (spi_register_board_info(spi_devices,ARRAY_SIZE(spi_devices)))
		printk(KERN_ERR "Error registering board info");


	i2c_adap = i2c_get_adapter(0);
	if (!i2c_adap){
		printk(KERN_ERR "Could not find i2c smbus. Check driver");
		platform_device_unregister(spi_device);
		return 0;
	}
	i2c_client=i2c_new_device(i2c_adap,&i2c_device);
	i2c_put_adapter(i2c_adap);

	return 0;
}

static void __exit io1000_exit(void){
	if (i2c_client)
		i2c_unregister_device(i2c_client);
	if (spi_device){
		spi_unregister_board_info(spi_devices,ARRAY_SIZE(spi_devices));
		platform_device_unregister(spi_device);
	}
}

module_init(io1000_init);
module_exit(io1000_exit);

MODULE_AUTHOR("Ricardo Ribalda");
MODULE_DESCRIPTION("IO1000 platform declarations");
MODULE_LICENSE("GPL");
