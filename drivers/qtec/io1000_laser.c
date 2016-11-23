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
#include <linux/mtd/partitions.h>
#include <linux/platform_data/gpio-qtec-lpc.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_data/ad5761.h>

#define DRIVER_NAME "IO1000-laser"

/*This has been tested with Jenkins IO1000_RevB-Laser#13 */

static const char *gpio_qtec_lpc_names_out0[]={
	"DRV_EN",
	"DACXY_LDAC",
	"DAC_nRESET",
};

static struct gpio_qtec_lpc_platform_data gpio_out0 = {
	.init_val = 0x7,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out0,
	.ngpio = 3,
};

static const char *gpio_qtec_lpc_names_out1[]={
	"LASER_PWM",
};

static struct gpio_qtec_lpc_platform_data gpio_out1 = {
	.init_val = -1,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out1,
	.ngpio = 1,
};

static const char *gpio_qtec_lpc_names_in0[]={
	"DRV_OK",
};

static struct gpio_qtec_lpc_platform_data gpio_in0 = {
	.direction = GPIO_QTEC_LPC_IN,
	.names = gpio_qtec_lpc_names_in0,
	.ngpio = 1,
};

static struct resource gpio_resources0[] = {
	[0] = {
		.start	= 0x041d,
		.end	= 0x041d,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources1[] = {
	[0] = {
		.start	= 0x041e,
		.end	= 0x041e,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources2[] = {
	[0] = {
		.start	= 0x041f,
		.end	= 0x041f,
		.flags	= IORESOURCE_IO,
	},
};

static struct ad5761_platform_data dacx = {
	.voltage_range = AD5761_VOLTAGE_RANGE_M10V_10V,
};

static struct ad5761_platform_data dacy = {
	.voltage_range = AD5761_VOLTAGE_RANGE_M10V_10V,
};

static struct ad5761_platform_data dacl = {
	.voltage_range = AD5761_VOLTAGE_RANGE_0V_5V,
};

static struct spi_board_info spi_devices[] = {
	{
		.modalias	= "ad5761r",
		.max_speed_hz	= 50 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select	= 0,
		.platform_data = &dacx,
		.mode = SPI_MODE_2,
	},
	{
		.modalias	= "ad5761r",
		.max_speed_hz	= 50 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select	= 1,
		.platform_data = &dacy,
		.mode = SPI_MODE_2,
	},
	{
		.modalias	= "ad5761r",
		.max_speed_hz	= 50 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select	= 2,
		.platform_data = &dacl,
		.mode = SPI_MODE_2,
	},
};

static struct resource spi_resources[] = {
	[0] = {
		.start	= 0x041b,
		.end	= 0x041c,
		.flags	= IORESOURCE_IO,
	},
	[1] = {
		.start	= 1,
		.end	= 1,
		.flags	= IORESOURCE_BUS,
	}
};

static struct regulator_consumer_supply ad5761_vrefs[] = {
	REGULATOR_SUPPLY("vref", "spi1.0"),
	REGULATOR_SUPPLY("vref", "spi1.1"),
};

static struct regulator_init_data vref_init_data = {
	.constraints.always_on = 1,
	.consumer_supplies = ad5761_vrefs,
	.num_consumer_supplies = ARRAY_SIZE(ad5761_vrefs),
};

static struct fixed_voltage_config vref_pdata = {
	.supply_name	= "2V5",
	.microvolts	= 2500000,
	.gpio		= -EINVAL,
	.enabled_at_boot = 1,
	.init_data	= &vref_init_data,
};

static struct platform_device_info platform_device_infos[] ={
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources0,
		.num_res	= ARRAY_SIZE(gpio_resources0),
		.data		= &gpio_out0,
		.size_data      = sizeof(gpio_out0),
	},
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources1,
		.num_res	= ARRAY_SIZE(gpio_resources1),
		.data		= &gpio_out1,
		.size_data      = sizeof(gpio_out1),
	},
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources2,
		.num_res	= ARRAY_SIZE(gpio_resources2),
		.data		= &gpio_in0,
		.size_data      = sizeof(gpio_in0),
	},
	{
		.name		= "spi-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= spi_resources,
		.num_res	= ARRAY_SIZE(spi_resources),
	},
	{
		.name		= "reg-fixed-voltage",
		.id		= PLATFORM_DEVID_AUTO,
		.data		= &vref_pdata,
		.size_data      = sizeof(vref_pdata),
	},
};

static struct platform_device *platform_devices[ARRAY_SIZE(platform_device_infos)];

static int __init io1000_laser_init(void){
	int i;

	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++){
		platform_devices[i]=platform_device_register_full(&platform_device_infos[i]);
		if (platform_devices[i]==NULL)
			printk(KERN_ERR "Error registering platform_devices %d\n",i);
	}

	if (spi_register_board_info(spi_devices,ARRAY_SIZE(spi_devices)))
		printk(KERN_ERR "Error registering board info");


	return 0;
}

static void __exit io1000_laser_exit(void){
	int i;

	spi_unregister_board_info(spi_devices,ARRAY_SIZE(spi_devices));

	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++)
		if (platform_devices[i])
			platform_device_unregister(platform_devices[i]);

}

module_init(io1000_laser_init);
module_exit(io1000_laser_exit);

MODULE_AUTHOR("Ricardo Ribalda");
MODULE_DESCRIPTION("IO1000 Laser Bitstream platform declarations");
MODULE_LICENSE("GPL");
