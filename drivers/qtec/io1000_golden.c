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

#define DRIVER_NAME "IO1000-golden"

static const char *gpio_qtec_lpc_names_out0[]={
	"OUT0",
	"OUT1",
	"OUT2",
	"OUT3",
	"OUT4",
	"OUT5",
	"OUT6",
	"OUT7",
};

static struct gpio_qtec_lpc_platform_data gpio_out0 = {
	.init_val = -1,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out0,
	.ngpio = 8,
};

static const char *gpio_qtec_lpc_names_out1[]={
	"OUT8",
	"OUT9",
	"OUT10",
	"OUT11",
	"OUT12",
	"OUT13",
	"OUT14",
	"OUT15",
};

static struct gpio_qtec_lpc_platform_data gpio_out1 = {
	.init_val = -1,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out1,
	.ngpio = 8,
};

static const char *gpio_qtec_lpc_names_out2[]={
	"OUT16",
	"OUT17",
	"OUT18",
	"OUT19",
	"OUT20",
	"OUT21",
	"OUT22",
	"OUT23",
};

static struct gpio_qtec_lpc_platform_data gpio_out2 = {
	.init_val = -1,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out2,
	.ngpio = 8,
};

static const char *gpio_qtec_lpc_names_in0[]={
	"IN0",
	"IN1",
	"IN2",
	"IN3",
	"IN4",
	"IN5",
	"IN6",
	"IN7",
};

static struct gpio_qtec_lpc_platform_data gpio_in0 = {
	.direction = GPIO_QTEC_LPC_IN,
	.names = gpio_qtec_lpc_names_in0,
	.ngpio = 8,
};

static const char *gpio_qtec_lpc_names_in1[]={
	"IN8",
	"IN9",
	"IN10",
	"IN11",
	"IN12",
	"IN13",
	"IN14",
	"IN15",
};

static struct gpio_qtec_lpc_platform_data gpio_in1 = {
	.direction = GPIO_QTEC_LPC_IN,
	.names = gpio_qtec_lpc_names_in1,
	.ngpio = 8,
};

static const char *gpio_qtec_lpc_names_aux[]={
	"AUX0",
	"AUX1",
	"AUX2",
	"AUX3",
	"AUX4",
	"AUX5",
};

static struct gpio_qtec_lpc_platform_data gpio_aux = {
	.init_val = -1,
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_aux,
	.ngpio = 6,
};

static struct resource gpio_resources0[] = {
	[0] = {
		.start	= 0x0410,
		.end	= 0x0410,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources1[] = {
	[0] = {
		.start	= 0x0414,
		.end	= 0x0414,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources2[] = {
	[0] = {
		.start	= 0x0418,
		.end	= 0x0418,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources3[] = {
	[0] = {
		.start	= 0x0420,
		.end	= 0x0420,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources4[] = {
	[0] = {
		.start	= 0x0428,
		.end	= 0x0428,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gpio_resources5[] = {
	[0] = {
		.start	= 0x041C,
		.end	= 0x041C,
		.flags	= IORESOURCE_IO,
	},
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
		.data		= &gpio_out2,
		.size_data      = sizeof(gpio_out2),
	},
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources3,
		.num_res	= ARRAY_SIZE(gpio_resources3),
		.data		= &gpio_in0,
		.size_data      = sizeof(gpio_in0),
	},
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources4,
		.num_res	= ARRAY_SIZE(gpio_resources4),
		.data		= &gpio_in1,
		.size_data      = sizeof(gpio_in1),
	},
	{
		.name		= "gpio-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= gpio_resources5,
		.num_res	= ARRAY_SIZE(gpio_resources5),
		.data		= &gpio_aux,
		.size_data      = sizeof(gpio_aux),
	},

};

static struct platform_device *platform_devices[ARRAY_SIZE(platform_device_infos)];

static int __init io1000_golden_init(void){
	int i;

	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++){
		platform_devices[i]=platform_device_register_full(&platform_device_infos[i]);
		if (platform_devices[i]==NULL)
			printk(KERN_ERR "Error registering platform_devices %d\n",i);
	}

	return 0;
}

static void __exit io1000_golden_exit(void){
	int i;

	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++)
		if (platform_devices[i])
			platform_device_unregister(platform_devices[i]);

}

module_init(io1000_golden_init);
module_exit(io1000_golden_exit);

MODULE_AUTHOR("Ricardo Ribalda");
MODULE_DESCRIPTION("IO1000 Golden Bitstream platform declarations");
MODULE_LICENSE("GPL");
