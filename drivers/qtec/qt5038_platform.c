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
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_data/leds-pca963x.h>
#include <linux/platform_data/gpio-qtec-lpc.h>
#include <linux/platform_data/pca953x.h>

#define DRIVER_NAME "qt5038-platform"

static const char *tca6416_names[16]={
	"PROG_B",
	"CCLK",
	"DIN",
	"M2",
	"M1",
	"M0",
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

static const char *gpio_qtec_lpc_names_out0[]={
	"QT5038_OUT0",
	"QT5038_OUT1",
	"QT5038_OUT2",
	"QT5038_OUT3",
	"QT5038_OUT4",
};

static struct gpio_qtec_lpc_platform_data gpio_out0 = {
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out0,
	.ngpio = 5,
};

static const char *gpio_qtec_lpc_names_out1[]={
	"RS485_DE_nRE",
	"RS485_TERM_EN",
	"RS485_LOOP_EN",
};

static struct gpio_qtec_lpc_platform_data gpio_out1 = {
	.direction = GPIO_QTEC_LPC_OUT,
	.names = gpio_qtec_lpc_names_out1,
	.ngpio = 3,
};

static const char *gpio_qtec_lpc_names_in[]={
	"QT5038_IN0",
	"QT5038_IN1",
	"QT5038_IN2",
	"QT5038_IN3",
	"QT5038_IN4",
};

static struct gpio_qtec_lpc_platform_data gpio_in = {
	.direction = GPIO_QTEC_LPC_IN,
	.names = gpio_qtec_lpc_names_in,
	.ngpio = 5,
};

/*68*/
static struct led_info leds68[] ={
	{
		.name = "Newtec0",
		.default_trigger = "default-on",
	},
	{
		.name = "Newtec1",
		.default_trigger = "default-on",
	},
	{
		.name = "Newtec2",
		.default_trigger = "default-on",
	},
	{
		.name = "Newtec3",
		.default_trigger = "default-on",
	},
	{
		.name = "Newtec4",
		.default_trigger = "default-on",
	},
	{
		.name = "Newtec5",
		.default_trigger = "default-on",
	},

};

static struct pca963x_platform_data pca9634_pdata68 = {
	.outdrv = PCA963X_OPEN_DRAIN,
	.leds = {
		.num_leds = ARRAY_SIZE(leds68),
		.leds= leds68,
	},
};

/*69*/
static struct led_info leds69[] ={
	{
		.name = "Qtec0",
	},
	{
		.name = "Qtec1",
	},
	{
		.name = "Qtec2",
	},
	{
		.name = "Qtec3",
	},
	{
		.name = "Qtec4",
	},
	{
		.name = "Qtec5",
	},

};

static struct pca963x_platform_data pca9634_pdata69 = {
	.outdrv = PCA963X_OPEN_DRAIN,
	.leds = {
		.num_leds = ARRAY_SIZE(leds69),
		.leds= leds69,
	},
};

/*6a*/
static struct led_info leds6a[] ={
	{
		.name = "SataLow",
	},
	{
		.name = "SataHigh",
	},
	{
		.name = "WifiLow",
	},
	{
		.name = "WifiHigh",
	},
	{
		.name = "EthLow",
	},
	{
		.name = "EthHigh",
	},
	{
		.name = "BlueTooth",
	},
	{
		.name = "NFC",
	},

};

static struct pca963x_platform_data pca9634_pdata6a = {
	.outdrv = PCA963X_TOTEM_POLE,
	.leds = {
		.num_leds = ARRAY_SIZE(leds6a),
		.leds= leds6a,
	},
};

/*6b*/
static struct led_info leds6b[] ={
	{
		.name = "LockGreen",
	},
	{
		.name = "LockRed",
	},
	{
		.name = "Audio",
	},
	{
		.name = "Alarm",
	},
	{
		.name = "Emergency",
	},

};

static struct pca963x_platform_data pca9634_pdata6b = {
	.outdrv = PCA963X_OPEN_DRAIN,
	.leds = {
		.num_leds = ARRAY_SIZE(leds6b),
		.leds= leds6b,
	},
};

/*6c*/
static struct led_info leds6c[] ={
	{
		.name = "BackUp",
		.default_trigger = "default-on",
	},
	{
		.name = "BackDown",
		.default_trigger = "default-on",
	},
	{
		.name = "HomeUp",
		.default_trigger = "default-on",
	},
	{
		.name = "HomeDown",
		.default_trigger = "default-on",
	},
	{
		.name = "StopUp",
		.default_trigger = "default-on",
	},
	{
		.name = "StopDown",
		.default_trigger = "default-on",
	},
	{
		.name = "PlayUp",
		.default_trigger = "default-on",
	},
	{
		.name = "PlayDown",
		.default_trigger = "default-on",
	},

};

static struct pca963x_platform_data pca9634_pdata6c = {
	.outdrv = PCA963X_OPEN_DRAIN,
	.leds = {
		.num_leds = ARRAY_SIZE(leds6c),
		.leds= leds6c,
	},
};

/*6d*/
static struct led_info leds6d[] ={
	{
		.name = "Undo",
	},
	{
		.name = "Web",
	},
	{
		.name = "Setup",
	},
	{
		.name = "Menu",
	},
	{
		.name = "OK",
	},
	{
	},
	{
	},
	{
		.name = "Power",
	},

};

static struct pca963x_platform_data pca9634_pdata6d = {
	.outdrv = PCA963X_OPEN_DRAIN,
	.leds = {
		.num_leds = ARRAY_SIZE(leds6d),
		.leds= leds6d,
	},
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &tca6416_pdata,
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x68),
		.platform_data = &pca9634_pdata68,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x69),
		.platform_data = &pca9634_pdata69,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x6a),
		.platform_data = &pca9634_pdata6a,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x6b),
		.platform_data = &pca9634_pdata6b,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x6c),
		.platform_data = &pca9634_pdata6c,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x6d),
		.platform_data = &pca9634_pdata6d,
	},
};

static struct i2c_client *i2c_client[ARRAY_SIZE(i2c_devices)];

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
		.start	= 0x0420,
		.end	= 0x0420,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device_info platform_device_infos[]={
	{
		.name		= "spi-qtec-lpc",
		.id		= PLATFORM_DEVID_AUTO,
		.res		= spi_resources,
		.num_res	= ARRAY_SIZE(spi_resources),
	},
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
		.data		= &gpio_in,
		.size_data      = sizeof(gpio_in),
	},
};

static struct mtd_partition qt5038_spi_flash_partitions[] = {
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

static struct flash_platform_data qt5038_spi_flash_data = {
	.name = "m25p80",
	.parts =  qt5038_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(qt5038_spi_flash_partitions),
	.type = "3S200AN",
};

static struct spi_board_info spi_devices[] = {
	{
		.modalias	= "m25p80",
		.max_speed_hz	= 66 * 1000 * 1000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data = &qt5038_spi_flash_data,
		.mode = SPI_MODE_3,
	},
};

static struct platform_device *platform_devices[ARRAY_SIZE(platform_device_infos)];

static int __init qt5038_init(void){
	int i;
	struct i2c_adapter *i2c_adap;

	i2c_adap = i2c_get_adapter(0);
	if (!i2c_adap){
		printk(KERN_ERR "Could not find i2c smbus. Check driver");
		return 0;
	}

	for (i=0;i<ARRAY_SIZE(i2c_devices);i++)
		i2c_client[i]=i2c_new_device(i2c_adap,&i2c_devices[i]);

	i2c_put_adapter(i2c_adap);

	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++){
		platform_devices[i]=platform_device_register_full(&platform_device_infos[i]);
		if (platform_devices[i]==NULL)
			printk(KERN_ERR "Error registering platform_devices %d\n",i);
	}

	if (spi_register_board_info(spi_devices,ARRAY_SIZE(spi_devices)))
		printk(KERN_ERR "Error registering spi devices");

	return 0;
}

static void __exit qt5038_exit(void){
	int i;

	for (i=0;i<ARRAY_SIZE(i2c_devices);i++)
		if (i2c_client[i])
			i2c_unregister_device(i2c_client[i]);

	spi_unregister_board_info(spi_devices,ARRAY_SIZE(spi_devices));
	for(i=0;i<ARRAY_SIZE(platform_device_infos);i++)
		platform_device_unregister(platform_devices[i]);

	return;
}

static struct usb_device_id qt5038_usb_table [] = {
	{ USB_DEVICE(0x0eef, 0x7321) }, //Auto launch if touchscreen is found
	{ }
};
MODULE_DEVICE_TABLE(usb, qt5038_usb_table);

module_init(qt5038_init);
module_exit(qt5038_exit);

MODULE_AUTHOR("Ricardo Ribalda");
MODULE_DESCRIPTION("QT5038 platform declarations");
MODULE_LICENSE("GPL");
