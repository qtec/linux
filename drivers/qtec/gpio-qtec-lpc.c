/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_data/gpio-qtec-lpc.h>

#define DRIVER_NAME "gpio-qtec-lpc"

#define QTEC_LPC_IOLEN 1
#define QTEC_LPC_N_GPIO 8
#define QTEC_LPC_GPIO 0

struct qtec_lpc_gpio {
	unsigned long portbase;
	struct mutex mutex;
	struct gpio_chip gpio_chip;
	int direction;
};

static void qtec_lpc_gpio_write(struct qtec_lpc_gpio *p, unsigned int reg, uint8_t value){
	dev_dbg(p->gpio_chip.parent,"W %lX %.2X",p->portbase+reg,value);
	outb(value,p->portbase+reg);
}

static uint8_t qtec_lpc_gpio_read(struct qtec_lpc_gpio *p, unsigned int reg){
	uint8_t ret;
	ret=inb(p->portbase+reg);
	dev_dbg(p->gpio_chip.parent,"R %lX %.2X",p->portbase+reg,ret);

	return ret;
}

static void qtec_lpc_gpio_set(struct gpio_chip *chip, unsigned offset, int value){
	struct qtec_lpc_gpio *p =  container_of(chip, struct qtec_lpc_gpio, gpio_chip);
	uint8_t reg;

	if (p->direction == GPIO_QTEC_LPC_IN)
		return;

	mutex_lock(&p->mutex);
	reg=qtec_lpc_gpio_read(p,QTEC_LPC_GPIO);
	if (value)
		reg |= BIT(offset);
	else
		reg &= ~BIT(offset);
	qtec_lpc_gpio_write(p,QTEC_LPC_GPIO,reg);
	mutex_unlock(&p->mutex);

	return;
}

static int qtec_lpc_gpio_get(struct gpio_chip *chip, unsigned offset){
	struct qtec_lpc_gpio *p =  container_of(chip, struct qtec_lpc_gpio, gpio_chip);
	uint8_t reg;

	reg=qtec_lpc_gpio_read(p,QTEC_LPC_GPIO);

	return (reg&BIT(offset))?1:0;
}

static int qtec_lpc_gpio_get_dir(struct gpio_chip *chip, unsigned offset){
	struct qtec_lpc_gpio *p =  container_of(chip, struct qtec_lpc_gpio, gpio_chip);

	//0 is output
	if (p->direction == GPIO_QTEC_LPC_OUT)
		return 0;
	return 1;
}

static int qtec_lpc_gpio_probe(struct platform_device *pdev){
	struct resource *res_port;
	struct qtec_lpc_gpio *p;
	struct gpio_qtec_lpc_platform_data *pdata= pdev->dev.platform_data;
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	if (pdata){
		p->direction= pdata->direction;
		p->gpio_chip.names = pdata->names;
		p->gpio_chip.ngpio = pdata->ngpio;
	} else {
		p->gpio_chip.ngpio = QTEC_LPC_N_GPIO;
		p->direction = GPIO_QTEC_LPC_IN;
	}

	platform_set_drvdata(pdev, p);

	//Request io resource
	res_port = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res_port){
		dev_err(&pdev->dev, "found no io resource\n");
		return -ENXIO;
	}

	if (!devm_request_region(&pdev->dev,res_port->start,QTEC_LPC_IOLEN,DRIVER_NAME)){
		dev_err(&pdev->dev, "Unable to request io region 0x%lx (%d)\n",(long)res_port->start,QTEC_LPC_IOLEN);
		return -EIO;
	}
	p->portbase=res_port->start;

	//mutex
	mutex_init(&p->mutex);

	//GPIO
	p->gpio_chip.get_direction = qtec_lpc_gpio_get_dir;
	p->gpio_chip.set = qtec_lpc_gpio_set;
	p->gpio_chip.get = qtec_lpc_gpio_get;
	p->gpio_chip.label = dev_name(&pdev->dev);
	p->gpio_chip.owner = THIS_MODULE;
	p->gpio_chip.base = -1;
	p->gpio_chip.parent = &pdev->dev;
	ret=gpiochip_add(&p->gpio_chip);
	if (ret)
		return ret;

	/*Init value*/
	if (pdata && p->direction==GPIO_QTEC_LPC_OUT && pdata->init_val>=0)
		qtec_lpc_gpio_write(p,QTEC_LPC_GPIO,pdata->init_val);

	dev_info(p->gpio_chip.parent, "QTEC_LPC GPIO driver at 0x%lx\n",p->portbase);

	return 0;
}

static int qtec_lpc_gpio_remove(struct platform_device *pdev){
	struct qtec_lpc_gpio *p = platform_get_drvdata(pdev);

	gpiochip_remove(&p->gpio_chip);

	return 0;
}

static struct platform_driver qtec_lpc_gpio_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= qtec_lpc_gpio_probe,
	.remove		= qtec_lpc_gpio_remove,
};

module_platform_driver(qtec_lpc_gpio_driver);

static struct platform_device_id qtec_lpc_spi_ids[] = {
	{
		.name           = DRIVER_NAME,
	},
	{ }
};

MODULE_DEVICE_TABLE(platform, qtec_lpc_spi_ids);

MODULE_DESCRIPTION("Qtec LPC GPIO driver");
MODULE_AUTHOR("Ricardo Ribalda");
MODULE_LICENSE("GPL");
