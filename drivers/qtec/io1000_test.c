/*
 *
 * IO1000 test module
 * Ricardo Ribalda - Qtec.com  2013
 * ricardo.ribalda@gmail.com
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#define DRIVER_NAME "IO1000_test"

#define IO1000_IRQ 12
#define REGA 0x43e
#define REGB 0x422
#define REGC 0x42a

static irqreturn_t irq_handler(int irq, void *data){
	unsigned int rega,regb,regc;

	rega=inb(REGA);
	regb=inb(REGB);
	regc=inb(REGC);
	printk(KERN_ERR "I01000 irq 0x%.2X 0x%.2X 0x%.2X\n",rega,regb,regc);

	return IRQ_HANDLED;
}

static int __init io100_test_init(void)
{
	int ret;

	ret=request_irq(IO1000_IRQ,irq_handler,IRQF_SHARED,DRIVER_NAME,DRIVER_NAME);

	if (ret){
		printk(KERN_ERR "Unable to request irq %d ret=%d\n",IO1000_IRQ,ret);
		return -EIO;
	}

	printk(KERN_ERR "Hello: I01000 irq at irq %d  IO regs 0x%.2X 0x%.2X 0x%.2X\n",IO1000_IRQ,REGA,REGB,REGC);

	return 0;
}

static void __exit io100_test_exit(void)
{
	printk(KERN_ERR "Bye bye: I01000 irq at irq %d  IO regs 0x%.2X 0x%.2X 0x%.2X\n",IO1000_IRQ,REGA,REGB,REGC);
	free_irq(IO1000_IRQ,DRIVER_NAME);
	return;
}

module_init(io100_test_init);
module_exit(io100_test_exit);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("IO1000 test module");
MODULE_LICENSE("GPL");
