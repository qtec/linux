/* qt5023_irq.c
 * qt523 camera head irq handling

   Copyright (C) 2011 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

  based on microblaze/kernel/intc.c
 * Copyright (C) 2007-2009 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2007-2009 PetaLogix
 * Copyright (C) 2006 Atmark Techno, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.


*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include "qt5023.h"

#define ISR 0x00			/* Interrupt Status Register */
#define IPR 0x04			/* Interrupt Pending Register */
#define IER 0x08			/* Interrupt Enable Register */
#define IAR 0x0c			/* Interrupt Acknowledge Register */
#define SIE 0x10			/* Set Interrupt Enable bits */
#define CIE 0x14			/* Clear Interrupt Enable bits */
#define IVR 0x18			/* Interrupt Vector Register */
#define MER 0x1c			/* Master Enable Register */

#define MER_ME (1<<0)
#define MER_HIE (1<<1)

#define QT5023_IRQ_START 128

static inline int qt5023_intc_write(struct qt5023_head *head, uint32_t reg, uint32_t value){
	dev_dbg(&head->pci_dev->dev, "intc W:0x%.2x 0x%.8x\n",reg,value);
	iowrite32(value,head->intc_io + reg);
	return 0;
}

static inline int qt5023_intc_read(struct qt5023_head *head, uint32_t reg, uint32_t *value){
	*value=ioread32(head->intc_io + reg);
	dev_dbg(&head->pci_dev->dev, "intc R:0x%.2x 0x%.8x\n",reg,*value);
	return 0;
}

static void intc_unmask(struct irq_data *d)
{
	struct qt5023_head *head=irq_data_get_irq_chip_data(d);
	unsigned long mask= 1<< d->hwirq;
	pr_debug("unmask: %d (%ld)\n", d->irq,d->hwirq);
	qt5023_intc_write(head,IAR,mask);
	qt5023_intc_write(head,SIE,mask);
}

static void intc_mask(struct irq_data *d)
{
	struct qt5023_head *head=irq_data_get_irq_chip_data(d);
	unsigned long mask= 1<< d->hwirq;
	pr_debug("mask: %d (%ld)\n", d->irq,d->hwirq);
	qt5023_intc_write(head, CIE, mask);
}

static int intc_probe(struct qt5023_head *head){

	/*
	 * Disable all external interrupts until they are
	 * explicity requested.
	 */
	qt5023_intc_write(head,IER,0);

	/* Acknowledge any pending interrupts just in case. */
	qt5023_intc_write(head,IAR,0xffffffff);

	/* Turn on the Master Enable. */
	qt5023_intc_write(head,MER,MER_HIE | MER_ME);

	return 0;
}

static void intc_remove(struct qt5023_head *head){
	/*
	 * Disable all external interrupts until they are
	 * explicity requested.
	 */
	qt5023_intc_write(head,IER,0);

	/* Acknowledge any pending interrupts just in case. */
	qt5023_intc_write(head,IAR,0xffffffff);

	/* Turn off the Master Enable. */
	qt5023_intc_write(head,MER,0);
}

static void qt5023_irq_handler(struct irq_desc *desc)
{
	struct qt5023_head *head=irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc) ;
	int phys_irq;
	int logical_irq;

	chained_irq_enter(chip,desc);

	while (1){
		//Find irq
		qt5023_intc_read(head,IVR,&phys_irq);
		if (phys_irq==0xffffffff)
			break;

		logical_irq=irq_linear_revmap(head->irq_dom, phys_irq);

		//handle
		if (likely(logical_irq!=-1))
			generic_handle_irq(logical_irq);
		else
			dev_err(&head->pci_dev->dev,
					"xintc: ERROR Unmapped irq %d triggered....\n",phys_irq);
	}

	chained_irq_exit(chip,desc);
	return;
}

static struct irq_chip qt5023_irq_chip_level = {
	.name = "Xilinx INTC" ,
	.irq_mask = intc_mask,
	.irq_mask_ack = intc_mask,
	.irq_unmask = intc_unmask,
};

static int qt5023_irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw){

	irq_set_chip_and_handler(virq, &qt5023_irq_chip_level,handle_level_irq);
	irq_set_chip_data(virq,h->host_data);

	return 0;
}

static int qt5023_irq_xlate(struct irq_domain *id, struct device_node *node, const u32 *intspec, unsigned int intsize, unsigned long *out_hwirq, unsigned int *out_type){
	unsigned char xilinx_intc_map_senses[] = {
		IRQ_TYPE_EDGE_RISING,
		IRQ_TYPE_EDGE_FALLING,
		IRQ_TYPE_LEVEL_HIGH,
		IRQ_TYPE_LEVEL_LOW,
	};

	if (intsize<2){
		printk("qt5023 xintc: Wrong intsize %d\n",intsize);
		return -EINVAL;
	}

	if (intspec[1]!=2){
		printk("qt5023 xintc: Wrong int type, we only support level high! %d\n",intspec[1]);
		return -EINVAL;
	}

	if (intspec[0]>=QT5023_NIRQ){
		printk("qt5023 xintc: Wrong hw irq %d\n",intspec[0]);
		return -EINVAL;
	}

	*out_type=xilinx_intc_map_senses[intspec[1]];
	*out_hwirq=intspec[0];

	return 0;
}

static struct irq_domain_ops qt5023_irq_ops = {
	.map = qt5023_irq_map,
	.xlate = qt5023_irq_xlate,
};

int qt5023_irq_save(struct qt5023_head *head,uint32_t *flags){
	qt5023_intc_write(head,MER,0);
	qt5023_intc_read(head,IER,flags);

	return 0;
}

int qt5023_irq_restore(struct qt5023_head *head,uint32_t flags){
	qt5023_intc_write(head,IER,0);
	qt5023_intc_write(head,IAR,0xffffffff);
	qt5023_intc_write(head,MER,MER_HIE | MER_ME);
	qt5023_intc_write(head,IER,flags);

	return 0;
}

int qt5023_irq_probe(struct qt5023_head *head,struct device_node *dn){
	struct resource mem;
	int ret;

	/*ioremap intc*/
	if (of_address_to_resource(dn,0,&mem)){
		dev_err(&head->pci_dev->dev,"intc: Cannot find address of intc\n");
		return -EIO;
	}

	head->intc_io=devm_ioremap(&head->pci_dev->dev, mem.start, resource_size(&mem));
	if (IS_ERR(head->intc_io))
		return PTR_ERR(head->intc_io);

	/*Init intc*/
	ret=intc_probe(head);
	if (ret){
		devm_iounmap(&head->pci_dev->dev, head->intc_io);
		return ret;
	}

	/*Irq domain*/
	head->irq_dom = irq_domain_add_linear(dn, QT5023_NIRQ, &qt5023_irq_ops, head);
	if(!head->irq_dom){
		dev_err(&head->pci_dev->dev,"intc: Could not allocate irq domain\n");
		intc_remove(head);
		devm_iounmap(&head->pci_dev->dev, head->intc_io);
		return -EIO;
	}

	/*irqchip*/
	irq_set_chained_handler_and_data(head->pci_dev->irq,qt5023_irq_handler,head);

	dev_info(&head->pci_dev->dev,"intc: allocated at 0x%.8lx with IRQ %d\n",
						(unsigned long)mem.start,head->pci_dev->irq);
	return 0;
}

void qt5023_irq_remove(struct qt5023_head *head){

	if (head->irq_dom)
		irq_domain_remove(head->irq_dom);
	head->irq_dom = NULL;

	intc_remove(head);

	if (!IS_ERR(head->intc_io))
		devm_iounmap(&head->pci_dev->dev, head->intc_io);
	head->intc_io = (void __iomem *) ERR_PTR(-EIO);

	return;
}
