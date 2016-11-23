/* qt5023.c Driver for Qtec qt5023 camera head

   Copyright (C) 2011-2014 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

*/
#ifndef QT5023_H
#define QT5023_H

#define DRIVER_NAME "qt5023"
#define QT5023_NIRQ 32

#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/irqdomain.h>
#include <linux/qtec/qtec_eeprom.h>

struct qt5023_head{
	uint32_t userid;
	struct pci_dev *pci_dev;
	struct device_node *fpga_flash_dn;
	void __iomem *intc_io;
	void __iomem *hwicap_io;
	void *of_mem;
	struct irq_domain *irq_dom;
	struct device_node *dev_node;
	struct qt5023_serial_info serial_head;
	struct qt5023_serial_info serial_body;
	struct device_node **prev_node;
	struct device_node **last_node;
	struct work_struct async_probe;
};

//irq
int qt5023_irq_probe(struct qt5023_head *head,struct device_node *dn);
void qt5023_irq_remove(struct qt5023_head *head);
int qt5023_irq_save(struct qt5023_head *head,uint32_t *flags);
int qt5023_irq_restore(struct qt5023_head *head,uint32_t flags);

//pcie bridge
int qt5023_pcie_probe(struct qt5023_head *head);
void qt5023_pcie_remove(struct qt5023_head *head);

//dmatest
int qt5023_dmatest_probe(struct qt5023_head *head);
void qt5023_dmatest_remove(struct qt5023_head *head);

//of
int qt5023_of_probe(struct qt5023_head *head);
void qt5023_of_remove(struct qt5023_head *head);

//hwicap
int qt5023_hwicap_probe(struct qt5023_head *head,struct device_node *dn);
int qt5023_hwicap_reset(struct qt5023_head *head, uint32_t address);
int qt5023_hwicap_userid(struct qt5023_head *head, uint32_t *id);
void qt5023_hwicap_remove(struct qt5023_head *head);

#endif
