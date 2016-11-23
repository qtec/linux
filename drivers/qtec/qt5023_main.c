/* qt5023.c Driver for Qtec qt5023 camera head

   Copyright (C) 2011 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

*/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/aer.h>
#include <linux/slab.h>
#include "qt5023.h"

#define QTEC_VENDOR_ID 0x07ec
#define QTEC_5023_PROD_ID 0x5023
#define QTEC_5046_PROD_ID 0x5046

static void qt5023_pci_remove(struct pci_dev *dev){
	struct qt5023_head *head = pci_get_drvdata(dev);

	qt5023_of_remove(head);
	pci_disable_msi(dev);
	pci_disable_device(dev);
	return;
}

static int qt5023_pci_probe(struct pci_dev *dev,
		const struct pci_device_id *id){
	struct qt5023_head *head;
	uint16_t command;

	head = devm_kzalloc(&dev->dev, sizeof(*head), GFP_KERNEL);
	if (!head)
		return -ENOMEM;
	/*PCI*/
	if (pci_enable_device(dev)){
		dev_err(&dev->dev,"Unable to enable pci\n");
		return -EIO;
	}

	//pci_set_master(dev);
	pci_read_config_word(dev,PCI_COMMAND,&command);
	command|=PCI_COMMAND_MEMORY;
	command|=PCI_COMMAND_MASTER;
	pci_write_config_word(dev,PCI_COMMAND,command);
	//pci_write_config_word(dev,PCI_CACHE_LINE_SIZE,0x8);
	pci_cleanup_aer_uncorrect_error_status(dev);
	pci_enable_pcie_error_reporting(dev);

	if(pci_enable_msi(dev)){
		dev_err(&dev->dev,"Unable to enable msi\n");
		goto err_nomsi;
	}

	pci_set_drvdata(dev,head);
	head->pci_dev=dev;

	/*Device tree*/
	if (qt5023_of_probe(head)){
		dev_err(&dev->dev,"Error probing OF\n");
		goto err_of;
	}

	return 0;

	qt5023_of_remove(head);
err_of:
	pci_disable_msi(dev);
err_nomsi:
	pci_disable_device(dev);
	return -EIO;
}

static struct pci_device_id qt5023_pci_ids[] ={
	{
		.vendor=QTEC_VENDOR_ID,
		.device=QTEC_5023_PROD_ID,
		.subvendor=PCI_ANY_ID,
		.subdevice=PCI_ANY_ID,
	},
	{
		.vendor=QTEC_VENDOR_ID,
		.device=QTEC_5046_PROD_ID,
		.subvendor=PCI_ANY_ID,
		.subdevice=PCI_ANY_ID,
	},
	{0,},
};
MODULE_DEVICE_TABLE(pci, qt5023_pci_ids);

static struct pci_driver qt5023_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = qt5023_pci_ids,
	.probe = qt5023_pci_probe,
	.remove = qt5023_pci_remove,
};

module_pci_driver(qt5023_pci_driver);

MODULE_DESCRIPTION("Driver for the QT5023 camera head");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL v2");
