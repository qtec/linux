/* qt5023_hwicap.c
 * qt5023 hwicap handler

   Copyright (C) 2013 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.

*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/pci.h>
#include "qt5023.h"

static inline int qt5023_hwicap_write(struct qt5023_head *head, uint32_t reg, uint32_t value){
	dev_dbg(&head->pci_dev->dev, "hwicap W:0x%.2x 0x%.8x\n",reg,value);
	iowrite32(value,head->hwicap_io + reg);
	return 0;
}

static inline int qt5023_hwicap_read(struct qt5023_head *head, uint32_t reg, uint32_t *value){
	*value=ioread32(head->hwicap_io + reg);
	dev_dbg(&head->pci_dev->dev, "hwicap R:0x%.2x 0x%.8x\n",reg,*value);
	return 0;
}

#define CONTROL 0x10c
#define DO_WRITE 0
#define DO_READ 1
#define DO_RESET (0xc)

#define STATUS 0x110
#define DONE 0

#define RFO 0x118
#define FIFO_WRITE 0x100
#define FIFO_READ 0x104
#define SZ 0x108

static int qt5023_retrain_pcie(struct qt5023_head *head){
	struct pci_dev *parent_dev = head->pci_dev->bus->self;
	u16 aux;
	int retries;

	/*pcie_capability_read_word(head->pci_dev,PCI_EXP_LNKSTA, &aux);
	if ( (aux & PCI_EXP_LNKSTA_CLS) == PCI_EXP_LNKSTA_CLS_5_0GB){
		dev_err(&head->pci_dev->dev,"hwicap: link speed already 5GBPS\n");
		return 0;
	}*/

	pcie_capability_read_word(parent_dev, PCI_EXP_LNKCTL, &aux);
	pcie_capability_write_word(parent_dev, PCI_EXP_LNKCTL, aux | PCI_EXP_LNKCTL_RL);

	for (retries=0;retries<1000;retries++){
		pcie_capability_read_word(parent_dev, PCI_EXP_LNKSTA, &aux);
		if (!(aux & PCI_EXP_LNKSTA_LT))
			return 0;
		msleep(1);
	}

	dev_err(&head->pci_dev->dev,"hwicap: Cannot retrain pcie\n");

	return -1;
}

static int qt5023_hwicap_reprog_generic(struct qt5023_head *head,uint32_t *bitfile, int n_words){
	int i;

	qt5023_hwicap_write(head,CONTROL,DO_RESET);
	qt5023_hwicap_write(head,CONTROL,0);

	/*Fill buffer*/
	for (i=0;i<n_words;i++)
		qt5023_hwicap_write(head,FIFO_WRITE,bitfile[i]);

	/*Start write*/
	qt5023_hwicap_write(head,CONTROL,BIT(DO_WRITE));

	msleep(1000);
	return qt5023_retrain_pcie(head);
}

static int qt5023_hwicap_userid_generic(struct qt5023_head *head,uint32_t *bitfile, int n_words, uint32_t *id){
	#define N_TRIES 1000
	int i;
	uint32_t aux;

	qt5023_hwicap_write(head,CONTROL,DO_RESET);
	qt5023_hwicap_write(head,CONTROL,0);

	/*Fill buffer*/
	for (i=0;i<n_words;i++)
		qt5023_hwicap_write(head,FIFO_WRITE,bitfile[i]);

	/*Start write*/
	qt5023_hwicap_write(head,CONTROL,BIT(DO_WRITE));

	for (i=0;i<N_TRIES;i++){
		qt5023_hwicap_read(head,STATUS,&aux);
		if (aux&BIT(DONE))
			break;
	}

	if (i==N_TRIES){
		dev_err(&head->pci_dev->dev,"hwicap: Cannot send userid cmd\n");
		return -1;
	}
	msleep(5);

	qt5023_hwicap_write(head,SZ,1);
	qt5023_hwicap_write(head,CONTROL,BIT(DO_READ));

	for (i=0;i<N_TRIES;i++){
		qt5023_hwicap_read(head,RFO,&aux);
		if (aux!=0)
			break;
	}

	if (i==N_TRIES){
		dev_err(&head->pci_dev->dev,"hwicap: Cannot read userid\n");
		return -1;
	}

	qt5023_hwicap_read(head,FIFO_READ,id);

	return 0;
}

static int qt5023_hwicap_userid_s6(struct qt5023_head *head,uint32_t *id){
	uint32_t bitfile[]={0xFFFF,0xFFFF,0xAA99,0x5566,0x2000,0x2000,0x2801 | (0x17<<5) ,0x2000,0x2000,0x2000};

	return qt5023_hwicap_userid_generic(head,bitfile,ARRAY_SIZE(bitfile),id);
}

static int qt5023_hwicap_userid_k7(struct qt5023_head *head,uint32_t *id){
	uint32_t bitfile[]={0xFFFFFFFF,0xAA995566,0x20000000,0x28000001 | (0xd<<13),0x20000000,0x20000000};

	return qt5023_hwicap_userid_generic(head,bitfile,ARRAY_SIZE(bitfile),id);
}

int qt5023_hwicap_userid(struct qt5023_head *head,uint32_t *id){

	if (head->pci_dev->device == 0x5023)
		return qt5023_hwicap_userid_s6(head,id);

	return qt5023_hwicap_userid_k7(head,id);
}

static int qt5023_hwicap_bootsts_s6(struct qt5023_head *head,uint32_t *id){
	uint32_t bitfile_bootsts[]={0xFFFF,0xFFFF,0xAA99,0x5566,0x2000,0x2000,0x2801 | (0x20<<5) ,0x2000,0x2000,0x2000};

	return qt5023_hwicap_userid_generic(head,bitfile_bootsts,ARRAY_SIZE(bitfile_bootsts),id);
}

static int qt5023_hwicap_reprog_s6(struct qt5023_head *head,uint32_t address){
	/*Described on ug380 page 126 (Table 7-1)*/
	uint32_t bitfile_reprog[]= {0xFFFF,0xFFFF,0xAA99,0x5566,0x2000,0x2000,0x3261,0x0044,0x3281,0x0300,\
	0x32A1,0x0044,0x32C1,0x0300,0x3301,0x2100,0x30A1,0x000E,0x2000,0x2000};
	uint32_t bootsts;
	int jmp_loc = 7;

	if (!address)
		bitfile_reprog[11+2]=0x0338;

	qt5023_hwicap_bootsts_s6(head, &bootsts);

	if ((bootsts & 0xff) != 0x1){
		dev_err(&head->pci_dev->dev,"hwicap: Previous jmp failed (0x%.4x). System running in fallback mode.\nPlease powercycle with allow_upgrade_golden and verify_bitstream options enabled\n",bootsts);
		jmp_loc = 11;
	}

	bitfile_reprog[jmp_loc]=(address+0x44)&0xffff; //0x44 is header size
	bitfile_reprog[jmp_loc+2]= 0x0300 | ((address>>16)&0xff);

	return	qt5023_hwicap_reprog_generic(head,bitfile_reprog,ARRAY_SIZE(bitfile_reprog));
}

static int qt5023_hwicap_reprog_k7(struct qt5023_head *head,uint32_t address){
	uint32_t bitfile[]= {0xFFFFFFFF, 0xAA995566,0x20000000, 0x30020001, 0x00000000, 0x30008001, 0x0000000f,0x20000000};

	bitfile[4]=address/2; //16 bit addressing

	return qt5023_hwicap_reprog_generic(head,bitfile,ARRAY_SIZE(bitfile));
}

static int qt5023_hwicap_reprog(struct qt5023_head *head,uint32_t address){

	if (head->pci_dev->device == 0x5023)
		return qt5023_hwicap_reprog_s6(head,address);

	return qt5023_hwicap_reprog_k7(head,address);
}


int qt5023_hwicap_reset(struct qt5023_head *head, uint32_t address){
	int i, len, n_dword;
	uint32_t *buffer;
	int ret;
	uint32_t flags;
	uint32_t userid;

	len=head->pci_dev->cfg_size;
	n_dword=len/4;

	//Save irq flags
	qt5023_irq_save(head,&flags);

	//alloc config
	buffer=vmalloc(len);
	if(!buffer)
		return -ENOMEM;

	//Save pcie config
	for (i=0;i<n_dword;i++)
		pci_user_read_config_dword(head->pci_dev, i, &buffer[i]);

	ret=qt5023_hwicap_reprog(head,address);
	if (ret){
		vfree(buffer);
		return ret;
	}

	//Restore pcie config
	for (i=0;i<n_dword;i++)
		pci_user_write_config_dword(head->pci_dev, i, buffer[i]);

	//Restore irq flags
	qt5023_irq_restore(head,flags);
	vfree(buffer);

	if (head->pci_dev->device == 0x5023){
		qt5023_hwicap_userid_s6(head, &userid);
		if (userid == 65534){
			dev_err(&head->pci_dev->dev,"hwicap: Failed jumping to user bitstream. Cannot continue.\nPlease powercycle with verify_bitstream option enabled\n");
			return -EINVAL;
		}
	}
	return 0;
}

int qt5023_hwicap_probe(struct qt5023_head *head,struct device_node *dn){
	struct resource mem;

	/*ioremap intc*/
	if (of_address_to_resource(dn,0,&mem)){
		dev_err(&head->pci_dev->dev,"hwicap: Cannot find address of hwicap\n");
		return -EIO;
	}

	head->hwicap_io=devm_ioremap(&head->pci_dev->dev, mem.start, resource_size(&mem));
	if (IS_ERR(head->hwicap_io)){
		dev_err(&head->pci_dev->dev,"hwicap: Cannot ioremap hwicap\n");
		return PTR_ERR(head->hwicap_io);
	}

	return 0;
}

void qt5023_hwicap_remove(struct qt5023_head *head){

	if (!IS_ERR(head->hwicap_io))
		devm_iounmap(&head->pci_dev->dev, head->hwicap_io);

	head->hwicap_io = (void __iomem *)ERR_PTR(-EIO);

}
