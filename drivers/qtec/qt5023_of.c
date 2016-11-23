/* qt5023_of.c
 * qt5023 Device tree parser

   Copyright (C) 2011-2013 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.

*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/firmware.h>
#include <linux/crc32.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include <linux/mtd/mtd.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "../mtd/mtdcore.h"
#include <linux/mtd/partitions.h>
#include "qt5023.h"

#define QT5023_FILENAME_SIZE 64

static bool use_fb4 = false;
module_param(use_fb4, bool, 0);
MODULE_PARM_DESC(use_fb4,
       "Use Frame Bus 4 when possible, "
       "0 (Default) - Use standard (fully featured) Fame bus, "
       "1 - Use parallel frame bus (4x faster with 1/4 of features).");

static bool allow_upgrade_golden = false;
module_param(allow_upgrade_golden, bool, 0);
MODULE_PARM_DESC(allow_upgrade_golden,
       "Allow upgrade golden bitstream, "
       "0 (Default) - Dont upgrade golden, "
       "1 - Upgrade golden. (Be careful You can brick your device!).");

static bool verify_bitstream = false;
module_param(verify_bitstream, bool, 0);
MODULE_PARM_DESC(verify_bitstream,
       "Verify whole bitstream, "
       "0 (Default) - No, "
       "1 - Yes (Slower boot).");

static struct of_device_id qt5023_of_main_bus [] ={
	{ .compatible = "main-bus",},
	{},
};

/*Serial*/
static int qt5023_of_verify_serial(struct qt5023_head *head,struct qt5023_serial_info *serial){
	uint32_t res;
	int i;

	if(memcmp(serial->magic,EMPTY_HEAD,sizeof(serial->magic))==0){
		dev_err(&head->pci_dev->dev,"serial: Eeprom is not programmed. You can program it with qt5022-scripts\n");
		return -1;
	}

	if(memcmp(serial->magic,MAGIC_QTEC,sizeof(serial->magic))){
		dev_err(&head->pci_dev->dev,"serial: Wrong magic header\n");
		return -1;
	}

	if (serial->version!=SERIAL_VERSION){
		dev_err(&head->pci_dev->dev,"serial: Wrong version, I can only understand version %d\n",SERIAL_VERSION);
		return -1;
	}

	serial->crc=be32_to_cpu((__force __be32) serial->crc);
	res=crc32(~0,(void *)serial,sizeof(*serial)-sizeof(serial->crc));
	res=~res;
	if (res!=serial->crc){
		dev_err(&head->pci_dev->dev,"serial: Wrong CRC!! 0x%x 0x%x\n",res,serial->crc);
		return -1;
	}

	//Take care of endianness
	for (i=0;i<N_VAR;i++)
		serial->var[i]=be32_to_cpu((__force __be32) serial->var[i]);

	return 0;
}

#define I2C_BODY_ADDRESS 0x57
#define I2C_HEAD_ADDRESS 0x51
#define I2C_HEAD_ADDRESS_ALT 0x54

static int qt5023_i2c_address(struct qt5023_head *head, bool body){
	int max_id= head->pci_dev->bus->number;
	struct pci_dev *dev = NULL;

	if (body)
		return I2C_BODY_ADDRESS;

	if (head->pci_dev->device == 0x5046)
		return I2C_HEAD_ADDRESS;

	for_each_pci_dev(dev)
		if ((dev->vendor==0x07ec) && (dev->bus->number > max_id))
				max_id = dev->bus->number;

	if ( head->pci_dev->bus->number == max_id )
		return I2C_HEAD_ADDRESS;

	//Second eye is enumerated first See #291
	return I2C_HEAD_ADDRESS_ALT;
}

#define XILINX_I2C_ADDR 0x30000
/*FIXME This is just a workaround until we cannot unpopulate heads */
static int qt5023_i2c_eeprom_read_5046(struct qt5023_head *head,struct qt5023_serial_info *serial,uint16_t i2c_addr){
	void __iomem *iomem;
	void *pserial=serial;
	int i;

	iomem = devm_ioremap_nocache(&head->pci_dev->dev,pci_resource_start(head->pci_dev,0)+XILINX_I2C_ADDR , 0x10000);
	if (IS_ERR(iomem))
		return -1;

	iowrite32(0xa,iomem+0x40);
	for (i=0;i<sizeof(struct qt5023_serial_info);i++){
		//Reversed engineered from the Linux driver
		iowrite32(0xf,iomem+0x120);
		iowrite32(0x2,iomem+0x100);
		iowrite32(0x1,iomem+0x100);
		ioread32(iomem+0x104);
		ioread32(iomem+0x114);
		iowrite32(i2c_addr*2+0x100,iomem+0x108);
		ioread32(iomem+0x114);
		iowrite32(i,iomem+0x108);
		ioread32(iomem+0x114);
		iowrite32(0x0,iomem+0x120);
		iowrite32(i2c_addr*2+0x101,iomem+0x108);
		iowrite32(0x201,iomem+0x108);
		ioread32(iomem+0x118);
		msleep(1);//FIXME USE BUSY BIT
		*(uint8_t *)(pserial+i)=(ioread32(iomem+0x10c))&0xff;
		iowrite32(0xff,iomem+0x120);
	}
	iowrite32(0xa,iomem+0x40);

	devm_iounmap(&head->pci_dev->dev,iomem);
	return 0;
}

static int _qt5023_of_read_serial(struct qt5023_head *head,struct qt5023_serial_info *serial, bool body){
	void *pserial=serial;
	int status,i;
	union i2c_smbus_data data;
	struct i2c_adapter *adap;
	int ret;
	uint16_t i2c_addr;

	i2c_addr = qt5023_i2c_address(head,body);

	if ((!body) &&(head->pci_dev->device == 0x5046)){
		ret=qt5023_i2c_eeprom_read_5046(head,serial,i2c_addr);
		if (ret)
			return ret;
		goto verify;
	}

	adap=i2c_get_adapter(0);
	if (!adap){
		dev_err(&head->pci_dev->dev,"Unable to find adapter 0\n");
		return -EIO;
	}

	for (i=0;i<sizeof(struct qt5023_serial_info);i++){
		status = i2c_smbus_xfer(adap, i2c_addr, 0,
				I2C_SMBUS_READ, i,
				I2C_SMBUS_BYTE_DATA, &data);
		if (status<0){
			dev_err(&head->pci_dev->dev,"Unable to read 0:0x%.2x:0x%.2x\n",i2c_addr,i);
			i2c_put_adapter(adap);
			memset(serial,0,sizeof(*serial));
			return status;
		}
		*(uint8_t *)(pserial+i)=data.byte&0xff;
	}
	i2c_put_adapter(adap);
verify:
	ret=qt5023_of_verify_serial(head,serial);
	if (ret)
		memset(serial,0,sizeof(*serial));

	return ret;
}

static int qt5023_of_read_serial(struct qt5023_head *head,struct qt5023_serial_info *serial, bool body){
	if (!_qt5023_of_read_serial(head,serial,body))
			return 0;
	dev_warn(&head->pci_dev->dev,"Error reading %s EEPROM, retrying once\n",body?"BODY":"HEAD");
	return _qt5023_of_read_serial(head,serial,body);
}

static int qt5023_of_name(struct qt5023_head *head,struct qt5023_serial_info *serial,char *name,char *sufix){
	char filename[QT5023_FILENAME_SIZE] = "";

	if (head->pci_dev->device == 0x5046)
		snprintf(filename,QT5023_FILENAME_SIZE,"qtec/qt5046/AXI_7K160T_");
	else if (head->pci_dev->revision==0)
		snprintf(filename,QT5023_FILENAME_SIZE,"qtec/qt5023/AXI_LX75T_");
	else
		snprintf(filename,QT5023_FILENAME_SIZE,"qtec/qt5023/AXI_LX100T_");

	if (strcmp(serial->product,"Q5CCDV00")==0 || strcmp(serial->product,"Q5NEWTECV00")==0)
		snprintf(name,QT5023_FILENAME_SIZE,"%sCCD.%s",filename,sufix);
	else if (strcmp(serial->product,"QCMOSISV00")==0){
		//Kintex cameras do not support non fb4 mode
		if (use_fb4 || head->pci_dev->device == 0x5046)
			snprintf(name,QT5023_FILENAME_SIZE,"%sCMOSIF-FB4.%s",filename,sufix);
		else if (serial->var[CMOSIS_BAYER]==1)
			snprintf(name,QT5023_FILENAME_SIZE,"%sCMOSIF-BAYER.%s",filename,sufix);
		else
			snprintf(name,QT5023_FILENAME_SIZE,"%sCMOSIF-MONO.%s",filename,sufix);
	}
	else if (strcmp(serial->product,"QGOGGLEV00")==0)
		snprintf(name,QT5023_FILENAME_SIZE,"%sDUAL_CMOSIF.%s",filename,sufix);
	else if (strcmp(serial->product,"QROICV00")==0)
		snprintf(name,QT5023_FILENAME_SIZE,"%sROIC.%s",filename,sufix);
	else
		snprintf(name,QT5023_FILENAME_SIZE,"%sBASIC.%s",filename,sufix);

	return 0;
}

static int qt5023_of_basic_name(struct qt5023_head *head,char *name, char *sufix){
	if (head->pci_dev->device == 0x5046)
		snprintf(name,QT5023_FILENAME_SIZE,"qtec/qt5046/AXI_7K160T_BASIC.%s",sufix);
	else if (head->pci_dev->revision==0)
		snprintf(name,QT5023_FILENAME_SIZE,"qtec/qt5023/AXI_LX75T_BASIC.%s",sufix);
	else
		snprintf(name,QT5023_FILENAME_SIZE,"qtec/qt5023/AXI_LX100T_BASIC.%s",sufix);
	return 0;
}

/*of*/
static int qt5023_of_fixup_dt(struct qt5023_head *head,struct qt5023_serial_info *serial){
	struct device_node *node=NULL;
	const __be32 *pvar;
	__be32 *var;
	unsigned int len;


	if (strcmp(serial->product,"Q5NEWTECV00")==0){
		of_node_get(head->dev_node);
		node=of_find_compatible_node(head->dev_node, NULL, "qtec,axi-framebus-gen-1.00.a");
		if (!node){
			dev_err(&head->pci_dev->dev,"Warning: Unable to find /axi/ccd_fg_0. Check dtb firmware file\n");
		}
		else{
			pvar=of_get_property(node,"qtec,bayer_chip",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(0);
			pvar=of_get_property(node,"qtec,n_ccd",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(3);
			of_node_put(node);
		}
	}

	if (strcmp(serial->product,"Q5CCDV00")==0){
		of_node_get(head->dev_node);
		node=of_find_compatible_node(head->dev_node, NULL, "qtec,axi-framebus-gen-1.00.a");
		if (!node){
			dev_err(&head->pci_dev->dev,"Warning: Unable to find /axi/ccd_fg_0. Check dtb firmware file\n");
		}
		else {
			pvar=of_get_property(node,"qtec,bayer_chip",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(serial->var[CCD_BAYER]);
			pvar=of_get_property(node,"qtec,n_ccd",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(serial->var[CCD_N]);
			of_node_put(node);
		}
	}

	if ((strcmp(serial->product,"QCMOSISV00")==0) ||(strcmp(serial->product,"QROICV00")==0) ||(strcmp(serial->product,"QGOGGLEV00")==0)) {
		of_node_get(head->dev_node);
		node=of_find_compatible_node(head->dev_node, NULL, "qtec,axi_cmosis_if-1.00.a");
		if (!node){
			dev_err(&head->pci_dev->dev,"Warning: Unable to find /axi/cmosis_fg_0. Check dtb firmware file\n");
		}
		else{
			pvar=of_get_property(node,"qtec,bayer_chip",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(serial->var[CMOSIS_BAYER]);

			if (serial->var[VERSION]>0){
				pvar=of_get_property(node,"qtec,offset",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_OFFSET]);
				pvar=of_get_property(node,"qtec,adc_gain",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_ADCGAIN]);
				pvar=of_get_property(node,"qtec,vramp",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_VRAMP]);
			}
			else
				dev_warn(&head->pci_dev->dev,"Warning: EEPROM formated with old version. Please upgrade!\n");
			of_node_put(node);
		}
	}

	if (node && strcmp(serial->product,"QGOGGLEV00")==0) { //Second eye
		of_node_get(node);
		node=of_find_compatible_node(node, NULL, "qtec,axi_cmosis_if-1.00.a");
		if (!node){
			dev_err(&head->pci_dev->dev,"Warning: Unable to find /axi/cmosis_fg_0. Check dtb firmware file\n");
		}
		else{
			pvar=of_get_property(node,"qtec,bayer_chip",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(serial->var[CMOSIS_BAYER1]);

			if (serial->var[VERSION]>0){
				pvar=of_get_property(node,"qtec,offset",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_OFFSET1]);
				pvar=of_get_property(node,"qtec,adc_gain",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_ADCGAIN1]);
				pvar=of_get_property(node,"qtec,vramp",&len);
				var=(__be32*)pvar;
				if (var && len)
					var[0]=cpu_to_be32(serial->var[CMOSIS_VRAMP1]);
			}
			else
				dev_warn(&head->pci_dev->dev,"Warning: EEPROM formated with old version. Please upgrade!\n");
			of_node_put(node);
		}
	}

	if (strcmp(serial->product,"QROICV00")==0){
		of_node_get(head->dev_node);
		node=of_find_compatible_node(head->dev_node, NULL, "qtec,axi-ingas-if-1.00.a");
		if (!node){
			dev_err(&head->pci_dev->dev,"Unable to find /axi/roic_fg_1. Check dtb firmware file\n");
		}
		else{
			int idx=(serial->var[VERSION]>0)?ROIC_INGAS1:ROIC_INGAS0;
			pvar=of_get_property(node,"qtec,ingas_chip",&len);
			var=(__be32*)pvar;
			if (var && len)
				var[0]=cpu_to_be32(serial->var[idx]);
			of_node_put(node);
		}
	}

	//Serial number
	for_each_of_allnodes_from(head->dev_node,node){
		if (!of_device_is_compatible(node, "qtec,axi_matrix_packer-1.00.a"))
			continue;
		pvar=of_get_property(node,"qtec,serial_number",&len);
		if (pvar && len){
			char *ser=(char *)pvar;
			snprintf(ser,len,"%6phN",serial->mac);
		}
		else
			dev_err(&head->pci_dev->dev,"Unable to find serial number placeholder. Check dtb firmware file\n");
		pvar=of_get_property(node,"qtec,bitstream_version",&len);
		var=(__be32*)pvar;
		if (var && len)
			var[0]=cpu_to_be32(head->userid);
		else
			dev_err(&head->pci_dev->dev,"Warning: Unable to find bitstream version placeholder. Check dtb firmware file\n");
		pvar=of_get_property(node,"qtec,head_i2c_address",&len);
		var=(__be32*)pvar;
		if (var && len)
			var[0]=cpu_to_be32(qt5023_i2c_address(head,false));
		else
			dev_err(&head->pci_dev->dev,"Warning: Unable to find i2c_addr version placeholder. Check dtb firmware file\n");
	}

	return 0;
}

static int qt5023_of_populate(struct qt5023_head *head){
	struct device_node *dn;
	int ret;
	const __be32 *pranges;
	__be32 *ranges;
	unsigned long flags;
	unsigned int len;
	static atomic_t nbus = ATOMIC_INIT(0);

	//Fix-up ranges
	of_node_get(head->dev_node);
	dn=of_find_compatible_node(head->dev_node, NULL, "main-bus");
	if(!dn){
		dev_err(&head->pci_dev->dev,"Unable to find main-bus\n");
		return -1;
	}

	//rename bus
	sprintf((char *)dn->full_name,"/axi%x", atomic_inc_return(&nbus));

	pranges=of_get_property(dn,"ranges",&len);
	if (pranges==NULL || len!=(8*sizeof(__be32))){
		of_node_put(dn);
		dev_err(&head->pci_dev->dev,"Unable to find bridge ranges (len=%d)\n",len);
		return -1;
	}
	ranges=(__be32*)pranges;
	ranges[2]=cpu_to_be32(pci_resource_start(head->pci_dev,0));
	ranges[6]=cpu_to_be32(pci_resource_start(head->pci_dev,2));
	of_node_put(dn);

	//interrupt controller
	of_node_get(head->dev_node);
	dn=of_find_node_with_property(head->dev_node, "interrupt-controller");
	if(!dn){
		dev_err(&head->pci_dev->dev,"Unable to find interrupt controller\n");
		return -1;
	}

	if (qt5023_irq_probe(head,dn)==-1){
		dev_err(&head->pci_dev->dev,"Unable to probe interrupt controller\n");
		return -1;

	}
	of_node_put(dn);

	//hwicap
	of_node_get(head->dev_node);
	dn=of_find_compatible_node(head->dev_node, NULL, "xlnx,axi-hwicap-2.03.a");
	if(!dn){
		dev_err(&head->pci_dev->dev,"Unable to find hwicap core\n");
		qt5023_irq_remove(head);
		return -1;
	}

	if (qt5023_hwicap_probe(head,dn)==-1){
		dev_err(&head->pci_dev->dev,"Unable to probe hwicap core\n");
		qt5023_irq_remove(head);
		of_node_put(dn);
		return -1;

	}
	of_node_put(dn);

	//flash
	of_node_get(head->dev_node);
	dn=of_find_node_with_property(head->dev_node, "fpga-flash");
	if(!dn){
		dev_err(&head->pci_dev->dev,"Unable to find fpga-flash. Cannot continue\n");
		return 0;
	}
	head->fpga_flash_dn=dn;
	of_node_put(dn);

	//Add node to device tree.
	raw_spin_lock_irqsave(&devtree_lock, flags);
	head->prev_node = &of_root;

	while(*head->prev_node)
		head->prev_node = &(*head->prev_node)->sibling;

	*head->prev_node = head->dev_node;
	head->last_node = &head->dev_node->sibling;

	while(*head->last_node)
		head->last_node = &(*head->last_node)->sibling;
	raw_spin_unlock_irqrestore(&devtree_lock, flags);

	head->pci_dev->dev.of_node = head->dev_node;
	ret=of_platform_populate(head->dev_node,qt5023_of_main_bus,NULL,&head->pci_dev->dev);
	if (ret){
		qt5023_irq_remove(head);

		raw_spin_lock_irqsave(&devtree_lock, flags);
		*head->prev_node = NULL;
		raw_spin_unlock_irqrestore(&devtree_lock, flags);

		head->pci_dev->dev.of_node = NULL;

		dev_err(&head->pci_dev->dev,"Error populating device tree... could not restore\n");
		return -1;
	}

	return ret;
}

static int qt5023_of_upgrade_write_mtd(struct qt5023_head *head,int mtdnum, const struct firmware *firmware, struct mtd_info *mtd){
	struct erase_info ei;
	size_t written,erase_size,write_size,offset=0;
	int ret;
	size_t header_size=mtd->erasesize*5;

	if (firmware->size > (mtd->size-(mtd->size%mtd->erasesize))){
		dev_err(&head->pci_dev->dev,"Unable to upgrade mtd%d, bitstream is too big...\n",mtdnum);
		return -EINVAL;
	}

	while (offset<firmware->size){
		if (offset==0 && (firmware->size> header_size) ){
			dev_err(&head->pci_dev->dev,"Upgrading header of mtd%d....\n",mtdnum);
			write_size=header_size;
			erase_size=write_size;
		}
		else{
			dev_err(&head->pci_dev->dev,"Upgrading body of mtd%d....\n",mtdnum);
			write_size=firmware->size-offset;
			erase_size=mtd->size-offset;
			erase_size-=erase_size%mtd->erasesize;
		}

		dev_err(&head->pci_dev->dev,"Erasing....\n");
		memset(&ei, 0, sizeof(struct erase_info));
		ei.mtd=mtd;
		ei.addr=offset;
		ei.len=erase_size;
		ret = mtd_erase(mtd, &ei);
		if (ret || ei.state==MTD_ERASE_FAILED){
			dev_err(&head->pci_dev->dev,"error: Cannot erase mtd%d\n",mtdnum);
			return ret;
		}

		dev_err(&head->pci_dev->dev,"Writing....\n");
		ret = mtd_write(mtd, offset, write_size, &written, firmware->data+offset);
		if (ret || (written!=write_size)){
			dev_err(&head->pci_dev->dev,"error: Cannot write mtd%d\n",mtdnum);
			return ret;
		}

		offset+=write_size;
	}


	dev_err(&head->pci_dev->dev,"Done with mtd%d!\n",mtdnum);

	return 0;
}

static struct mtd_info *qt5023_find_mtd_name(struct qt5023_head *head, char *mtdname){
	unsigned long expiration = jiffies + 10*HZ;
	struct mtd_info *mtd;
	static bool requested_module = false;

	//Try for 10 seconds for the mtd to be ready
	do{
		mtd_for_each_device(mtd){
			if ((mtd->dev.parent->of_node==head->fpga_flash_dn) &&
					(strcmp(mtdname,mtd->name)==0))
				return mtd;
		}
		schedule();
		if (!requested_module){
			request_module("spi-xilinx");
			requested_module = true;
		}
	}while(time_before(jiffies,expiration));

	return NULL;
}

#define MINI_SIZE (64*1024)
static int qt5023_of_upgrade_mtd(struct qt5023_head *head, char *filename,char *mtdname, bool allow_upgrade, int do_jmp){
	const struct firmware *firmware;
	struct mtd_info *mtd;
	int ret;
	uint8_t *buffer;
	size_t read;
	int mtdnum;
	int i;
	uint32_t mtd_offset;
	bool whole_verify;

	whole_verify = (allow_upgrade && verify_bitstream)?true:false;

	mtd = qt5023_find_mtd_name(head,mtdname);
	if (!mtd){
		dev_err(&head->pci_dev->dev,"error: cannot find MTD device with name %s\n",mtdname);
		return -1;
	}
	mtdnum=mtd->index;

	ret= request_firmware(&firmware, filename,&head->pci_dev->dev);
	if (ret<0){
		dev_err(&head->pci_dev->dev,"of: Unable to find %s\n",filename);
		return ret;
	}

	buffer=vmalloc(whole_verify?firmware->size:MINI_SIZE);
	if (!buffer){
		release_firmware(firmware);
		return -ENOMEM;
	}

	mtd = get_mtd_device(mtd, -1);
	if (IS_ERR(mtd)) {
		ret = PTR_ERR(mtd);
		release_firmware(firmware);
		vfree(buffer);
		dev_err(&head->pci_dev->dev,"error: cannot get MTD device\n");
		return ret;
	}

	//Check start and end of the mtd.
	for (i=0;i<(whole_verify?1:2);i++){
		long offset,size;

		if (i==0){
			offset=0;
			size=whole_verify?firmware->size:MINI_SIZE;
		}
		else{
			offset=firmware->size-MINI_SIZE;
			size=MINI_SIZE;
		}

		if (offset<0)
			offset=0;

		if (offset+size > firmware->size)
			size=firmware->size-offset;

		ret = mtd_read(mtd, offset, size, &read, buffer);
		if (ret){
			put_mtd_device(mtd);
			release_firmware(firmware);
			vfree(buffer);
			dev_err(&head->pci_dev->dev,"error: Cannot read mtd%d\n",mtdnum);
			return ret;
		}

		if (memcmp(buffer,firmware->data+offset,size)){
			for (i=0;i<size;i++)
				if (buffer[i] != 0xff)
					break;
			if (!allow_upgrade && i!=size)
				dev_err(&head->pci_dev->dev,"Outdated mtd%d, but ignoring because allow_upgrade_golden is set to false\n",mtdnum);
			else{
				dev_err(&head->pci_dev->dev,"Outdated mtd%d, Upgrading with %s\n",mtdnum,filename);
				dev_err(&head->pci_dev->dev,"DO NOT POWER OFF!!!!!!!!!!!\n");
				ret= qt5023_of_upgrade_write_mtd(head,mtdnum,firmware,mtd);
				if (ret){
					put_mtd_device(mtd);
					vfree(buffer);
					release_firmware(firmware);
					dev_err(&head->pci_dev->dev,"error: Failed to write mtd%d\n",mtdnum);
					return ret;
				}
			}
			break;
		}
	}
	mtd_offset=mtd_get_device_offset(mtd);
	put_mtd_device(mtd);
	if (do_jmp){
		ret=qt5023_hwicap_reset(head,mtd_offset);
		if (ret){
			dev_err(&head->pci_dev->dev,"error: Failed to jump to new bitfile\n");
			vfree(buffer);
			release_firmware(firmware);
			return -EIO;
		}
	}

	vfree(buffer);
	release_firmware(firmware);

	return 0;
}

static int qt5023_of_check_compatibility(struct qt5023_head *head){
	if (strcmp(head->serial_head.product,"Q5CCDV00")==0 || strcmp(head->serial_head.product,"Q5NEWTECV00")==0){
		if (head->serial_body.var[VOLT_SENSOR]!=15000){
			dev_err(&head->pci_dev->dev,"error: Head requires 15000 mV, but body provides %dmV\n",head->serial_body.var[VOLT_SENSOR]);
			return -1;
		}
	}

	return 0;
}

static int qt5023_load_dt(struct qt5023_head *head, char *filename, struct device_node **node, void **of_mem){
	int ret;
	const struct firmware *firmware;

	dev_info(&head->pci_dev->dev,"of: Using dtb firmware file: %s\n",filename);

	ret = request_firmware(&firmware, filename,&head->pci_dev->dev);
	if (ret<0){
		dev_err(&head->pci_dev->dev,"of: Unable to find %s\n",filename);
		return ret;
	}

	*of_mem =vmalloc(firmware->size);
	if (!*of_mem){
		release_firmware(firmware);
		return -ENOMEM;
	}

	memcpy(*of_mem,firmware->data,firmware->size);
	release_firmware(firmware);

	of_fdt_unflatten_tree(*of_mem, NULL, node);
	of_node_set_flag(*node, OF_DETACHED);
	ret = of_resolve_phandles(*node);
	if (ret){
		dev_err(&head->pci_dev->dev,"of: Error resolving phandles\n");
		vfree(*of_mem);
		return ret;
	}

	return 0;
}

static void qt5023_async_probe(struct work_struct *work){
	struct qt5023_head *head = container_of(work, struct qt5023_head , async_probe);
	char filename[QT5023_FILENAME_SIZE] = "";
	struct device_node *node;
	void *of_mem;

	//Upgrade golden
	qt5023_of_basic_name(head,filename,(head->pci_dev->device == 0x5023)?"bin":"kbpi");
	if (qt5023_of_upgrade_mtd(head,filename,"Golden Bitstream",!!allow_upgrade_golden,false)){
		dev_err(&head->pci_dev->dev,"Error checking golden bistream\n");
		return;
	}

	//Check head
	if ((qt5023_of_read_serial(head,&head->serial_body,true)<0) || strcmp(head->serial_body.product,"QT5023RevB")){
		head->serial_body.var[VOLT_SENSOR]=15000;
		dev_err(&head->pci_dev->dev,"Error reading body serial. Assuming default HW\n");
	}

	if (qt5023_of_read_serial(head,&head->serial_head,false)){
		dev_err(&head->pci_dev->dev,"Error reading head serial. Assuming headless\n");
		return;
	}

	if (qt5023_of_check_compatibility(head)){
		dev_err(&head->pci_dev->dev,"Head is not compatible with body. Do not load user bitstream\n");
		return;
	}

	//Update user bitstream and jmp
	qt5023_of_name(head,&head->serial_head,filename,(head->pci_dev->device == 0x5023)?"bin":"kbpi");
	if (qt5023_of_upgrade_mtd(head,filename,"User Bitstream",true,true)){
		dev_err(&head->pci_dev->dev,"Unable to jmp to user bitstream.\n");
		qt5023_of_remove(head);
		return;
	}

	qt5023_of_name(head,&head->serial_head,filename,"dtb");
	if (qt5023_load_dt(head,filename, &node, &of_mem)){
		dev_err(&head->pci_dev->dev,"Error loading user device tree\n");
		return;
	}

	qt5023_hwicap_userid(head,&head->userid);
	qt5023_of_remove(head);

	//Load user device tree
	head->dev_node = node;
	head->of_mem = of_mem;
	qt5023_of_fixup_dt(head,&head->serial_head);

	qt5023_of_populate(head);

	dev_info(&head->pci_dev->dev,"QT%.4X camera head (build id #%d) ready!\n",head->pci_dev->device, head->userid);
}

int qt5023_of_probe(struct qt5023_head *head){
	char filename[QT5023_FILENAME_SIZE] = "";
	int ret;

	qt5023_of_basic_name(head,filename, "dtb");

	ret = qt5023_load_dt(head,filename, &head->dev_node, &head->of_mem);
	if (ret){
		dev_err(&head->pci_dev->dev,"Error loading basic device tree\n");
		return -EIO;
	}


	ret = qt5023_of_populate(head);
	if (ret){
		dev_err(&head->pci_dev->dev,"Error populating basic device tree\n");
		return ret;
	}

	INIT_WORK(&head->async_probe, qt5023_async_probe);
	schedule_work(&head->async_probe);

	return 0;
}

void qt5023_of_remove(struct qt5023_head *head){
	unsigned long flags;

	if (!head->pci_dev->dev.of_node)
		return;

	of_platform_depopulate(&head->pci_dev->dev);
	qt5023_irq_remove(head);
	qt5023_hwicap_remove(head);
	head->pci_dev->dev.of_node = NULL;

	raw_spin_lock_irqsave(&devtree_lock, flags);
	*head->prev_node = *head->last_node;
	raw_spin_unlock_irqrestore(&devtree_lock, flags);

	vfree(head->of_mem);
	return;
}
