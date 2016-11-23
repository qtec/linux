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
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME "qtec_mem"

static unsigned int size=64;
module_param(size, uint, 0400);
MODULE_PARM_DESC(size, "Required size (in MiB)");

static struct qtec_mem{
	void *iomem;
	dma_addr_t dma_handle;
	unsigned long len;
	struct class *class;
	dev_t first;
	struct cdev cdev;
	struct device *device;
}qtec_mem;

static ssize_t qtec_mem_read(struct file *filp, char __user *buff,
		    size_t count, loff_t *offset){
	struct qtec_mem *priv=&qtec_mem;

	return simple_read_from_buffer(buff, count, offset, priv->iomem, priv->len);
}

static ssize_t qtec_mem_write(struct file *filp, const char __user * buff,
		size_t count, loff_t *offset){
	struct qtec_mem *priv=&qtec_mem;

	return simple_write_to_buffer(priv->iomem, priv->len, offset, buff, count);
}

static int qtec_mem_mmap(struct file *flip, struct vm_area_struct *vma){
	int ret;
	long size = vma->vm_end - vma->vm_start ; //Not +1 or this crashes
	struct qtec_mem *priv=&qtec_mem;

	if (vma->vm_pgoff&PAGE_MASK){
		dev_err(priv->device,"Offset is not page aligned\n");
		return -EIO;
	}

	if ((size+(vma->vm_pgoff<<PAGE_SHIFT))  > priv->len){
		dev_err(priv->device,"Trying to map 0x%lx bytes, but only 0x%lx has been allocated on boot\n",
				size+(vma->vm_pgoff<<PAGE_SHIFT) ,priv->len);
		return -ENOMEM;
	}

	ret=remap_pfn_range(vma,vma->vm_start,PFN_DOWN(priv->dma_handle)+vma->vm_pgoff,
			size,vma->vm_page_prot);
	if (ret)
		return ret;

	return 0;
}

static const struct file_operations qtec_mem_fops = {
	.owner	= THIS_MODULE,
	.mmap	= qtec_mem_mmap,
	.read   = qtec_mem_read,
	.write   = qtec_mem_write,
};

static void qtec_mem_end(void){
	struct qtec_mem *priv=&qtec_mem;
	if (priv->iomem)
		dma_free_coherent(priv->device,priv->len,priv->iomem,priv->dma_handle);
	device_destroy(priv->class,priv->first);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->first, 1);
	class_destroy(priv->class);
}

static int qtec_mem_probe(void){
	struct qtec_mem *priv=&qtec_mem;
	int ret;

	priv->len=size*1024*1024;

	priv->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(priv->class)){
		printk(KERN_ERR "qtec_mem: Unable to create class\n");
		return PTR_ERR(priv->class);
	}

	ret=alloc_chrdev_region(&priv->first,0,1,DRIVER_NAME);
	if (ret){
		class_destroy(priv->class);
		printk(KERN_ERR "qtec_mem: Unable to alloc chrdev region\n");
		return  ret;
	}
	cdev_init(&priv->cdev,&qtec_mem_fops);
	ret =cdev_add(&priv->cdev,priv->first,1);
	if (ret){
		unregister_chrdev_region(priv->first, 1);
		class_destroy(priv->class);
		printk(KERN_ERR "qtec_mem: Unable to alloc chrdev region\n");
		return ret;
	}
	priv->device= device_create
			(priv->class,NULL,priv->first,
			 &priv->cdev,DRIVER_NAME);
	if (IS_ERR(priv->device)){
		cdev_del(&priv->cdev);
		unregister_chrdev_region(priv->first, 1);
		class_destroy(priv->class);
		printk(KERN_ERR "qtec_mem: Unable to create device\n");
		return PTR_ERR(priv->class);
	}

	priv->device->dma_mask=&x86_dma_fallback_dev.coherent_dma_mask;
	dma_set_coherent_mask(priv->device, DMA_BIT_MASK(64));
	priv->iomem = dma_alloc_coherent(priv->device,priv->len, &priv->dma_handle,GFP_KERNEL);
	if (priv->iomem)
		dev_info(priv->device, "qtec_mem: Allocated 0x%lx bytes\n",priv->len);
	else
		dev_err(priv->device, "qtec_mem: Unable to allocated 0x%lx bytes\n",priv->len);

	return 0;
}

static int __init qtec_mem_init(void)
{
	if (size<1){
		printk(KERN_ERR "qtec_mem: Unable to create qtec_mem device. qtec_dmamem_size is smaller than a PAGE\n");
		return -EIO;
	}
	return qtec_mem_probe();
}

static void __exit qtec_mem_exit(void)
{
	return qtec_mem_end();
}

static struct of_device_id qt5023_video_of_match[] = {
	{ .compatible = "qtec,axi_matrix_packer-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qt5023_video_of_match);

module_init(qtec_mem_init);
module_exit(qtec_mem_exit);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@uam.es>");
MODULE_DESCRIPTION("Qtec contiguos memory");
MODULE_LICENSE("GPL");
