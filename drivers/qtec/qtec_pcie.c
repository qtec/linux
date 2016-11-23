/* qtec_pcie.c
 * qt523 PCIe bridge

   Copyright (C) 2011-2013 Qtechnology
     Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>

 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.

*/
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "qtec_pcie"

#define LINKDOWN (1<<0)
#define STREAMINGERR (1<<2)
#define HOTRESET (1<<3)
#define SLVUNSUPREQ (1<<20)
#define SLVUNEXCOMP (1<<21)
#define SLVCOMPTIMEOUT (1<<22)
#define SLVERRPOISON (1<<23)
#define SLVCOMPABORT (1<<24)
#define SLVILEGALBURST (1<<25)
#define MSTDECERR (1<<26)
#define MSTSLVERR (1<<27)
#define MSTERRPOISON (1<<28)

#define PCI_IRQ_DECODE 0x138
#define PCI_IRQ_MASK 0x13c

struct qtec_pcie{
	void __iomem *iomem;
	struct platform_device *pdev;
};

static inline int qtec_pcie_write(struct qtec_pcie *priv, uint32_t reg, uint32_t value){
	dev_dbg(&priv->pdev->dev, "pcie W:0x%.2x 0x%.8x\n",reg,value);
	iowrite32(value,priv->iomem+reg);
	return 0;
}

static inline int qtec_pcie_read(struct qtec_pcie *priv, uint32_t reg, uint32_t *value){
	*value=ioread32(priv->iomem+reg);
	dev_dbg(&priv->pdev->dev, "pcie R:0x%.2x 0x%.8x\n",reg,*value);
	return 0;
}

static int qtec_pcie_enable_all(struct qtec_pcie *priv){
	uint32_t value=0;

	//ACK old irqs
	qtec_pcie_read(priv,PCI_IRQ_DECODE,&value);
	qtec_pcie_write(priv,PCI_IRQ_DECODE,value);

	value|=LINKDOWN;
	value|=STREAMINGERR;
	value|=HOTRESET;
	value|=SLVUNSUPREQ;
	value|=SLVUNEXCOMP;
	value|=SLVCOMPTIMEOUT;
	value|=SLVERRPOISON;
	value|=SLVCOMPABORT;
	value|=SLVILEGALBURST;
	value|=MSTDECERR;
	value|=MSTSLVERR;
	value|=MSTERRPOISON;
	qtec_pcie_write(priv,PCI_IRQ_MASK,value);
	return 0;
}

static int qtec_pcie_disable_all(struct qtec_pcie *priv){
	qtec_pcie_write(priv,PCI_IRQ_MASK,0);
	return 0;
}

static irqreturn_t qtec_pcie_handler(int irq, void *data){
	uint32_t value;
	uint32_t ack_value=0;
	struct qtec_pcie *priv=data;

	qtec_pcie_read(priv,PCI_IRQ_DECODE,&value);

	if (value&LINKDOWN){
		dev_err(&priv->pdev->dev, "pcie: LinkDown Error\n");
		ack_value|=LINKDOWN;
	}
	if(value&STREAMINGERR){
		dev_err(&priv->pdev->dev, "pcie: Streaming Error\n");
		ack_value|=STREAMINGERR;
	}
	if (value&HOTRESET){
		dev_err(&priv->pdev->dev, "pcie: Hot Reset");
		ack_value|=HOTRESET;
	}
	if (value&SLVUNSUPREQ){
		dev_err(&priv->pdev->dev, "pcie: Slave Unsupported Request\n");
		ack_value|=SLVUNSUPREQ;
	}
	if (value&SLVUNEXCOMP){
		dev_err(&priv->pdev->dev, "pcie: Slave Unexpected Completion\n");
		ack_value|=SLVUNEXCOMP;
	}
	if (value&SLVCOMPTIMEOUT){
		dev_err(&priv->pdev->dev, "pcie: Slave Completion Timeout\n");
		ack_value|=SLVCOMPTIMEOUT;
	}
	if (value&SLVERRPOISON){
		dev_err(&priv->pdev->dev, "pcie: Slave Error Poison\n");
		ack_value|=SLVERRPOISON;
	}
	if (value&SLVCOMPABORT){
		dev_err(&priv->pdev->dev, "pcie: Slave Completion Abort\n");
		ack_value|=SLVCOMPABORT;
	}
	if (value&SLVILEGALBURST){
		dev_err(&priv->pdev->dev, "pcie: Slave Ilegal Burst\n");
		ack_value|=SLVCOMPABORT;
	}
	if (value&MSTDECERR){
		dev_err(&priv->pdev->dev, "pcie: Master Decode Error\n");
		ack_value|=MSTDECERR;
	}
	if (value&MSTSLVERR){
		dev_err(&priv->pdev->dev, "pcie: Master Slave Error\n");
		ack_value|=MSTSLVERR;
	}
	if (value&MSTERRPOISON){
		dev_err(&priv->pdev->dev, "pcie: Master Error Poison\n");
		ack_value|=MSTERRPOISON;
	}

	if (ack_value!=value)
		dev_err(&priv->pdev->dev, "pcie: Error 0x%.8x (DS820)\n",value);


	if (unlikely(ack_value==0)){
		dev_err(&priv->pdev->dev, "pcie: Spureous IRQ 0x%8x\n",value);
		return 0;
	}
	else
		qtec_pcie_write(priv,PCI_IRQ_DECODE,ack_value);

	return IRQ_HANDLED;
}


static int qtec_pcie_of_probe(struct platform_device *pdev){
	struct resource res;
	struct qtec_pcie *priv;
	int ret;

	/*Alloc priv*/
	priv=(struct qtec_pcie *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv){
		dev_err(&pdev->dev, "Unable to alloc priv\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

	/*iomem*/
	if (of_address_to_resource(pdev->dev.of_node,0,&res)){
		dev_err(&pdev->dev, "Unable to get memory address\n");
		return -EIO;
	}
	priv->iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem)){
		dev_err(&pdev->dev, "Unable to ioremap memory\n");
		return PTR_ERR(priv->iomem);
	}

	/*irq*/
	if (of_irq_to_resource(pdev->dev.of_node,0,&res)==0){
		dev_err(&pdev->dev, "Unable to get irq\n");
		return -EIO;
	}
	ret=devm_request_irq(&pdev->dev,res.start,qtec_pcie_handler,IRQF_SHARED,DRIVER_NAME,priv);
	if (ret)
		return ret;

	/*Enable all*/
	qtec_pcie_enable_all(priv);
	dev_info(&pdev->dev,"Axi-pcie bridge supervisor\n");
	return 0;
}

static int qtec_pcie_of_remove(struct platform_device *pdev){
	struct qtec_pcie *priv=platform_get_drvdata(pdev);

	qtec_pcie_disable_all(priv);
	return 0;
}

static struct of_device_id qtec_pcie_of_match[] = {
	{ .compatible = "xlnx,axi-pcie-1.00.a",},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_pcie_of_match);

static struct platform_driver qtec_pcie_plat_driver = {
	.probe		= qtec_pcie_of_probe,
	.remove		= qtec_pcie_of_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_pcie_of_match,
	},
};

module_platform_driver(qtec_pcie_plat_driver);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("AXI/PCIe bridge");
MODULE_LICENSE("GPL");
