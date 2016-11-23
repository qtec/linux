 /* qt523 PCIe speed test

   Copyright (C) 2014 Qtechnology
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
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME "qtec_pcitest"

static unsigned int size=64;
module_param(size, uint, 0400);
MODULE_PARM_DESC(size, "DMA Test size (in MiB)");

static bool memtest=false;
module_param(memtest, bool, 0400);
MODULE_PARM_DESC(memtest, "Do memory test");

static bool dmatest=true;
module_param(dmatest, bool, 0400);
MODULE_PARM_DESC(dmatest, "Do dma test");

static bool test64=false;
module_param(test64, bool, 0400);
MODULE_PARM_DESC(test64, "Do 64 bit test");

#define MAX_APERTURES 6
#define APERTURE_REG_LEN 8
struct qtec_pcitest_pcie_aperture{
	int index;
	uint32_t fpga_addr;
	unsigned long length;
};

/*CDMA*/
#define CDMACR 0x0
#define CDMA_TAILPTREN 1
#define CDMA_RESET 2
#define CDMA_SGMODE 3
#define CDMA_ERRIRQEN 14
#define CDMA_IOCIRQEN 12
#define CDMA_IRQTHRESHOLD 16
#define CDMA_IRQTHRESHOLD_LEN 8

#define CDMASR 0x4
#define CDMA_IDLE 1
#define CDMA_IRQSLVERR  5
#define CDMA_IRQDECERR  6
#define CDMA_IRQSGINTERR  8
#define CDMA_IRQSGSLVERR 9
#define CDMA_IRQSGDECERR 10
#define CDMA_IRQIOC 12
#define CDMA_IRQERR 14

#define CDMACURDESC 0x8

#define CDMATAILDESC 0x10

#define MAX_DESC_SIZE 8388607
#define MAX_DESC 255
struct axi_cdma_desc{
	uint32_t next;
	uint32_t reserved0;
	uint32_t source;
	uint32_t reserved1;
	uint32_t dest;
	uint32_t reserved2;
	uint32_t length;
	uint32_t status;
	uint32_t reserved[8];
};

struct qtec_pcitest{
	struct platform_device *pdev;

	//cdma
	void __iomem *cdma_iomem;
	int cdma_n_desc;
	void __iomem *cdma_desc_iomem;
	uint32_t cdma_desc_base_addr;

	//apertures
	void __iomem *aperture_iomem;
	struct qtec_pcitest_pcie_aperture apertures[MAX_APERTURES];
	int n_apertures;

	//qtec mem
	void *cma_iomem;
	long cma_len;
	dma_addr_t cma_handle;

	unsigned long circular_length;
	uint32_t circular_address;
	void __iomem *circular_iomem;

	struct timeval tv_init,tv_end;

	struct completion dma_done;
	bool fromfpga;
	long max_dma;
	long test_size;
};

#define APERTURE_OFFSET 0x208
static inline int qtec_pcitest_aperture_write(struct qtec_pcitest *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "aperture W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->aperture_iomem+offset+APERTURE_OFFSET);
	return 0;
}

static inline int qtec_pcitest_aperture_read(struct qtec_pcitest *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->aperture_iomem+offset+APERTURE_OFFSET);
	dev_dbg(&priv->pdev->dev, "aperture R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}


static inline int qtec_pcitest_cdma_write(struct qtec_pcitest *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "cdma W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->cdma_iomem+offset);
	return 0;
}

static inline int qtec_pcitest_cdma_read(struct qtec_pcitest *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->cdma_iomem+offset);
	dev_dbg(&priv->pdev->dev, "cdma R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#define CDMA_IRQ_ALL (BIT(CDMA_IRQERR)|BIT(CDMA_IRQIOC))
static irqreturn_t qtec_pcitest_cdma_irq_handler(int irq, void *data){
	struct qtec_pcitest *priv=data;
	uint32_t aux=0;
	long timediff;
	uint32_t speed;


	qtec_pcitest_cdma_read(priv,CDMASR,&aux);
	qtec_pcitest_cdma_write(priv,CDMASR,aux); //ACK irq ASAP

	do_gettimeofday(&priv->tv_end);

	if (unlikely((aux & CDMA_IRQ_ALL)==0)){
		dev_err(&priv->pdev->dev, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(BIT(CDMA_IRQERR)&aux)){
		uint32_t curdesc;
		qtec_pcitest_cdma_read(priv,CDMACURDESC,&curdesc);
		dev_err(&priv->pdev->dev, "Error processing descriptor 0x%.8x\n",curdesc);

		if (BIT(CDMA_IRQSGDECERR)&aux)
			dev_err(&priv->pdev->dev, "Scatter Gather Decode Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSGSLVERR)&aux)
			dev_err(&priv->pdev->dev, "Scatter Gather Slave Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSGINTERR)&aux)
			dev_err(&priv->pdev->dev, "Scatter Gather Internal Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQDECERR)&aux)
			dev_err(&priv->pdev->dev, "Decode Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSLVERR)&aux)
			dev_err(&priv->pdev->dev, "Slave Error IRQ 0x%.8x\n",aux);

		//Reset core
		if ((BIT(CDMA_IRQSGDECERR)&aux)|| (BIT(CDMA_IRQSGSLVERR)&aux) || (BIT(CDMA_IRQSGINTERR)&aux) ||
			(BIT(CDMA_IRQDECERR)&aux) ||(BIT(CDMA_IRQSLVERR)&aux))
				qtec_pcitest_cdma_write(priv,CDMACR,BIT(CDMA_RESET));
	}

	timediff = priv->tv_end.tv_sec - priv->tv_init.tv_sec;
	timediff *= 1000000;
	timediff += priv->tv_end.tv_usec;
	timediff -= priv->tv_init.tv_usec;

	speed = priv->test_size/timediff;
	//MB to MiB
	speed *= 954;
	speed /= 1000;

	dev_err(&priv->pdev->dev, "Test %s for size:0x%lx byte",(priv->fromfpga)?"FPGA->PC":"PC->FPGA",priv->test_size);
	dev_err(&priv->pdev->dev,"   time:%ld usec speed: %ld MiBps \n",timediff,(long)speed);

	complete(&priv->dma_done);

	return IRQ_HANDLED;
}

static int qtec_pcitest_cdma_init(struct qtec_pcitest *priv){
	uint32_t aux=BIT(CDMA_RESET);
	unsigned long timeout = jiffies + HZ;

	qtec_pcitest_cdma_write(priv,CDMACR,aux);

	while (BIT(CDMA_RESET)&aux){
		if (time_before(timeout,jiffies)){
			dev_err(&priv->pdev->dev, "Timeout while reseting cdma\n");
			return -1;
		}
		qtec_pcitest_cdma_read(priv,CDMACR,&aux);
	}

	return 0;
}

static int qt5032_parse_apertures(struct qtec_pcitest *priv, struct device_node *node){
	int n_ap;
	int max_aperture=MAX_APERTURES-1;
	int min_aperture=0;
	uint32_t apertures_val[MAX_APERTURES*2];
	int ret;

	ret=of_property_read_u32_array(priv->pdev->dev.of_node,"qtec,aperture-addr",apertures_val,2);
	if (ret==0){
		min_aperture=apertures_val[0];
		max_aperture=apertures_val[0]+apertures_val[1]-1;
		dev_info(&priv->pdev->dev, "Using apertures %d-%d\n",min_aperture,max_aperture);
	}

	if (of_property_read_u32_array(node,"qtec,apertures",apertures_val,(max_aperture+1)*2)!=0){
		dev_err(&priv->pdev->dev, "Unable to get aperture geometry\n");
		return -EIO;
	}

	priv->n_apertures = max_aperture-min_aperture+1;
	for (n_ap=0;n_ap<priv->n_apertures;n_ap++){
		priv->apertures[n_ap].fpga_addr=apertures_val[(min_aperture+n_ap)*2];
		priv->apertures[n_ap].length=apertures_val[((min_aperture+n_ap)*2)+1];
		priv->apertures[n_ap].index=min_aperture+n_ap;
		dev_dbg(&priv->pdev->dev, "Appertures %d 0x%x 0x%lx \n",priv->apertures[n_ap].index,priv->apertures[n_ap].fpga_addr,priv->apertures[n_ap].length);
	}
	return 0;
}


static int qtec_pcitest_dmatest_size(struct  qtec_pcitest *priv, bool fromfpga, long size){
	int last_apperture=-1;
	struct axi_cdma_desc *desc=priv->cdma_desc_iomem;
	int ndesc=0;
	long offset=0;
	long apperture_offset;
	uint32_t aux;

	qtec_pcitest_cdma_init(priv);

	while (offset<size){
		long len=size-offset;

		if ((last_apperture<0)||(apperture_offset>=priv->apertures[last_apperture].length)){
			uint64_t ap_addr;
			last_apperture ++;

			if (last_apperture>priv->n_apertures)
				break;

			ap_addr=priv->cma_handle+offset;
			apperture_offset= ap_addr & (priv->apertures[last_apperture].length-1);
			ap_addr -= apperture_offset;
			qtec_pcitest_aperture_write(priv,(priv->apertures[last_apperture].index*APERTURE_REG_LEN)+4,ap_addr&0xffffffff);
			qtec_pcitest_aperture_write(priv,(priv->apertures[last_apperture].index*APERTURE_REG_LEN)+0,(ap_addr>>32)&0xffffffff);
		}

		len=min_t(long,len,MAX_DESC_SIZE);
		len=min_t(long,len,priv->apertures[last_apperture].length-apperture_offset);
		desc[ndesc].next=priv->cdma_desc_base_addr+((ndesc+1)*sizeof(*desc));
		desc[ndesc].source=priv->circular_address+offset;
		desc[ndesc].dest=priv->apertures[last_apperture].fpga_addr+apperture_offset;
		desc[ndesc].status=0;
		desc[ndesc].length=len;

		if (!fromfpga){
			aux=desc[ndesc].source;
			desc[ndesc].source=desc[ndesc].dest;
			desc[ndesc].dest=aux;
		}

		ndesc++;
		offset +=len;
		apperture_offset += len;

		if (ndesc==MAX_DESC)
			break;
	}

	priv->test_size=offset;
	priv->fromfpga=fromfpga;

	if (ndesc)
		desc[ndesc-1].next=priv->cdma_desc_base_addr;
	wmb();

	//Clear SG (Otherwise curdesc is ignored)
	aux= ndesc << CDMA_IRQTHRESHOLD;
	aux|=BIT(CDMA_ERRIRQEN)|BIT(CDMA_IOCIRQEN)|BIT(CDMA_TAILPTREN)|BIT(CDMA_SGMODE);
	qtec_pcitest_cdma_write(priv,CDMACR,aux);

	qtec_pcitest_cdma_write(priv,CDMACURDESC,priv->cdma_desc_base_addr);

	//Launch dma
	do_gettimeofday(&priv->tv_init);
	qtec_pcitest_cdma_write(priv,CDMATAILDESC,priv->cdma_desc_base_addr+(ndesc-1)*sizeof(*desc));

	return 0;
}

static int qtec_pcitest_dmatest(struct qtec_pcitest *priv){
	long len;
	int dest;

	dev_info(&priv->pdev->dev, "PCI CDMA test. Max size 0x%lx\n",priv->max_dma);

	for (dest=0;dest<=1;dest++)
		for (len=0x2000;len<=priv->max_dma;len<<=1){
			qtec_pcitest_dmatest_size(priv,(dest!=0),len);
			if (!wait_for_completion_timeout(&priv->dma_done,HZ))
				dev_err(&priv->pdev->dev, "Timeout on test with size 0x%lx\n",len);
		}

	return 0;
}

static int qtec_pcitest_pattern(struct qtec_pcitest *priv, u64 pattern ,u64 *start, u64 *end){
	u64 *p;

	for (p=start; p<end; p++)
		*p = pattern;

	for (p=start; p<end; p++){
		if (*p == pattern)
			continue;
		dev_err(&priv->pdev->dev, "Error at offset 0x%lx (%p)\n",p-start,p);
		dev_err(&priv->pdev->dev, "written 0x%llx read 0x%llx, diff=0x%llx\n",pattern,*p,*p ^ pattern);
		return -1;
	}

	return 0;
}
static int qtec_pcitest_memtest(struct qtec_pcitest *priv){
	u64 *start, *end;
	static u64 patterns[] = {
		0x7a6c7258554e494cULL, /* yeah ;-) */
		0x0123456789abcdefULL,
		0xffffffffffffffffULL,
		0x5555555555555555ULL,
		0xaaaaaaaaaaaaaaaaULL,
		0x1111111111111111ULL,
		0x2222222222222222ULL,
		0x4444444444444444ULL,
		0x8888888888888888ULL,
		0x3333333333333333ULL,
		0x6666666666666666ULL,
		0x9999999999999999ULL,
		0xccccccccccccccccULL,
		0x7777777777777777ULL,
		0xbbbbbbbbbbbbbbbbULL,
		0xddddddddddddddddULL,
		0xeeeeeeeeeeeeeeeeULL,
		0,
	};
	int i;
	u64 mem = (u64) priv->circular_iomem;

	dev_info(&priv->pdev->dev, "PCI Memory test. Max size 0x%lx\n",priv->circular_length);

	start = (u64 *) ALIGN(mem, sizeof(patterns[0]));
	end = (u64 *) ALIGN(mem+priv->circular_length, sizeof(patterns[0]));

	for (i=0;i<ARRAY_SIZE(patterns);i++){
		dev_info(&priv->pdev->dev, "Testing pattern 0x%lx\n",(long)patterns[i]);
		qtec_pcitest_pattern(priv,patterns[i],start,end);
	}
	return 0;
}

#define PAT64 0x0123456789abcdefULL
static int qtec_pcitest_64btest(struct qtec_pcitest *priv){
	volatile u64 *start = priv->circular_iomem;
	volatile u64 read;
	int i;

	dev_info(&priv->pdev->dev, "64 bit test\n");

	for (i=0;i<8;i++){
		start[i] = PAT64;
		read = start[i];
		if (read != PAT64)
			dev_info(&priv->pdev->dev, "Error at offset 0x%x 0x%llx != 0x%llx\n",i*8,read,PAT64);
	}

	return 0;
}

static int qtec_pcitest_of_remove(struct platform_device *pdev){
	struct qtec_pcitest *priv=platform_get_drvdata(pdev);

	qtec_pcitest_cdma_init(priv);

	return 0;
}

static int qtec_pcitest_of_probe(struct platform_device *pdev){
	struct device_node *node;
	struct resource res;
	struct qtec_pcitest *priv;
	int ret;

	/*Alloc priv*/
	priv=(struct qtec_pcitest *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv){
		dev_err(&pdev->dev, "Unable to alloc priv\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

	/*cdma*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,cdma",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get cdma phandle\n");
		return -EIO;
	}
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma address\n");
		return ret;
	}

	priv->cdma_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->cdma_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap cdma\n");
		return PTR_ERR(priv->cdma_iomem);
	}

	ret=of_irq_to_resource(node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get cdma irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,qtec_pcitest_cdma_irq_handler,0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request cdma irq\n");
		return ret;
	}
	of_node_put(node);

	/*descriptors*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,desc_mem",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get cdma descriptor node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptor address\n");
		return ret;
	}

	priv->cdma_desc_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->cdma_desc_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap descriptors\n");
		return PTR_ERR(priv->cdma_desc_iomem);
	}

	priv->cdma_n_desc=resource_size(&res)/sizeof(struct axi_cdma_desc);
	if (priv->cdma_n_desc>((1<<CDMA_IRQTHRESHOLD_LEN)-1))
		priv->cdma_n_desc=(1<<CDMA_IRQTHRESHOLD_LEN)-1;
	ret=of_property_read_u32(node,"reg-axi",&priv->cdma_desc_base_addr);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptors base address\n");
		return -EIO;
	}
	of_node_put(node);

	/*video mem*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,circular_buffer",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get circular buffer descriptor node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get circular buffer descriptor address\n");
		return ret;
	}
	priv->circular_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->cdma_desc_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap circular memory\n");
		return PTR_ERR(priv->cdma_desc_iomem);
	}
	priv->circular_length=resource_size(&res);
	ret=of_property_read_u32(node,"reg-axi",&priv->circular_address);
	if (ret){
		dev_err(&pdev->dev, "Unable to get circular buffer base address\n");
		return -EIO;
	}
	of_node_put(node);

	/*aperture*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,pcie_bridge",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get pcie bridge node\n");
		return -EIO;
	}
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get aperture address\n");
		return ret;
	}

	//Dont request or qtec_pcie wont load
	priv->aperture_iomem  = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (IS_ERR(priv->aperture_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap pci bridge\n");
		return PTR_ERR(priv->aperture_iomem);
	}

	ret=qt5032_parse_apertures(priv,node);
	if (ret)
		return ret;

	of_node_put(node);

	priv->cma_len=size*1024*1024;
	dma_set_coherent_mask(&priv->pdev->dev, DMA_BIT_MASK(64));
	priv->cma_iomem = dma_alloc_coherent(&priv->pdev->dev,priv->cma_len, &priv->cma_handle,GFP_KERNEL);
	if (!priv->cma_iomem){
		dev_err(&pdev->dev, "Unable to alloc enough memory, Try removing qtec_mem module\n");
		return -EIO;
	}

	priv->max_dma=min_t(long,priv->cma_len,priv->circular_length);
	priv->max_dma=min_t(long,priv->max_dma,MAX_DESC*MAX_DESC_SIZE);

	init_completion(&priv->dma_done);

	if (memtest)
		qtec_pcitest_memtest(priv);

	if (dmatest)
		qtec_pcitest_dmatest(priv);

	if (test64)
		qtec_pcitest_64btest(priv);

	dma_free_coherent(&priv->pdev->dev,priv->cma_len,priv->cma_iomem,priv->cma_handle);

	return 0;
}

static struct of_device_id qtec_pcitest_of_match[] = {
	{ .compatible = "qtec,memory_test-1.00.a"},
	{ .compatible = "qtec,axi_matrix_packer-1.00.a"},
	{ /* EOL */}
};

static struct platform_driver qtec_pcitest_plat_driver = {
	.probe		= qtec_pcitest_of_probe,
	.remove		= qtec_pcitest_of_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_pcitest_of_match,
	},
};

module_platform_driver(qtec_pcitest_plat_driver);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("AXI/PCIe speed test");
MODULE_LICENSE("GPL");
