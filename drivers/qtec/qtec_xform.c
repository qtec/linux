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
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-vmalloc.h>

#define DRIVER_NAME "qtec_xform"

#define QUIRK_FB4 BIT(0)
#define QUIRK_FIFO_MONITOR BIT(1)
#define QUIRK_COMPACT_ONLY BIT(2)
#define QUIRK_GAIN_ONLY BIT(3)
#define QUIRK_GAIN_SCALE BIT(4)

enum xform_types {LEGACY_XFORM, FB4_XFORM};

struct qtec_xform_buf{
	struct vb2_buffer vb2_buf; //Must be first in struct
};

struct qtec_xform_dist_initial_pixel{
	uint16_t start_line;
	uint16_t start_col;
};

struct qtec_xform_dist_pixel{
	uint8_t cy;
	uint8_t cx;
	uint8_t gain;
	uint8_t move;
};

#define MAX_DESC_SIZE ((8*1024*1024)-1)
struct qtec_xform_axi_cdma_desc{
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

struct qtec_xform{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct v4l2_ctrl_handler ctrl_handler;

	struct work_struct work;

	uint32_t quirk;

	//vdev
	struct v4l2_device v4l2_dev;

	//vb gain
	struct vb2_queue vb2_vidq_gain;
	struct video_device	   vdev_gain;
	struct mutex vdev_gain_mutex;

	//vb dist
	struct vb2_queue vb2_vidq_dist;
	struct video_device	   vdev_dist;
	struct mutex vdev_dist_mutex;

	//vb gain read back
	struct vb2_queue vb2_vidq_gain_rb;
	struct video_device	   vdev_gain_rb;
	struct mutex vdev_gain_rb_mutex;

	//vb dist read back
	struct vb2_queue vb2_vidq_dist_rb;
	struct video_device	   vdev_dist_rb;
	struct mutex vdev_dist_rb_mutex;

	/*io*/
	void __iomem *iomem;
	unsigned int buffer_size_pixels;
	struct platform_device *pdev;

	/*Controls*/
	struct v4l2_ctrl	   *gain_enable;
	struct v4l2_ctrl	   *dist_enable;
	struct v4l2_ctrl	   *extra_gain;
	struct v4l2_ctrl	   *dist_buffer_size;
	struct v4l2_ctrl	   *fifo_size;
	struct v4l2_ctrl	   *min_fifo_level;
	struct v4l2_ctrl	   *xform_hflip;
	struct v4l2_ctrl	   *gain_scale;

	bool fg_is_streaming;
	bool xform_is_streaming;
	struct v4l2_mbus_framefmt format_in;
	struct v4l2_mbus_framefmt format_out;
	bool independent_dma;

#define QUEUE 0
#define BUFFER 1
#define RESIZED 2
	struct v4l2_mbus_framefmt format_gain[2];
	struct v4l2_mbus_framefmt format_dist[3];

	/*Dist mem*/
	void __iomem *dist_mem;
	uint32_t dist_mem_base_addr;
	unsigned long dist_mem_len;
	unsigned long dist_mem_last_offset;

	/*User structures*/
	unsigned long usr_gain_size;
	void *usr_gain_mem;
	void *usr_dist_mem;
	unsigned long usr_dist_size; //One extra line
	unsigned long usr_dist_size_real;

	//dma
	void __iomem *dma_iomem;
	int dma_n_desc;
	int dma_n_desc_preloaded;
	void __iomem *dma_desc_iomem;
	uint32_t dma_desc_base_addr;
	int dma_running_desc;
	uint32_t dma_last_circular;
};

#define PARAM_BASEADDR 0x0

#define PARAM_LEN 0x4
#define ACTIVE 0
#define TOTAL 16

#define H_PARAMS 0x8

#define V_PARAMS 0xc

#define BUFF_PARAMS 0x10
#define PRE_SIZE 0
#define PRE_SIZE_LEN 24
#define EXTRA_GAIN 24
#define EXTRA_GAIN_LEN 8

#define CTRL 0x14
#define VERSION_MASK 0xff
#define SIZE_KBYTES 8
#define SIZE_KBYTES_MASK 0xff
#define STAT_BYPASS 22
#define STAT_RUN 23
#define IRQ_STATUS 24
#define IRQENA 25
#define URIRQENA 26
#define URIRQ_STATUS 27
#define RUN 31

#define FIFO 0x18
#define FIFO_MASK 0xffff
#define FIFO_LEVEL 0
#define FIFO_SIZE 16

#define GAIN_SCALE 0x1c

/*FB4*/
#define CTRL_FB4 0
#define ENABLE_FB4 0
#define IRQ_URUN_ENA_FB4 1
#define VERSION_FB4 24
#define ENABLE_ST_FB4 16
#define IRQ_URUN_FB4 17

#define OFFSET_FB4 0x4

/*DMA*/
#define CDMA_IRQTHRESHOLD_LEN 8
#define DMACR 0x0
#define RS 0
#define RESET 2
#define IOC_IRQEN 12
#define ERR_IRQEN 14
#define IRQTHRESHOLD 16

#define DMASR 0x04
#define DMAINTERR 4
#define DMASLVERR 5
#define DMADECERR 6
#define SGINTERR 8
#define SGSLVERR 9
#define SGDECERR 10
#define IOC_IRQ 12
#define DLY_IRQ 13
#define ERR_IRQ 14

#define CURDESC 0x08
#define TAILDESC 0x10

static inline int qtec_xform_dma_write(struct qtec_xform *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "xform (dma) W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->dma_iomem+offset);
	return 0;
}

static inline int qtec_xform_dma_read(struct qtec_xform *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->dma_iomem+offset);
	dev_dbg(&priv->pdev->dev, "xform (dma) R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static inline int qtec_xform_write(struct qtec_xform *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "xform W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem+offset);
	return 0;
}

static inline int qtec_xform_read(struct qtec_xform *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "xform R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static irqreturn_t qtec_xform_irq_handler_legacy(int irq, void *data){
	struct qtec_xform *priv=data;
	uint32_t aux;

	qtec_xform_read(priv,CTRL,&aux);
	qtec_xform_write(priv,CTRL,aux);//ACK irq asap

	if (unlikely(! (aux & (BIT(IRQ_STATUS) | BIT(URIRQ_STATUS))))){
		v4l2_err(&priv->sd, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (aux & BIT(URIRQ_STATUS))
		v4l2_err(&priv->sd, "Underrun IRQ! 0x%.8x\n",aux);

	//Nothing is done today...
	return IRQ_HANDLED;
}

static irqreturn_t qtec_xform_irq_handler_fb4(int irq, void *data){
	struct qtec_xform *priv=data;
	uint32_t aux;

	qtec_xform_read(priv,CTRL_FB4,&aux);
	qtec_xform_write(priv,CTRL_FB4,aux);//ACK irq asap

	if (aux & IRQ_URUN_FB4)
		v4l2_err(&priv->sd, "Xform buffer Underrun IRQ 0x%.8x\n",aux);
	else
		v4l2_err(&priv->sd, "Spureous IRQ 0x%.8x\n",aux);

	//Nothing is done today...
	return IRQ_HANDLED;
}

#define S_RXEOF 26
#define S_RXSOF 27
#define S_DMAINTERR 28
#define S_DMASLVERR 29
#define S_DMADECERR 30
#define S_CMPLT 31
#define DMA_IRQ_ALL (BIT(ERR_IRQ)|BIT(DLY_IRQ)|BIT(IOC_IRQ))

static irqreturn_t qtec_xform_irq_handler_dma(int irq, void *data){
	struct qtec_xform *priv=data;
	uint32_t aux;

	qtec_xform_dma_read(priv,DMASR,&aux);
	qtec_xform_dma_write(priv,DMASR,aux);//ACK irq asap
		if (unlikely((aux & DMA_IRQ_ALL)==0)){
		v4l2_err(&priv->v4l2_dev, "DMA Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(aux & (BIT(ERR_IRQ) | BIT(DLY_IRQ)))){
		uint32_t curdesc;
		qtec_xform_dma_read(priv,CURDESC,&curdesc);

		v4l2_err(&priv->v4l2_dev, "DMA: Error processing descriptor 0x%.8x (0x%.8x)\n", curdesc, aux);

		if (aux & BIT(DMAINTERR))
			v4l2_err(&priv->v4l2_dev, "DMA Internal Error 0x%.8x\n",aux);

		if (aux & BIT(DMASLVERR))
			v4l2_err(&priv->v4l2_dev, "DMA Slave Error 0x%.8x\n",aux);

		if (aux & BIT(DMADECERR))
			v4l2_err(&priv->v4l2_dev, "DMA Decode Error 0x%.8x\n",aux);

		if (aux & BIT(SGINTERR))
			v4l2_err(&priv->v4l2_dev, "DMA SG Internal Error 0x%.8x\n",aux);

		if (aux & BIT(SGSLVERR))
			v4l2_err(&priv->v4l2_dev, "DMA SG Slave Error 0x%.8x\n",aux);

		if (aux & BIT(SGDECERR))
			v4l2_err(&priv->v4l2_dev, "DMA SG Decode Error 0x%.8x\n",aux);

		if (unlikely(aux & BIT(DLY_IRQ)))
			v4l2_err(&priv->v4l2_dev, "DMA: Delay IRQ 0x%.8x\n", aux);
	}

	if (likely(aux & IOC_IRQ)){
		uint32_t status;
		struct qtec_xform_axi_cdma_desc __iomem *desc=priv->dma_desc_iomem;

		while ((status = ioread32(&desc[priv->dma_running_desc].status)) & BIT(S_CMPLT)){
			dev_dbg(&priv->pdev->dev, "xform desc %d: status 0x%.8x\n", priv->dma_running_desc,ioread32(&desc[priv->dma_running_desc].status));

			if (status & BIT(S_DMAINTERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Internal Error\n",priv->dma_running_desc);
			if (status & BIT(S_DMASLVERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Slave Error\n",priv->dma_running_desc);
			if (status & BIT(S_DMADECERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Decode Error\n",priv->dma_running_desc);

			iowrite32(0,&desc[priv->dma_running_desc].status);
			qtec_xform_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+priv->dma_running_desc*sizeof(*desc));
			priv->dma_running_desc ++;
			if (priv->dma_running_desc == priv->dma_n_desc_preloaded)
				priv->dma_running_desc = 0;
		}
		dev_dbg(&priv->pdev->dev, "xform desc %d: status = 0x%.8x\n", priv->dma_running_desc,ioread32(&desc[priv->dma_running_desc].status));
	}

	return IRQ_HANDLED;
}

static int qtec_xform_set_extra_gain(struct qtec_xform *priv, uint32_t val){

	if (priv->quirk & QUIRK_FB4){
		qtec_xform_write(priv,OFFSET_FB4,val);
	} else {
		uint32_t aux;
		qtec_xform_read(priv,BUFF_PARAMS,&aux);
		aux&=~ (((1<<EXTRA_GAIN_LEN)-1)<<EXTRA_GAIN);
		aux|=val<<EXTRA_GAIN;
		qtec_xform_write(priv,BUFF_PARAMS,aux);
	}

	return 0;
}

static bool qtec_xform_is_compact(uint32_t code){
	switch(code){
		case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			return true;
		default:
			return false;
	}
}

static inline int qtec_xform_width_divider(struct qtec_xform *priv){
	if (qtec_xform_is_compact(priv->format_in.code) &&
		!(priv->quirk & QUIRK_COMPACT_ONLY))
		return 5;
	return 1;
}

static inline int qtec_xform_width(struct qtec_xform *priv){

	return priv->format_in.width/qtec_xform_width_divider(priv);;
}

inline static bool qtec_xform_do(struct qtec_xform *priv, bool dist){
	if (dist)
		return priv->dist_enable->val  && !(priv->dist_enable->flags&V4L2_CTRL_FLAG_INACTIVE);

	return priv->gain_enable->val && !(priv->gain_enable->flags&V4L2_CTRL_FLAG_INACTIVE);
}

static bool qtec_xform_can_run(struct qtec_xform *priv, bool dist){

	if ( !qtec_xform_is_compact(priv->format_in.code) !=
		!(priv->quirk & QUIRK_COMPACT_ONLY))
		return false;

	//Wrong sizes
	if (dist)
		return (memcmp(&priv->format_dist[BUFFER],&priv->format_in,sizeof(priv->format_in))) == 0;

	return (memcmp(&priv->format_gain[BUFFER],&priv->format_out,sizeof(priv->format_out))) == 0;
}

static int qtec_xform_get_geometry(struct qtec_xform *priv, void *iomem, unsigned int *width, unsigned int *height){
	unsigned int final_lines=0;
	unsigned int final_cols=0;
	struct qtec_distortion *dist_image=iomem;
	int xform_width=qtec_xform_width(priv);
	int in=0;
	int line,col;

	for(line=0;line<priv->format_in.height;line++){
		int this_col=0;

		//Number of cols
		for(col=0;col<xform_width;col++){
			if ((dist_image[in].col >= 0) && (dist_image[in].line >= 0))
				this_col++;
			in++;
		}
		if (!final_cols)
			final_cols=this_col;

		if (this_col){
			final_lines++;
			if (final_cols!=this_col){
				dev_err(&priv->pdev->dev, "Inconsistent number of columns on line %d %d!=%d\n",line,final_cols,this_col);
				return -1;
			}
		}

	}

	if (!final_cols || !final_lines || (final_cols%4)){
		dev_err(&priv->pdev->dev, "Unsupported final geometry %dx%d (width must be multiple of 4)",final_cols,final_lines);
		return -1;
	}

	if ((priv->quirk & QUIRK_COMPACT_ONLY) && (final_cols%5)){
		dev_err(&priv->pdev->dev, "Unsupported final geometry %dx%d (width must be multiple of 5)",final_cols,final_lines);
		return -1;
	}

	if (qtec_xform_is_compact(priv->format_in.code) && !(priv->quirk & QUIRK_COMPACT_ONLY))
		final_cols*=5;

	*width=final_cols;
	*height=final_lines;

	return 0;
}

static int qtec_xform_program_dma(struct qtec_xform *priv,uint32_t new_offset,size_t len){
	int j=0;
	uint32_t frame_size;
	uint32_t last_offset=0;
	struct qtec_xform_axi_cdma_desc __iomem *desc=priv->dma_desc_iomem;
	uint32_t aux;

	if (((len+MAX_DESC_SIZE-1)/MAX_DESC_SIZE)>priv->dma_n_desc){
		dev_err(&priv->pdev->dev, "Not enough descriptors to start the xform. Aborting\n");
		return -ENOMEM;
	}

	if (priv->dist_mem_len < len){
		dev_err(&priv->pdev->dev, "Not enough distortion memory xform. Aborting\n");
		return -ENOMEM;
	}

	frame_size = len;
	while (j<priv->dma_n_desc){
		int dlen= (frame_size >MAX_DESC_SIZE) ? MAX_DESC_SIZE:frame_size;

		memset_io(&desc[j],0,sizeof(*desc));
		iowrite32(priv->dma_desc_base_addr+((j+1)*sizeof(*desc)),&desc[j].next);
		iowrite32(priv->dist_mem_base_addr + new_offset + last_offset,&desc[j].source);
		aux = dlen;
		if (frame_size == len)
			aux |=  BIT(S_RXSOF);
		if (frame_size == dlen){
			aux |=  BIT(S_RXEOF);
			priv->dma_n_desc_preloaded = j +1;
			frame_size = len;
			last_offset = 0;
		}else{
			frame_size -=dlen;
			last_offset += dlen;
		}
		iowrite32(aux,&desc[j].length);
		dev_dbg(&priv->pdev->dev, "DMA Descriptor %.2d Addr:0x%.8x Next:0x%.8x len:0x%.8x\n",j,desc[j].source,desc[j].next,desc[j].length);
		j ++;
	}
	//Loop the descriptors
	iowrite32(priv->dma_desc_base_addr,&desc[priv->dma_n_desc_preloaded-1].next);

	dev_dbg(&priv->pdev->dev, "DMA Descriptor %.3d Addr:0x%.8x Next:0x%.8x len:0x%.8x\n",priv->dma_n_desc_preloaded-1,desc[priv->dma_n_desc_preloaded-1].source,desc[priv->dma_n_desc_preloaded-1].next,desc[priv->dma_n_desc_preloaded-1].length);

	priv->dma_running_desc = 0;

	//Only reset the dma engine if it is not shared with other cores
	if (priv->independent_dma){
		dev_dbg(&priv->pdev->dev, "Restaring DMA");
		qtec_xform_dma_write(priv,DMACR,BIT(RESET));
		qtec_xform_dma_write(priv,DMACR,0);
		while(1){
			qtec_xform_dma_read(priv,DMACR,&aux);
			if (!(aux & BIT(RESET)))
				break;
		}
	}
	qtec_xform_dma_write(priv,DMACR,0);
	qtec_xform_dma_write(priv,CURDESC,priv->dma_desc_base_addr);
	qtec_xform_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+((priv->dma_n_desc_preloaded-1)*sizeof(*desc)));
	aux = BIT(IOC_IRQEN) | BIT(ERR_IRQEN);
	aux |= 1<< IRQTHRESHOLD;
	aux |= BIT(RS);
	qtec_xform_dma_write(priv,DMACR,aux);
	//FIXME, check if it is needed
	qtec_xform_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+((priv->dma_n_desc_preloaded-1)*sizeof(*desc)));

	return 0;
}

static int qtec_xform_convert_user_files_fb4(struct qtec_xform *priv,void __iomem *iomem,size_t len){
	if (priv->format_in.code != MEDIA_BUS_FMT_QTEC_FB4_RGBX){
		memcpy(iomem,priv->usr_gain_mem,len);
		return 0;
	} else{
		size_t i;
		len/=4;
		for(i=0;i<len;i++)
			memset(iomem+4*i,((uint8_t *)priv->usr_gain_mem)[i],4);
	}
	return 0;
}

#define SKIP_LINE 0x8000
#define LAST_LINE 0x4000
#define SKIP_COL 7
#define MASK_DIFF 0x7
#define MIN_DIFF -4
#define MAX_DIFF 3
#define COL_DIFF 3
static int qtec_xform_convert_user_files_legacy(struct qtec_xform *priv,void __iomem *iomem){
	struct qtec_xform_dist_pixel __iomem *pix=iomem;
	struct qtec_xform_dist_initial_pixel __iomem *pix_ini=iomem;
	int in_dist, in_gain, out;
	int line,col;
	int do_gain = qtec_xform_do(priv, false);
	int do_dist = qtec_xform_do(priv, true);
	int do_hflip=v4l2_ctrl_g_ctrl(priv->xform_hflip);
	int xform_width=qtec_xform_width(priv);
	unsigned int buffer_len_lines=priv->buffer_size_pixels/xform_width;
	unsigned int buffer_pre_lines;
	uint8_t *gain_image=priv->usr_gain_mem;
	struct qtec_distortion *dist_image=priv->usr_dist_mem;
	int prod_line=0;

	if (priv->quirk & QUIRK_COMPACT_ONLY)
		buffer_len_lines &= ~0x1;

	buffer_pre_lines=buffer_len_lines/2;
	if (priv->quirk & QUIRK_COMPACT_ONLY)
		buffer_pre_lines &= ~0x1;

	out=0;
	in_gain=0;
	for(line=0;line<priv->format_in.height;line++){
		int last_col;
		int last_line;
		int min_line;
		int max_line;
		int min_col;
		int max_col;
		bool skip_line;
		uint16_t aux;

		//Fill initial location
		in_dist =line*xform_width;
		if (do_hflip)
			in_dist += xform_width - 1;

		//Borders
		min_line=line-(buffer_len_lines-buffer_pre_lines);
		if (priv->quirk & QUIRK_COMPACT_ONLY)
			min_line++; //Temp hack for ticket #122
		if (min_line<0)
			min_line=0;

		max_line=line+buffer_pre_lines-2;//-2 because you need an extra pixel
		if (priv->quirk & QUIRK_COMPACT_ONLY)
			max_line--; //Temp hack for ticket #122
		max_line=clamp(max_line,0,(int)(priv->format_in.height-2));
		min_col=0;
		max_col=xform_width-1;

		//Init line
		last_line=line;
		if (do_hflip)
			last_col = max_col;
		else
			last_col=0;
		skip_line=false;
		if (do_dist){
			int offset =0;
			for (col=0;col<xform_width;col++){
				if (do_hflip)
					offset = in_dist - col;
				else
					offset = in_dist + col;
				if ((dist_image[offset].line >=0) && (dist_image[offset].col >=0))
					break;
			}
			if (col==xform_width)
				skip_line=true;
			else {
				last_line=dist_image[offset].line;
				last_col=dist_image[offset].col;

			}
		}

		last_line=clamp(last_line,min_line,max_line);
		last_col=clamp(last_col,min_col,max_col);

		iowrite16(last_line-line+(buffer_len_lines-buffer_pre_lines),
				&pix_ini[out].start_line);
		aux = last_col;
		if (skip_line)
			aux |= SKIP_LINE;
		else
			if (++prod_line == priv->format_out.height)
				aux |= LAST_LINE;
		iowrite16(aux,&pix_ini[out].start_col);

		out++;
		for(col=0;col<xform_width;col++){
			//Fill pixel
			int next_line, next_col;
			int diff_line, diff_col;
			bool is_skipped= do_dist &&
				((dist_image[in_dist].line <0) ||(dist_image[in_dist].col <0));
			uint8_t aux;

			//Cx and Xy
			if ( do_dist && !is_skipped ) {
				iowrite8(dist_image[in_dist].col_res>>8,&pix[out].cx);
				iowrite8(dist_image[in_dist].line_res>>8,&pix[out].cy);
				if (dist_image[in_dist].line<last_line)
					iowrite8(0,&pix[out].cy);
				if (dist_image[in_dist].line>last_line)
					iowrite8(0xff,&pix[out].cy);
				if (dist_image[in_dist].col<last_col)
					iowrite8(0,&pix[out].cx);
				if (dist_image[in_dist].col>last_col)
					iowrite8(0xff,&pix[out].cx);
			}
			else{
				iowrite8(0,&pix[out].cx);
				iowrite8(0,&pix[out].cy);
			}

			//Gain
			if (do_gain && !is_skipped && !skip_line)
				iowrite8(gain_image[in_gain++],&pix[out].gain);
			else
				iowrite8(0xff,&pix[out].gain);

			//Jump
			if (!do_dist || skip_line){
				next_line=line;
				if (do_hflip)
					next_col= xform_width-col-2;
				else
					next_col=col+1;
			}
			else {
				int next;
				int offset = in_dist;
				for (next=1;(col+next)<xform_width;next++){
					if (do_hflip)
						offset = in_dist - next;
					else
						offset = in_dist + next;
					if ((dist_image[offset].line>=0) &&
						(dist_image[offset].col>=0))
						break;
				}
				next_line=dist_image[offset].line;
				next_col=dist_image[offset].col;
			}

			next_line=clamp(next_line,min_line,max_line);
			next_col=clamp(next_col,min_col,max_col);


			diff_line=next_line-last_line;
			diff_col=next_col-last_col;

			diff_line=clamp(diff_line,MIN_DIFF,MAX_DIFF);
			diff_col=clamp(diff_col,MIN_DIFF,MAX_DIFF);
			last_col+=diff_col;
			last_line+=diff_line;

			aux=diff_line&MASK_DIFF;
			aux|=(diff_col&MASK_DIFF)<<COL_DIFF;
			if (is_skipped && !skip_line)
				aux|=BIT(SKIP_COL);

			iowrite8(aux,&pix[out].move);

			if (do_hflip)
				in_dist--;
			else
				in_dist++;
			out++;
		}
	}

	return 0;
}

static int qtec_xform_fill(struct qtec_xform *priv){
	uint32_t param_len;
	uint32_t new_ofsset;
	int xform_width=qtec_xform_width(priv);
	int do_gain = qtec_xform_do(priv, false);
	int do_dist = qtec_xform_do(priv, true);

	if (!do_gain && !do_dist)
		return -1;

	if (do_dist &&	!qtec_xform_can_run(priv,true))
		return -EINVAL;

	if (do_gain &&	!qtec_xform_can_run(priv,false))
		return -EINVAL;

	if (priv->quirk & QUIRK_FB4)
		param_len=xform_width*priv->format_in.height*
			((priv->format_in.code == MEDIA_BUS_FMT_QTEC_FB4_RGBX)?4:1);
	else
		param_len=(xform_width+1)*priv->format_in.height
			*sizeof(struct qtec_xform_dist_pixel);

	if (param_len>priv->dist_mem_len)
		return -ENOMEM;

	//Find new offset
	new_ofsset=priv->dist_mem_last_offset+param_len;
	if ((new_ofsset+param_len)>priv->dist_mem_len)
		new_ofsset=0;

	if (priv->quirk & QUIRK_FB4)
		qtec_xform_convert_user_files_fb4(priv,priv->dist_mem+new_ofsset,param_len);
	else
		qtec_xform_convert_user_files_legacy(priv,priv->dist_mem+new_ofsset);

	if (do_gain)
		qtec_xform_set_extra_gain(priv,v4l2_ctrl_g_ctrl(priv->extra_gain));
	else
		qtec_xform_set_extra_gain(priv,1); // 0xff+1 = no gain

	if (priv->quirk & QUIRK_FB4) {
		if (qtec_xform_program_dma(priv,new_ofsset,param_len))
			return -ENOMEM;
	} else
		qtec_xform_write(priv,PARAM_BASEADDR,priv->dist_mem_base_addr+new_ofsset);
	priv->dist_mem_last_offset=new_ofsset;

	return 0;
}

static int qtec_xform_stop_fb4(struct qtec_xform *priv){
	uint32_t aux;
	unsigned long expiration;

	//Disable IRQs
	qtec_xform_write(priv,CTRL_FB4,BIT(ENABLE_FB4));

	//Stop DMA
	qtec_xform_dma_write(priv,DMACR,0);

	//Reset DMA
	if (priv->independent_dma){
		dev_err(&priv->pdev->dev, "Restaring DMA");
		qtec_xform_dma_write(priv,DMACR,BIT(RESET));
		qtec_xform_dma_write(priv,DMACR,0);
		while(1){
			qtec_xform_dma_read(priv,DMACR,&aux);
			if (!(aux & BIT(RESET)))
				break;
		}
	}

	//Needed to make sure the dma is stopped
	//for some reason, the not halt bit is never
	//cleared on the status register. Finn will look
	//into it.
	msleep(100);

	qtec_xform_write(priv,CTRL_FB4,0);
	expiration=jiffies+2*HZ;
	do {
		qtec_xform_read(priv,CTRL_FB4,&aux);
		if ((aux & BIT(ENABLE_ST_FB4))==0)
			return 0;
	}while(time_before(expiration,jiffies)==0);

	return -ETIMEDOUT;
}

static int qtec_xform_stop_legacy(struct qtec_xform *priv){
	uint32_t aux;
	unsigned long expiration;

	qtec_xform_read(priv,CTRL,&aux);
	aux&=~BIT(RUN);
	aux&=~BIT(IRQENA);
	aux|=BIT(IRQ_STATUS);
	qtec_xform_write(priv,CTRL,aux);

	expiration=jiffies+2*HZ;
	do {
		qtec_xform_read(priv,CTRL,&aux);
		if ((aux & BIT(STAT_RUN))==0)
			return 0;
	}while(time_before(expiration,jiffies)==0);

	return -ETIMEDOUT;

	return 0;
}

static int qtec_xform_stop(struct qtec_xform *priv){
	int ret;

	if (priv->xform_is_streaming == false)
		return 0;

	priv->xform_is_streaming = false;

	if (priv->quirk & QUIRK_FB4)
		ret = qtec_xform_stop_fb4(priv);
	else
		ret = qtec_xform_stop_legacy(priv);

	if (ret)
		dev_err(&priv->pdev->dev, "Timeout while stopping\n");

	return 0;
}


static int qtec_xform_hblank(struct qtec_xform *priv){
	struct v4l2_ctrl *ctrl;
	int hblank;

	ctrl=v4l2_ctrl_find(priv->sd.v4l2_dev->ctrl_handler,V4L2_CID_HBLANK);
	if (ctrl)
		hblank=v4l2_ctrl_g_ctrl(ctrl);
	else
		return 1;

	if (qtec_xform_is_compact(priv->format_in.code) &&
		!(priv->quirk & QUIRK_COMPACT_ONLY))
		hblank/=5;

	return hblank;
}

static int qtec_xform_start_legacy(struct qtec_xform *priv){
	uint32_t aux;
	uint32_t pre_size;
	int hblank;
	int vblank;
	int ret;
	struct v4l2_ctrl *ctrl;
	int xform_width;

	xform_width=qtec_xform_width(priv);

	hblank=qtec_xform_hblank(priv);

	ctrl=v4l2_ctrl_find(priv->sd.v4l2_dev->ctrl_handler,V4L2_CID_VBLANK);
	if (ctrl)
		vblank=v4l2_ctrl_g_ctrl(ctrl);
	else
		vblank=1;

	//param_len
	qtec_xform_write(priv,PARAM_LEN,
			(xform_width+1)*priv->format_in.height
			*sizeof(struct qtec_xform_dist_pixel));

	//sizes
	aux=xform_width<<ACTIVE;
	aux|=(xform_width+hblank)<<TOTAL;
	qtec_xform_write(priv,H_PARAMS,aux);

	aux=priv->format_in.height<<ACTIVE;
	aux|=(priv->format_in.height+vblank)<<TOTAL;
	qtec_xform_write(priv,V_PARAMS,aux);

	//buff_params
	if (priv->quirk & QUIRK_COMPACT_ONLY) {
		/*From Finn
		 *
		BUFFER_SIZE is now the size of each of the even and odd buffers.

		BUFFER_SIZE = 184 * 1024 / 2 = 94208 pixels

		BUFFER_LINES = int(BUFFER_SIZE/IMG_WIDTH)
		PREFILL_LINES = int(BUFFER_LINES/2)

		REG_BUF_LIMIT = BUFFER_SIZE - (BUFFER_LINES-PREFILL_LINES)*IMG_WIDTH // this is the bit field [17:1] of reg_buf_limit register at offset 0x10
		REG_BUF_LINE = 0 // this it bit-0 of reg_buf_limit register at offset 0x10
		 *
		 **/
		int buf_size=priv->buffer_size_pixels/2;
		int buf_lines;
		int pre_lines;

		buf_lines=buf_size/xform_width;
		pre_lines=buf_lines/2;

		pre_size = buf_size;
		pre_size -= (buf_lines-pre_lines)*xform_width;
		aux=pre_size<<1;
	}
	else{
		qtec_xform_read(priv,BUFF_PARAMS,&aux);
		aux&=~ (((1<<PRE_SIZE_LEN)-1)<<PRE_SIZE);
		pre_size=priv->buffer_size_pixels/xform_width;
		pre_size/=2;
		pre_size*=xform_width;
		pre_size+=priv->buffer_size_pixels%xform_width;
		aux |= pre_size << PRE_SIZE;
	}
	qtec_xform_write(priv,BUFF_PARAMS,aux);

	if (priv->quirk & QUIRK_FIFO_MONITOR)
		qtec_xform_write(priv, FIFO, 0);

	//run
	ret=qtec_xform_fill(priv);
	if (ret)
		return ret;

	aux = BIT(RUN);
	aux |= BIT(IRQENA);
	qtec_xform_write(priv,CTRL,aux);

	return 0;
}

static int qtec_xform_start_fb4(struct qtec_xform *priv){
	int ret;

	ret=qtec_xform_fill(priv);
	if (ret)
		return ret;
	qtec_xform_write(priv,CTRL_FB4,BIT(IRQ_URUN_ENA_FB4) | BIT(ENABLE_FB4));

	return 0;
}


static int qtec_xform_start(struct qtec_xform *priv){
	int ret;
	int do_gain=v4l2_ctrl_g_ctrl(priv->gain_enable) && !(priv->gain_enable->flags&V4L2_CTRL_FLAG_INACTIVE);
	int do_dist=v4l2_ctrl_g_ctrl(priv->dist_enable)  && !(priv->dist_enable->flags&V4L2_CTRL_FLAG_INACTIVE);

	if (!do_dist && !do_gain)
		return 0;

	if (priv->quirk & QUIRK_FB4)
		ret = qtec_xform_start_fb4(priv);
	else
		ret = qtec_xform_start_legacy(priv);

	if (ret)
		return ret;

	priv->xform_is_streaming = true;

	return 0;
}

/*Subdev*/
static int qtec_xform_get_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct qtec_xform *priv = container_of(subdev, struct qtec_xform, sd);

	if (format->pad)
		return -EINVAL;

	format->format = priv->format_out;

	return 0;
}

static void qtec_xform_update_enable(struct qtec_xform *priv){
	if (qtec_xform_can_run(priv,true))
		priv->dist_enable->flags&=~V4L2_CTRL_FLAG_INACTIVE;
	else{
		priv->dist_enable->val = priv->dist_enable->cur.val = 0;
		priv->dist_enable->flags|=V4L2_CTRL_FLAG_INACTIVE;
	}

	priv->format_out = (qtec_xform_do(priv,true))?priv->format_dist[RESIZED]:priv->format_in;

	if (qtec_xform_can_run(priv,false))
		priv->gain_enable->flags&=~V4L2_CTRL_FLAG_INACTIVE;
	else{
		priv->gain_enable->val = priv->gain_enable->cur.val = 0;
		priv->gain_enable->flags|=V4L2_CTRL_FLAG_INACTIVE;
	}

}

static int qtec_xform_set_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct qtec_xform *priv = container_of(subdev, struct qtec_xform, sd);
	struct v4l2_mbus_framefmt new_fmt;

	if (format->pad)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY){
		if (v4l2_ctrl_g_ctrl(priv->dist_enable) && !memcmp(&priv->format_in,fmt,sizeof(*fmt)))
			*fmt = priv->format_out;

		if (cfg)
			cfg->try_fmt = *fmt;
		return 0;
	}

	memset(&new_fmt,0,sizeof(new_fmt));
	new_fmt.width=fmt->width;
	new_fmt.height=fmt->height;
	new_fmt.code=fmt->code;
	new_fmt.field=fmt->field;

	priv->format_in = new_fmt;

	qtec_xform_update_enable(priv);

	return 0;
}

static int qtec_xform_s_stream(struct v4l2_subdev *subdev, int enable){
	struct qtec_xform *priv = container_of(subdev, struct qtec_xform, sd);

	if (enable){
		priv->fg_is_streaming=true;
		return qtec_xform_start(priv);
	}
	priv->fg_is_streaming=false;
	return qtec_xform_stop(priv);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_xform_s_register(struct v4l2_subdev *subdev,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_xform *priv = container_of(subdev, struct qtec_xform, sd);
	uint32_t val=reg->val;

	if (reg->reg<0x10000000)
		qtec_xform_write(priv, reg->reg, val);
	else
		iowrite32(val,priv->dist_mem+reg->reg-0x10000000);

	return 0;
}

static int qtec_xform_g_register(struct v4l2_subdev *subdev,
			      struct v4l2_dbg_register *reg)
{
	struct qtec_xform *priv = container_of(subdev, struct qtec_xform, sd);
	uint32_t val;

	if (reg->reg<0x10000000)
		qtec_xform_read(priv, reg->reg, &val);
	else
		val=ioread32(priv->dist_mem+reg->reg-0x10000000);

	reg->val=val;
	reg->size=4;
	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_xform_pad_ops = {
	.set_fmt = qtec_xform_set_fmt,
	.get_fmt = qtec_xform_get_fmt,
};

static const struct v4l2_subdev_video_ops qtec_xform_video_ops = {
	.s_stream = qtec_xform_s_stream,
};

static const struct v4l2_subdev_core_ops qtec_xform_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_xform_g_register,
	.s_register = qtec_xform_s_register,
	#endif
};

static const struct v4l2_subdev_ops qtec_xform_ops = {
	.core = &qtec_xform_core_ops,
	.video = &qtec_xform_video_ops,
	.pad = &qtec_xform_pad_ops,
};

/*Controls*/

static void qtec_xform_checkrun(struct work_struct *work){
	struct qtec_xform *priv=container_of(work, struct qtec_xform , work);

	if (qtec_xform_do(priv,true) || qtec_xform_do(priv,false))
		qtec_xform_start(priv);
	else
		qtec_xform_stop(priv);

	return;
}

static int qtec_xform_s_ctrl_aux(struct v4l2_ctrl *ctrl)
{
	struct qtec_xform *priv = container_of(ctrl->handler, struct qtec_xform, ctrl_handler);

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	switch (ctrl->id){
		case QTEC_XFORM_CID_ENABLE_GAIN_MAP:
			if (!qtec_xform_can_run(priv,false) && ctrl->val)
				return -EINVAL;
			qtec_xform_update_enable(priv);
			if (!priv->fg_is_streaming)
				break;
			schedule_work(&priv->work);
			break;
		case QTEC_XFORM_CID_ENABLE_DIST_MAP:
			if (!qtec_xform_can_run(priv,true) && ctrl->val)
				return -EINVAL;
			qtec_xform_update_enable(priv);
			if (!priv->fg_is_streaming)
				break;
			if (memcmp(&priv->format_dist[RESIZED],&priv->format_in,sizeof(priv->format_in))){
				dev_err(&priv->pdev->dev, "Cannot change size on the fly fg:%dx%d -> xform:%dx%d.\n",priv->format_in.width, priv->format_in.height, priv->format_out.width,priv->format_out.height);
				return -EINVAL;
			}
			schedule_work(&priv->work);
			break;
		case QTEC_XFORM_CID_EXTRA_GAIN:
			if (!priv->xform_is_streaming)
				return 0;
			if (priv->gain_enable->val && !(priv->gain_enable->flags&V4L2_CTRL_FLAG_INACTIVE))
				qtec_xform_set_extra_gain(priv,ctrl->val);
				//Cannot use g_ctrl because the lock is held
			break;
		case QTEC_XFORM_CID_HFLIP:
			if (!priv->xform_is_streaming)
				return 0;
			schedule_work(&priv->work);
			break;
		case QTEC_XFORM_CID_GAIN:
			qtec_xform_write(priv,GAIN_SCALE,ctrl->val/4);
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int qtec_xform_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	uint32_t aux;
	struct qtec_xform *priv = container_of(ctrl->handler, struct qtec_xform, ctrl_handler);

	switch (ctrl->id) {
		case QTEC_XFORM_CID_MIN_FIFO_LEVEL:
			qtec_xform_read(priv,FIFO,&aux);
			aux >>= FIFO_LEVEL;
			aux &= FIFO_MASK;
			ctrl->val =  aux;
			return 0;
	}

	return -EINVAL;
}

static int qtec_xform_s_ctrl(struct v4l2_ctrl *ctrl){
	int ret;

	ret=qtec_xform_s_ctrl_aux(ctrl);

	if (ret)
		ctrl->val=ctrl->cur.val;

	return ret;
}

static const struct v4l2_ctrl_ops qtec_xform_ctrl_ops = {
	.s_ctrl = qtec_xform_s_ctrl,
};

static const struct v4l2_ctrl_ops qtec_xform_ctrl_ops_volatile = {
	.g_volatile_ctrl = qtec_xform_g_volatile_ctrl,
};

/*VB2*/
static int qtec_xform_vb2_queue_setup_input(struct vb2_queue *vq,
		unsigned int *nbufs,
		unsigned int *num_planes, unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct qtec_xform *priv = vb2_get_drv_priv(vq);
	bool gain_dev = (vq==&priv->vb2_vidq_gain_rb);

	if (!gain_dev && (priv->quirk & QUIRK_GAIN_ONLY))
		return -EINVAL;

	*num_planes = 1;
	if (gain_dev)
		sizes[0] =  priv->format_gain[BUFFER].width * priv->format_gain[BUFFER].height;
	else
		sizes[0] =  priv->format_dist[BUFFER].width * priv->format_dist[BUFFER].height * sizeof(struct qtec_distortion);

	return sizes[0] ? 0: EINVAL;
}

static int qtec_xform_vb2_queue_setup_output(struct vb2_queue *vq,
		unsigned int *nbufs,
		unsigned int *num_planes, unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct qtec_xform *priv = vb2_get_drv_priv(vq);
	bool gain_dev = (vq==&priv->vb2_vidq_gain);
	int xform_width=qtec_xform_width(priv);

	if (gain_dev){
		sizes[0] = priv->format_out.width/qtec_xform_width_divider(priv);
		sizes[0] *= priv->format_out.height;

		if (sizes[0]!=priv->usr_gain_size){
			vfree(priv->usr_gain_mem);
			priv->usr_gain_mem=vmalloc(sizes[0]);
			if (!priv->usr_gain_mem){
				priv->usr_gain_size=0;
				return -EINVAL;
			}
			priv->usr_gain_size=sizes[0];
		}
		priv->format_gain[QUEUE] = priv->format_out;

	} else {
		unsigned long alloc_size;
		//No distortion
		if (priv->quirk & QUIRK_GAIN_ONLY)
			return -EINVAL;

		sizes[0] = xform_width;
		sizes[0] *= priv->format_in.height;
		sizes[0] *= sizeof(struct qtec_distortion);


		//1 extra line for the border
		alloc_size=sizes[0]+xform_width*sizeof(struct qtec_distortion);
		if (alloc_size!=priv->usr_dist_size){
			vfree(priv->usr_dist_mem);
			priv->usr_dist_mem=vmalloc(alloc_size);
			if (!priv->usr_dist_mem){
				priv->usr_dist_size=0;
				return -EINVAL;
			}
			priv->usr_dist_size=alloc_size;
			priv->usr_dist_size_real=sizes[0];
			priv->format_dist[QUEUE] = priv->format_in;
		}
	}

	*num_planes = 1;
	return 0;
}

static void qtec_xform_vb2_buf_queue_input(struct vb2_buffer *vb)
{
	struct qtec_xform *priv = vb2_get_drv_priv(vb->vb2_queue);
	void *buffer_data=vb2_plane_vaddr(vb,0);
	bool gain_dev = (vb->vb2_queue==&priv->vb2_vidq_gain_rb);
	unsigned long buf_len = gain_dev ? priv->usr_gain_size : priv->usr_dist_size;

	memcpy(buffer_data, gain_dev ? priv->usr_gain_mem : priv->usr_dist_mem, buf_len);

	vb2_set_plane_payload(vb, 0, buf_len);
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
}

static void qtec_xform_vb2_buf_queue_output(struct vb2_buffer *vb)
{
	struct qtec_xform *priv = vb2_get_drv_priv(vb->vb2_queue);
	void *buffer_data=vb2_plane_vaddr(vb,0);
	unsigned long size=vb2_get_plane_payload(vb,0);
	bool gain_dev = (vb->vb2_queue==&priv->vb2_vidq_gain);
	bool do_refill = true;

	if (gain_dev){
		memcpy(priv->usr_gain_mem,buffer_data,size);
		priv->format_gain[BUFFER]=priv->format_gain[QUEUE];
	}
	else{
		unsigned int width,height;

		//Check xform geometry
		if (qtec_xform_get_geometry(priv,buffer_data,&width,&height)){
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
			return;
		}

		if ((priv->xform_is_streaming) &&
				((width!=priv->format_out.width) || (height!=priv->format_out.height))){
			dev_err(&priv->pdev->dev, "Wrong geometry from the distortion file Expected:%dx%d -> file:%dx%d. File ignored\n",priv->format_out.width,priv->format_out.height,width,height);
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
			return;
		}

		priv->format_dist[RESIZED].width=width;
		priv->format_dist[RESIZED].height=height;
		priv->format_dist[RESIZED].code=priv->format_dist[QUEUE].code;
		priv->format_dist[RESIZED].field=priv->format_dist[QUEUE].field;

		memcpy(priv->usr_dist_mem,buffer_data,size);
		priv->format_dist[BUFFER]=priv->format_dist[QUEUE];
		if (priv->fg_is_streaming && (!priv->dist_enable->val) &&
				((width!=priv->format_in.width) || (height!=priv->format_in.height))){
			dev_err(&priv->pdev->dev, "Warning: Geometry cannot be changed online. Stream-off to enable distortion\n");
			do_refill = false;
		}
	}

	qtec_xform_update_enable(priv);

	if (!do_refill){
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		return;
	}

	if ((priv->quirk & QUIRK_FB4) && priv->xform_is_streaming){
		dev_err(&priv->pdev->dev, "FB4 Xform does not support online set of gain main. New map will be applied after pipeline restart \n");
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		return;
	}

	if (priv->xform_is_streaming)
		if (qtec_xform_fill(priv))
			qtec_xform_stop(priv);

	if (priv->fg_is_streaming)
		schedule_work(&priv->work);
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	return;
}

static void qtec_xform_stop_streaming(struct vb2_queue *vq)
{
	vb2_wait_for_all_buffers(vq);
}

static const struct vb2_ops qtec_xform_vb2_ops_input = {
	.queue_setup		= qtec_xform_vb2_queue_setup_input,
	.buf_queue		= qtec_xform_vb2_buf_queue_input,
	.stop_streaming		= qtec_xform_stop_streaming,
};

static const struct vb2_ops qtec_xform_vb2_ops_output = {
	.queue_setup		= qtec_xform_vb2_queue_setup_output,
	.buf_queue		= qtec_xform_vb2_buf_queue_output,
	.stop_streaming		= qtec_xform_stop_streaming,
};

/*
 * IOCTLS
 */
static int qtec_xform_vidioc_querycap(struct file *file, void *pr,
		struct v4l2_capability *cap)
{
	struct qtec_xform *priv=video_drvdata(file);
	struct video_device *vd= video_devdata(file);
	bool output_dev = (vd==&priv->vdev_gain || vd==&priv->vdev_dist);

	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, DRIVER_NAME);
	snprintf(cap->bus_info, sizeof(cap->bus_info),"platform:%s",dev_name(&priv->pdev->dev));
	if (output_dev)
		cap->device_caps = V4L2_CAP_VIDEO_OUTPUT |
			V4L2_CAP_READWRITE | V4L2_CAP_STREAMING ;
	else
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
			V4L2_CAP_READWRITE | V4L2_CAP_STREAMING ;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int qtec_xform_enum_output(struct file *file, void *priv,
				struct v4l2_output *out)
{
	if (out->index > 0)
		return -EINVAL;

	out->type = V4L2_OUTPUT_TYPE_MODULATOR; //Why not?
	sprintf(out->name, "Output %u", out->index);
	return 0;
}

static int qtec_xform_enum_input(struct file *file, void *priv,
				struct v4l2_input *in)
{
	if (in->index > 0)
		return -EINVAL;

	in->type = V4L2_INPUT_TYPE_CAMERA;
	sprintf(in->name, "Input %u", in->index);
	return 0;
}

static int qtec_xform_g_io(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int qtec_xform_s_io(struct file *file, void *priv, unsigned int i)
{
	if (i>0)
		return -EINVAL;
	return 0;
}

static int qtec_xform_enum_fmt_vid(struct file *file, void  *pr,
					struct v4l2_fmtdesc *f)
{
	struct qtec_xform *priv=video_drvdata(file);
	struct video_device *vd= video_devdata(file);
	bool gain_dev = (vd==&priv->vdev_gain);

	if (f->index>0)
		return -EINVAL;

	if (gain_dev){
		strlcpy(f->description, "8 bit 0.8 gain", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_GREY;
	}
	else{
		strlcpy(f->description, "32 bits Qtec distortion", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_QTEC_DISTORTION;
	}

	return 0;
}

static int qtec_xform_g_fmt_vid(struct file *file, void *pr,
			struct v4l2_format *f)
{
	struct qtec_xform *priv=video_drvdata(file);
	struct video_device *vd= video_devdata(file);
	bool gain_dev = (vd==&priv->vdev_gain || vd==&priv->vdev_gain_rb);
	bool output_dev = (vd==&priv->vdev_gain || vd==&priv->vdev_dist);

	memset(f,0,sizeof(*f));//Compliance test

	f->type = output_dev ? V4L2_BUF_TYPE_VIDEO_OUTPUT :  V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix.field = priv->format_in.field;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	if (gain_dev){
		if (output_dev){
			f->fmt.pix.width = priv->format_out.width/qtec_xform_width_divider(priv);
			f->fmt.pix.height = priv->format_out.height;
		} else {
			f->fmt.pix.width = priv->format_gain[BUFFER].width;
			f->fmt.pix.height = priv->format_gain[BUFFER].height;
		}
		f->fmt.pix.bytesperline =f->fmt.pix.width;
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
	} else {
		if (output_dev){
			f->fmt.pix.width = priv->format_in.width/qtec_xform_width_divider(priv);
			f->fmt.pix.height = priv->format_in.height;
		} else {
			f->fmt.pix.width = priv->format_dist[BUFFER].width;
			f->fmt.pix.height = priv->format_dist[BUFFER].height;
		}
		f->fmt.pix.bytesperline =f->fmt.pix.width*sizeof(struct qtec_distortion);
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_QTEC_DISTORTION;
	}

	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	f->fmt.pix.priv = 0;

	return f->fmt.pix.sizeimage ? 0 : EINVAL;
}

static const struct v4l2_ioctl_ops qtec_xform_ioctl_ops = {
	/*Driver specific*/
	.vidioc_querycap	= qtec_xform_vidioc_querycap,
	.vidioc_enum_fmt_vid_out  = qtec_xform_enum_fmt_vid,
	.vidioc_enum_fmt_vid_cap  = qtec_xform_enum_fmt_vid,
	.vidioc_try_fmt_vid_out   = qtec_xform_g_fmt_vid,
	.vidioc_g_fmt_vid_out     = qtec_xform_g_fmt_vid,
	.vidioc_s_fmt_vid_out     = qtec_xform_g_fmt_vid,
	.vidioc_try_fmt_vid_cap   = qtec_xform_g_fmt_vid,
	.vidioc_g_fmt_vid_cap     = qtec_xform_g_fmt_vid,
	.vidioc_s_fmt_vid_cap     = qtec_xform_g_fmt_vid,
	.vidioc_enum_output       = qtec_xform_enum_output,
	.vidioc_g_output          = qtec_xform_g_io,
	.vidioc_s_output          = qtec_xform_s_io,
	.vidioc_enum_input        = qtec_xform_enum_input,
	.vidioc_g_input           = qtec_xform_g_io,
	.vidioc_s_input           = qtec_xform_s_io,

	/*vb2*/
	.vidioc_reqbufs       = vb2_ioctl_reqbufs,
	.vidioc_create_bufs   = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
	.vidioc_querybuf      = vb2_ioctl_querybuf,
	.vidioc_qbuf          = vb2_ioctl_qbuf,
	.vidioc_dqbuf         = vb2_ioctl_dqbuf,
	.vidioc_streamon      = vb2_ioctl_streamon,
	.vidioc_streamoff     = vb2_ioctl_streamoff,
	.vidioc_expbuf        = vb2_ioctl_expbuf,

	/*v4l2*/
	.vidioc_log_status    = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * FOPS
 */

static int xform_fop_release(struct file *file)
{
	struct qtec_xform *priv=video_drvdata(file);
	struct video_device *vd= video_devdata(file);
	bool gain_dev = (vd==&priv->vdev_gain);
	unsigned long size;
	bool output_dev = (vd==&priv->vdev_gain || vd==&priv->vdev_dist);

	if (!output_dev)
		return vb2_fop_release(file);

	size = gain_dev ? priv->usr_gain_size : priv->usr_dist_size_real;

	if ( size && file->f_pos && (file->f_pos % size))
		dev_err(&priv->pdev->dev, "Invalid size of xform file: %lld instead of %ld\n", file->f_pos % size, size);

	return vb2_fop_release(file);
}

static const struct v4l2_file_operations qtec_xform_v4l_fops = {
	.owner		= THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = xform_fop_release,
	.write          = vb2_fop_write,
	.read          = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static struct video_device qtec_xform_v4l_template = {
	.name = DRIVER_NAME,

	.fops = &qtec_xform_v4l_fops,
	.ioctl_ops = &qtec_xform_ioctl_ops,
	.release = video_device_release_empty,
	.vfl_dir = VFL_DIR_M2M,
};

static struct v4l2_ctrl *qtec_xform_add_custom_control_enabled(struct qtec_xform *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_xform_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	};
	ctrl.name=name,
	ctrl.id=id;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_xform_add_custom_control_gain(struct qtec_xform *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_xform_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = 0,
		.max = 255,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	};

	ctrl.id=id;
	ctrl.name=name;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_xform_add_custom_control_gain_scale(struct qtec_xform *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_xform_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 4,
		.min = 0,
		.max = 0xffff*4,
		.def = 0x1000*4,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	};

	ctrl.id=id;
	ctrl.name=name;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_xform_add_custom_control_buffer_size(struct qtec_xform *priv, char *name, int id, int val){
	static struct v4l2_ctrl_config ctrl = {
		.ops = NULL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 1,
		.max = 1024*1024,
	};

	ctrl.id=id;
	ctrl.def = val;
	ctrl.name = name;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}


static struct v4l2_ctrl *qtec_xform_add_custom_control_fifo_level(struct qtec_xform *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_xform_ctrl_ops_volatile,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.def = 1024,
		.max = 0xffff,
	};

	ctrl.id = id;
	ctrl.name = name;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

uint8_t qtec_xform_read_version(struct qtec_xform *priv){
	uint32_t aux;

	if (priv->quirk & QUIRK_FB4){
		qtec_xform_read(priv,CTRL_FB4,&aux);
		return (aux>>VERSION_FB4) & VERSION_MASK;
	} else {
		qtec_xform_read(priv,CTRL,&aux);
		return aux&VERSION_MASK;
	}
}

static struct of_device_id qtec_xform_of_match[] = {
	{ .compatible = "qtec,axi_fb_img_xform-1.03.a", .data = (void *)LEGACY_XFORM},
	{ .compatible = "qtec,axi_fb_img_xform-4.00.a", .data = (void *)FB4_XFORM},
	{ /* EOL */}
};

static int qtec_xform_init_vdev(struct qtec_xform *priv, struct vb2_queue * queue, struct video_device *vdev, struct mutex  *mutex, bool output_device){
	int ret;

	queue->type = output_device ? V4L2_BUF_TYPE_VIDEO_OUTPUT : V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->io_modes |= output_device ? VB2_WRITE : VB2_READ;
	queue->drv_priv = priv;
	queue->buf_struct_size = sizeof (struct qtec_xform_buf);
	queue->mem_ops = &vb2_vmalloc_memops;
	queue->ops = output_device? &qtec_xform_vb2_ops_output : &qtec_xform_vb2_ops_input;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(queue);
	if (ret){
		dev_err(&priv->pdev->dev, "Unable to init vb2 queue\n");
		return ret;
	}
	mutex_init(mutex);
	*vdev = qtec_xform_v4l_template;
	vdev->lock=mutex;
	vdev->v4l2_dev=&priv->v4l2_dev;
	vdev->queue=queue;
	video_set_drvdata(vdev,priv);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret){
		dev_err(&priv->pdev->dev, "Unable to register video device\n");
		vb2_queue_release(queue);
	}

	return ret;
}

/*
 * Probe/Remove
 */
static atomic_t qtec_xform_sub_instance = ATOMIC_INIT(0);
static atomic_t qtec_xform_instance = ATOMIC_INIT(0);
static int qtec_xform_probe(struct platform_device *pdev){
	struct qtec_xform *priv;
	struct resource res;
	int ret;
	struct device_node *node;
	uint32_t aux;
	uint8_t version;
	const struct of_device_id *of_id =
			of_match_device(qtec_xform_of_match, &pdev->dev);

	priv=(struct qtec_xform *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

	if (of_id->data != LEGACY_XFORM)
		priv->quirk |= QUIRK_FB4;

	/*
	 * Device tree parsing
	 */
	ret=of_address_to_resource(pdev->dev.of_node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma address\n");
		return ret;
	}

	priv->iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem)){
		dev_err(&pdev->dev, "Unable to ioremap packer\n");
		return PTR_ERR(priv->iomem);
	}

	node=of_parse_phandle(pdev->dev.of_node,"qtec,dist_mem",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get dist mem node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get dist mem address\n");
		return ret;
	}
	priv->dist_mem_len=resource_size(&res);

	priv->dist_mem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->dist_mem)){
		dev_err(&pdev->dev, "Unable to ioremap dist mem\n");
		return PTR_ERR(priv->dist_mem);
	}
	ret=of_property_read_u32(node,"reg-axi",&priv->dist_mem_base_addr);
	if (ret){
		dev_err(&pdev->dev, "Unable to get dist mem base address\n");
		return -EIO;
	}
	of_node_put(node);

	/*Version*/
	version = qtec_xform_read_version(priv);
	switch(version){
		case 0x40:
			priv->buffer_size_pixels = 1;
			priv->quirk |= QUIRK_GAIN_ONLY;
			break;
		case 0x22:
		case 0x21:
			qtec_xform_read(priv,CTRL,&aux);
			priv->buffer_size_pixels = (aux>>SIZE_KBYTES)&SIZE_KBYTES_MASK;
			priv->buffer_size_pixels *= 1024;
			/* Fall through */
		case 0x20:
			priv->quirk |= QUIRK_COMPACT_ONLY;
			break;
		case 0x15:
			priv->quirk |= QUIRK_GAIN_SCALE;
			/* Fall through */
		case 0x14:
			priv->quirk |= QUIRK_FIFO_MONITOR;
			/* Fall through */
		case 0x13:
			break;
		default:
			dev_err(&pdev->dev, "Unknown core version 0x%x\n",version);
			return -EIO;
	}

	/*IRQ handling*/
	ret=of_irq_to_resource(pdev->dev.of_node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get packer irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,
			(priv->quirk & QUIRK_FB4)?qtec_xform_irq_handler_fb4:qtec_xform_irq_handler_legacy,
			0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request xform irq\n");
		return ret;
	}

	/*FB4*/
	if (priv->quirk & QUIRK_FB4){

		/*dma*/
		node=of_parse_phandle(pdev->dev.of_node,"qtec,dma",0);
		if (!node){
			dev_err(&pdev->dev, "Unable to get dma phandle\n");
			return -EIO;
		}
		ret=of_address_to_resource(node,0,&res);
		if (ret){
			dev_err(&pdev->dev, "Unable to get dma address\n");
			return ret;
		}

		priv->dma_iomem  = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(priv->dma_iomem)){
			dev_err(&pdev->dev, "Unable to ioremap dma\n");
			return PTR_ERR(priv->dma_iomem);
		}

		ret=of_irq_to_resource(node,0,&res);
		if(!ret){
			dev_err(&pdev->dev, "Unable to get dma irq\n");
			return -EIO;
		}

		ret=devm_request_irq(&pdev->dev,res.start,qtec_xform_irq_handler_dma,0,DRIVER_NAME,priv);
		if (ret){
			dev_err(&pdev->dev, "Unable to request dma irq\n");
			return ret;
		}
		of_node_put(node);

		/*descriptors*/
		node=of_parse_phandle(pdev->dev.of_node,"qtec,dma_desc_mem",0);
		if (!node){
			dev_err(&pdev->dev, "Unable to get dma descriptor node\n");
			return -EIO;
		}

		ret=of_address_to_resource(node,0,&res);
		if (ret){
			dev_err(&pdev->dev, "Unable to get dma descriptor address\n");
			return ret;
		}

		priv->dma_desc_iomem  = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(priv->dma_desc_iomem)){
			dev_err(&pdev->dev, "Unable to ioremap descriptors\n");
			return PTR_ERR(priv->dma_desc_iomem);
		}

		priv->dma_n_desc=min_t(int,256,resource_size(&res)/sizeof(struct qtec_xform_axi_cdma_desc));
		ret=of_property_read_u32(node,"reg-axi",&priv->dma_desc_base_addr);
		if (ret){
			dev_err(&pdev->dev, "Unable to get dma descriptors base address\n");
			return -EIO;
		}
		of_node_put(node);

		priv->independent_dma = of_property_read_bool(pdev->dev.of_node,"qtec,independent_dma");
	}

	/*Buffer size*/
	if (!(priv->quirk & QUIRK_GAIN_ONLY)){
		ret=of_property_read_u32(pdev->dev.of_node,"qtec,buffer_size_pixels",
						&priv->buffer_size_pixels);
		if (ret){
			dev_err(&pdev->dev, "Unable to get buffer size pixel\n");
			return -EIO;
		}
	}

	/*initial values*/
	INIT_WORK(&priv->work, qtec_xform_checkrun);
	priv->fg_is_streaming = false;
	priv->xform_is_streaming = false;
	priv->dist_mem_last_offset=0;

	priv->format_in.width=1024;
	priv->format_in.height=768;
	priv->format_in.code=MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP;
	priv->format_in.field=V4L2_FIELD_NONE;
	priv->format_in.colorspace=V4L2_COLORSPACE_SRGB;
	priv->format_out = priv->format_in;
	//Host device shall set_fmt asap

	priv->usr_gain_size=0;
	priv->usr_dist_size=0;

	/*
	 * Subdev
	 */
	v4l2_subdev_init(&priv->sd, &qtec_xform_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_xform_sub_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	/*Controls*/
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 16); //16 is just a guess for the hash table

	priv->dist_enable=qtec_xform_add_custom_control_enabled(priv,"Distortion Map",QTEC_XFORM_CID_ENABLE_DIST_MAP);
	priv->gain_enable=qtec_xform_add_custom_control_enabled(priv,"Gain Map",QTEC_XFORM_CID_ENABLE_GAIN_MAP);
	priv->extra_gain=qtec_xform_add_custom_control_gain(priv,"Extra Gain for Gain Map",QTEC_XFORM_CID_EXTRA_GAIN);
	if (!(priv->quirk & QUIRK_GAIN_ONLY)){
		priv->dist_buffer_size=qtec_xform_add_custom_control_buffer_size(priv,"Distortion buffer size",QTEC_XFORM_CID_DIST_BUFFER_SIZE,priv->buffer_size_pixels);
		priv->xform_hflip=qtec_xform_add_custom_control_enabled(priv,"Xform HFLIP",QTEC_XFORM_CID_HFLIP);
	}
	if (priv->quirk & QUIRK_FIFO_MONITOR){
		qtec_xform_read(priv,FIFO,&aux);
		aux >>= FIFO_SIZE;
		aux &= FIFO_MASK;
		priv->fifo_size=qtec_xform_add_custom_control_buffer_size(priv,"FIFO size",QTEC_XFORM_CID_FIFO_SIZE,aux);
		priv->min_fifo_level=qtec_xform_add_custom_control_fifo_level(priv,"Minimum FIFO level",QTEC_XFORM_CID_MIN_FIFO_LEVEL);
	}
	if (priv->quirk & QUIRK_GAIN_SCALE)
		priv->gain_scale=qtec_xform_add_custom_control_gain_scale(priv,"Xform Gain Scale",QTEC_XFORM_CID_GAIN);
	if (priv->ctrl_handler.error) {
		video_unregister_device(&priv->vdev_dist);
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	priv->gain_enable->flags|=V4L2_CTRL_FLAG_INACTIVE;
	priv->dist_enable->flags|=V4L2_CTRL_FLAG_INACTIVE;
	//Init controls
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	/*v4l2 device*/
	v4l2_device_set_name(&priv->v4l2_dev, DRIVER_NAME, &qtec_xform_instance);
	ret=v4l2_device_register(&pdev->dev,&priv->v4l2_dev);
	if (ret){
		dev_err(&pdev->dev, "Unable to register v4l2 device\n");
		goto err_device_reg;
	}

	if (qtec_xform_init_vdev(priv, &priv->vb2_vidq_dist, &priv->vdev_dist, &priv->vdev_dist_mutex, true))
		goto err_vb2_que_init1;

	if (qtec_xform_init_vdev(priv, &priv->vb2_vidq_gain, &priv->vdev_gain, &priv->vdev_gain_mutex, true))
		goto err_vb2_que_init2;

	if (qtec_xform_init_vdev(priv, &priv->vb2_vidq_dist_rb, &priv->vdev_dist_rb, &priv->vdev_dist_rb_mutex, false))
		goto err_vb2_que_init3;

	if (qtec_xform_init_vdev(priv, &priv->vb2_vidq_gain_rb, &priv->vdev_gain_rb, &priv->vdev_gain_rb_mutex, false))
		goto err_vb2_que_init4;

	v4l2_info(&priv->sd, "qtec_xform version 0x%.2x size %d pixels V4L2 subdevice registered as %s\n",version, priv->buffer_size_pixels, priv->sd.name);
	return 0;

	err_vb2_que_init4:
	video_unregister_device(&priv->vdev_dist_rb);
	vb2_queue_release(&priv->vb2_vidq_dist_rb);
	err_vb2_que_init3:
	video_unregister_device(&priv->vdev_gain);
	vb2_queue_release(&priv->vb2_vidq_gain);
	err_vb2_que_init2:
	video_unregister_device(&priv->vdev_dist);
	vb2_queue_release(&priv->vb2_vidq_dist);
	err_vb2_que_init1:
	v4l2_device_unregister(&priv->v4l2_dev);
	err_device_reg:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_device_unregister_subdev(&priv->sd);
	return ret;

}

static int qtec_xform_remove(struct platform_device *pdev){
	struct qtec_xform *priv=platform_get_drvdata(pdev);

	if (priv->usr_gain_mem)
		vfree(priv->usr_gain_mem);
	if (priv->usr_dist_mem)
		vfree(priv->usr_dist_mem);

	video_unregister_device(&priv->vdev_gain_rb);
	vb2_queue_release(&priv->vb2_vidq_gain_rb);
	video_unregister_device(&priv->vdev_dist_rb);
	vb2_queue_release(&priv->vb2_vidq_dist_rb);
	video_unregister_device(&priv->vdev_gain);
	vb2_queue_release(&priv->vb2_vidq_gain);
	video_unregister_device(&priv->vdev_dist);
	vb2_queue_release(&priv->vb2_vidq_dist);
	v4l2_device_unregister(&priv->v4l2_dev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

MODULE_DEVICE_TABLE(of, qtec_xform_of_match);

static struct platform_driver qtec_xform_plat_driver = {
	.probe		= qtec_xform_probe,
	.remove		= qtec_xform_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_xform_of_match,
	},
};

module_platform_driver(qtec_xform_plat_driver);

MODULE_DESCRIPTION("Qtec xform v4l2 module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
