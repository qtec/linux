/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/log2.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include "qtec_ccd.h"

#define DRIVER_NAME "qtec_ccd"
#define DEF_FIVAL {1,24} //24fps

static const struct qtec_ccd_format {
	u32 mbus_format;
	int n_ccd;
	bool is_bayer_chip;
}ccd_formats[] = {
	//1ccd mono
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_MONO,
		.n_ccd = 1,
		.is_bayer_chip = false,
	},
	//1ccd bayer
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGB,
		.n_ccd = 1,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_BGGR,
		.n_ccd = 1,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_GRBG,
		.n_ccd = 1,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_GBRG,
		.n_ccd = 1,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGGB,
		.n_ccd = 1,
		.is_bayer_chip = true,
	},
	//3ccd
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGB,
		.n_ccd = 3,
		.is_bayer_chip = false,
	},
	//5ccd
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP,
		.n_ccd = 5,
		.is_bayer_chip = false,
	},

};

int qtec_ccd_write_timegen(struct qtec_ccd *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "timegen W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_timegen+offset);
	return 0;
}

int qtec_ccd_read_timegen(struct qtec_ccd *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_timegen+offset);
	dev_dbg(&priv->pdev->dev, "timegen R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

int qtec_ccd_write_fg(struct qtec_ccd *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "fg W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_fg+offset);
	return 0;
}

int qtec_ccd_read_fg(struct qtec_ccd *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_fg+offset);
	dev_dbg(&priv->pdev->dev, "fg R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

int qtec_ccd_write_bayer(struct qtec_ccd *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "bayer W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_bayer+offset);
	return 0;
}

int qtec_ccd_read_bayer(struct qtec_ccd *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_bayer+offset);
	dev_dbg(&priv->pdev->dev, "bayer R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

int qtec_ccd_write_trig(struct qtec_ccd *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "trig W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_trig+offset);
	return 0;
}

int qtec_ccd_read_trig(struct qtec_ccd *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_trig+offset);
	dev_dbg(&priv->pdev->dev, "trig R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

int qtec_ccd_write_ad(struct qtec_ccd *priv, int channel, uint16_t reg, uint32_t value){
	uint8_t buffer[5];
	struct spi_transfer transfer ={
		.tx_buf = buffer,
		.rx_buf = NULL,
		.bits_per_word=8,
		.len=5,
	};
	struct spi_message message;

	dev_dbg(&priv->pdev->dev, "AD%d W:0x%.2x 0x%.8x\n",channel,reg,value);

	buffer[0] = reg&0xff;
	reg >>=8;
	buffer[1] = reg&0xf;
	buffer[1] |= (value&0xf)<<4;
	value>>=4;
	buffer[2] = value&0xff;
	value>>=8;
	buffer[3] = value&0xff;
	value>>=8;
	buffer[4] = value&0xff;

	spi_message_init(&message);
	spi_message_add_tail(&transfer,&message);

	return spi_sync(priv->ccd_spi[channel],&message);
}

#define STAT 0
#define IRQ_STATUS 2
static void qtec_ccd_fg_error(struct work_struct *work);
static irqreturn_t qtec_ccd_irq_handler(int irq, void *data){
	struct qtec_ccd *priv=data;
	uint32_t aux;

	qtec_ccd_read_fg(priv,STAT,&aux);

	if (unlikely((aux & BIT(IRQ_STATUS))==0)){
		v4l2_err(&priv->sd, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	qtec_ccd_fg_stop(priv);//If it is not stopped ASAP system is stalled

	v4l2_err(&priv->sd, "Framegen IRQ! Restarting sensor\n");
	schedule_work(&priv->fg_error_wk);

	return IRQ_HANDLED;
}

static int v4l2_ctrl_modify_range_cond(struct v4l2_ctrl *ctrl,
                            s64 min, s64 max, u64 step, s64 def)
{
       if ((ctrl->minimum == min) &&   (ctrl->maximum == max) &&
               (ctrl->step == step) && (ctrl->default_value == def))
               return 0;

       return v4l2_ctrl_modify_range(ctrl,min,max,step,def);
}

static int qtec_ccd_update_exposure_range(struct qtec_ccd *priv){
	uint64_t min;
	uint64_t max;
	uint32_t def;
	int i;

	def=priv->exp_time[0]->default_value;

	if (priv->trig_mode->val == SELF_TIMED){
		min=(uint64_t)HTOTAL * (uint64_t)1000000;
		do_div(min,priv->pixel_clk);
		max= (int64_t) priv->fival.numerator * (int64_t)1000000;
		do_div(max,priv->fival.denominator);
		max-=min; //-1 ext time
	}
	else{
		min=(uint64_t)0x100 * (uint64_t)1000000;
		do_div(min,priv->bus_clk);
		max= (uint64_t) 0xffffff00 * (uint64_t)1000000;
		do_div(max,priv->bus_clk);
	}

	def=clamp(def,(uint32_t) min, (uint32_t) max);

	for (i=0;i<priv->n_ccd;i++)
		v4l2_ctrl_modify_range_cond(priv->exp_time[i],min,max,1,def);

	return 0;
}

static int qtec_ccd_find_eye(struct qtec_ccd *priv){
	int ret;
	qtec_ccd_tg_start(priv);
	qtec_ccd_ad_start(priv);
	ret=qtec_ccd_fg_calibrate_doutphase(priv);
	qtec_ccd_ad_stop(priv);
	qtec_ccd_tg_stop(priv);

	return ret;
}

static int qtec_ccd_stop(struct qtec_ccd *priv){
	qtec_ccd_trig_stop(priv);
	qtec_ccd_fg_stop(priv);
	qtec_ccd_bayer_stop(priv);
	qtec_ccd_tg_stop(priv);
	qtec_ccd_ad_stop(priv);
	priv->streaming =false;
	return 0;
}

static int qtec_ccd_clamp_interval(struct qtec_ccd *priv,struct v4l2_fract *fival);
static int qtec_ccd_start(struct qtec_ccd *priv){

	qtec_ccd_clamp_interval(priv,&priv->fival);
	qtec_ccd_update_exposure_range(priv);

	qtec_ccd_tg_start(priv);
	qtec_ccd_bayer_start(priv);
	qtec_ccd_ad_start(priv);
	if (qtec_ccd_fg_sync(priv)){
		qtec_ccd_fg_stop(priv);
		qtec_ccd_bayer_stop(priv);
		qtec_ccd_tg_stop(priv);
		qtec_ccd_ad_start(priv);
		v4l2_err(&priv->sd, "Error syncing, check hardware\n");
		return -EIO;
	}
	qtec_ccd_ad_mode_normal(priv);
	if(qtec_ccd_fg_start(priv)){
		qtec_ccd_fg_stop(priv);
		qtec_ccd_bayer_stop(priv);
		qtec_ccd_tg_stop(priv);
		qtec_ccd_ad_start(priv);
		v4l2_err(&priv->sd, "Error syncing, check hardware\n");
		return -EIO;
	}

	priv->streaming =true;
	if ((priv->trig_mode->val != SELF_TIMED))
		qtec_ccd_tg_start(priv); //Set final exp_mode
	qtec_ccd_fg_stream(priv); //Start streaming to FG
	qtec_ccd_trig_start(priv);

	return 0;
}

static void qtec_ccd_fg_error(struct work_struct *work){
	struct qtec_ccd *priv=container_of(work, struct qtec_ccd , fg_error_wk);
	struct v4l2_ctrl *ctrl;


	ctrl=v4l2_ctrl_find(priv->sd.v4l2_dev->ctrl_handler,QTEC_VIDEO_CID_RESET_PIPELINE);
	if (!ctrl){
		v4l2_err(&priv->sd, "Unable to find reset pipeline ctrl\n");
		return;
	}

	v4l2_ctrl_s_ctrl(ctrl, 1);
	return;
}

static int qtec_ccd_s_stream(struct v4l2_subdev *subdev, int enable){
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	if (enable)
		return  qtec_ccd_start(priv);

	return qtec_ccd_stop(priv);
}

static int qtec_ccd_enum_mbus_code(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);
	int i,index=-1;

	if (code->pad)
		return -EINVAL;

	for (i=0;i<ARRAY_SIZE(ccd_formats);i++){
		if (priv->is_bayer_chip!=ccd_formats[i].is_bayer_chip)
			continue;
		if (priv->n_ccd!=ccd_formats[i].n_ccd)
			continue;
		index++;
		if (index==code->index){
			code->code=ccd_formats[i].mbus_format;
			return 0;
		}
	}

	return -EINVAL;
}

static int qtec_ccd_min_size(struct qtec_ccd *priv,u32 format,
		unsigned int *min_width, unsigned int *min_height){

	//At least 2x2 pixels on bayer formats
	switch(format){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
			*min_width=2;
			*min_height=2;
			break;
		default:
			*min_width=1;
			*min_height=1;
			break;
	}

	return 0;
}

static int qtec_ccd_min_borders(struct qtec_ccd *priv,u32 format,
		unsigned int *min_left, unsigned int *min_top){
	if ((format == MEDIA_BUS_FMT_QTEC_LEGACY_GRBG) ||
		(format == MEDIA_BUS_FMT_QTEC_LEGACY_BGGR))
		*min_left=1;
	else
		*min_left=0;

	if ((format == MEDIA_BUS_FMT_QTEC_LEGACY_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_LEGACY_GRBG))
		*min_top=1;
	else
		*min_top=0;

	return 0;
}

static int qtec_ccd_max_size(struct qtec_ccd *priv,u32 code,
		unsigned int *max_width, unsigned int *max_height){
	int min_left,min_top;

	qtec_ccd_min_borders(priv,code,&min_left,&min_top);
	*max_width=CCD_WIDTH-min_left;
	*max_height=CCD_HEIGHT-min_top;
	return 0;
}

#define MAX_SKIPS 4
static int qtec_ccd_update_readout(struct qtec_ccd *priv){
	struct {
		enum qtec_ccd_readout_mode mode;
		int len;
		bool hw_skip;
	} read[(MAX_CROP+2)*2];
	int n_readouts=0;
	int from;
	int safe_zone_head,safe_zone_tail;
	int i,j;

	if ((priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB)&& (priv->n_ccd ==1)){
		safe_zone_head = 3;
		safe_zone_tail = 1;
	}
	else{
		safe_zone_head = 2;
		safe_zone_tail= 0;
	}

	//pre read
	read[n_readouts].mode=NORMAL;
	read[n_readouts].len=VVSG+VDUMMY+VBLANK_HEAD+VBORDER_HEAD;
	n_readouts++;

	//normal reads
	for (i=0;i<priv->n_crop+1;i++){
		int last_line=(i==0)?0:priv->crop[i-1].top+priv->crop[i-1].height+safe_zone_tail;
		int top= (i==priv->n_crop)?CCD_HEIGHT: priv->crop[i].top;
		int height=(i==priv->n_crop)?(VBORDER_TAIL+VBLANK_TAIL):priv->crop[i].height;
		int skip_len=top-last_line-safe_zone_head;

		if (skip_len > LINE_SKIP){
			read[n_readouts].mode=SKIP;
			read[n_readouts].len=skip_len/LINE_SKIP;
			read[n_readouts].hw_skip=false;
			last_line += read[n_readouts].len*LINE_SKIP;
			n_readouts++;
		}
		read[n_readouts].mode=NORMAL;
		read[n_readouts].len=top+height-last_line;
		if (i!=priv->n_crop)
			read[n_readouts].len+=safe_zone_tail;
		n_readouts++;
	}

	//Pick 4 best skips
	for (i=0 ; i< MAX_SKIPS ;i++){
		int maxval = -1;
		int maxread = -1;
		for (j=0;j<n_readouts;j++)
			if ((read[j].mode == SKIP) && (read[j].len>maxval) && (read[j].hw_skip == false)){
				maxval = read[j].len;
				maxread = j;
			}
		if (maxread != -1)
			read[maxread].hw_skip = true;
	}
	for (i=0;i<n_readouts;i++)
		if ((read[i].mode ==SKIP) && (!read[i].hw_skip)){
			read[i].len*=LINE_SKIP;
			read[i].mode=NORMAL;
		}

	//flatten struct
	from=0;
	for (i=1;i<n_readouts;i++){
		if (read[from].mode == read[i].mode)
			read[from].len+=read[i].len;
		else{
			from++;
			read[from]=read[i];
		}
	}
	n_readouts=from+1;

	//Copy structure
	for (i=0;i<n_readouts;i++){
		priv->readout.read[i].mode=read[i].mode;
		priv->readout.read[i].len=read[i].len;
	}
	priv->readout.n=n_readouts;

	for(i=0;i<priv->readout.n;i++)
		dev_dbg(&priv->pdev->dev, "READOUT %d mode:%d len:%d\n",i,priv->readout.read[i].mode,priv->readout.read[i].len);

	qtec_ccd_clamp_interval(priv,&priv->fival);
	return 0;
}

static int qtec_ccd_crop_height(struct qtec_ccd *priv){
	int height=0,i;

	for (i=0;i<priv->n_crop;i++)
		if (priv->crop[i].height<1)
			return 0;
		else
			height+=priv->crop[i].height;

	return height;
}

static bool qtec_ccd_is_bayer(struct qtec_ccd *priv){
	switch(priv->format.code){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
			return true;
		default:
			return false;
	}
}

static int qtec_ccd_crop_sort_check(struct qtec_ccd *priv){
	struct v4l2_rect aux;
	int i,j;
	int interpol_separation=((priv->n_ccd==1) && ( priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB))?2:0;

	//bubble sort
	for (i=0;i<priv->n_crop-1;i++)
		for (j=i;j<priv->n_crop;j++)
			if (priv->crop[i].top>priv->crop[j].top){
				aux=priv->crop[i];
				priv->crop[i]=priv->crop[j];
				priv->crop[j]=aux;
			}

	for (i=1;i<priv->n_crop;i++)
		if ((priv->crop[i].top < (priv->crop[i-1].top+priv->crop[i-1].height+interpol_separation)))
			return -1; //selection overlap

	return 0;
}

static int qtec_ccd_update_crop(struct qtec_ccd *priv){
	int max_left,max_top,min_left,min_top;
	bool is_bayer=qtec_ccd_is_bayer(priv);
	int i;
	int crop_height;

	qtec_ccd_min_borders(priv,priv->format.code,&min_left,&min_top);

	//width
	if (priv->n_crop > MAX_CROP)
		priv->n_crop=1;
	for (i=0;i<priv->n_crop;i++)
		priv->crop[i].width=priv->format.width;
	max_left=CCD_WIDTH-priv->crop[0].width;
	priv->crop[0].left=clamp(priv->crop[0].left,min_left,max_left);
	if (((min_left&1)!=(priv->crop[0].left&1)) && is_bayer)
			priv->crop[0].left++;
	if (priv->crop[0].left>max_left)
		priv->crop[0].left-=2;
	for (i=1;i<priv->n_crop;i++)
		priv->crop[i].left=priv->crop[0].left;

	//height
	crop_height=qtec_ccd_crop_height(priv);
	max_top=CCD_HEIGHT-priv->format.height;
	if ((crop_height != priv->format.height) || qtec_ccd_crop_sort_check(priv))  {
		priv->crop[0].height=priv->format.height;
		priv->n_crop=1;
	}

	for (i=0;i<priv->n_crop;i++){
		priv->crop[i].top=clamp(priv->crop[i].top,min_top,max_top);
		if ((min_top&1)!=(priv->crop[i].top&1))
			priv->crop[i].top++;
		if (priv->crop[i].top>max_top)
			priv->crop[i].top-=2;
		max_top+=priv->crop[i].height;
		min_top=priv->crop[i].height+priv->crop[i].top;
	}

	/*for(i=0;i<priv->n_crop;i++)
		v4l2_err(&priv->sd,"CROP %d left:%d width:%d top:%d height:%d\n",i,priv->crop[i].left,priv->crop[i].width,priv->crop[i].top,priv->crop[i].height) ;*/
	qtec_ccd_update_readout(priv);

	return 0;
}

static int qtec_ccd_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *s){
	struct qtec_ccd *priv = container_of(sd, struct qtec_ccd, sd);
	int i;

	if (priv->streaming)
		return -EBUSY;

	if (s->target == V4L2_SEL_TGT_COMPOSE_ACTIVE)
		return 0;

	if (s->target != V4L2_SEL_TGT_CROP_ACTIVE)
		return -EINVAL;

	if (s->rectangles> MAX_CROP)
		return -EINVAL;

	if (s->rectangles){
		for (i=0;i<s->rectangles;i++)
			priv->crop[i]=s->pr[i].r;
		priv->n_crop=s->rectangles;
	}
	else{
		priv->crop[0]=s->r;
		priv->n_crop=1;
	}

	qtec_ccd_update_crop(priv);
	return 0;
}

static int qtec_ccd_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *s){
	struct qtec_ccd *priv = container_of(sd, struct qtec_ccd, sd);
	int32_t max_width,max_height;
	int min_left,min_top;
	int i;

	switch (s->target){
		case V4L2_SEL_TGT_CROP_ACTIVE:
		case V4L2_SEL_TGT_COMPOSE_ACTIVE:
			if (s->rectangles==0){
				if (priv->n_crop>1)
					return -ENOSPC;
				s->r=priv->crop[0];
			}
			else{
				if (priv->n_crop>s->rectangles)
					return -ENOSPC;
				for (i=0;i<priv->n_crop;i++)
					s->pr[i].r=priv->crop[i];
				s->rectangles=priv->n_crop;
			}

			return 0;
		case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		case V4L2_SEL_TGT_CROP_BOUNDS:
			if (qtec_ccd_max_size(priv,priv->format.code,&max_width,&max_height))
				return -EINVAL;
			qtec_ccd_min_borders(priv,priv->format.code,&min_left,&min_top);
			s->rectangles = 0;
			s->r.left = min_left;
			s->r.top = min_top;
			s->r.width = max_width+min_left;
			s->r.height = max_height+min_top;
			return 0;
		default:
			return -EINVAL;
	}

	return 0;
}

static int qtec_ccd_check_avail_fmt(struct qtec_ccd *priv,u32 format){
	int i;
	for (i=0;i<ARRAY_SIZE(ccd_formats);i++)
		if ((priv->n_ccd == ccd_formats[i].n_ccd) &&
			(priv->is_bayer_chip == ccd_formats[i].is_bayer_chip))
				if (format==ccd_formats[i].mbus_format)
					return 0;
	return -EINVAL;
}

static int qtec_ccd_try_fmt(struct v4l2_subdev *subdev,struct v4l2_mbus_framefmt *fmt){
	unsigned int max_width,max_height;
	unsigned int min_width,min_height;
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	if (qtec_ccd_check_avail_fmt(priv,fmt->code)){
		struct v4l2_subdev_mbus_code_enum code = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.index = 0,
		};

		qtec_ccd_enum_mbus_code(subdev,NULL,&code);
		fmt->code = code.code;
	}

	qtec_ccd_max_size(priv,fmt->code,&max_width,&max_height);
	qtec_ccd_min_size(priv,fmt->code,&min_width,&min_height);

	v4l_bound_align_image(&fmt->width, min_width, max_width, 0,
			      &fmt->height, min_height, max_height, ilog2(min_height), 0);
	fmt->colorspace=V4L2_COLORSPACE_SRGB;
	fmt->field=V4L2_FIELD_NONE;

	return 0;
}

static int qtec_ccd_get_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	if (format->pad)
		return -EINVAL;

	format->format=priv->format;

	return 0;
}

static int qtec_ccd_set_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	if (format->pad)
		return -EINVAL;

	qtec_ccd_try_fmt(subdev,fmt);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY){
		if (cfg)
			cfg->try_fmt = *fmt;
		return 0;
	}

	if (priv->streaming)
		return -EBUSY;

	priv->format=*fmt;
	qtec_ccd_update_crop(priv);

	return 0;
}

static int qtec_ccd_enum_fsize(struct v4l2_subdev *subdev, struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);
	unsigned int max_width,max_height;
	unsigned int min_width,min_height;

	if (fse->index!=0)
		return -EINVAL;

	if (qtec_ccd_max_size(priv,fse->code,&max_width,&max_height))
		return -EINVAL;

	if (qtec_ccd_min_size(priv,fse->code,&min_width,&min_height))
		return -EINVAL;

	fse->min_width=min_width;
	fse->max_width=max_width;
	fse->max_width-=max_width%min_width;
	fse->min_height=min_height;
	fse->max_height=max_height;

	return 0;
}

int qtec_ccd_readout_lines(struct qtec_ccd *priv){
	int lines=0,i;

	for (i=0;i<priv->readout.n;i++)
		lines+=priv->readout.read[i].len;

	return lines;
}

static int qtec_ccd_max_tpf(struct qtec_ccd *priv,
		struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){

	if (priv->trig_mode->val == SELF_TIMED){
		fival->numerator = HTOTAL*TIMEGEN_MAX_LINES;
		fival->denominator = priv->pixel_clk;
	}
	else{
		fival->numerator = 0xffffffff;
		fival->denominator = priv->pixel_clk;
	}
	return 0;
}

static int qtec_ccd_min_tpf(struct qtec_ccd *priv,
		struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){

	if (memcmp(&format,&priv->format,sizeof(format))==0)
		fival->numerator = HTOTAL*qtec_ccd_readout_lines(priv);
	else{
		//Optimistic estimation. All lines not read are skipped.
		int skip_lines=(CCD_HEIGHT-format.height)/LINE_SKIP;
		fival->numerator = HTOTAL * (VTOTAL_MAX -skip_lines*LINE_SKIP);
	}
	fival->denominator = priv->pixel_clk;
	return 0;
}

#define FRACT_CMP(a, OP, b)	\
	((u64)(a).numerator * (b).denominator  OP  (u64)(b).numerator * (a).denominator)
static int qtec_ccd_clamp_interval(struct qtec_ccd *priv,struct v4l2_fract *fival){
	struct v4l2_fract interval;
	int ret;

	ret=qtec_ccd_min_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,<,interval))
		*fival=interval;

	ret=qtec_ccd_max_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,>,interval))
		*fival=interval;

	return 0;

}

static int qtec_ccd_g_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	fival->pad=0;
	fival->interval=priv->fival;

	return 0;
}

static int qtec_ccd_s_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);


	fival->pad=0;
	if (fival->interval.denominator==0)
		fival->interval=(struct v4l2_fract) DEF_FIVAL;
	qtec_ccd_clamp_interval(priv,&fival->interval);

	priv->fival=fival->interval;

	qtec_ccd_update_exposure_range(priv);

	if (priv->streaming)
		qtec_ccd_tg_start(priv);

	return 0;
}

static int qtec_ccd_enum_frameintervals(struct v4l2_subdev *subdev, struct v4l2_frmivalenum *fival){
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);
	int ret;
	struct v4l2_mbus_framefmt format;

	if (fival->index!=0)
		return -EINVAL;

	if (qtec_ccd_check_avail_fmt(priv,fival->pixel_format)!=0)
		return -EINVAL;

	format.code=fival->pixel_format;
	format.width=fival->width;
	format.height=fival->height;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;

	ret= qtec_ccd_min_tpf(priv, format, &fival->stepwise.min);
	if (ret)
		return ret;
	ret= qtec_ccd_max_tpf(priv, format, &fival->stepwise.max);
	if (ret)
		return ret;

	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1; //compliance

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_ccd_s_register(struct v4l2_subdev *subdev,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);

	if (reg->reg >= 0x4000)
		qtec_ccd_write_bayer(priv,reg->reg-0x4000,reg->val);
	else
	if (reg->reg >= 0x3000)
		qtec_ccd_write_timegen(priv,reg->reg-0x3000,reg->val);
	else
	if (reg->reg >= 0x2000)
		qtec_ccd_write_trig(priv,reg->reg-0x2000,reg->val);
	else
	if (reg->reg >= 0x1000)
		qtec_ccd_write_fg(priv,reg->reg-0x1000,reg->val);
	else
		qtec_ccd_write_ad(priv,CHAN_ALL,reg->reg,reg->val);
	return 0;
}

static int qtec_ccd_g_register(struct v4l2_subdev *subdev,
			      struct v4l2_dbg_register *reg)
{
	struct qtec_ccd *priv = container_of(subdev, struct qtec_ccd, sd);
	uint32_t val=0;

	if (reg->reg >= 0x4000)
		qtec_ccd_read_bayer(priv,reg->reg-0x4000,&val);
	else
	if (reg->reg >= 0x3000)
		qtec_ccd_read_timegen(priv,reg->reg-0x3000,&val);
	else
	if (reg->reg >= 0x2000)
		qtec_ccd_read_trig(priv,reg->reg-0x2000,&val);
	else
	if (reg->reg >= 0x1000)
		qtec_ccd_read_fg(priv,reg->reg-0x1000,&val);

	reg->val=val;
	reg->size=4;

	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_ccd_pad_ops = {
	.get_selection = qtec_ccd_get_selection,
	.set_selection = qtec_ccd_set_selection,
	.enum_frame_size = qtec_ccd_enum_fsize,
	.set_fmt = qtec_ccd_set_fmt,
	.get_fmt = qtec_ccd_get_fmt,
	.enum_mbus_code = qtec_ccd_enum_mbus_code,
};

static const struct v4l2_subdev_video_ops qtec_ccd_video_ops = {
	.s_stream = qtec_ccd_s_stream,
	.g_frame_interval = qtec_ccd_g_frame_interval,
	.s_frame_interval = qtec_ccd_s_frame_interval,
	.enum_frameintervals = qtec_ccd_enum_frameintervals,
};

static const struct v4l2_subdev_core_ops qtec_ccd_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_ccd_g_register,
	.s_register = qtec_ccd_s_register,
	#endif
};
static const struct v4l2_subdev_ops qtec_ccd_ops = {
	.core = &qtec_ccd_core_ops,
	.video = &qtec_ccd_video_ops,
	.pad = &qtec_ccd_pad_ops,
};

static int qtec_ccd_init_fmt(struct qtec_ccd *priv){
	unsigned int max_width,max_height;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev_mbus_code_enum code = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.index = 0,
	};

	priv->n_crop=1;
	qtec_ccd_enum_mbus_code(&priv->sd,NULL,&code);
	format.format.code = code.code;
	qtec_ccd_max_size(priv,format.format.code,&max_width,&max_height);
	format.format.width=max_width;
	format.format.height=max_height;
	qtec_ccd_set_fmt(&priv->sd,NULL,&format);
	return 0;
}

static int qtec_ccd_s_ctrl_aux(struct v4l2_ctrl *ctrl)
{
	struct qtec_ccd *priv = container_of(ctrl->handler, struct qtec_ccd, ctrl_handler);

	switch (ctrl->id){
		case QTEC_CCD_CID_MODE_TRIG:
		case QTEC_CCD_CID_FLASH_POL:
		case QTEC_CCD_CID_TRIG_POL:
			if (priv->streaming)
				return -EBUSY;
	};

	switch (ctrl->id){
		case QTEC_CCD_CID_EXT_TRIG_DELAY:
		case QTEC_CCD_CID_MODE_TRIG:
		case QTEC_CCD_CID_FLASH_POL:
		case QTEC_CCD_CID_TRIG_POL:
		case QTEC_CCD_CID_MANUAL_TRIGGER:
			return qtec_ccd_trig_ctrl(priv,ctrl);

		case QTEC_CCD_CID_RED_EXPOSURE:
		case QTEC_CCD_CID_GREEN_EXPOSURE:
		case QTEC_CCD_CID_BLUE_EXPOSURE:
		case QTEC_CCD_CID_IR1_EXPOSURE:
		case QTEC_CCD_CID_IR2_EXPOSURE:
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			if (priv->trig_mode->val == SELF_TIMED)
				return qtec_ccd_tg_ctrl(priv,ctrl);
			else
				return qtec_ccd_trig_ctrl(priv,ctrl);

		case QTEC_CCD_CID_RED_CDS:
		case QTEC_CCD_CID_GREEN_CDS:
		case QTEC_CCD_CID_BLUE_CDS:
		case QTEC_CCD_CID_IR1_CDS:
		case QTEC_CCD_CID_IR2_CDS:
		case QTEC_CCD_CID_CDS:
		case QTEC_CCD_CID_RED_VGAGAIN:
		case QTEC_CCD_CID_GREEN_VGAGAIN:
		case QTEC_CCD_CID_BLUE_VGAGAIN:
		case QTEC_CCD_CID_IR1_VGAGAIN:
		case QTEC_CCD_CID_IR2_VGAGAIN:
		case V4L2_CID_GAIN:
			return qtec_ccd_ad_ctrl(priv,ctrl);
	}


	return 0;
}

static int qtec_ccd_s_ctrl(struct v4l2_ctrl *ctrl){
	int ret;

	ret=qtec_ccd_s_ctrl_aux(ctrl);

	if (ret)
		ctrl->val=ctrl->cur.val;

	return ret;
}

static int qtec_ccd_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	struct qtec_ccd *priv = container_of(ctrl->handler, struct qtec_ccd, ctrl_handler);

	switch (ctrl->id) {
		case V4L2_CID_HBLANK:
			ctrl->val=HTOTAL-priv->format.width;
			break;
		case V4L2_CID_VBLANK:
			ctrl->val=qtec_ccd_readout_lines(priv)-priv->format.height;
			break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops qtec_ccd_ctrl_ops_volatile = {
	.g_volatile_ctrl = qtec_ccd_g_volatile_ctrl,
};

static const struct v4l2_ctrl_ops qtec_ccd_ctrl_ops = {
	.s_ctrl = qtec_ccd_s_ctrl,
};

static int qtec_ccd_init_pols(struct qtec_ccd *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_ccd_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def=0,
	};
	ctrl.name="Invert Flash Polarity",
	ctrl.id=QTEC_CCD_CID_FLASH_POL;
	priv->flash_pol=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	ctrl.name="Invert Trigger Polarity",
	ctrl.id=QTEC_CCD_CID_TRIG_POL;
	priv->trig_pol=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);

	return 0;
}

static int qtec_ccd_init_ext_trig(struct qtec_ccd *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_ccd_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min =0,
		.def =0,
		.id =QTEC_CCD_CID_EXT_TRIG_DELAY,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.name = "External Trigger Delay"
	};
	uint64_t max;

	max= (uint64_t)0xffffffff*(uint64_t)1000000;
	do_div(max,priv->pixel_clk);

	ctrl.max=(uint32_t)max;
	priv->ext_trig_delay=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);

	return 0;
}

static int qtec_ccd_init_sensor_type(struct qtec_ccd *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_ccd_ctrl_ops,
		.type = V4L2_CTRL_TYPE_STRING,
		.step = 1,
		.min = 1,
		.max = 32,
		.id = QTEC_VIDEO_CID_SENSOR_TYPE,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Sensor Type",
	};

	priv->sensor_type=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	if (!priv->sensor_type)
		return -1;

	snprintf(priv->sensor_type->p_cur.p_char,32,"%dxICX204 %s",priv->n_ccd,(priv->is_bayer_chip)?"Bayer":"");

	return 0;
}

static int qtec_ccd_init_trig_mode(struct qtec_ccd *priv){
	static const char * const trig_mode_menu[] = {
		"Self Timed",
		"External Trigger",
		"External Exposure",
		NULL
	};
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_ccd_ctrl_ops,
		.type = V4L2_CTRL_TYPE_MENU,
		.qmenu = trig_mode_menu,
		.menu_skip_mask = 0x0,
		.min = 0,
		.max = 2,
		.def = SELF_TIMED,
		.name = "Trigger Mode",
		.id = QTEC_CCD_CID_MODE_TRIG,
	};

	priv->trig_mode=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	return 0;
}

static int qtec_ccd_init_manual_trig(struct qtec_ccd *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_ccd_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.id =QTEC_CCD_CID_MANUAL_TRIGGER,
		.name = "Manual Trigger"
	};

	priv->manual_trigger=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	return 0;
}

static int qtec_ccd_init_exposures(struct qtec_ccd *priv){
	int i;
	char *names[]={
		"Red Exposure Absolute",
		"Green Exposure Absolute",
		"Blue Exposure Absolute",
		"IR1 Exposure Absolute",
		"IR2 Exposure Absolute",
	};
	int ids[] = {
		QTEC_CCD_CID_RED_EXPOSURE,
		QTEC_CCD_CID_GREEN_EXPOSURE,
		QTEC_CCD_CID_BLUE_EXPOSURE,
		QTEC_CCD_CID_IR1_EXPOSURE,
		QTEC_CCD_CID_IR2_EXPOSURE,

	};

	if (priv->n_ccd == 1 ){
		priv->exp_time[0]=v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_ccd_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE, 0, 0xffffff, 1,20000);
		return 0;
	}

	for (i=0;i<priv->n_ccd;i++){
		static struct v4l2_ctrl_config ctrl = {
			.ops = &qtec_ccd_ctrl_ops,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.step = 1,
			.max = 0xffffff,
			.def = 20000,
			.flags = V4L2_CTRL_FLAG_SLIDER,
		};
		ctrl.name=names[i],
		ctrl.id=ids[i];
		priv->exp_time[i]=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	}

	return 0;
}

static int qtec_ccd_init_cds(struct qtec_ccd *priv){
	int i;
	char *names[]={
		"Red CDS",
		"Green CDS",
		"Blue CDS",
		"IR1 CDS",
		"IR2 CDS",
		"CDS",
	};
	int ids[] = {
		QTEC_CCD_CID_RED_CDS,
		QTEC_CCD_CID_GREEN_CDS,
		QTEC_CCD_CID_BLUE_CDS,
		QTEC_CCD_CID_IR1_CDS,
		QTEC_CCD_CID_IR2_CDS,
		QTEC_CCD_CID_CDS,
	};

	for (i=0;i<priv->n_ccd;i++){
		static struct v4l2_ctrl_config ctrl = {
			.ops = &qtec_ccd_ctrl_ops,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0,
			.max = 3,
			.step = 1,
			.def = 3,
			.flags = V4L2_CTRL_FLAG_SLIDER,
		};
		ctrl.name=(priv->n_ccd!=1)?names[i]:names[ARRAY_SIZE(names)-1],
		ctrl.id=(priv->n_ccd!=1)?ids[i]:ids[ARRAY_SIZE(names)-1];
		priv->cds[i]=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	}

	return 0;
}

static int qtec_ccd_init_vgagain(struct qtec_ccd *priv){
	int i;
	char *names[]={
		"Red VGA Gain",
		"Green VGA Gain",
		"Blue VGA Gain",
		"IR1 VGA Gain",
		"IR2 VGA Gain",
	};
	int ids[] = {
		QTEC_CCD_CID_RED_VGAGAIN,
		QTEC_CCD_CID_GREEN_VGAGAIN,
		QTEC_CCD_CID_BLUE_VGAGAIN,
		QTEC_CCD_CID_IR1_VGAGAIN,
		QTEC_CCD_CID_IR2_VGAGAIN,
	};

	if (priv->n_ccd==1)
		priv->vga_gain[0]=v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_ccd_ctrl_ops,
			V4L2_CID_GAIN, 0-CALIB_0DB,  36623-CALIB_0DB,1,0);
	else
		for (i=0;i<priv->n_ccd;i++){
		static struct v4l2_ctrl_config ctrl = {
			.ops = &qtec_ccd_ctrl_ops,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.min = 0-CALIB_0DB,
			.max = 36623-CALIB_0DB,
			.step = 1,
			.def = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER,
		};
		ctrl.name=names[i];
		ctrl.id=ids[i];
		priv->vga_gain[i]=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	}

	return 0;
}


static int of_dev_node_match(struct device *dev, void *data)
{
        return dev->of_node == data;
}

static struct spi_device *of_find_spi_device_by_node(struct device_node *node)
{
	struct device *dev;

	dev = bus_find_device(&spi_bus_type, NULL, node,
					 of_dev_node_match);

	return to_spi_device(dev);
}

static int qtec_ccd_probe_spi_devices(struct qtec_ccd *priv){
	char *spi_devices[]= {"qtec,ad_red","qtec,ad_green","qtec,ad_blue","qtec,ad_ir1","qtec,ad_ir2","qtec,ad_all"};
	struct device_node *node;
	int i;

	for(i=0;i<ARRAY_SIZE(spi_devices);i++){
		node=of_parse_phandle(priv->pdev->dev.of_node,spi_devices[i],0);
		if (!node){
			dev_err(&priv->pdev->dev, "Unable to find phandle %s\n",spi_devices[i]);
			return -EIO;
		}
		priv->ccd_spi[i]=of_find_spi_device_by_node(node);
		of_node_put(node);
		if (!priv->ccd_spi[i]){
			dev_err(&priv->pdev->dev, "Unable to find spi device %s\n",spi_devices[i]);
			return -EIO;
		}

	}
	return 0;
}

static void __iomem *qtec_ccd_probe_sub_devices(struct qtec_ccd *priv,char *name){
	struct device_node *node;
	void __iomem *mem;
	int ret;
	struct resource res;

	node=of_parse_phandle(priv->pdev->dev.of_node,name,0);
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&priv->pdev->dev, "Unable to get %s address\n",name);
		return NULL;
	}

	mem  = devm_ioremap_resource(&priv->pdev->dev, &res);
	of_node_put(node);
	if (IS_ERR(mem)){
		dev_err(&priv->pdev->dev, "Unable to ioremap %s memory\n",name);
		return NULL;
	}

	return mem;
}

static atomic_t qtec_ccd_instance = ATOMIC_INIT(0);
static int qtec_ccd_probe(struct platform_device *pdev){
	struct qtec_ccd *priv;
	struct resource res;
	int ret;
	uint32_t aux;

	priv=(struct qtec_ccd *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

	INIT_WORK(&priv->fg_error_wk, qtec_ccd_fg_error);

	//Dt parse
	ret=of_address_to_resource(pdev->dev.of_node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get address\n");
		return ret;
	}
	priv->iomem_fg  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem_fg)){
		dev_err(&pdev->dev, "Unable to ioremap memory\n");
		return PTR_ERR(priv->iomem_fg);
	}

	ret=of_irq_to_resource(pdev->dev.of_node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get packer irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,qtec_ccd_irq_handler,0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request fg irq\n");
		return ret;
	}

	priv->iomem_trig  = qtec_ccd_probe_sub_devices(priv,"qtec,trigger");
	if (!priv->iomem_trig)
		return -EIO;

	priv->iomem_timegen  = qtec_ccd_probe_sub_devices(priv,"qtec,timegen");
	if (!priv->iomem_timegen)
		return -EIO;

	priv->iomem_bayer  = qtec_ccd_probe_sub_devices(priv,"qtec,bayer");
	if (!priv->iomem_bayer)
		return -EIO;

	ret = qtec_ccd_probe_spi_devices(priv);
	if (ret)
		return -EIO;

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bayer_chip",&aux);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse is_bayer_chip property\n");
		return -EIO;
	}
	priv->is_bayer_chip=(aux)?true:false;

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,n_ccd",&aux);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse n_ccd property\n");
		return -EIO;
	}
	aux=clamp(aux,(uint32_t)1,(uint32_t)MAX_CCD);
	priv->n_ccd=aux;

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,pixel_clk",&priv->pixel_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse pixel_clk property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bus_clk",&priv->bus_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse bus_clk property\n");
		return -EIO;
	}

	//Subdev
	v4l2_subdev_init(&priv->sd, &qtec_ccd_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_ccd_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	//Controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 16); //16 is just a guess for the hash table
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	qtec_ccd_init_exposures(priv);
	qtec_ccd_init_trig_mode(priv);
	qtec_ccd_init_cds(priv);
	qtec_ccd_init_vgagain(priv);
	qtec_ccd_init_pols(priv);
	qtec_ccd_init_ext_trig(priv);
	qtec_ccd_init_manual_trig(priv);
	qtec_ccd_init_sensor_type(priv);
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_ccd_ctrl_ops_volatile,
			V4L2_CID_HBLANK, 0, 0xffffff, 1,0);
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_ccd_ctrl_ops_volatile,
			V4L2_CID_VBLANK, 0, 0xffffff, 1,0);
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	//Init controls
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	priv->hblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY ;
	priv->vblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY;

	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	//Init video infrasctuctures
	qtec_ccd_init_fmt(priv);
	priv->fival=(struct v4l2_fract) DEF_FIVAL;
	qtec_ccd_update_exposure_range(priv);
	priv->streaming=false;

	//Check sync
	ret=qtec_ccd_find_eye(priv);
	if (ret)
		v4l2_err(&priv->sd, "Error syncing, check hardware\n");

	v4l2_info(&priv->sd, "qtec_ccd %d ccds%s V4L2 subdevice registered as %s\n",
			priv->n_ccd, (priv->is_bayer_chip)?" (Bayer)":"",priv->sd.name);

	return 0;
}

static int __exit qtec_ccd_remove(struct platform_device *pdev){
	struct qtec_ccd *priv=platform_get_drvdata(pdev);

	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static struct of_device_id qtec_ccd_of_match[] = {
	{ .compatible = "qtec,axi-framebus-gen-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_ccd_of_match);

static struct platform_driver qtec_ccd_plat_driver = {
	.probe		= qtec_ccd_probe,
	.remove		= qtec_ccd_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_ccd_of_match,
	},
};

module_platform_driver(qtec_ccd_plat_driver);

MODULE_DESCRIPTION("CCD module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
