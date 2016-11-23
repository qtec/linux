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
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include "qtec_ccd.h"

#define CTRL 0x0
#define EXT_TRIG_WIDTH 1
#define EXT_TRIG_POL 2
#define EXT_TRIG_ENA 3
#define SW_TRIG 4
#define FLASH_POL 5

#define MAPPING_LEN 4
#define MAPPING 12

#define DELAY_TIMER (0x4)
#define EXP_TIMER_BASE (0x8)

static uint32_t qtec_ccd_trig_time2clk_delay(struct qtec_ccd *priv, int32_t time){
	uint64_t res=0;

	res=(int64_t)time*(int64_t)priv->bus_clk;
	do_div(res,1000000);

	res=clamp(res,(uint64_t)0x1,(uint64_t)0xffffffff);

	return res;
}

static uint32_t qtec_ccd_trig_time2clk_exp(struct qtec_ccd *priv, int32_t time){
	uint64_t res=0;

	res=(int64_t)time*(int64_t)priv->bus_clk;
	do_div(res,1000000);

	if (res & 0x80) //0x100 Rounding
		res +=0x80;

	res=clamp(res,(uint64_t)0x100,(uint64_t)0xffffffff); //Minimun value is 0x100

	return res;
}

static int qtec_ccd_update_trig_exposure(struct qtec_ccd *priv){
	int max_exp_channel=0;
	uint32_t mapping=0;
	uint32_t reg;
	int i;

	for (i=1;i<priv->n_ccd;i++){
		if (priv->exp_time[i]->val>priv->exp_time[max_exp_channel]->val)
			max_exp_channel=i;
	}

	for (i=0;i<MAX_CCD;i++){
		int chan=(max_exp_channel+i)%MAX_CCD;
		uint32_t val=qtec_ccd_trig_time2clk_exp(priv,(chan>=priv->n_ccd)?0:priv->exp_time[chan]->val);
		qtec_ccd_write_trig(priv,EXP_TIMER_BASE+i*4,val);
		mapping |= i<<(MAPPING_LEN*chan);
	}

	qtec_ccd_read_trig(priv,CTRL,&reg);
	reg &= ~(((1<<(MAPPING_LEN*MAX_CCD))-1)<<MAPPING);
	reg |= mapping<<MAPPING;
	qtec_ccd_write_trig(priv,CTRL,reg); //All exposures are applied here.

	return 0;

}

int qtec_ccd_trig_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl){
	uint32_t reg;

	switch (ctrl->id){
		case QTEC_CCD_CID_RED_EXPOSURE:
		case QTEC_CCD_CID_GREEN_EXPOSURE:
		case QTEC_CCD_CID_BLUE_EXPOSURE:
		case QTEC_CCD_CID_IR1_EXPOSURE:
		case QTEC_CCD_CID_IR2_EXPOSURE:
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			qtec_ccd_update_trig_exposure(priv);
			break;

		case QTEC_CCD_CID_EXT_TRIG_DELAY:
			qtec_ccd_write_trig(priv,DELAY_TIMER,qtec_ccd_trig_time2clk_delay(priv,priv->ext_trig_delay->val));
			break;
		case QTEC_CCD_CID_MANUAL_TRIGGER:
			qtec_ccd_read_trig(priv,CTRL,&reg);
			reg |= BIT(SW_TRIG);
			qtec_ccd_write_trig(priv,CTRL,reg);
			reg &= ~BIT(SW_TRIG);
			qtec_ccd_write_trig(priv,CTRL,reg);
			break;
		case QTEC_CCD_CID_MODE_TRIG:
			qtec_ccd_read_trig(priv,CTRL,&reg);
			if (ctrl->val==EXT_EXPOSURE)
				reg |= BIT(EXT_TRIG_WIDTH);
			else
				reg &= BIT(EXT_TRIG_WIDTH);
			if (ctrl->val!=SELF_TIMED)
				reg |= BIT(EXT_TRIG_ENA);
			qtec_ccd_write_trig(priv,CTRL,reg);

			break;
		case QTEC_CCD_CID_FLASH_POL:
			qtec_ccd_read_trig(priv,CTRL,&reg);
			if (ctrl->val)
				reg |= BIT(FLASH_POL);
			else
				reg &= ~BIT(FLASH_POL);
			qtec_ccd_write_trig(priv,CTRL,reg);
			break;
		case QTEC_CCD_CID_TRIG_POL:
			qtec_ccd_read_trig(priv,CTRL,&reg);
			if (ctrl->val)
				reg |= BIT(EXT_TRIG_POL);
			else
				reg &= ~BIT(EXT_TRIG_POL);
			qtec_ccd_write_trig(priv,CTRL,reg);
			break;
	}

	return 0;
}

int qtec_ccd_trig_stop(struct qtec_ccd *priv){
	return 0;
}

int qtec_ccd_trig_start(struct qtec_ccd *priv){
	qtec_ccd_update_trig_exposure(priv);
	return 0;
}
