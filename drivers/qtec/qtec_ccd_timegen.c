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

#define HSYNC_WIDTH (HDELAY+SHIFT_END+HDUMMY+HBLANK_HEAD+HBORDER_HEAD+40)
#define VSYNC_WIDTH (VVSG+VDUMMY+VBLANK_HEAD+VBORDER_HEAD+5)

/*Vertical Shifting*/
/*ICX204AL page 18. Naming of the CXD1267AN*/
#define V1_POL		1
#define V2_POL		0
#define V3_POL		1
#define V4_POL		0
/*Normal mode*/
#define V1_TOG1		39
#define V1_TOG2		119
#define V2_TOG1		0
#define V2_TOG2		99
#define V3_TOG1		39
#define V3_TOG2		119
#define V4_TOG1		59
#define V4_TOG2		139
#define SHIFT_END	162
#define V1_TOG1_SKIP	V1_TOG1
#define V1_TOG2_SKIP	V1_TOG2
#define V2_TOG1_SKIP	V2_TOG1
#define V2_TOG2_SKIP	V2_TOG2
#define V3_TOG1_SKIP	V3_TOG1
#define V3_TOG2_SKIP	V3_TOG2
#define V4_TOG1_SKIP	V4_TOG1
#define V4_TOG2_SKIP	V4_TOG2
#define SHIFT_END_SKIP	SHIFT_END
/*Readout*/
/*ICX204AL page 14. Naming of the CXD1267AN*/
#define READOUT_LINE	0
#define V2R_TOG1	SHIFT_END
#define V2R_TOG2	806
#define V2R_TYPE	1 /* 1 force high */
#define VSG1_TOG1_VAL	804
#define VSG1_TOG2_VAL	855
#define VSG2_TOG1_VAL	VSG1_TOG1_VAL
#define VSG2_TOG2_VAL	VSG1_TOG2_VAL
/*Subck*/
/*ICX204AL page 18. Naming of the CXD1267AN*/
/* It is 126 instead of 108 because we run it at 30Mhz! (kgm)*/
#define SUBCK_DELAY	0
#define SUB_TOG1	59
#define SUB_TOG2	126

/*Registers*/
#define HD_PERIOD (0*4)
#define VD_PERIOD (1*4)
#define HD_WIDTH (2*4)
#define VD_WIDTH (3*4)
#define VTP_PRESET (4*4)

#define CONTROL (5*4)
#define RUN 0
#define PAGE_NXT 1
#define PAGE_USED 2
#define HINVERT 7
#define EXP_MODE 5

#define VSG_ENABLE (8*4)
#define VSG1_TOG1 (9*4)
#define VSG1_TOG2 (0xa*4)
#define VSG2_TOG1 (0xb*4)
#define VSG2_TOG2 (0xc*4)
#define SVT (0x10*4)
#define SVTV1_12 (0x11*4)
#define SVTV2_12 (0x12*4)
#define SVTV3_12 (0x13*4)
#define SVTV4_12 (0x14*4)
#define SVTV2_12 (0x12*4)
#define SVTLINE (0x18*4)
#define SVTV1_34 (0x19*4)
#define SVTV2_34 (0x1a*4)
#define SVTV3_34 (0x1b*4)
#define SVTV4_34 (0x1c*4)
#define SVT_T1 12
#define SVT_T2 0

#define REGION_CHANGE (0x20*4)

#define SUBCKTOG (0x30*4)
#define SUBCK_T2 0
#define SUBCK_T1 16

#define VT_SIZE (8*4)
#define VT (0x40*4)
#define INTERVAL 0
#define REPEAT 9
#define VT_POL1 14
#define VT_POL2 15
#define VT_POL3 16
#define VT_POL4 17

#define SUBCKCTRL (0x38*4)
#define SUBCK_EN 23

#define V1 (0x41*4)
#define V2 (0x42*4)
#define V3 (0x43*4)
#define V4 (0x44*4)
#define T1 9
#define T2 0

#define REGION_MAX 16

#define MODE_NORMAL 0
#define MODE_SYNC 2
#define MODE_ASYNC 3

static int qtec_ccd_tg_vperiod(struct qtec_ccd *priv){
	uint64_t lines;

	if ((priv->trig_mode->val != SELF_TIMED))
		return qtec_ccd_readout_lines(priv);

	lines= (int64_t)priv->fival.numerator * (int64_t) priv->pixel_clk;
	do_div(lines,priv->fival.denominator);
	do_div(lines,HTOTAL);

	return (int)lines;
}

static int qtec_ccd_tg_syncs(struct qtec_ccd *priv){
	int lines=qtec_ccd_tg_vperiod(priv);
	qtec_ccd_write_timegen(priv,HD_PERIOD,HTOTAL-1);
	qtec_ccd_write_timegen(priv,VD_PERIOD,lines);
	qtec_ccd_write_timegen(priv,HD_WIDTH,HSYNC_WIDTH);
	qtec_ccd_write_timegen(priv,VD_WIDTH,VSYNC_WIDTH);
	qtec_ccd_write_timegen(priv,VTP_PRESET,HDELAY-2);
	return 0;
}

static int qtec_ccd_tg_timing(struct qtec_ccd *priv){
	uint32_t reg;
	int line_change=0,i;

	for (i=0;i<REGION_MAX;i++){
		if (i<priv->readout.n && priv->readout.read[i].mode ==SKIP)
			reg=LINE_SKIP<<REPEAT;
		else
			reg=1<<REPEAT;
		reg |= (V1_POL)?BIT(VT_POL1):0;
		reg |= (V2_POL)?BIT(VT_POL2):0;
		reg |= (V3_POL)?BIT(VT_POL3):0;
		reg |= (V4_POL)?BIT(VT_POL4):0;
		reg |= (HDELAY+SHIFT_END)<<INTERVAL;
		qtec_ccd_write_timegen(priv,VT + i*VT_SIZE,reg);

		reg = (V1_TOG1_SKIP+HDELAY-2) << T1;
		reg |= (V1_TOG2_SKIP+HDELAY-2) << T2;
		qtec_ccd_write_timegen(priv,V1 + i*VT_SIZE,reg);

		reg = (V2_TOG1_SKIP+HDELAY-2) << T1;
		reg |= (V2_TOG2_SKIP+HDELAY-2) << T2;
		qtec_ccd_write_timegen(priv,V2 + i*VT_SIZE,reg);

		reg = (V3_TOG1_SKIP+HDELAY-2) << T1;
		reg |= (V3_TOG2_SKIP+HDELAY-2) << T2;
		qtec_ccd_write_timegen(priv,V3 + i*VT_SIZE,reg);

		reg = (V4_TOG1_SKIP+HDELAY-2) << T1;
		reg |= (V4_TOG2_SKIP+HDELAY-2) << T2;
		qtec_ccd_write_timegen(priv,V4 + i*VT_SIZE,reg);


		if (i<priv->readout.n)
			line_change += priv->readout.read[i].len;

		qtec_ccd_write_timegen(priv,REGION_CHANGE+i*4,line_change);
	}

	return 0;
}

static int qtec_ccd_tg_special_timing(struct qtec_ccd *priv){
	uint32_t reg;
	qtec_ccd_write_timegen(priv,SVT,BIT(1));
	qtec_ccd_write_timegen(priv,SVTLINE,1);
	qtec_ccd_write_timegen(priv,SVTV1_12,0);
	qtec_ccd_write_timegen(priv,SVTV1_34,0);
	reg = (V2R_TOG1+HDELAY)<<SVT_T1;
	reg |= (V2R_TOG2+HDELAY)<<SVT_T2;
	qtec_ccd_write_timegen(priv,SVTV2_12,reg);
	qtec_ccd_write_timegen(priv,SVTV2_34,0xffffff);
	qtec_ccd_write_timegen(priv,SVTV3_12,0);
	qtec_ccd_write_timegen(priv,SVTV3_34,0);
	qtec_ccd_write_timegen(priv,SVTV4_12,0);
	qtec_ccd_write_timegen(priv,SVTV4_34,0);
	qtec_ccd_write_timegen(priv,VSG_ENABLE,1);
	qtec_ccd_write_timegen(priv,VSG1_TOG1,VSG1_TOG1_VAL+HDELAY);
	qtec_ccd_write_timegen(priv,VSG1_TOG2,VSG1_TOG2_VAL+HDELAY);
	qtec_ccd_write_timegen(priv,VSG2_TOG1,VSG2_TOG1_VAL+HDELAY);
	qtec_ccd_write_timegen(priv,VSG2_TOG2,VSG2_TOG2_VAL+HDELAY);
	return 0;
}

static int qtec_ccd_tg_time2lines(struct qtec_ccd *priv, int time){
	uint64_t lines;
	int lines_int;
	lines=(int64_t)time * priv->pixel_clk;
	do_div(lines,HTOTAL);
	do_div(lines,1000000);
	lines_int=lines;

	return lines_int;
}

static int qtec_ccd_tg_exposure(struct qtec_ccd *priv,int chan, int time){
	uint32_t reg;
	int lines=qtec_ccd_tg_time2lines(priv,time);
	int vperiod=qtec_ccd_tg_vperiod(priv);

	lines=clamp(lines,2,vperiod-3);

	reg= vperiod-lines;
	reg |= BIT(SUBCK_EN);
	qtec_ccd_write_timegen(priv,SUBCKCTRL+chan*4,reg);
	return 0;
}

static int qtec_ccd_tg_subck(struct qtec_ccd *priv){
	uint32_t reg,i;

	reg = (SUB_TOG1+HDELAY-1) << SUBCK_T1;
	reg |= (SUB_TOG2+HDELAY-1) << SUBCK_T2;

	for (i=0;i<MAX_CCD;i++){
		int time = 1;
		qtec_ccd_write_timegen(priv,SUBCKTOG+i*4,reg);

		if ((priv->trig_mode->val == SELF_TIMED) && (i<priv->n_ccd))
			time = priv->exp_time[i]->val;

		qtec_ccd_tg_exposure(priv,i,time);
	}

	return 0;
}

static int qtec_ccd_tg_cntrl(struct qtec_ccd *priv){
	uint32_t reg,new_reg;
	qtec_ccd_read_timegen(priv,CONTROL,&reg);

	new_reg = BIT(RUN);

	if (((reg&BIT(RUN)) == 0) != ((reg&BIT(PAGE_USED)) == 0))
		new_reg |= BIT(PAGE_NXT);

	if (priv->n_ccd == 1)
		new_reg |= BIT(HINVERT);

	if ((priv->trig_mode->val == SELF_TIMED)||(!priv->streaming))
		new_reg |= MODE_NORMAL << EXP_MODE;
	else
		new_reg |= MODE_SYNC << EXP_MODE;

	qtec_ccd_write_timegen(priv,CONTROL,new_reg);
	return 0;
}

int qtec_ccd_tg_stop(struct qtec_ccd *priv){
	qtec_ccd_write_timegen(priv,CONTROL,0x0);
	return 0;
}

int qtec_ccd_tg_start(struct qtec_ccd *priv){

	qtec_ccd_tg_syncs(priv);
	qtec_ccd_tg_timing(priv);
	qtec_ccd_tg_special_timing(priv);
	qtec_ccd_tg_subck(priv);
	qtec_ccd_tg_cntrl(priv);
	return 0;
}

int qtec_ccd_tg_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl){
	uint32_t reg;

	if (!priv->streaming)
		return 0;

	//Workaround for #468
	qtec_ccd_read_timegen(priv,CONTROL,&reg);
	if (reg&BIT(PAGE_USED))
		reg|=BIT(PAGE_NXT);
	else
		reg&=~BIT(PAGE_NXT);
	qtec_ccd_write_timegen(priv,CONTROL,reg);

	qtec_ccd_tg_start(priv);

	return 0;
}
