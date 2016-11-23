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

//Values
#define N_SCP 9
#define DOUTPHASE_N_OFFSET 0x20
#define DOUTPHASE_TESTMODE_VALUE 0x4
#define CLAMPLEVEL_VALUE 0x64
#define HCLK_MODE_VAL 2
#define TIMING_DRIVER_DELAY 17
#define LVDSSYNCW0_VALUE 0xaa55
#define LVDSSYNCW1_VALUE 0xce3c
#define LVDSTEST_PATTERN_VALUE 0x2c2c
#define SERIALIZER_DELAY 18

//Registers
#define AFEREG 0x0
#define CLAMPLEVEL_EN 3

#define STARTUP 0x1
#define STARTUP_EN 3

#define CDSGAIN 0x4
#define VGAGAIN 0x5
#define CLAMPLEVEL 0x6

#define HBLKMISC1 0x8
#define SWRST 0x10

#define HBLKMISC2 0x9
#define H1_POL_BIT 21
#define H2_POL_BIT 22

#define HBLKMISC3 0xa
#define HDLEN 0xb
#define HDLEN_ODD 0
#define HDLEN_EVEN 13

#define CLPOB0 0xc
#define CLPOB1 0xd
#define PBLK0 0xe
#define PBLK1 0xf

#define OUTCTRL 0x11
#define TGCORE 0x14

#define IOCTRL 0x23
#define IOVDD 2
#define HCLK_MODE 5

#define HPATNUM 0x28
#define FIELDNUM 0x2a
#define FIELSEL1 0x2b
#define FIELSEL2 0x2c

#define H1REG 0x30
#define H2REG 0x31
#define RGREG 0x33
#define POS 0
#define NEG 8
#define POL 16

#define HDRIVE 0x35
#define DRIVE_H1 0
#define DRIVE_H2 4
#define DRIVE_RG 20

#define SAMPLEREG 0x36
#define SHD 0
#define SHP 6
#define SAMPLEWIDTH 12

#define DOUTPHASE 0x37
#define DOUTPHASE_P 0
#define DOUTPHASE_N 6
#define DOUTPHASE_TESTMODE 12

#define LVDSPWR 0x40
#define LVDSPWR_START 2
#define TCLK 0
#define DOUT1 3

#define LVDSMISC 0x41
#define LVDSMISC_VALUE 0x200

#define LVDSSYNCW 0x43
#define LVDSSYNCW0 0x44
#define LVDSSYNCW1 0x45

#define LVDSTEST 0x4b
#define LVDSTEST_EN 1

#define LVDS_SYNC1 0x4c

#define LVDS_SYNC2 0x4d
#define CWEN 14

#define LVDSTEST_PATTERN 0x72
#define LVDSTEST_PATTERN_EN 24

//Hpat
#define HPAT_SIZE 0x10
#define HPAT_START 0x800
#define HBLKTOGO1 0x0
#define HBLKTOGO2 0x1
#define HBLKTOGO3 0x2
#define HBLKTOGE1 0x3
#define HBLKTOGE2 0x4
#define HBLKTOGE3 0x5
#define HBLKSTART1 0x6
#define HBLKSTART2 0x7
#define T1 0
#define T2 13

//Field
#define SCP_BASE 0
#define SCP_0 0
#define SCP_1 13

#define HPAT1 0x5
#define HPAT2 0x6
#define FIELD_UNUSED 0x7

#define CLPOB 0x8
#define CLPOB_POL 0
#define CLPOB_PAT 9

#define CLPOBMASK1 0x9
#define CLPOBMASK1_END1 13
#define CLPOBMASK2 0xa
#define CLPOBMASK3 0xb
#define MASK_START 0
#define MASK_END 13

#define PBLK 0xc
#define PBLK_POL 0
#define PBLK_PAT 9

#define PBLKMASK1 0xd
#define PBLKMASK2 0xe
#define PBLKMASK3 0xf

/*Horizontal Shifting*/
/*ICX204AL page 7 and AD9970 page13*/
#define H2_TOG1 27
#define H2_TOG2 (H2_TOG1+32) //After H2_TOG1
#define H2_TOG1_INV 7		//exp value
#define H2_TOG2_INV 41		//exp value
#define H2_POL 1
#define H2_DRIVE 3
#define H2_DRIVE_INV 7
#define H1_TOG1 (H2_TOG1+2)
#define H1_TOG2 (H1_TOG1+32)
#define H1_TOG1_INV 3 //exp value
#define H1_TOG2_INV 43 //exp value
#define H1_POL 0
#define H1_DRIVE 3
#define H1_DRIVE_INV 7
#define RG_TOG1 (H1_TOG1+3) //After H1_TOG1
#define RG_TOG2 (RG_TOG1+13)
#define RG_TOG1_INV 16 //exp value
#define RG_TOG2_INV 32 //exp value
#define RG_POL 1
#define RG_DRIVE 3
#define RG_DRIVE_INV 7
#define SHDLOC (H2_TOG1-18)
#define SHPLOC (H2_TOG2-1)
#define SHDLOC_INV 21 //exp value
#define SHPLOC_INV 42  //exp value
#define SWIDTH_INV 16
#define SWIDTH 21
#define VEND_DELAY 0

#define HBLANK_SEC 5

static int qtec_ccd_ad_startup(struct qtec_ccd *priv){
	qtec_ccd_write_ad(priv,CHAN_ALL,SWRST,1);
	qtec_ccd_write_ad(priv,CHAN_ALL,STARTUP,BIT(STARTUP_EN));
	return 0;
}

static int qtec_ccd_ad_hpat(struct qtec_ccd *priv){
	int i;
	uint32_t reg;
	int offset;

	for (i=NORMAL;i<=SKIP;i++){
		offset=i*HPAT_SIZE+HPAT_START;

		reg=(HDELAY+0-HCOUNTER_DELAY)<<T1;
		if (i==NORMAL)
			reg|=(HDELAY+(SHIFT_END+1)-HCOUNTER_DELAY)<<T2;
		else
			reg|=(HDELAY+1-HCOUNTER_DELAY)<<T2;
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGO1+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGE1+offset,reg);
		reg=0x1fff<<T1;
		reg|=0x1fff<<T2;
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGO2+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGO3+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGE2+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKTOGE3+offset,reg);

		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKSTART1+offset,0);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKSTART2+offset,0);

		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKMISC1+offset,0);
		if (priv->n_ccd == 1){
			reg=(H1_POL)?BIT(H1_POL_BIT):0;
			reg|=(H2_POL)?BIT(H2_POL_BIT):0;
		}
		else{
			reg=(H1_POL)?0:BIT(H1_POL_BIT);
			reg|=(H2_POL)?0:BIT(H2_POL_BIT);
		}
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKMISC2+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,HBLKMISC3+offset,0);
		reg=HTOTAL<<HDLEN_ODD;
		reg|=HTOTAL<<HDLEN_EVEN;
		qtec_ccd_write_ad(priv,CHAN_ALL,HDLEN+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,CLPOB0+offset,0);
		if (i==NORMAL){
			reg=(HDELAY-HBLANK_TAIL-HCOUNTER_DELAY+HBLANK_SEC)<<T1;
			reg|=(HDELAY-HCOUNTER_DELAY-1-HBLANK_SEC)<<T2;
		}
		else{
			reg=0x1fff<<T1;
			reg|=0<<T2;
		}
		qtec_ccd_write_ad(priv,CHAN_ALL,CLPOB1+offset,reg);
		qtec_ccd_write_ad(priv,CHAN_ALL,PBLK0+offset,0);
		if (i==NORMAL){
			reg=(HDELAY-HCOUNTER_DELAY)<<T1;
			reg|=(HDELAY+SHIFT_END+1-HCOUNTER_DELAY)<<T2;
		}
		else{
			reg=0<<T1;
			reg|=0x1fff<<T2;
		}
		qtec_ccd_write_ad(priv,CHAN_ALL,PBLK1+offset,reg);
	}
	return 0;
}

static int qtec_ccd_ad_field(struct qtec_ccd *priv){
	int i;
	uint32_t reg;
	int offset;
	int last_line=0;

	offset=HPAT_START+2*(HPAT_SIZE);

	for (i=0;i<((N_SCP+1)/2);i++){
		reg=last_line<<SCP_0;
		if ((i*2)<priv->readout.n)
			last_line+=priv->readout.read[i*2].len;
		if (i!=(((N_SCP+1)/2)-1))
			reg|=last_line<<SCP_1;
		if (((i*2)+1)<priv->readout.n)
			last_line+=priv->readout.read[i*2+1].len;
		qtec_ccd_write_ad(priv,CHAN_ALL,offset+SCP_BASE+i,reg);
	}

	reg=0;
	for (i=0;i<5;i++)
		if (priv->readout.n>=i)
			reg|=(priv->readout.read[i].mode)<<5*i;
	qtec_ccd_write_ad(priv,CHAN_ALL,HPAT1+offset,reg);
	reg=0;
	for (i=5;i<10;i++)
		if (priv->readout.n>=i)
			reg|=(priv->readout.read[i].mode)<<5*(i-5);
	qtec_ccd_write_ad(priv,CHAN_ALL,HPAT2+offset,reg);
	qtec_ccd_write_ad(priv,CHAN_ALL,FIELD_UNUSED+offset,0);

	reg=0x1ff<<CLPOB_POL;
	reg|=0x1ff<<CLPOB_PAT;
	qtec_ccd_write_ad(priv,CHAN_ALL,CLPOB+offset,reg);

	reg= 0<<MASK_START;
	reg|= (VVSG+VDUMMY-1)<<MASK_END;
	qtec_ccd_write_ad(priv,CHAN_ALL,CLPOBMASK1+offset,reg);

	reg= (qtec_ccd_readout_lines(priv)-VEND_DELAY)<<MASK_START;
	reg|= 0x1fff<<MASK_END;
	qtec_ccd_write_ad(priv,CHAN_ALL,CLPOBMASK2+offset,reg);

	reg= 0x1fff<<MASK_START;
	reg|= 0<<MASK_END;
	qtec_ccd_write_ad(priv,CHAN_ALL,CLPOBMASK3+offset,reg);

	reg=0x1ff<<PBLK_POL;
	reg|=1<<PBLK_PAT;
	qtec_ccd_write_ad(priv,CHAN_ALL,PBLK+offset,reg);

	reg= 0x1fff<<MASK_START;
	reg|= 0<<MASK_END;
	qtec_ccd_write_ad(priv,CHAN_ALL,PBLKMASK1+offset,reg);
	qtec_ccd_write_ad(priv,CHAN_ALL,PBLKMASK2+offset,reg);
	qtec_ccd_write_ad(priv,CHAN_ALL,PBLKMASK3+offset,reg);

	return 0;
}

static int qtec_ccd_ad_field_hpat(struct qtec_ccd *priv){
	qtec_ccd_ad_hpat(priv);
	qtec_ccd_ad_field(priv);
	qtec_ccd_write_ad(priv,CHAN_ALL,HPATNUM,2);
	qtec_ccd_write_ad(priv,CHAN_ALL,FIELDNUM,1);
	qtec_ccd_write_ad(priv,CHAN_ALL,FIELSEL1,0);
	qtec_ccd_write_ad(priv,CHAN_ALL,FIELSEL2,0);

	return 0;
}

static int qtec_ccd_ad_timming(struct qtec_ccd *priv){
	uint32_t reg;

	reg=   ((priv->n_ccd==1)?H1_TOG1_INV:(H1_TOG1+64-TIMING_DRIVER_DELAY)%64)<<POS;
	reg |= ((priv->n_ccd==1)?H1_TOG2_INV:(H1_TOG2+64-TIMING_DRIVER_DELAY)%64)<<NEG;
	reg |= ((priv->n_ccd==1)?(1-H1_POL):H1_POL) <<POL;
	qtec_ccd_write_ad(priv,CHAN_ALL,H1REG,reg);

	reg=   ((priv->n_ccd==1)?H2_TOG1_INV:(H2_TOG1+64-TIMING_DRIVER_DELAY)%64)<<POS;
	reg |= ((priv->n_ccd==1)?H2_TOG2_INV:(H2_TOG2+64-TIMING_DRIVER_DELAY)%64)<<NEG;
	reg |= ((priv->n_ccd==1)?(1-H2_POL):H2_POL) <<POL;
	qtec_ccd_write_ad(priv,CHAN_ALL,H2REG,reg);

	reg=   ((priv->n_ccd==1)?RG_TOG1_INV:(RG_TOG1+64-TIMING_DRIVER_DELAY)%64)<<POS;
	reg |= ((priv->n_ccd==1)?RG_TOG2_INV:(RG_TOG2+64-TIMING_DRIVER_DELAY)%64)<<NEG;
	reg |= RG_POL <<POL;
	qtec_ccd_write_ad(priv,CHAN_ALL,RGREG,reg);

	reg=   ((priv->n_ccd==1)?SHDLOC_INV:SHDLOC) <<SHD;
	reg |= ((priv->n_ccd==1)?SHPLOC_INV:SHPLOC) <<SHP;
	reg |= ((priv->n_ccd==1)?SWIDTH_INV:SWIDTH) <<SAMPLEWIDTH;
	qtec_ccd_write_ad(priv,CHAN_ALL,SAMPLEREG,reg);

	reg=   ((priv->n_ccd==1)?H1_DRIVE_INV:H1_DRIVE) << DRIVE_H1;
	reg |= ((priv->n_ccd==1)?H2_DRIVE_INV:H2_DRIVE) << DRIVE_H2;
	reg |= ((priv->n_ccd==1)?RG_DRIVE_INV:RG_DRIVE) << DRIVE_RG;
	qtec_ccd_write_ad(priv,CHAN_ALL,HDRIVE,reg);

	return 0;
}

static int qtec_ccd_ad_db2val(struct qtec_ccd *priv, int32_t db){
	int32_t val;

	val=db+CALIB_0DB; //Workaround for #443
	val=(val*10);
	val+=(359/2);
	val/=359;

	val = clamp(val,0,1023);

	return val;
}

static int qtec_ccd_ad_afe(struct qtec_ccd *priv){
	uint32_t reg;
	int i;

	reg=BIT(IOVDD);
	reg|=HCLK_MODE_VAL << HCLK_MODE;
	qtec_ccd_write_ad(priv,CHAN_ALL,IOCTRL,reg);

	for (i=0;i<priv->n_ccd;i++){
		qtec_ccd_write_ad(priv,i,VGAGAIN,qtec_ccd_ad_db2val(priv,priv->vga_gain[i]->val));
		qtec_ccd_write_ad(priv,i,CDSGAIN,priv->cds[i]->val);
	}
	qtec_ccd_write_ad(priv,CHAN_ALL,CLAMPLEVEL,CLAMPLEVEL_VALUE);
	qtec_ccd_write_ad(priv,CHAN_ALL,AFEREG,BIT(CLAMPLEVEL_EN));

	return 0;
}

int qtec_ccd_ad_dout_phase(struct qtec_ccd *priv,int chan,int doutphase);
static int qtec_ccd_ad_lvds(struct qtec_ccd *priv){
	int i;

	for (i=0;i<priv->n_ccd;i++)
		qtec_ccd_ad_dout_phase(priv,i,priv->dout_phase[i]);

	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSPWR, BIT(TCLK)|BIT(DOUT1) | LVDSPWR_START);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSMISC,LVDSMISC_VALUE);

	qtec_ccd_write_ad(priv,CHAN_ALL,OUTCTRL,1);

	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSSYNCW,2);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSSYNCW0,LVDSSYNCW0_VALUE);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSSYNCW1,LVDSSYNCW1_VALUE);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDS_SYNC1,0);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDS_SYNC2,0);

	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSTEST_PATTERN,LVDSTEST_PATTERN_VALUE | BIT(LVDSTEST_PATTERN_EN) );
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSTEST,BIT(LVDSTEST_EN));

	return 0;
}

int qtec_ccd_ad_dout_phase(struct qtec_ccd *priv,int chan,int doutphase){
	uint32_t reg;
	reg= doutphase << DOUTPHASE_P;
	reg|=(doutphase+DOUTPHASE_N_OFFSET) << DOUTPHASE_N;
	reg|= DOUTPHASE_TESTMODE_VALUE << DOUTPHASE_TESTMODE;
	qtec_ccd_write_ad(priv,chan,DOUTPHASE,reg);
	return 0;
}

int qtec_ccd_ad_mode_normal(struct qtec_ccd *priv){

	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSTEST_PATTERN,0);
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDS_SYNC2,(HDELAY+SERIALIZER_DELAY-HCOUNTER_DELAY) | BIT(CWEN) );
	qtec_ccd_write_ad(priv,CHAN_ALL,LVDSTEST,0);

	return 0;
}

int  qtec_ccd_ad_start(struct qtec_ccd *priv){
	qtec_ccd_ad_startup(priv);
	qtec_ccd_ad_field_hpat(priv);
	qtec_ccd_ad_timming(priv);
	qtec_ccd_ad_afe(priv);
	/*Sleep for at least 500 usecs (page 37 AD9970)*/
	msleep(1);
	/*Sleep for at least 100 usecs (page 37 AD9970)*/
	qtec_ccd_write_ad(priv,CHAN_ALL,TGCORE,1);
	msleep(1);
	qtec_ccd_ad_lvds(priv);
	return 0;
}

int  qtec_ccd_ad_stop(struct qtec_ccd *priv){
	qtec_ccd_write_ad(priv,CHAN_ALL,OUTCTRL,0);
	return 0;
}

int qtec_ccd_ad_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl){
	int i;
	static const int qtec_v4l2[][3] ={
		{QTEC_CCD_CID_CDS,CHAN_RED,true},
		{QTEC_CCD_CID_RED_CDS,CHAN_RED,true},
		{QTEC_CCD_CID_GREEN_CDS,CHAN_GREEN,true},
		{QTEC_CCD_CID_BLUE_CDS,CHAN_BLUE,true},
		{QTEC_CCD_CID_IR1_CDS,CHAN_IR1,true},
		{QTEC_CCD_CID_IR2_CDS,CHAN_IR2,true},
		{V4L2_CID_GAIN,CHAN_RED,false},
		{QTEC_CCD_CID_RED_VGAGAIN,CHAN_RED,false},
		{QTEC_CCD_CID_GREEN_VGAGAIN,CHAN_GREEN,false},
		{QTEC_CCD_CID_BLUE_VGAGAIN,CHAN_BLUE,false},
		{QTEC_CCD_CID_IR1_VGAGAIN,CHAN_IR1,false},
		{QTEC_CCD_CID_IR2_VGAGAIN,CHAN_IR2,false},
		{QTEC_CCD_CID_IR2_VGAGAIN,CHAN_IR2,false},
	};

	if (!priv->streaming)
		return 0;

	for (i=0;i<ARRAY_SIZE(qtec_v4l2);i++)
		if (ctrl->id == qtec_v4l2[i][0])
			break;

	if (i==ARRAY_SIZE(qtec_v4l2))
		return -EINVAL;

	if (qtec_v4l2[i][2])
		qtec_ccd_write_ad(priv,qtec_v4l2[i][1],CDSGAIN,ctrl->val);
	else
		qtec_ccd_write_ad(priv,qtec_v4l2[i][1],VGAGAIN,qtec_ccd_ad_db2val(priv,ctrl->val));

	return 0;
}
