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

#define DATA_TRAIN 0x2c
#define MAX_STEP 27
#define MAX_PHASE 4
#define WINDOW_MIN 5
#define N_WINDOW 8

//Registers
#define STAT 0x0
#define IRQ_ACK 0
#define FSLOCK 1
#define BITCHG 12
#define BUSY 13
#define DATA_LSB 16

#define HOR (0x1*4)
#define HOR_START 0
#define HOR_END 16

#define CONTROL (0x2*4)
#define FSRUN 1
#define IRQ_ENA 2
#define N_CHAN 4
#define STREAM 7
#define CE 15
#define ENABLE 16
#define CLR 17
#define INC 20
#define RST 21
#define CAL 22
#define BITSLIP 23

#define VER_BASE (0x4*4)
#define VER_START 0
#define VER_END 16

static inline int qtec_ccd_fg_wait_busy(struct qtec_ccd *priv){
	uint32_t reg;
	unsigned long expiration=jiffies+HZ;

	do {
		qtec_ccd_read_fg(priv, STAT,&reg);
		if ((reg & BIT(BUSY))==0)
			return 0;
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "IDELAY expired\n");
	return -1;
}

static inline int qtec_ccd_fg_toggle(struct qtec_ccd *priv, int reg, uint32_t bits){
	reg |= bits;
	qtec_ccd_write_fg(priv, CONTROL,reg);
	reg |= BIT(ENABLE);
	qtec_ccd_write_fg(priv, CONTROL,reg);
	reg &= ~(bits | BIT(ENABLE));
	qtec_ccd_write_fg(priv, CONTROL,reg);
	return qtec_ccd_fg_wait_busy(priv);
}

static inline bool qtec_ccd_fg_valid_data(uint8_t data){
	if (hweight8(data)==hweight8(DATA_TRAIN))
		return true;
	return false;
}

static int qtec_ccd_fg_find_eye(struct qtec_ccd *priv,int chan, bool go_center){
	uint32_t reg;
	uint8_t do_sync[MAX_STEP+1];
	int best_len=-1,best_loc=-1,loc;

	qtec_ccd_write_fg(priv, CONTROL,0);
	reg=chan<<N_CHAN;
	qtec_ccd_write_fg(priv, CONTROL,reg);

	qtec_ccd_fg_toggle(priv,reg,BIT(CAL));
	qtec_ccd_fg_toggle(priv,reg,BIT(RST));

	for(loc=0;loc<MAX_STEP;loc++){
		uint32_t stat;
		qtec_ccd_fg_toggle(priv,reg,BIT(CLR)); //clear bitchange and meas for 1 msec
		msleep(1);
		qtec_ccd_read_fg(priv, STAT,&stat);
		if ((stat&BIT(BITCHG)))
			do_sync[loc]=0xff;
		else
			do_sync[loc]=(stat>>DATA_LSB)&0xff;
		dev_dbg(&priv->pdev->dev, "chan %d loc %d value 0x%x\n",chan,loc,do_sync[loc]);
		qtec_ccd_fg_toggle(priv,reg,BIT(INC)|BIT(CE));
	}

	for(loc=0;loc<MAX_STEP;loc++){
		int i, len=1;
		if (qtec_ccd_fg_valid_data(do_sync[loc])==false)
			continue;
		for (i=(loc+1);i<MAX_STEP;i++){
			if (do_sync[i]!=do_sync[loc])
				break;
			len++;
		}
		if (len >best_len){
			best_len=len;
			best_loc=loc;
		}

	}

	if (best_len < WINDOW_MIN)
		return -1;

	best_loc = (best_loc+(best_len/2))%MAX_STEP;

	if (go_center){
		int i;
		for (loc=(MAX_STEP-1);loc!=best_loc;loc--)
			qtec_ccd_fg_toggle(priv,reg,BIT(CE));

		for (i=0;i<8;i++){
			uint32_t data;
			qtec_ccd_read_fg(priv, STAT,&data);
			data=(data>>DATA_LSB)&0xff;
			dev_dbg(&priv->pdev->dev, "bslip %d value 0x%x\n",i,data);
			if (data==DATA_TRAIN)
				break;
			qtec_ccd_fg_toggle(priv,reg,BIT(BITSLIP));
		}

		if (i==8)
			return -1;
	}

	return best_len;
}

static int qtec_ccd_fg_line2readout(struct qtec_ccd *priv, int line){
	int i;
	int last_border=0;
	int new_border=0;
	int last_readout=0;

	for (i=0;i<priv->readout.n;i++){
		if (priv->readout.read[i].mode ==SKIP)
			new_border+=priv->readout.read[i].len*LINE_SKIP;
		else
			new_border+=priv->readout.read[i].len;

		if (line<new_border){
			int offset=line-last_border;
			if (priv->readout.read[i].mode ==SKIP)
				offset/=5;
			return last_readout+offset;
		}
		last_border=new_border;
		last_readout+=priv->readout.read[i].len;
	}

	return -1;

}

static int qtec_ccd_fg_setup_windows(struct qtec_ccd *priv){
	int start, stop;
	int i;

	start = (SHIFT_END+HDUMMY+HBLANK_HEAD+HBORDER_HEAD+priv->crop[0].left);
	stop = start + priv->crop[0].width-1;

	if ((priv->n_ccd==1) && ( priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB)){
		start--;
		stop++;
	}
	qtec_ccd_write_fg(priv, HOR,(start << HOR_START) | (stop << HOR_END));
	for (i=0;i<priv->n_crop;i++){
		start = VVSG+VDUMMY+VBLANK_HEAD+VBORDER_HEAD +priv->crop[i].top;
		start = qtec_ccd_fg_line2readout(priv,start);
		stop = start + priv->crop[i].height-1;

		if ((priv->n_ccd==1) && ( priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB)){
			start--;
			stop++;
		}
		qtec_ccd_write_fg(priv, VER_BASE+i*4 ,(start << VER_START) | (stop << VER_END));
	}
	for (;i<N_WINDOW;i++)
		qtec_ccd_write_fg(priv, VER_BASE+i*4 ,(start << VER_START) | (stop << VER_END));

	return 0;
}

int qtec_ccd_fg_calibrate_doutphase(struct qtec_ccd *priv){
	int chan;
	int ret=0;
	char report[64]= "";

	qtec_ccd_fg_setup_windows(priv);

	for (chan=0;chan<priv->n_ccd;chan++){
		int best_phase=-1;
		int best_eye=-1;
		int phase;
		for (phase=0;phase<MAX_PHASE;phase++){
			int eye;
			qtec_ccd_ad_dout_phase(priv,chan,phase);
			eye=qtec_ccd_fg_find_eye(priv,chan,0);
			if (eye>best_eye){
				best_eye=eye;
				best_phase=phase;
			}
			if (eye==MAX_STEP)
				break;
		}
		snprintf(report,64,"%s%d (%d) ",report,best_eye,best_phase);
		if (best_eye < 0)
			ret =-1;
		//Save best phase
		priv->dout_phase[chan]=best_phase;
	}

	if (ret)
		v4l2_err(&priv->sd, "Could not sync with CCDs, check hardware [ %s]\n",report);
	else
		v4l2_info(&priv->sd, "On sync with CCDs [ %s]\n",report);

	return ret;
}

int qtec_ccd_fg_sync(struct qtec_ccd *priv){
	int ret,chan;
	qtec_ccd_fg_setup_windows(priv);
	for (chan=0;chan<priv->n_ccd;chan++){
		ret=qtec_ccd_fg_find_eye(priv,chan,1);
		if (ret<0)
			return ret;
	}

	return 0;
}

int qtec_ccd_fg_stop(struct qtec_ccd *priv){
	qtec_ccd_write_fg(priv, CONTROL,0);
	qtec_ccd_write_fg(priv,STAT,BIT(IRQ_ACK));

	return 0;
}

int qtec_ccd_fg_start(struct qtec_ccd *priv){
	uint32_t reg;
	unsigned long expiration=jiffies+HZ;

	qtec_ccd_write_fg(priv,STAT,BIT(IRQ_ACK));

	reg=(priv->n_ccd-1) << N_CHAN;
	reg |=BIT(FSRUN);
	qtec_ccd_write_fg(priv, CONTROL,reg);
	do {
		qtec_ccd_read_fg(priv, STAT,&reg);
		if (reg & BIT(FSLOCK) )
			return 0;
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "Could not wordsync with CCDs, check hardware\n");

	return -1;
}

int qtec_ccd_fg_stream(struct qtec_ccd *priv){
	uint32_t reg;
	qtec_ccd_read_fg(priv, CONTROL,&reg);
	reg |=BIT(STREAM);
	reg |=BIT(IRQ_ENA);
	qtec_ccd_write_fg(priv, CONTROL,reg);
	return 0;
}
