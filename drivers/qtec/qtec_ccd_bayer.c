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

#define N_SECTOR 8

//Registers
#define SECTOR_BASE 0
#define ALIGN_COL 12
#define ALIGN_ROW 13
#define ENABLE 31

int qtec_ccd_bayer_stop(struct qtec_ccd *priv){
	int i;

	for (i=0;i<N_SECTOR;i++)
		qtec_ccd_write_bayer(priv,SECTOR_BASE+i*4 ,0);

	return 0;
}

int qtec_ccd_bayer_start(struct qtec_ccd *priv){
	int i;
	uint32_t reg;

	if (!((priv->n_ccd==1) && ( priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB)))
		return qtec_ccd_bayer_stop(priv);

	for (i=0;i<priv->n_crop;i++){
		reg = priv->crop[i].height+2;
		if ((priv->crop[i].left&1))
			reg |= BIT(ALIGN_COL);
		if ((priv->crop[i].top&1)==0)
			reg |= BIT(ALIGN_ROW);
		reg |= BIT(ENABLE);
		qtec_ccd_write_bayer(priv, SECTOR_BASE+i*4,reg);
	}

	for (;i<N_SECTOR;i++)
		qtec_ccd_write_bayer(priv, SECTOR_BASE+i*4,0xfff);

	return 0;
}

