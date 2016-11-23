/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */

#ifndef QTEC_CCD_H
#define QTEC_CCD_H

#define CCD_WIDTH 1024
#define CCD_HEIGHT 768

/*Pixel structure*/
/*ICX204AL page 1 and 9*/
#define SHIFT_END		162
#define HDUMMY			29
#define HBLANK_HEAD		3
#define HBORDER_HEAD		5
#define HACTIVE			CCD_WIDTH
#define HBORDER_TAIL		5
#define HBLANK_TAIL		40
#define HTOTAL			(SHIFT_END+HDUMMY+HBLANK_HEAD+HBORDER_HEAD+HACTIVE+\
								HBORDER_TAIL+HBLANK_TAIL)
#define HDELAY ((HBORDER_TAIL+HBLANK_TAIL)+HCOUNTER_DELAY)
#define HCOUNTER_DELAY 12

#define VVSG		1
#define VDUMMY		1
#define VBLANK_HEAD	7
#define VBORDER_HEAD	5
#define VACTIVE_MAX	CCD_HEIGHT
#define VBORDER_TAIL	6
#define VBLANK_TAIL	2
#define VTOTAL_MAX	(VVSG+VDUMMY+VBLANK_HEAD+VBORDER_HEAD+VACTIVE_MAX+\
			VBORDER_TAIL+VBLANK_TAIL)

#define TIMEGEN_MAX_LINES 0xffff

#define CALIB_0DB 3051

#define MAX_READOUTS 9
#define LINE_SKIP 5
enum qtec_ccd_readout_mode {NORMAL=0, SKIP};
struct qtec_ccd_readout{
	unsigned int n;
	struct {
		enum qtec_ccd_readout_mode mode;
		unsigned int len;
	} read[MAX_READOUTS];
};

enum channel_names {CHAN_RED=0,CHAN_GREEN,CHAN_BLUE,CHAN_IR1,CHAN_IR2,CHAN_ALL};
#define MAX_CCD CHAN_ALL

enum mode_trig {SELF_TIMED=0, EXT_TRIG, EXT_EXPOSURE};

#define MAX_CROP 8
struct qtec_ccd{
	struct v4l2_subdev sd; //NEEDS to be first!!!!

	struct platform_device *pdev;
	void __iomem *iomem_fg;
	void __iomem *iomem_trig;
	void __iomem *iomem_timegen;
	void __iomem *iomem_bayer;
	struct spi_device *ccd_spi[MAX_CCD+1];

	struct work_struct fg_error_wk;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	   *hblank;
	struct v4l2_ctrl	   *vblank;
	struct v4l2_ctrl *exp_time[MAX_CCD];
	struct v4l2_ctrl *cds[MAX_CCD];
	struct v4l2_ctrl *vga_gain[MAX_CCD];
	struct v4l2_ctrl *trig_mode;
	struct v4l2_ctrl *trig_pol;
	struct v4l2_ctrl *flash_pol;
	struct v4l2_ctrl *ext_trig_delay;
	struct v4l2_ctrl *manual_trigger;
	struct v4l2_ctrl *sensor_type;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop[MAX_CROP];
	int n_crop;
	struct v4l2_fract fival;
	struct qtec_ccd_readout readout;
	int dout_phase[MAX_CCD];

	uint32_t pixel_clk;
	uint32_t bus_clk;

	int n_ccd;
	bool is_bayer_chip;

	bool streaming;
};

//main
int qtec_ccd_readout_lines(struct qtec_ccd *priv);
int qtec_ccd_write_timegen(struct qtec_ccd *priv, uint32_t offset, uint32_t value);
int qtec_ccd_read_timegen(struct qtec_ccd *priv, uint32_t offset, uint32_t *value);
int qtec_ccd_write_ad(struct qtec_ccd *priv, int channel, uint16_t reg, uint32_t value);
int qtec_ccd_write_fg(struct qtec_ccd *priv, uint32_t offset, uint32_t value);
int qtec_ccd_read_fg(struct qtec_ccd *priv, uint32_t offset, uint32_t *value);
int qtec_ccd_write_bayer(struct qtec_ccd *priv, uint32_t offset, uint32_t value);
int qtec_ccd_read_bayer(struct qtec_ccd *priv, uint32_t offset, uint32_t *value);
int qtec_ccd_write_trig(struct qtec_ccd *priv, uint32_t offset, uint32_t value);
int qtec_ccd_read_trig(struct qtec_ccd *priv, uint32_t offset, uint32_t *value);

//tg
int qtec_ccd_tg_start(struct qtec_ccd *priv);
int qtec_ccd_tg_stop(struct qtec_ccd *priv);
int qtec_ccd_tg_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl);

//ad
int qtec_ccd_ad_start(struct qtec_ccd *priv);
int qtec_ccd_ad_stop(struct qtec_ccd *priv);
int qtec_ccd_ad_mode_normal(struct qtec_ccd *priv);
int qtec_ccd_ad_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl);
int qtec_ccd_ad_dout_phase(struct qtec_ccd *priv,int chan,int doutphase);

//fg
int qtec_ccd_fg_calibrate_doutphase(struct qtec_ccd *priv);
int qtec_ccd_fg_sync(struct qtec_ccd *priv);
int qtec_ccd_fg_stop(struct qtec_ccd *priv);
int qtec_ccd_fg_start(struct qtec_ccd *priv);
int qtec_ccd_fg_stream(struct qtec_ccd *priv);

//bayer
int qtec_ccd_bayer_start(struct qtec_ccd *priv);
int qtec_ccd_bayer_stop(struct qtec_ccd *priv);

//trigger
int qtec_ccd_trig_ctrl(struct qtec_ccd *priv,struct v4l2_ctrl *ctrl);
int qtec_ccd_trig_stop(struct qtec_ccd *priv);
int qtec_ccd_trig_start(struct qtec_ccd *priv);

#endif //QTEC_CCD_H
