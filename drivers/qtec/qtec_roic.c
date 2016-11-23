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
#include <linux/mtd/mtd.h>
#include "../mtd/mtdcore.h"
#include <linux/i2c.h>
#include <linux/io.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define DRIVER_NAME "qtec_roic"
#define DEF_FIVAL {1,24} //24fps

#define MAX_CROP 1

#define CTRL 0x0
#define RUN 0
#define SENSOR_TYPE 1
#define PHASE 2
#define IF_CH 3
#define SPI_BUSY 5
#define MODE_TRIG 6
#define TRIG_POL 8
#define TRIG_ENA 9
#define TRIG_SW 10
#define FLASH_POL 11
#define FB_ENABLE 12
#define XFLIP 13
#define FB_RESET 14
#define TRIG_OVERFLOW 15
#define FR_ERROR 16
#define WSLOCK 17

#define HPARAM 0x4
#define HSTART 16
#define HEND 0

#define VPARAM 0x8
#define VSTART 16
#define VEND 0

#define TOTALS 0xc
#define HTOTAL 12
#define VTOTAL 0

#define FRAME_DELAY 0x10

#define EXPOSURE 0x14

#define FRAME_CNT 0x18

#define SPI_WORD0 0x1c
#define SPI_WORD1 0x20

#define GPIO 0x24
#define TEMP 0x28

#define HBLANK 17

#define HTOTAL_EXTRA_MB (10+17) //10 extra found experimentaly
#define HTOTAL_EXTRA_IG (16)
#define VTOTAL_EXTRA_IG (2)
#define VTOTAL_EXTRA_MB (1)

#define MIN_CLK_AD   2500000
#define CLK_AD_SLOW  5000000
#define MAX_CLK_IG   5000000
#define MAX_CLK_MB  14000000

enum {LWIR=0,SENSOR};
enum mode_trig {SELF_TIMED=0, EXT_TRIG, EXT_EXPOSURE};
enum {MODE_SELF_TIMED=0, MODE_TRIG_DELAY=1, MODE_TRIG_NODELAY=2, MODE_EXT_EXPOSURE=3};

struct qtec_roic{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct platform_device *pdev;
	void __iomem *iomem;
	void __iomem *iomem_pll;
	struct spi_device *dac_lwir;
	struct spi_device *dac_sensor;
	struct spi_device *ad_lwir;
	struct mtd_info *flash_lwir;

	struct work_struct work;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	   *mode_trig;
	struct v4l2_ctrl	   *flash_pol;
	struct v4l2_ctrl	   *trig_pol;
	struct v4l2_ctrl	   *exp;
	struct v4l2_ctrl	   *manual_trigger;
	struct v4l2_ctrl	   *ext_trig_delay;
	struct v4l2_ctrl	   *hflip;
	struct v4l2_ctrl	   *vflip;
	struct v4l2_ctrl	   *hblank;
	struct v4l2_ctrl	   *vblank;
	struct v4l2_ctrl	   *sensor_type;
	struct v4l2_ctrl	   *gain;
	struct v4l2_ctrl	   *volt_gok;
	struct v4l2_ctrl	   *volt_vsk;
	struct v4l2_ctrl	   *volt_gfid;
	struct v4l2_ctrl	   *volt_detcom;
	struct v4l2_ctrl	   *volt_ad_gain;
	struct v4l2_ctrl	   *volt_ad_offset;
	struct v4l2_ctrl	   *temp;
	struct v4l2_ctrl	   *trigger_overflow;
	struct v4l2_ctrl	   *itr;
	struct v4l2_ctrl	   *imro;
	struct v4l2_ctrl	   *ndro;
	struct v4l2_ctrl	   *re;
	struct v4l2_ctrl	   *rst;
	struct v4l2_ctrl	   *oe_en;
	struct v4l2_ctrl	   *pw;
	struct v4l2_ctrl	   *i;
	struct v4l2_ctrl	   *ap;
	struct v4l2_ctrl	   *bw;
	struct v4l2_ctrl	   *ts;
	struct v4l2_ctrl	   *auto_off;
	struct v4l2_ctrl	   *pclk;
	struct v4l2_ctrl	   *vtemp_1;
	struct v4l2_ctrl	   *vtemp_2;
	struct v4l2_ctrl	   *vtemp_pin;
	struct v4l2_ctrl           *dead_pixels;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop[MAX_CROP];
	int n_crop;
	struct v4l2_fract fival;
	bool streaming;

	uint32_t bus_clk;
	uint32_t pixel_clk;
	uint32_t base_clk;
	uint32_t max_clk;

	bool pll_7series;

	bool is_ingas_chip;
};

static inline int qtec_roic_write(struct qtec_roic *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "fg W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem+offset);
	return 0;
}

static inline int qtec_roic_read(struct qtec_roic *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "fg R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static inline int qtec_roic_write_pll(struct qtec_roic *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "pll W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_pll+offset);
	return 0;
}

static inline int qtec_roic_read_pll(struct qtec_roic *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_pll+offset);
	dev_dbg(&priv->pdev->dev, "pll R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#define PLL_CTRL (0x1f*4)
#define PLL_CTRL7 (0xff*4)
#define PLL_RESET (BIT(31)|BIT(0))
#define PLL_LOCK 31
#define PLL_AUX_LOCK 30
static int qtec_roic_stop_pll(struct qtec_roic *priv){

	qtec_roic_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);

	return 0;
}

static int qtec_roic_start_pll(struct qtec_roic *priv){
	unsigned long expiration;
	uint32_t aux;

	qtec_roic_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,0);
	expiration=jiffies+HZ;
	do{
		qtec_roic_read_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,&aux);
		if ((aux&BIT(PLL_LOCK))&&(aux&BIT(PLL_AUX_LOCK)))
			return 0;
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "Timeout waiting for pll lock 0%x\n", aux);

	return -EIO;
}

#define PLLBIT(reg,bit,value) if (value) reg|=1<<bit; else reg&=~(1<<bit)
#define MAX_DIV 64
#define MAX_PHASE 511
//1cycle is 8*div
static int qtec_roic_set_clk_div_phase(struct qtec_roic *priv,int div,int div2,int phase0,int phase2){
	uint32_t aux;
	int high_time,low_time;
	int edge;

	if (div>MAX_DIV || div2>MAX_DIV){
		dev_err(&priv->pdev->dev, "Invalid dividers %d %d. Abort!!!!!\n",div*2, div2*2);
		return -EINVAL;
	}

	if ( phase0>MAX_PHASE || phase2>MAX_PHASE ){
		dev_err(&priv->pdev->dev, "Invalid phases %d %d. Abort!!!!!\n",phase0, phase2);
		return -EINVAL;
	}


	priv->pixel_clk=priv->base_clk/(div*2);
	dev_info(&priv->pdev->dev, "Setting pixel clk to %d with div %dx phase0 %d phase2 %d\n",priv->pixel_clk,div,phase0,phase2);

	qtec_roic_stop_pll(priv);

	//CLK0
	edge= ((((50*div)%100)*2)+50)/100;
	high_time=div/2;
	low_time=div-high_time;
	if (priv->pll_7series){
		qtec_roic_read_pll(priv,0x8*4,&aux);
		aux  &= BIT(12);
		aux |= (phase0 & 0x7) << 13;
		aux |= high_time << 6;
		aux |= low_time;
		qtec_roic_write_pll(priv,0x8*4,aux);
		qtec_roic_read_pll(priv,0x9*4,&aux);
		aux  &= ~0x3ff;
		aux |= edge << 7;
		aux |= (phase0 >> 3) & 0x3f;
		qtec_roic_write_pll(priv,0x9*4,aux);
	}
	else{

		qtec_roic_read_pll(priv,0x5*4,&aux);
		PLLBIT(aux,15,phase0&BIT(3+5));
		PLLBIT(aux,13,phase0&BIT(3+4));
		PLLBIT(aux,11,phase0&BIT(3+2));
		PLLBIT(aux,10,phase0&BIT(3+3));
		PLLBIT(aux,9,phase0&BIT(3+1));
		PLLBIT(aux,8,phase0&BIT(3+0));
		qtec_roic_write_pll(priv,0x5*4,aux);

		qtec_roic_read_pll(priv,0x6*4,&aux);
		PLLBIT(aux,2,edge);
		qtec_roic_write_pll(priv,0x6*4,aux);

		qtec_roic_read_pll(priv,0x9*4,&aux);
		PLLBIT(aux,13,phase0&BIT(1));
		PLLBIT(aux,12,phase0&BIT(2));
		qtec_roic_write_pll(priv,0x9*4,aux);

		qtec_roic_read_pll(priv,0x10*4,&aux);
		PLLBIT(aux,14,high_time&BIT(3));
		PLLBIT(aux,13,high_time&BIT(5));
		PLLBIT(aux,12,high_time&BIT(4));
		qtec_roic_write_pll(priv,0x10*4,aux);

		qtec_roic_read_pll(priv,0x11*4,&aux);
		PLLBIT(aux,3,high_time&BIT(0));
		PLLBIT(aux,1,high_time&BIT(2));
		PLLBIT(aux,0,high_time&BIT(1));
		PLLBIT(aux,2,phase0&BIT(0));
		qtec_roic_write_pll(priv,0x11*4,aux);

		qtec_roic_read_pll(priv,0xb*4,&aux);
		PLLBIT(aux,15,low_time&BIT(5));
		PLLBIT(aux,9,low_time&BIT(4));
		qtec_roic_write_pll(priv,0xb*4,aux);

		qtec_roic_read_pll(priv,0xd*4,&aux);
		PLLBIT(aux,6,low_time&BIT(3));
		PLLBIT(aux,5,low_time&BIT(0));
		PLLBIT(aux,4,low_time&BIT(2));
		qtec_roic_write_pll(priv,0xd*4,aux);

		qtec_roic_read_pll(priv,0xf*4,&aux);
		PLLBIT(aux,2,low_time&BIT(1));
		qtec_roic_write_pll(priv,0xf*4,aux);
	}

	//CLK2
	edge= ((((50*div2)%100)*2)+50)/100;
	high_time=div2/2;
	low_time=div2-high_time;
	if (priv->pll_7series){
		qtec_roic_read_pll(priv,0xc*4,&aux);
		aux  &= BIT(12);
		aux |= (phase2 & 0x7) << 13;
		aux |= high_time << 6;
		aux |= low_time;
		qtec_roic_write_pll(priv,0xc*4,aux);
		qtec_roic_read_pll(priv,0xd*4,&aux);
		aux  &= ~0x3ff;
		aux |= edge << 7;
		aux |= (phase2 >> 3) & 0x3f;
		qtec_roic_write_pll(priv,0xd*4,aux);
	}
	else{
		qtec_roic_read_pll(priv,0x8*4,&aux);
		PLLBIT(aux,13,low_time&BIT(5));
		PLLBIT(aux,10,low_time&BIT(4));
		PLLBIT(aux,9,low_time&BIT(3));
		PLLBIT(aux,8,low_time&BIT(2));
		PLLBIT(aux,7,low_time&BIT(0));
		PLLBIT(aux,15,phase2&BIT(2));
		PLLBIT(aux,12,phase2&BIT(1));
		PLLBIT(aux,6,phase2&BIT(3+5));
		PLLBIT(aux,5,phase2&BIT(3+3));
		PLLBIT(aux,4,phase2&BIT(3+4));
		PLLBIT(aux,3,phase2&BIT(3+1));
		PLLBIT(aux,2,phase2&BIT(3+2));
		PLLBIT(aux,1,phase2&BIT(3+0));
		qtec_roic_write_pll(priv,0x8*4,aux);

		qtec_roic_read_pll(priv,0x9*4,&aux);
		PLLBIT(aux,9,high_time&BIT(4));
		PLLBIT(aux,7,high_time&BIT(3));
		PLLBIT(aux,6,high_time&BIT(2));
		PLLBIT(aux,5,high_time&BIT(0));
		PLLBIT(aux,4,high_time&BIT(1));
		PLLBIT(aux,3,edge);
		qtec_roic_write_pll(priv,0x9*4,aux);

		qtec_roic_read_pll(priv,0x11*4,&aux);
		PLLBIT(aux,7,high_time&BIT(5));
		PLLBIT(aux,6,low_time&BIT(1));
		PLLBIT(aux,2,phase2&BIT(0));
		qtec_roic_write_pll(priv,0x11*4,aux);
	}

	return qtec_roic_start_pll(priv);
}

static int qtec_roic_set_clk_freq(struct qtec_roic *priv,int freq, int degrees){
	int max_clk;
	int div,div2,phase2;

	max_clk=freq;
	max_clk*=2;
	div=(priv->base_clk+(max_clk-1))/max_clk;
	div2 = div*2;

	//1cycle is 8*div
	phase2=(8*div2*degrees)/360;
	return qtec_roic_set_clk_div_phase(priv,div,div*2,0,phase2);
}

static int qtec_roic_set_clk(struct qtec_roic *priv){

	if (priv->is_ingas_chip)
		return qtec_roic_set_clk_freq(priv,MAX_CLK_IG,0);
	else
		return qtec_roic_set_clk_freq(priv,MAX_CLK_MB,0);
}

static inline int qtec_roic_write_dac(struct qtec_roic *priv, int chip,  uint8_t cmd, uint8_t addr, uint32_t value){
	uint8_t txbuffer[4];
	struct spi_transfer transfer ={
		.tx_buf = txbuffer,
		.rx_buf = NULL,
		.bits_per_word=8,
		.len=sizeof(txbuffer),
	};
	struct spi_message message;
	struct spi_device *spidev;

	dev_dbg(&priv->pdev->dev, "dac %d: cmd 0x%.2x addr 0x%.2x val 0x%.3x\n",chip,cmd,addr,value);

	txbuffer[0]=cmd&0xf;
	txbuffer[1]=(addr&0xf)<<4;
	txbuffer[1]|=(value>>16)&0xf;
	txbuffer[2]=(value>>8)&0xff;
	txbuffer[3]=value&0xff;

	spidev=(chip==LWIR)?priv->dac_lwir:priv->dac_sensor;
	spi_message_init(&message);
	spi_message_add_tail(&transfer,&message);
	return spi_sync(spidev,&message);
}

static inline int __qtec_roic_write_ad(struct qtec_roic *priv, uint16_t addr, uint8_t value){
	uint8_t txbuffer[3];
	struct spi_transfer transfer ={
		.tx_buf = txbuffer,
		.rx_buf = NULL,
		.bits_per_word=8,
		.len=sizeof(txbuffer),
	};
	struct spi_message message;

	dev_dbg(&priv->pdev->dev, "ad W:0x%.2x 0x%.2x\n",addr,value);

	txbuffer[0]=(addr>>8)&0x1f;
	txbuffer[1]=addr&0xff;
	txbuffer[2]=value&0xff;

	spi_message_init(&message);
	spi_message_add_tail(&transfer,&message);
	return spi_sync(priv->ad_lwir,&message);
}

static inline int qtec_roic_write_ad(struct qtec_roic *priv, uint16_t addr, uint8_t value){
	int ret;

	ret=__qtec_roic_write_ad(priv, addr, value);
	if (ret)
		return ret;
	return __qtec_roic_write_ad(priv, 0xff, 1);//transfer
}

static int qtec_roic_pga_chip(struct qtec_roic *priv,int *req_val,int *chip_gain);

static int qtec_roic_wait_frame(struct qtec_roic *priv){
	uint32_t framecnt;
	uint32_t aux;
	unsigned long expiration;

	qtec_roic_read(priv,FRAME_CNT,&framecnt);
	expiration=jiffies+(HZ/5);
	do{
		qtec_roic_read(priv,FRAME_CNT,&aux);
		if (framecnt!=aux)
			return 0;
		msleep(100);
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "Timeout waiting frame 0x%8x\n",aux);

	return 1;
}

static inline int qtec_roic_send_serial_link_ingas_geom(struct qtec_roic *priv){
	uint32_t cfg;
	unsigned long expiration;
	uint32_t aux;

	cfg=BIT(31); //Start
	cfg|=BIT(30); //Win
	cfg |= (priv->crop[0].left/8)<<(22+2);
	cfg |= (priv->crop[0].top/2)<<15;
	cfg |= (priv->crop[0].width/8)<<(7+2);
	cfg |= ((priv->crop[0].height/2)-1)<<0;
	dev_dbg(&priv->pdev->dev, "Sending SERDAT 0x%x\n",cfg);
	qtec_roic_write(priv,SPI_WORD0,0);
	qtec_roic_write(priv,SPI_WORD1,cfg);//change to word1
	expiration=jiffies+HZ;
	do{
		qtec_roic_read(priv,CTRL,&aux);
		if (!(aux&BIT(SPI_BUSY)))
			break;
	}while(time_before(jiffies,expiration));

	return 0;
}

static inline int qtec_roic_send_serial_link_ingas(struct qtec_roic *priv){
	uint32_t cfg;
	unsigned long expiration;
	uint32_t aux;
	int chip_pga;

	qtec_roic_pga_chip(priv,&priv->gain->val,&chip_pga);

	cfg=BIT(31); //Start
	cfg|=(priv->itr->val)<<29;
	cfg|=(chip_pga)<<28;
	cfg|=(priv->pw->val)<<26;
	cfg|=(priv->i->val)<<23;
	cfg|=(priv->ap->val)<<20;
	cfg|=(priv->bw->val)<<18;
	cfg|=(priv->imro->val)<<17;
	cfg|=(priv->ndro->val)<<16;
	cfg|=(priv->ts->val)<<9;
	cfg|=(priv->re->val)<<2;
	cfg|=(priv->rst->val)<<1;
	cfg|=(priv->oe_en->val)<<0;
	cfg|=(priv->hflip->val)?BIT(7):0;
	cfg|=(priv->vflip->val)?BIT(6):0;
	cfg|=BIT(4);//4chan
	cfg|=BIT(3);//4chan
	dev_dbg(&priv->pdev->dev, "Sending SERDAT 0x%x\n",cfg);
	qtec_roic_write(priv,SPI_WORD0,0);
	qtec_roic_write(priv,SPI_WORD1,cfg);//Change to word1
	expiration=jiffies+HZ;
	do{
		qtec_roic_read(priv,CTRL,&aux);
		if (!(aux&BIT(SPI_BUSY)))
			break;
	}while(time_before(jiffies,expiration));

	return 0;
}

static inline int qtec_roic_send_serial_link_mb(struct qtec_roic *priv){
	uint64_t cfg=0;
	int chip_pga;
	unsigned long expiration;
	uint32_t aux;

	qtec_roic_pga_chip(priv,&priv->gain->val,&chip_pga);

	cfg|=BIT(64-1);//START
	cfg|=((uint64_t)4)<<(64-5);//SELQ
	cfg|=((uint64_t)1)<<(64-6);//Reserved
	cfg|=((uint64_t)chip_pga)<<(64-12);//gain
	if (priv->hflip->val){
		cfg|=((uint64_t)(priv->crop[0].left/2))<<(64-54);
		cfg|=((uint64_t)((priv->crop[0].left+priv->crop[0].width)/2-1))<<(64-45);
	}
	else{
		cfg|=(uint64_t)1<<(64-13);//upcol
		cfg|=((uint64_t)(priv->crop[0].left/2))<<(64-45);
		cfg|=((uint64_t)((priv->crop[0].left+priv->crop[0].width)/2-1))<<(64-54);
	}
	if (priv->vflip->val){
		cfg|=((uint64_t)priv->crop[0].top)<<(64-36);
		cfg|=((uint64_t)(priv->crop[0].top+priv->crop[0].height-1))<<(64-26);
	}
	else{
		cfg|=(uint64_t)1<<(64-14);//row
		cfg|=((uint64_t)priv->crop[0].top)<<(64-26);
		cfg|=((uint64_t)(priv->crop[0].top+priv->crop[0].height-1))<<(64-36);
	}

	dev_dbg(&priv->pdev->dev, "Sending SERDAT 0x%llx\n",(unsigned long long)cfg);
	qtec_roic_write(priv,SPI_WORD0,cfg&0xffffffff);
	qtec_roic_write(priv,SPI_WORD1,(cfg>>32)&0xffffffff);

	expiration=jiffies+HZ;
	do{
		qtec_roic_read(priv,CTRL,&aux);
		if (!(aux&BIT(SPI_BUSY)))
			return 0;
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "Timeout sending serial data 0x%llx\n",(unsigned long long)cfg);

	return -1;
}

static inline int qtec_roic_send_serial_link(struct qtec_roic *priv){
	if (priv->is_ingas_chip)
		return qtec_roic_send_serial_link_ingas(priv);
	else
		return qtec_roic_send_serial_link_mb(priv);
}

static int qtec_roic_init_ad(struct qtec_roic *priv){
	static const uint16_t ad_values[][2]={
		{0x5,0x3f}, //Write to all channels
		{0x8,0x1}, //chip off
		{0x8,0x3}, //rst chip
		{0x8,0x0}, //start chip
		{0x21,0x4},//14 bits
		{0x14,0x0}, //Non invert
		{0xd,0x0},//No test pattern
		{0x15,0x30},//100 ohm
		{0x22,0x0},//Start channels
	};
	static const uint16_t mb_values[][2]={
		{0x5,0xc}, //Channels 2 and 3
		{0x22,0x1},//Power off
		{0x5,0x3}, //Channels 0 and 1
		//{0x14,0x4},//Invert Temp workaround #546
		{0x5,0x33}, //Write to other channels
	};
	int i;

	for (i=0;i<ARRAY_SIZE(ad_values);i++)
		if (qtec_roic_write_ad(priv,ad_values[i][0],ad_values[i][1]))
			return -1;

	if (!priv->is_ingas_chip)
		for (i=0;i<ARRAY_SIZE(mb_values);i++)
			if (qtec_roic_write_ad(priv,mb_values[i][0],mb_values[i][1]))
				return -1;

	return 0;
}

static int qtec_roic_ad_slow(struct qtec_roic *priv){
	qtec_roic_write_ad(priv,0x21,0xc);
	return 0;
}

static int qtec_roic_ad_normal(struct qtec_roic *priv){
	qtec_roic_write_ad(priv,0x21,0x4);
	return 0;
}

static int qtec_roic_ad_start(struct qtec_roic *priv){
	qtec_roic_write_ad(priv,0x22,0x0);
	return 0;
}
static int qtec_roic_ad_stop(struct qtec_roic *priv){
	qtec_roic_write_ad(priv,0x22,0x1);
	return 0;
}

static int qtec_roic_init_fg(struct qtec_roic *priv){
	//Set sensor type and wake up from tristate
	qtec_roic_write(priv,CTRL,(priv->is_ingas_chip)?0:BIT(SENSOR_TYPE));
	return 0;
}

static int qtec_roic_dac_write_array(struct qtec_roic *priv, int dac, const uint32_t values[][3], int len){
	int i;
	for (i=0;i<len;i++)
		if (qtec_roic_write_dac(priv,dac,values[i][0],values[i][1],values[i][2]))
			return -1;
	return 0;
}

static int qtec_roic_init_voltages(struct qtec_roic *priv){
	static const uint32_t dac_init[][3]={
		{5,0,0}, //rst val =0
		{7,0,0}, //rst
		{8,0,0}, //int_ref =0
	};
	static const uint32_t lwir_dac_ingas[][3]={
		//{4,0,0x3f0},//poweroff 4-7
		{0,0,0xfffff},//5.5 v
		{0,1,0xfffff},//5.5 v
		{0,2,0x8cccd},//2.75 v (Finn)
		{0,3,0x4072c},//0.5035v v //Considering that the multiplier is 0.8 #390
		{0,4,0},//0v
		{0,5,0},//0v
		{0,6,0},//0v
		{0,7,0},//0v
	};
	static const uint32_t lwir_dac_mb[][3]={
		//{4,0,0x3f0},//poweroff 4-7
		{0,0,0xa7905},//3.6 v
		{0,1,0xa7905},//3.6 v
		{0,2,0x47b00},//1.4v (from Finn)
		{0,3,0x2b800},//0.425 v //Considering that the multiplier is 0.8 #390
		{0,4,0},//0v
		{0,5,0},//0v
		{0,6,0},//0v
		{0,7,0},//0v
	};
	static const uint32_t sensor_dac_mb[][3]={
		//{4,0,0x394},//poweroff 2,4,7
		{0,0,0x9999a},//3.6v
		{0,1,0x40000},//1.5v
		{0,2,0},//0v
		{0,3,0xc4fdf},//4.617v
		{0,4,0},//0v
		{0,5,0x80000},//3.0v
		{0,6,0x62222},//2.3v
		{0,7,0},//0v
	};
	static const uint32_t sensor_dac_ingas[][3]={
		//{4,0,0x3f7},//poweroff all except 3
		{0,0,0},
		{0,1,0},
		{0,2,0},
		{0,3,0xad3a},
		{0,4,0},
		{0,5,0},
		{0,6,0},
		{0,7,0},
	};

	//Enable ADs power
	qtec_roic_write(priv,GPIO,BIT(0));

	qtec_roic_dac_write_array(priv,LWIR,dac_init,ARRAY_SIZE(dac_init));
	qtec_roic_dac_write_array(priv,SENSOR,dac_init,ARRAY_SIZE(dac_init));

	if (priv->is_ingas_chip){
		qtec_roic_dac_write_array(priv,LWIR,lwir_dac_ingas,ARRAY_SIZE(lwir_dac_ingas));
		qtec_roic_dac_write_array(priv,SENSOR,sensor_dac_ingas,ARRAY_SIZE(sensor_dac_ingas));
	}
	else{
		qtec_roic_dac_write_array(priv,LWIR,lwir_dac_mb,ARRAY_SIZE(lwir_dac_mb));
		qtec_roic_dac_write_array(priv,SENSOR,sensor_dac_mb,ARRAY_SIZE(sensor_dac_mb));
	}

	qtec_roic_write_dac(priv,LWIR, 6 ,0,0xff); //LDAC
	qtec_roic_write_dac(priv,SENSOR, 6 ,0,0xff); //LDAC

	return 0;
}

#define GOK 0
#define VSK 3
#define GFID 5
#define VDETCOM 3
#define ADOFFSET 2
#define ADGAIN 3
static int qtec_roic_set_voltages(struct qtec_roic *priv,int output, uint32_t value){
	uint64_t aux;
	int chip;

	switch (output){
		case QTEC_ROIC_CID_VOLT_GOK:
		case QTEC_ROIC_CID_VOLT_VSK:
		case QTEC_ROIC_CID_VOLT_GFID:
			aux= (uint64_t) value * (uint64_t) 0x100000; //Resolution of the ad
			aux += 3000;
			do_div (aux,6000); // mvolt to volt * 2.4 *2.5
			break;
		case QTEC_ROIC_CID_VOLT_VDETCOM:
			aux = value-4669;
			aux*=1430525;
			aux/=1000;
			//0 is 4669
			//0xfffff is 5402
			break;
		case QTEC_ROIC_CID_VOLT_AD_OFFSET:
			aux= (uint64_t) value * (uint64_t) 0x100000; //Resolution of the ad
			aux += 2500;
			do_div (aux,5000); // mvolt to volt * 2 *2.5
			break;
		case QTEC_ROIC_CID_VOLT_AD_GAIN:
			aux= (uint64_t) value * (uint64_t) 0x100000; //Resolution of the ad
			aux += 900;
			do_div (aux,1800); // mvolt to volt * 0.72 *2.5
			break;
		default:
			return -1;
	}
	switch (output){
		case QTEC_ROIC_CID_VOLT_GOK:
			chip=SENSOR;output=GOK; break;
		case QTEC_ROIC_CID_VOLT_VSK:
			chip=SENSOR;output=VSK; break;
		case QTEC_ROIC_CID_VOLT_GFID:
			chip=SENSOR;output=GFID; break;
		case QTEC_ROIC_CID_VOLT_VDETCOM:
			chip=SENSOR;output=VDETCOM; break;
		case QTEC_ROIC_CID_VOLT_AD_OFFSET:
			chip=LWIR;output=ADOFFSET; break;
		case QTEC_ROIC_CID_VOLT_AD_GAIN:
			chip=LWIR;output=ADGAIN; break;
	}


	value=aux;
	value=clamp_t(uint32_t, value,0,0xfffff);
	qtec_roic_write_dac(priv,chip,0,output,value);
	return qtec_roic_write_dac(priv,chip, 6 ,0,0xff); //LDAC
}

#define MIN_FRAME_DELAY 2
static int qtec_roic_set_ext_trig_delay(struct qtec_roic *priv,uint32_t time){
	uint64_t delay64;

	delay64=(uint64_t )time * (uint64_t)priv->pixel_clk;
	do_div(delay64,1000000);
	delay64=clamp(delay64,(uint64_t)MIN_FRAME_DELAY,(uint64_t)0xffffffff);

	qtec_roic_write(priv,FRAME_DELAY,delay64);
	return 0;
}

static int _qtec_roic_set_frame_delay(struct qtec_roic *priv,struct v4l2_fract fival){
	uint64_t delay64;
	delay64=(uint64_t) fival.numerator * (uint64_t) priv->pixel_clk;
	do_div(delay64,fival.denominator);
	delay64=clamp(delay64,(uint64_t)MIN_FRAME_DELAY,(uint64_t)0xffffffff);
	qtec_roic_write(priv,FRAME_DELAY,delay64&0xffffffff);

	return 0;
}
static int qtec_roic_set_frame_delay(struct qtec_roic *priv){
	if (v4l2_ctrl_g_ctrl(priv->mode_trig) == SELF_TIMED){
		return _qtec_roic_set_frame_delay(priv,priv->fival);
	}
	else
		return qtec_roic_set_ext_trig_delay(priv,v4l2_ctrl_g_ctrl(priv->ext_trig_delay));

}

static int qtec_roic_min_tpf(struct qtec_roic *priv,
	struct v4l2_mbus_framefmt format, struct v4l2_fract *fival);
static int qtec_roic_set_min_frame_delay(struct qtec_roic *priv){
	struct v4l2_fract interval;
	qtec_roic_min_tpf(priv, priv->format, &interval);
	return _qtec_roic_set_frame_delay(priv,interval);
}

static int _qtec_roic_set_exposure(struct qtec_roic *priv, uint32_t val){
	uint64_t exp64;
	uint32_t exp;

	exp64=(uint64_t)val*(uint64_t) priv->pixel_clk;
	do_div(exp64,1000000);
	exp=exp64&0xffffffff;
	if (!priv->is_ingas_chip){
		exp=(priv->format.width/2)+17-exp; //From Finn
		exp=clamp_t(uint32_t, exp,HBLANK,(priv->format.width/2)+2); // /2 is number of channels +2 is pipeline
	}
	qtec_roic_write(priv,EXPOSURE,exp);

	return 0;
}

static int qtec_roic_set_exposure(struct qtec_roic *priv){
	return _qtec_roic_set_exposure(priv,priv->exp->val);
}

static int qtec_roic_stop(struct qtec_roic *priv){
	qtec_roic_write(priv,CTRL,(priv->is_ingas_chip)?0:BIT(SENSOR_TYPE)|BIT(FB_RESET));
	qtec_roic_write(priv,CTRL,(priv->is_ingas_chip)?0:BIT(SENSOR_TYPE));

	qtec_roic_ad_stop(priv);
	if (priv->auto_off->val)
		qtec_roic_write(priv,GPIO,0);
	priv->streaming=0;

	return 0;
}

static int qtec_roic_update_exposure_range(struct qtec_roic *priv);
static int qtec_roic_clamp_interval(struct qtec_roic *priv,struct v4l2_fract *fival);

static int qtec_roic_try_sync(struct qtec_roic *priv){
	uint32_t aux;
	unsigned long expiration;
	if (priv->is_ingas_chip){
		aux= 2 <<IF_CH;
	}
	else{
		aux =BIT(SENSOR_TYPE);
		aux|=BIT(PHASE);
		aux|= 1 <<IF_CH;
	}
	aux|= MODE_SELF_TIMED << MODE_TRIG;
	aux|=BIT(RUN);
	aux|=BIT(WSLOCK);
	qtec_roic_write(priv,CTRL,aux);
	expiration=jiffies+HZ/500;
	do{
		qtec_roic_read(priv,CTRL,&aux);
		if (aux&BIT(WSLOCK))
			return 1;;
	}while(time_before(jiffies,expiration));
	return 0;
}

#define MAX_EYE 12
#define MAX_TRIES 2
static int qtec_roic_sync(struct qtec_roic *priv){
	char do_sync[MAX_EYE+1];
	int i,j;
	int best_phase =-1;
	int best_len=-1;
	int tries;

	qtec_roic_ad_stop(priv);
	qtec_roic_write(priv,GPIO,BIT(0));
	qtec_roic_ad_start(priv);

	for (i=0;i<MAX_EYE;i++)
		do_sync[i]=0;
	//Try
	for (tries=0;tries<MAX_TRIES;tries++){
		for (i=0;i<MAX_EYE;i++){
			if (do_sync[i]!=tries)
				continue;
			qtec_roic_write_ad(priv,0x16,i);
			do_sync[i]+=qtec_roic_try_sync(priv);
			qtec_roic_write(priv,CTRL,(priv->is_ingas_chip)?0:BIT(SENSOR_TYPE));
		}
	}

	for (i=0;i<MAX_EYE;i++){
		int len;
		if (do_sync[i]!=MAX_TRIES)
			continue;
		len=1;
		for (j=1;j<MAX_EYE;j++){
			if (do_sync[(j+i)%MAX_EYE]!=MAX_TRIES)
				break;
			len++;
		}
		if (len>best_len){
			best_len=len;
			best_phase=i;
		}

	}

	for (i=0;i<MAX_EYE;i++)
		do_sync[i]+='0';
	do_sync[MAX_EYE]='\0';

	if (best_len==-1){
		v4l2_err(&priv->sd, "Could not sync with AD chip, check hardware [%s]\n",do_sync);
		return -1;
	}

	best_phase+=best_len/2;
	best_phase%=MAX_EYE;
	qtec_roic_write_ad(priv,0x16,best_phase);

	v4l2_info(&priv->sd, "Found a %d units lenght eye with center in %d [%s]\n", best_len,best_phase,do_sync);
	return 0;
}

#ifdef SYNC_EXPERIMENT
static int qtec_roic_sync_experiment(struct qtec_roic *priv){
	int div;

	for (div=1;div<=64;div++){
		dev_err(&priv->pdev->dev, "Setting div to %d, %dHz\n",div,priv->base_clk/(div*2));
		qtec_roic_set_clk_div_phase(priv,div,div*2,0);
		qtec_roic_sync(priv);
	}

	return 0;
}
#endif

static int qtec_roic_start(struct qtec_roic *priv){
	uint32_t aux;
	unsigned long expiration;
	int delay;
	uint32_t res;

	qtec_roic_write(priv,GPIO,BIT(0));
	qtec_roic_ad_start(priv);

	//Clamp frame_interval
	qtec_roic_clamp_interval(priv,&priv->fival);

	//Set max and min of exposure
	qtec_roic_update_exposure_range(priv);

	qtec_roic_set_min_frame_delay(priv);
	_qtec_roic_set_exposure(priv,0);

	//Set HPARAM
	delay=(priv->is_ingas_chip)?84:50;
	aux=(0+delay)<<HSTART;
	aux|=(priv->format.width+delay-1)<<HEND;
	qtec_roic_write(priv,HPARAM,aux);

	//Set VPARAM
	delay=(priv->is_ingas_chip)?3:2;
	aux=(0+delay)<<VSTART;
	aux|=(0+priv->format.height+delay-1)<<VEND;
	qtec_roic_write(priv,VPARAM,aux);

	//Set TOTALS
	if (priv->is_ingas_chip){
		aux=((priv->format.width/8)+HTOTAL_EXTRA_IG-1)<<HTOTAL; //8 is number of channels and ddr
		aux|=(priv->format.height+VTOTAL_EXTRA_IG-1)<<VTOTAL;
	}
	else{
		aux=((priv->format.width/2)+HTOTAL_EXTRA_MB-1)<<HTOTAL; //2 is number of channels
		aux|=(priv->format.height+VTOTAL_EXTRA_MB-1)<<VTOTAL;
	}
	qtec_roic_write(priv,TOTALS,aux);

	//ctrl register
	if (priv->is_ingas_chip){
		aux=0;
		aux|= 2 <<IF_CH;
	}
	else{
		aux =BIT(SENSOR_TYPE);
		aux|=BIT(PHASE);
		aux|= 1 <<IF_CH;
	}

	if (priv->hflip->val)
		aux|=BIT(XFLIP);
	aux|= MODE_SELF_TIMED << MODE_TRIG;
	if (v4l2_ctrl_g_ctrl(priv->flash_pol))
		aux|=BIT(FLASH_POL);
	else
		aux&=~BIT(FLASH_POL);

	if (v4l2_ctrl_g_ctrl(priv->trig_pol))
		aux|=BIT(TRIG_POL);
	else
		aux&=~BIT(TRIG_POL);

	aux|=BIT(RUN);
	aux|=BIT(WSLOCK);
	qtec_roic_write(priv,CTRL,aux);

	expiration=jiffies+HZ;
	do{
		qtec_roic_read(priv,CTRL,&res);
		if (res&BIT(WSLOCK))
			break;
	}while(time_before(jiffies,expiration));

	if(!time_before(jiffies,expiration)){
		v4l2_err(&priv->sd, "Timeout word syncing 0x%.8x\n",res);
		qtec_roic_stop(priv);
		return -EINVAL;
	}

	aux|=BIT(TRIG_ENA);
	qtec_roic_write(priv,CTRL,aux);

	if (priv->is_ingas_chip){
		qtec_roic_wait_frame(priv);
		qtec_roic_wait_frame(priv);//Needs two frames!
		qtec_roic_send_serial_link_ingas_geom(priv);
	}

	qtec_roic_wait_frame(priv);
	qtec_roic_send_serial_link(priv);
	switch (priv->mode_trig->val){
		case SELF_TIMED:
			aux|= MODE_SELF_TIMED << MODE_TRIG;
			break;
		case EXT_TRIG:
			aux|= MODE_TRIG_DELAY << MODE_TRIG;
			break;
		case EXT_EXPOSURE:
			aux|= MODE_EXT_EXPOSURE << MODE_TRIG;
			break;
	}

	//Enable fb at real speed
	qtec_roic_set_frame_delay(priv);

	//Exposure
	qtec_roic_set_exposure(priv);

	qtec_roic_write(priv,CTRL,aux);

	//Enable FB, needs to be done in two steps #449
	if (priv->mode_trig->val != SELF_TIMED)
		qtec_roic_wait_frame(priv);
	aux|=BIT(FB_ENABLE);
	qtec_roic_write(priv,CTRL,aux);
	priv->streaming=1;

	return 0;
}

static int qtec_roic_s_stream(struct v4l2_subdev *subdev, int enable){
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);
	if (enable)
		return qtec_roic_start(priv);
	else
		return qtec_roic_stop(priv);
}

static int qtec_roic_max_size(struct qtec_roic *priv,u32 format,
		unsigned int *max_width, unsigned int *max_height){

	if (format!= MEDIA_BUS_FMT_QTEC_LEGACY_MONO)
		return -EINVAL;

	if (priv->is_ingas_chip){
		*max_width=320;
		*max_height=256;
	}
	else{
		*max_width=1024;
		*max_height=768;
	}

	return 0;
}

static int qtec_roic_min_size(struct qtec_roic *priv,u32 format,
		unsigned int *min_width, unsigned int *min_height){

	if (format!= MEDIA_BUS_FMT_QTEC_LEGACY_MONO)
		return -EINVAL;

	if (priv->is_ingas_chip){
		*min_width=128;
		*min_height=8;
	}
	else{
		*min_width=160;
		*min_height=80;
	}
	return 0;
}

static int qtec_roic_enum_mbus_code(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index)
		return -EINVAL;

	code->code= MEDIA_BUS_FMT_QTEC_LEGACY_MONO;

	return 0;
}

static int qtec_roic_try_fmt(struct v4l2_subdev *subdev,struct v4l2_mbus_framefmt *fmt){
	unsigned int max_width=0,max_height=0;
	unsigned int min_width=0,min_height=0;
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);

	if (qtec_roic_max_size(priv,fmt->code,&max_width,&max_height)==EINVAL){
		struct v4l2_subdev_mbus_code_enum code = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.index = 0,
		};
		//Wrong code
		qtec_roic_enum_mbus_code(subdev,NULL,&code);
		fmt->code = code.code;
		qtec_roic_max_size(priv,fmt->code,&max_width,&max_height);
	}

	qtec_roic_min_size(priv,fmt->code,&min_width,&min_height);

	v4l_bound_align_image(&fmt->width, min_width, max_width, (priv->is_ingas_chip)?4:2,
			      &fmt->height, min_height, max_height, (priv->is_ingas_chip)?1:0, 0);
	fmt->colorspace=V4L2_COLORSPACE_SRGB;
	fmt->field=V4L2_FIELD_NONE;

	return 0;
}

static int qtec_roic_enum_fsize(struct v4l2_subdev *subdev, struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);
	unsigned int max_width,max_height;
	unsigned int min_width,min_height;

	if (fse->index!=0)
		return -EINVAL;

	if (qtec_roic_max_size(priv,fse->code,&max_width,&max_height))
		return -EINVAL;

	if (qtec_roic_min_size(priv,fse->code,&min_width,&min_height))
		return -EINVAL;

	fse->min_width=min_width;
	fse->max_width=max_width;
	fse->max_height-=max_width%min_width;
	fse->min_height=min_height;

	return 0;
}

static int qtec_roic_get_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);

	if (format->pad)
		return -EINVAL;

	format->format=priv->format;

	return 0;
}

static int v4l2_ctrl_modify_range_cond(struct v4l2_ctrl *ctrl,
                            s64 min, s64 max, u64 step, s64 def)
{
       if ((ctrl->minimum == min) &&   (ctrl->maximum == max) &&
               (ctrl->step == step) && (ctrl->default_value == def))
               return 0;

       return v4l2_ctrl_modify_range(ctrl,min,max,step,def);
}

static int qtec_roic_update_exposure_range(struct qtec_roic *priv){
	uint32_t min,max,def;
	uint64_t val_64;

	def=priv->exp->default_value;
	if (priv->is_ingas_chip){
		/*Exp valid between 110 pixel clk and T-220*/
		val_64=110*1000000;
		do_div(val_64,priv->pixel_clk);
		min=val_64;
		val_64=(int64_t)priv->fival.numerator * (int64_t)1000000;
		do_div(val_64,priv->fival.denominator);
		max=val_64;
		max-=2*min;
	}
	else{
		/*Exp valid between 15 pixel clk and H*/
		val_64=15*1000000;
		do_div(val_64,priv->pixel_clk);
		min=val_64;
		val_64=(priv->format.width/2)*1000000;
		do_div(val_64,priv->pixel_clk);
		max=val_64;
	}

	if (def>max)
		def=max;
	v4l2_ctrl_modify_range_cond(priv->exp,min,max,1,def);
	return 0;
}

static int qtec_roic_pga_chip_ingas(struct qtec_roic *priv,int *req_val,int *chip_gain){
	int pgadb=*req_val;

	if (pgadb > 13000){//10x
		*chip_gain=0; //4
		*req_val=26000;
	}
	else{
		*chip_gain=1;
		*req_val=0;
	}

	return 0;
}

static int qtec_roic_pga_chip_mb(struct qtec_roic *priv,int *req_val,int *chip_gain){
	int pgadb=*req_val;

	if (pgadb > 9500){//3
		*chip_gain=0; //4
		*req_val=12000;
	}
	else if (pgadb > 4400){//1.66
		*chip_gain=1; //2
		*req_val=6000;
	}
	else if (pgadb > 1300){ //1.166
		*chip_gain=2; //1.33
		*req_val=2300;
	}
	else{
		*chip_gain=3; //1.0
		*req_val=0;
	}

	return 0;
}

static int qtec_roic_pga_chip(struct qtec_roic *priv,int *req_val,int *chip_gain){
	if (priv->is_ingas_chip)
		return qtec_roic_pga_chip_ingas(priv,req_val,chip_gain);
	else
		return qtec_roic_pga_chip_mb(priv,req_val,chip_gain);
}

static int qtec_roic_update_crop(struct qtec_roic *priv){
	int max_left,max_top,min_left=0,min_top=0;

	if (qtec_roic_max_size(priv,priv->format.code,&max_left,&max_top))
		return -EINVAL;
	max_left-=priv->format.width;
	max_top-=priv->format.height;

	priv->n_crop =1;
	priv->crop[0].width=priv->format.width;
	priv->crop[0].height=priv->format.height;
	priv->crop[0].left=clamp(priv->crop[0].left,min_left,max_left);
	priv->crop[0].top=clamp(priv->crop[0].top,min_top,max_top);

	if (priv->is_ingas_chip){
		priv->crop[0].left -= priv->crop[0].left%8;
		priv->crop[0].top -= priv->crop[0].top%2;
	}

	return 0;
}

static int qtec_roic_set_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);

	if (format->pad)
		return -EINVAL;

	qtec_roic_try_fmt(subdev,fmt);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY){
		if (cfg)
			cfg->try_fmt = *fmt;
		return 0;
	}

	if (priv->streaming)
		return -EBUSY;

	priv->format=*fmt;
	qtec_roic_clamp_interval(priv,&priv->fival);
	qtec_roic_update_exposure_range(priv);
	qtec_roic_update_crop(priv);

	return 0;
}

static int qtec_roic_min_tpf(struct qtec_roic *priv,
	struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){

	if (priv->is_ingas_chip)
		fival->numerator=((format.width/8)+HTOTAL_EXTRA_IG)*(format.height+VTOTAL_EXTRA_IG)+1;
	else
		fival->numerator=13+(((format.width/2)+HTOTAL_EXTRA_MB)*(format.height+VTOTAL_EXTRA_MB));//(2 channels)
	fival->denominator=priv->pixel_clk;
	return 0;
}

static int qtec_roic_max_tpf(struct qtec_roic *priv,
		struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){
	fival->numerator=0x7fffffff;
	if (v4l2_ctrl_g_ctrl(priv->mode_trig) == SELF_TIMED)
		fival->denominator=priv->pixel_clk;
	else
		fival->denominator=1;
	return 0;
}

static int qtec_roic_enum_frameintervals(struct v4l2_subdev *subdev, struct v4l2_frmivalenum *fival){
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);
	int ret;
	struct v4l2_mbus_framefmt format;

	if (fival->index!=0)
		return -EINVAL;

	if (fival->pixel_format != MEDIA_BUS_FMT_QTEC_LEGACY_MONO)
		return -EINVAL;

	format.code=fival->pixel_format;
	format.width=fival->width;
	format.height=fival->height;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;

	ret= qtec_roic_min_tpf(priv, format, &fival->stepwise.min);
	if (ret)
		return ret;
	ret= qtec_roic_max_tpf(priv, format, &fival->stepwise.max);
	if (ret)
		return ret;

	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1; //compliance
	//fival->stepwise.step.denominator = priv->pixel_clk;

	return 0;
}

static int qtec_roic_g_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);

	fival->pad=0;
	fival->interval=priv->fival;

	return 0;
}

#define FRACT_CMP(a, OP, b)	\
	((u64)(a).numerator * (b).denominator  OP  (u64)(b).numerator * (a).denominator)
static int qtec_roic_clamp_interval(struct qtec_roic *priv,struct v4l2_fract *fival){
	struct v4l2_fract interval;
	int ret;

	ret=qtec_roic_min_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,<,interval))
		*fival=interval;

	ret=qtec_roic_max_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,>,interval))
		*fival=interval;

	return 0;
}

static int qtec_roic_s_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);


	fival->pad=0;
	if (fival->interval.denominator==0)
		fival->interval=(struct v4l2_fract) DEF_FIVAL;
	qtec_roic_clamp_interval(priv,&fival->interval);
	priv->fival=fival->interval;
	qtec_roic_update_exposure_range(priv);
	qtec_roic_set_frame_delay(priv);
	return 0;
}

static int qtec_roic_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
                            struct v4l2_subdev_selection *s){
	struct qtec_roic *priv = container_of(sd, struct qtec_roic, sd);
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
	qtec_roic_update_crop(priv);
	return 0;
}

static int qtec_roic_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
                            struct v4l2_subdev_selection *s){
	struct qtec_roic *priv = container_of(sd, struct qtec_roic, sd);
	int32_t max_width,max_height;
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
			if (qtec_roic_max_size(priv,priv->format.code,&max_width,&max_height))
				return -EINVAL;
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = max_width;
			s->r.height = max_height;
			s->rectangles = 0;
			return 0;
		default:
			return -EINVAL;
	}
	return 0;
}

static int qtec_roic_init_fmt(struct qtec_roic *priv){
	unsigned int max_width,max_height;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = 0,
	};

	qtec_roic_enum_mbus_code(&priv->sd,NULL,&code);
	format.format.code = code.code;
	qtec_roic_max_size(priv,format.format.code,&max_width,&max_height);
	format.format.width=max_width;
	format.format.height=max_height;
	qtec_roic_set_fmt(&priv->sd, NULL, &format);
	return 0;
}

static uint32_t qtec_roic_max_delay_ext_trig(struct qtec_roic *priv){
	uint64_t delay=0xffffffff;

	delay *= (uint64_t) 1000000;

	do_div(delay,priv->pixel_clk);

	return delay & 0x7fffffff;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_roic_s_register(struct v4l2_subdev *subdev,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);

	if (reg->reg >= 0x3000)
		qtec_roic_write_dac(priv,SENSOR,(reg->reg-0x3000)&0xf,((reg->reg-0x3000)>>4)&0xf,reg->val);
	else if (reg->reg >= 0x2000)
		qtec_roic_write_dac(priv,LWIR,(reg->reg-0x2000)&0xf,((reg->reg-0x2000)>>4)&0xf,reg->val);
	else if (reg->reg >= 0x1000)
		qtec_roic_write_ad(priv,reg->reg-0x1000,reg->val);
	else
		qtec_roic_write(priv,reg->reg,reg->val);

	return 0;
}

static int qtec_roic_g_register(struct v4l2_subdev *subdev,
			      struct v4l2_dbg_register *reg)
{
	struct qtec_roic *priv = container_of(subdev, struct qtec_roic, sd);
	uint32_t val;

	if (reg->reg >= 0x1000)
		return -EINVAL;
	qtec_roic_read(priv,reg->reg,&val);
	reg->val=val;
	reg->size=4;

	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_roic_pad_ops = {
	.get_selection = qtec_roic_get_selection,
	.set_selection = qtec_roic_set_selection,
	.enum_frame_size = qtec_roic_enum_fsize,
	.set_fmt = qtec_roic_set_fmt,
	.get_fmt = qtec_roic_get_fmt,
	.enum_mbus_code = qtec_roic_enum_mbus_code,
};

static const struct v4l2_subdev_video_ops qtec_roic_video_ops = {
	.s_stream = qtec_roic_s_stream,
	.g_frame_interval = qtec_roic_g_frame_interval,
	.s_frame_interval = qtec_roic_s_frame_interval,
	.enum_frameintervals = qtec_roic_enum_frameintervals,
};

static const struct v4l2_subdev_core_ops qtec_roic_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_roic_g_register,
	.s_register = qtec_roic_s_register,
	#endif
};
static const struct v4l2_subdev_ops qtec_roic_ops = {
	.core = &qtec_roic_core_ops,
	.video = &qtec_roic_video_ops,
	.pad = &qtec_roic_pad_ops,
};

static int qtec_roic_read_dac1000(struct qtec_roic *priv, int32_t *val){
	union i2c_smbus_data data;
	struct i2c_adapter *adap;
	int32_t value;
	int ret;

	adap=i2c_get_adapter(0);
	if (!adap){
		dev_err(&priv->pdev->dev, "Unable to find i2c adapter\n");
		return -1;
	}
	ret = i2c_smbus_xfer(adap, 0x48, 0,I2C_SMBUS_READ, 0, I2C_SMBUS_WORD_DATA, &data);
	i2c_put_adapter(adap);
	value=swab16(data.word);

	dev_dbg(&priv->pdev->dev, "temp R:%d (0x%4x)\n",value,data.word);
	*val=value;

	return ret;
}

static int qtec_roic_get_temperature(struct qtec_roic *priv, struct v4l2_ctrl *ctrl){
	int ret;
	int32_t value;

	ret=qtec_roic_read_dac1000(priv,&value);
	if (ret){
		ctrl->val=0;
		dev_err(&priv->pdev->dev, "Error reading temperature\n");
		return 0;
	}

	if (priv->is_ingas_chip){
		value*=3300;
		value+=0x400;
		value/=0x800; // to mV

		value -= 740; //26.85 C is 0.740 V

		value *= -100000;
		value /= 148; //-1.48 mV per degree (0.00148 mV per mC)

		value+= 26850;
	}
	else{
		value*=3600;
		value+=0x400;
		value/=0x800; // to mV

		value-=1788; //30C is 1.788V

		value *=-100000;
		value /= 645;//-6.45mV per degree (0.00645 mV per mC)

		value+=30000;
	}

	ctrl->val=value;

	return 0;
}

static int qtec_roic_get_vtemp_pin(struct qtec_roic *priv, struct v4l2_ctrl *ctrl){
	int ret;
	int32_t value;

	ret=qtec_roic_read_dac1000(priv,&value);
	if (ret){
		ctrl->val=0;
		dev_err(&priv->pdev->dev, "Error reading vtemp_pin\n");
		return 0;
	}

	value*=3600;
	value+=0x400;
	value/=0x800; // to mV
	ctrl->val=value;

	return 0;
}

static int qtec_roic_get_vtemp(struct qtec_roic *priv, struct v4l2_ctrl *ctrl){
	int32_t val;

	qtec_roic_read(priv,TEMP,&val);
	if (ctrl->id==QTEC_ROIC_CID_VTEMP_2)
		val=(val>>16)&0xffff;
	else
		val=val&0xffff;

	//To volts
	val *= (priv->volt_ad_gain->val*2);
	val += 0x2000;
	val /= 0x4000;

	//From -ad_gain to +ad_gain
	val-=priv->volt_ad_gain->val;

	//filter
	val *=100;
	val /= 82;

	//Remove offset
	val*=2;
	val+=priv->volt_ad_offset->val;

	ctrl->val=val;

	return 0;
}

static void qtec_roic_update_range(struct work_struct *work){
	struct qtec_roic *priv=container_of(work, struct qtec_roic , work);

	qtec_roic_clamp_interval(priv,&priv->fival);
	qtec_roic_update_exposure_range(priv);
	return;
}

/*
 * Controls
 */
static int qtec_roic_s_ctrl_aux(struct v4l2_ctrl *ctrl)
{
	struct qtec_roic *priv = container_of(ctrl->handler, struct qtec_roic, ctrl_handler);
	uint32_t aux;

	switch (ctrl->id){
		case QTEC_ROIC_CID_MODE_TRIG:
		case QTEC_ROIC_CID_FLASH_POL:
		case QTEC_ROIC_CID_PIXEL_CLK:
			if (priv->streaming)
				return -EBUSY;
	};

	switch (ctrl->id){
		case QTEC_ROIC_CID_ITR:
		case QTEC_ROIC_CID_IMRO:
		case QTEC_ROIC_CID_NDRO:
		case QTEC_ROIC_CID_RE:
		case QTEC_ROIC_CID_RST:
		case QTEC_ROIC_CID_OE_EN:
		case QTEC_ROIC_CID_PW:
		case QTEC_ROIC_CID_I:
		case QTEC_ROIC_CID_AP:
		case QTEC_ROIC_CID_BW:
		case QTEC_ROIC_CID_TS:
		case V4L2_CID_HFLIP:
		case V4L2_CID_VFLIP:
		case V4L2_CID_GAIN:
			if (priv->streaming){
				qtec_roic_send_serial_link(priv);
				if (ctrl->id==V4L2_CID_HFLIP){
					uint32_t xflip;
					aux=ctrl->val;
					qtec_roic_read(priv,CTRL,&aux);
					aux&=~BIT(XFLIP);
					xflip=!!ctrl->val;
					aux|=xflip<<XFLIP;
					qtec_roic_write(priv,CTRL,aux);
				}
			}
			break;
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			return qtec_roic_set_exposure(priv);
		case QTEC_ROIC_CID_VOLT_GOK:
		case QTEC_ROIC_CID_VOLT_VSK:
		case QTEC_ROIC_CID_VOLT_GFID:
		case QTEC_ROIC_CID_VOLT_VDETCOM:
		case QTEC_ROIC_CID_VOLT_AD_OFFSET:
		case QTEC_ROIC_CID_VOLT_AD_GAIN:
			return qtec_roic_set_voltages(priv,ctrl->id,ctrl->val);
		case QTEC_ROIC_CID_MANUAL_TRIGGER:
			qtec_roic_read(priv,CTRL,&aux);
			aux|=BIT(TRIG_SW);
			qtec_roic_write(priv,CTRL,aux);
			aux&=~BIT(TRIG_SW);
			qtec_roic_write(priv,CTRL,aux);
			return 0;
		case QTEC_ROIC_CID_EXT_TRIG_DELAY:
			if (priv->mode_trig->val == EXT_TRIG)
				return qtec_roic_set_ext_trig_delay(priv,ctrl->val);
			else
				return 0;
		case QTEC_ROIC_CID_AUTO_OFF:
			if (!priv->streaming){
				if (ctrl->val)
					qtec_roic_write(priv,GPIO,0);
				else
					qtec_roic_write(priv,GPIO,BIT(0));
			}
			return 0;
		case QTEC_ROIC_CID_PIXEL_CLK:
			if (ctrl->val<CLK_AD_SLOW)
				qtec_roic_ad_slow(priv);
			else
				qtec_roic_ad_normal(priv);
			qtec_roic_set_clk_freq(priv,ctrl->val,0);
			qtec_roic_sync(priv);
			qtec_roic_stop(priv);
			ctrl->val=priv->pixel_clk;
			schedule_work(&priv->work);
			return 0;
		case QTEC_ROIC_CID_EXT_TRIG_OVERFLOW:
			qtec_roic_read(priv,CTRL,&aux);
			if (! (aux&BIT(TRIG_OVERFLOW)))
				return 0;
			aux|=BIT(TRIG_OVERFLOW);
			qtec_roic_write(priv,CTRL,aux);
			return 0;
		default:
			return 0;
	}

	return 0;
}

static int qtec_roic_s_ctrl(struct v4l2_ctrl *ctrl){
	int ret;

	ret=qtec_roic_s_ctrl_aux(ctrl);

	if (ret)
		ctrl->val=ctrl->cur.val;

	return ret;

}

static int qtec_roic_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	struct qtec_roic *priv = container_of(ctrl->handler, struct qtec_roic, ctrl_handler);
	uint32_t aux;

	switch (ctrl->id) {
		case QTEC_ROIC_CID_TEMPERATURE:
			if (!priv->streaming && priv->auto_off->val){
				dev_warn(&priv->pdev->dev, "Sensor is powered off, returning invalid temperature\n");
				return 0;
			}
			return qtec_roic_get_temperature(priv,ctrl);
		case V4L2_CID_HBLANK:
			ctrl->val=HBLANK;
			break;
		case V4L2_CID_VBLANK:
			ctrl->val=0;
			break;
		case QTEC_ROIC_CID_EXT_TRIG_OVERFLOW:
			qtec_roic_read(priv,CTRL,&aux);
			ctrl->val=(aux&BIT(TRIG_OVERFLOW))?1:0;
			return 0;
		case QTEC_ROIC_CID_VTEMP_1:
		case QTEC_ROIC_CID_VTEMP_2:
			return qtec_roic_get_vtemp(priv,ctrl);
		case QTEC_ROIC_CID_VTEMP_PIN:
			return qtec_roic_get_vtemp_pin(priv,ctrl);

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops qtec_roic_ctrl_ops_volatile_write = {
	.g_volatile_ctrl = qtec_roic_g_volatile_ctrl,
	.s_ctrl = qtec_roic_s_ctrl,
};

static const struct v4l2_ctrl_ops qtec_roic_ctrl_ops_volatile = {
	.g_volatile_ctrl = qtec_roic_g_volatile_ctrl,
};

static const struct v4l2_ctrl_ops qtec_roic_ctrl_ops = {
	.s_ctrl = qtec_roic_s_ctrl,
};

static const char * const mode_trig_menu[] = {
	"Self Timed",
	"External Trigger",
	"External Exposure",
	NULL
};

static struct v4l2_ctrl *qtec_roic_add_custom_control_trig(struct qtec_roic *priv, char *name, int id,int def){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_MENU,
		.qmenu = mode_trig_menu,
		.menu_skip_mask = 0,
		.min = 0,
		.max = 2,
	};
	ctrl.name=name,
	ctrl.id=id;
	ctrl.def=def;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_roic_add_custom_control_inv(struct qtec_roic *priv, char *name, int id,int def){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
	};
	ctrl.name=name,
	ctrl.id=id;
	ctrl.def=def;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_roic_add_custom_control(struct qtec_roic *priv, char *name, int id,int min, int max,int def,int flags){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
	};

	ctrl.id=id;
	ctrl.min=min;
	ctrl.max=max;
	ctrl.name=name;
	ctrl.def=def;
	ctrl.flags = flags;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_roic_add_custom_control_bool_volatile(struct qtec_roic *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops_volatile_write,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
	};

	ctrl.id=id;
	ctrl.name=name;
	ctrl.flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_roic_add_custom_control_sensor_type(struct qtec_roic *priv){
	struct v4l2_ctrl *c;
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_STRING,
		.step = 1,
		.min = 1,
		.max = 32,
		.id = QTEC_VIDEO_CID_SENSOR_TYPE,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Sensor Type",
	};

	c=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	if (!c)
		return NULL;

	if (priv->is_ingas_chip)
		snprintf(c->p_cur.p_char,32,"FPA-320x256-C");
	else
		snprintf(c->p_cur.p_char,32,"UL 05 25 1 - 026");

	return c;
}

static struct v4l2_ctrl *qtec_roic_add_button_control(struct qtec_roic *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BUTTON,
	};
	ctrl.name=name,
	ctrl.id=id;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

#define N_DEAD_PIXELS 200
static struct v4l2_ctrl *qtec_roic_add_custom_control_dead_pixels(struct qtec_roic *priv){
	struct v4l2_ctrl *c;
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_roic_ctrl_ops,
		.type = V4L2_CTRL_TYPE_POINT,
		.id = QTEC_VIDEO_CID_DEAD_PIXELS,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Dead Pixels",
		.dims =  {N_DEAD_PIXELS},
		.elem_size = sizeof(struct v4l2_point),
	};
	int i;

	c=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	if (!c){
		printk(KERN_ERR "%s:%d\n",__FILE__,__LINE__);
		return NULL;
	}

	//FIXME
	for (i=0;i<N_DEAD_PIXELS;i++){
		c->p_cur.p_point[i].x=N_DEAD_PIXELS-i;
		c->p_cur.p_point[i].y=i;
	}

	return c;
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

static struct spi_device *qtec_roic_init_spi_dev(struct qtec_roic *priv, char *name){
	struct device_node *node;
	struct spi_device *spidev;
	int ret;

	node=of_parse_phandle(priv->pdev->dev.of_node,name,0);
	if (!node){
		dev_err(&priv->pdev->dev, "Unable to parse %s phandle\n",name);
		return NULL;
	}
	spidev=of_find_spi_device_by_node(node);
	of_node_put(node);
	if (!spidev){
		dev_err(&priv->pdev->dev, "Unable to find %s spi dev\n",name);
		return NULL;
	}
	ret=spi_setup(spidev);
	if (ret){
		dev_err(&priv->pdev->dev, "Unable to setup %s spi dev\n",name);
		return NULL;
	}

	return spidev;
}

static int qtec_roic_init_spi_devices(struct qtec_roic *priv){

	priv->dac_lwir=qtec_roic_init_spi_dev(priv,"qtec,dac_lwir");
	if (!priv->dac_lwir)
		return -1;
	priv->dac_sensor=qtec_roic_init_spi_dev(priv,"qtec,dac_sensor");
	if (!priv->dac_sensor)
		return -1;
	priv->ad_lwir=qtec_roic_init_spi_dev(priv,"qtec,ad_lwir");
	if (!priv->ad_lwir)
		return -1;

	return 0;
}

static struct mtd_info *qtec_roic_find_mtd_name(struct qtec_roic *priv, struct device_node *node, char *name){
	struct mtd_info *mtd;
	mtd_for_each_device(mtd)
		if ((mtd->dev.parent->of_node==node) &&
				(strcmp(name,mtd->name)==0))
			return mtd;
	return NULL;
}

static int qtec_roic_init_flash_lwir(struct qtec_roic *priv){
	struct device_node *node;
	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,flash",0);
	if (!node){
		dev_err(&priv->pdev->dev, "Unable to parse qtec,flash phandle\n");
		return -1;
	}

	priv->flash_lwir= qtec_roic_find_mtd_name(priv, node, "Sensor Data");
	if (!priv->flash_lwir){
		dev_err(&priv->pdev->dev, "Unable to find mtd device for qtec,flash phandle\n");
		return -1;
	}

	return 0;
}

static atomic_t qtec_roic_instance = ATOMIC_INIT(0);
static int qtec_roic_probe(struct platform_device *pdev){
	struct qtec_roic *priv;
	struct resource res;
	int ret;
	struct device_node *node;
	uint32_t aux;
	int min_clk;
	uint32_t max_clk;
	const char *pll_name;

	priv=(struct qtec_roic *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;
	INIT_WORK(&priv->work,qtec_roic_update_range);

	//Dt parse
	ret=of_address_to_resource(pdev->dev.of_node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get address\n");
		return ret;
	}

	priv->iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem)){
		dev_err(&pdev->dev, "Unable to ioremap memory\n");
		return PTR_ERR(priv->iomem);
	}

	ret = qtec_roic_init_spi_devices(priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to init spi devices\n");
		return -EIO;
	}

	ret = qtec_roic_init_flash_lwir(priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to init lwir flash\n");
		return -EIO;
	}

	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,pll",0);
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get pll address\n");
		return ret;
	}

	priv->iomem_pll  = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (IS_ERR(priv->iomem_pll)){
		dev_err(&pdev->dev, "Unable to ioremap pll memory\n");
		return PTR_ERR(priv->iomem);
	}

	ret=of_property_read_string(node, "qtec,pll_type",&pll_name);
	if (!ret && !strcmp(pll_name, "PLLE2_ADV"))
		priv->pll_7series = true;
	else
		priv->pll_7series = false;

	of_node_put(node);

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,ingas_chip",&aux);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse ingas_chip property\n");
		return -EIO;
	}
	priv->is_ingas_chip=(aux)?true:false;

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bus_clk",&priv->bus_clk); //NOT USED!!
	if (ret){
		dev_err(&pdev->dev, "Unable to parse bus_clk property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,base_clk",&priv->base_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse base_clk property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,max_clk",&priv->max_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse max_clk property\n");
		return -EIO;
	}

	//Set appropiate clks
	if (qtec_roic_set_clk(priv)){
		dev_err(&pdev->dev, "Unable to set clks\n");
		return -EIO;
	}

	//Set baseboard voltage (before ctrl!)
	qtec_roic_init_fg(priv);
	qtec_roic_init_voltages(priv);
	qtec_roic_init_ad(priv);

	//Subdev
	v4l2_subdev_init(&priv->sd, &qtec_roic_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_roic_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	//Controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 16); //16 is just a guess for the hash table

	priv->hflip = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1,(priv->is_ingas_chip)?0:1);
	priv->vflip = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1,(priv->is_ingas_chip)?0:1);
	priv->mode_trig = qtec_roic_add_custom_control_trig(priv,"Trigger Mode",QTEC_ROIC_CID_MODE_TRIG,SELF_TIMED);
	priv->flash_pol = qtec_roic_add_custom_control_inv(priv,"Invert Flash Polarity",QTEC_ROIC_CID_FLASH_POL,0);
	priv->trig_pol = qtec_roic_add_custom_control_inv(priv,"Invert Trigger Polarity",QTEC_ROIC_CID_TRIG_POL,0);
	priv->exp = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE, 0, 0xffffff, 1,20000);
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops_volatile,
			V4L2_CID_HBLANK, 0, 0xffffff, 1,0);
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops_volatile,
			V4L2_CID_VBLANK, 0, 0xffffff, 1,0);
	priv->temp = qtec_roic_add_custom_control(priv,"Sensor Temperature",QTEC_ROIC_CID_TEMPERATURE,-20000,100000,0,V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);
	priv->manual_trigger = qtec_roic_add_button_control(priv,"Manual Trigger",QTEC_ROIC_CID_MANUAL_TRIGGER);
	priv->ext_trig_delay = qtec_roic_add_custom_control(priv,"External Trigger Delay",QTEC_ROIC_CID_EXT_TRIG_DELAY,0x0,
			qtec_roic_max_delay_ext_trig(priv),0,V4L2_CTRL_FLAG_SLIDER);
	priv->trigger_overflow = qtec_roic_add_custom_control_bool_volatile(priv,"External Trigger Overflow", QTEC_ROIC_CID_EXT_TRIG_OVERFLOW);
	priv->auto_off = qtec_roic_add_custom_control_inv(priv,"Auto Power Off",QTEC_ROIC_CID_AUTO_OFF,1);

	if (priv->is_ingas_chip){
		priv->volt_detcom = qtec_roic_add_custom_control(priv,"VDETCOM Voltage",QTEC_ROIC_CID_VOLT_VDETCOM,4700,5402,4700,V4L2_CTRL_FLAG_SLIDER);
		priv->volt_ad_offset = qtec_roic_add_custom_control(priv,"AD Offset Voltage",QTEC_ROIC_CID_VOLT_AD_OFFSET,0,5000,2750,V4L2_CTRL_FLAG_SLIDER);
		priv->volt_ad_gain = qtec_roic_add_custom_control(priv,"AD Gain Voltage",QTEC_ROIC_CID_VOLT_AD_GAIN,0,1800,504,V4L2_CTRL_FLAG_SLIDER);
		priv->itr = qtec_roic_add_custom_control_inv(priv,"ITR Integrate Then Readout mode",QTEC_ROIC_CID_ITR,0);
		priv->gain = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops, V4L2_CID_GAIN, 0, 26000, 1,26000);

		priv->imro = qtec_roic_add_custom_control_inv(priv,"IMRO Integrate Readout Mode",QTEC_ROIC_CID_IMRO,0);
		priv->ndro = qtec_roic_add_custom_control_inv(priv,"NDRM Non Destructive Readout Mode",QTEC_ROIC_CID_NDRO,0);
		priv->re = qtec_roic_add_custom_control_inv(priv,"RE Reference Output Enambe",QTEC_ROIC_CID_RE,0);
		priv->rst = qtec_roic_add_custom_control_inv(priv,"RST Global Reset",QTEC_ROIC_CID_RST,0);
		priv->oe_en = qtec_roic_add_custom_control_inv(priv,"OE_EN Skimming Enable",QTEC_ROIC_CID_OE_EN,0);
		priv->pw = qtec_roic_add_custom_control(priv,"PW Power Control",QTEC_ROIC_CID_PW,0,3,2,V4L2_CTRL_FLAG_SLIDER);
		priv->i = qtec_roic_add_custom_control(priv,"I Master Current",QTEC_ROIC_CID_I,0,7,4, V4L2_CTRL_FLAG_SLIDER);
		priv->ap = qtec_roic_add_custom_control(priv,"AP CTIA Bias",QTEC_ROIC_CID_AP,0,7,4, V4L2_CTRL_FLAG_SLIDER);
		priv->bw = qtec_roic_add_custom_control(priv,"BW CTIA Bandwidth Control",QTEC_ROIC_CID_BW,0,3,0, V4L2_CTRL_FLAG_SLIDER);
		priv->ts = qtec_roic_add_custom_control(priv,"TS Test Control",QTEC_ROIC_CID_TS,0,255,0, V4L2_CTRL_FLAG_SLIDER);
		max_clk=MAX_CLK_IG;
	}
	else{
		priv->volt_gok = qtec_roic_add_custom_control(priv,"GOK Voltage",QTEC_ROIC_CID_VOLT_GOK,1000,3600,3600, V4L2_CTRL_FLAG_SLIDER);
		priv->volt_vsk = qtec_roic_add_custom_control(priv,"VSK Voltage",QTEC_ROIC_CID_VOLT_VSK,2500,5500,4802, V4L2_CTRL_FLAG_SLIDER);
		priv->volt_gfid = qtec_roic_add_custom_control(priv,"GFID Voltage",QTEC_ROIC_CID_VOLT_GFID,650,3600,3300, V4L2_CTRL_FLAG_SLIDER);
		priv->gain = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_roic_ctrl_ops, V4L2_CID_GAIN, 0, 12000, 1,0);
		priv->volt_ad_offset = qtec_roic_add_custom_control(priv,"AD Offset Voltage",QTEC_ROIC_CID_VOLT_AD_OFFSET,0,5000,1400, V4L2_CTRL_FLAG_SLIDER);
		priv->volt_ad_gain = qtec_roic_add_custom_control(priv,"AD Gain Voltage",QTEC_ROIC_CID_VOLT_AD_GAIN,0,1800,425, V4L2_CTRL_FLAG_SLIDER);
		priv->vtemp_pin = qtec_roic_add_custom_control(priv,"Vtemp_pin",QTEC_ROIC_CID_VTEMP_PIN,-5000,+5000,0,V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);
		priv->vtemp_1 = qtec_roic_add_custom_control(priv,"Vtemp 1",QTEC_ROIC_CID_VTEMP_1,-5000,+5000,0,V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);
		priv->vtemp_2 = qtec_roic_add_custom_control(priv,"Vtemp 2",QTEC_ROIC_CID_VTEMP_2,-5000,+5000,0,V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);
		max_clk=MAX_CLK_MB;
	}
	max_clk=min(max_clk,priv->max_clk);
	min_clk=priv->base_clk/(MAX_DIV*2);
	min_clk=max(min_clk,MIN_CLK_AD);
	priv->pclk = qtec_roic_add_custom_control(priv,"Pixel CLock",QTEC_ROIC_CID_PIXEL_CLK,min_clk,max_clk,priv->pixel_clk, 0);
	priv->sensor_type = qtec_roic_add_custom_control_sensor_type(priv);
	priv->dead_pixels = qtec_roic_add_custom_control_dead_pixels(priv);

	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	priv->pclk->flags|=V4L2_CTRL_FLAG_READ_ONLY;//Do not resync
	priv->hblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY;
	priv->vblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY;

	//Init controls
	priv->streaming=0;
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	priv->pclk->flags&=~V4L2_CTRL_FLAG_READ_ONLY;

	//Connect to subdev
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	//Init video infrasctuctures
	priv->fival=(struct v4l2_fract) DEF_FIVAL;
	qtec_roic_init_fmt(priv);
	qtec_roic_update_exposure_range(priv);

	//Find best window
	if (qtec_roic_sync(priv))
		return -EIO;

#ifdef SYNC_EXPERIMENT
	qtec_roic_sync_experiment(priv);
	return -EIO;
#endif

	//Leave system stopped
	qtec_roic_stop(priv);

	v4l2_info(&priv->sd, "qtec_roic V4L2 subdevice registered as %s\n", priv->sd.name);

	return 0;
}

static int __exit qtec_roic_remove(struct platform_device *pdev){
	struct qtec_roic *priv=platform_get_drvdata(pdev);

	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static struct of_device_id qtec_roic_of_match[] = {
	{ .compatible = "qtec,axi-ingas-if-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_roic_of_match);

static struct platform_driver qtec_roic_plat_driver = {
	.probe		= qtec_roic_probe,
	.remove		= qtec_roic_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_roic_of_match,
	},
};

module_platform_driver(qtec_roic_plat_driver);

MODULE_DESCRIPTION("ROIC module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
