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
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/log2.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define DRIVER_NAME "qtec_cmosis"
#define DEF_FIVAL {1,24} //24fps

#define MASK(x) (BIT(x)-1)

enum cmosis_id {CMV2000v2=0x20,CMV2000v3=0x23,CMV4000v2=0x40,CMV4000v3=0x43, CMV8000v1=0x81, CMV12000v2=0x2};
enum mosaic_mode {MONO, BAYER, MOSAIC5x5, BAND100};

#define MAX_CROP 8

#define QUIRK_BINNING (BIT(0))
#define QUIRK_SPI (BIT(1))
#define QUIRK_SPI_NO_FOT (BIT(2))
#define QUIRK_SPI_IDLE (BIT(3))
#define QUIRK_FAST_BINNING (BIT(4))
#define QUIRK_BURST_224 (BIT(5))
#define QUIRK_PROG_BITMODE (BIT(6))
#define QUIRK_IRQ_OVERFLOW (BIT(7))
#define QUIRK_FREQ_DELAY (BIT(8))
#define QUIRK_VERSION_SENSOR (BIT(9))
#define QUIRK_SPI_16_BITS (BIT(10))
#define QUIRK_SYNC_ONLY_USED_CHANS (BIT(11))
#define QUIRK_IDELAY (BIT(12))
#define QUIRK_PROG_BITMODE_IGNORE (BIT(13))
#define QUIRK_IRQ_SYNC_ERROR (BIT(14))
#define QUIRK_FPN (BIT(15))
#define QUIRK_FLASH_DISABLE (BIT(16))
#define QUIRK_FB4 (BIT(17))
#define QUIRK_PROG_FLASH_WIDTH (BIT(18))
#define QUIRK_PHASE_INC (BIT(19))
#define QUIRK_IDELAY_MONITOR (BIT(19))
#define QUIRK_NO_PHASE_IDELAY (BIT(20))

#define ABS(X)  (((X)<0) ? (-(X)) : (X))

static DEFINE_MUTEX(spi_mutex);

struct fb4_config {
	int hbin;
	u32 code;
	int left;
	int right;
};

#define MAX_CHANNELS 16

struct qtec_cmosis{
	struct v4l2_subdev sd; //NEEDS to be first!!!!

	struct platform_device *pdev;
	void __iomem *iomem;
	void __iomem *iomem_pll;
	struct spi_device *cmosis_spi;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	   *mode_trig;
	struct v4l2_ctrl	   *phase;
	struct v4l2_ctrl	   *flash_pol;
	struct v4l2_ctrl	   *flash_disable;
	struct v4l2_ctrl	   *trig_pol;
	struct v4l2_ctrl	   *pga_gain;
	struct v4l2_ctrl	   *offset;
	struct v4l2_ctrl	   *adc_gain;
	struct v4l2_ctrl	   *exp;
	struct v4l2_ctrl	   *dual_exp;
	struct v4l2_ctrl	   *hflip;
	struct v4l2_ctrl	   *vflip;
	struct v4l2_ctrl	   *hblank;
	struct v4l2_ctrl	   *vblank;
	struct v4l2_ctrl	   *manual_trigger;
	struct v4l2_ctrl	   *ext_trig_delay;
	struct v4l2_ctrl	   *trigger_overflow;
	struct v4l2_ctrl	   *temperature;
	struct v4l2_ctrl	   *sensor_type;
	struct v4l2_ctrl	   *vramp;
	struct v4l2_ctrl	   *h_binning;
	struct v4l2_ctrl	   *v_binning;
	struct v4l2_ctrl	   *output_scaler;
	struct v4l2_ctrl	   *bayer_skipping;
	struct v4l2_ctrl	   *fpn;
	struct v4l2_ctrl	   *nchans_ctrl;
	struct v4l2_ctrl	   *bitmode;
	struct v4l2_ctrl	   *resp_curve;
	struct v4l2_ctrl	   *exp_step;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop[MAX_CROP];
	int n_crop;
	struct v4l2_fract fival;

	uint32_t bus_clk;
	uint32_t pixel_clk;

	int32_t of_offset;
	int32_t of_adc_gain;
	int32_t of_vramp;

	uint8_t version;
	uint32_t quirk;

	bool initialized;
	unsigned int mosaic_mode;
	bool streaming;
	uint32_t sync_word;
	int max_channels; //Maximum connected channels
	int sync_channels; //Channels that could sync
	int idelay[MAX_CHANNELS+1];
	enum cmosis_id chip_id;
	uint8_t fot_len;
	uint8_t slot_len;
	bool calibration_done;
	bool spi_mutex;
	bool dual_eye;
	int32_t calibration_temperature;
	bool stop_idelay_monitor;

	struct mutex idelay_mutex;

	bool pll_7series;

	bool trig_overflow_v32;

	int nchans;

	struct task_struct *kthread;

	struct fb4_config fb4_config;
	struct work_struct async_idelay;
};

/*Cmosis common definitions*/
#define REG_FLIP 40
#define REG_FLIP_8M 39
#define REG_FLIP_12M 69
#define REG_EXPOSURE 42
#define REG_EXPOSURE_8M 41
#define REG_EXPOSURE_12M 71
#define REG_EXPOSURE_STEP 45
#define REG_NSLOPES 54
#define REG_VRAMP1 98
#define REG_VRAMP2 99
#define REG_VRAMP1_8M 115
#define REG_VRAMP2_8M 116
#define REG_VRAMP_12M 109
#define REG_OFFSET 100
#define REG_OFFSET_8M 80
#define REG_OFFSET_12M_BOT 87
#define REG_OFFSET_12M_TOP 88
#define REG_PGA2 121
#define REG_PGA2_MASK 0x1
#define REG_PGA10 102
#define REG_PGA10_MASK 0x3
#define REG_PGA_8M 118
#define REG_PGA_12M 115
#define REG_GAIN 103
#define REG_GAIN_8M 117
#define REG_ID 125
#define REG_ID_8M 90
#define REG_TEMP 126
#define REG_TEMP_8M 88
#define REG_TEMP_12M 127

#define VFLIP_IMG 1
#define HFLIP_IMG 0

/*Framegen common definitions*/
#define CONTROL 0x0
#define RUN 0
#define MODE_OUT 1
#define MODE_OUT_LEN 2
#define IF_CH 3
#define WSYNCRUN 5
#define MODE_TRIG 6
#define MODE_TRIG_LEN 2
#define TRIG_POL 8
#define TRIG_ENABLE 9
#define TRIG_SW 10
#define FLASH_POL 11
#define WSLOCK 12
#define XFLIP 13
#define YFLIP 14
#define FB_RESET 15
#define FLASH_DISABLE 16
#define BURST_224 16
#define CH_SWAP 17
#define TRIG_OVERFLOW 18
#define BLCH 19
#define BLCH_LEN 5
#define VERSION 24

#define HPARAM 0x4
#define HSTART 16
#define HEND 0

#define FRAME_DELAY 0x8

#define CMV_CTRL 0xc
#define RESET 0
#define NUM_BITS 1
#define CMV_FREQ_DELAY 2
#define CMV_LOCK_BIT 3
#define SENSOR_TYPE 4
#define DATA_MODE 8
#define DLY_DATA_CHG 11
#define DLY_CH 12
#define DLY_CLR_CHG 17
#define CH_BUSY 18
#define CH_RST 19
#define CH_CAL 20
#define DLY_DIR 21
#define DLY_RUN 22
#define DLY_BUSY 23
#define DLY_CNT 24
#define DLY_MS 31

#define FRAMECNT 0x10
#define NOT_IDLE 16
#define NOT_IDLE_LEN 8
#define NOT_IDLE_MASK (((1<<NOT_IDLE_LEN)-1)<<NOT_IDLE)

#define BINNING 0x14
#define BINNING_FB4 0x18
#define H_BINNING 0
#define V_BINNING 8
#define BINSCALE 16

#define LINE_SKIP 0x14

#define BRAM_FB1 0x18
#define BRAM_FB4 0x3c
#define BRAM_ADDR 0
#define BRAM_WE 15
#define BRAM_DATA 16

#define FPN 0x0
#define LINCOMB 0x1000
#define COEFF 0x2000

#define SPI_CTRL 0x1c
#define SPI_EN_FOT 0
#define SPI_EN_EXP 1
#define SPI_EN 2
#define SPI_INT 3
#define SPI_RST 4
#define SPI_ACK 5
#define SPI_RDY 8

#define SPI_V7_DLY 2
#define SPI_V7_IDLE 3
#define SPI_V7_IDLERUN 4
#define SPI_V7_EN 5
#define SPI_V7_RST 7
#define SPI_V7_ACK 8
#define SPI_V7_RDY 9
#define SPI_V7_OVERRUN 10
#define MAX_SPI_CMD 32

#define SPI_DATA 0x20

#define TRIG_FILTER 0x24
#define GLOBAL_TIMER 0x28

#define TRC_OVERFLOW 0x2C
#define OVERFLOW_CNT 0
#define OVERFLOW_EN 12
#define OVERFLOW_STATUS 13
#define SYNC_ERR_EN 14
#define SYNC_ERR_STATUS 15

#define N_TRACERS 3
#define TRC_CTRL 0x30
#define TRC_IDX 0
#define TRC_IRQ 4
#define TRC_IRQEN 8
#define TRC_EN 12
#define TRC_POL 16
#define TRC_EMPTY 20
#define TRC_FULL 24
#define TRC_BLANK 28

#define IDELAY_DATA 0x34
#define FRAME_ERROR_STATUS 0x38

#define FLASH_TIME 0x40

enum mode_trig {SELF_TIMED=0, EXT_TRIG, EXT_EXPOSURE, IDLE};
enum mode_trig_hw {MODE_SELF_TIMED=0, MODE_TRIG_DELAY, MODE_TRIG_NODELAY, MODE_EXT_EXPOSURE};
enum mode_out {MODE_DECIM=0,MODE_SINGLE,MODE_COMPACT};
enum mode_out_fb4 {MODE_MONO = 0, MODE_BAYER};
enum if_ch {IF_2CH=0, IF_4CH, IF_8CH, IF_16CH};

static const struct qtec_cmosis_format {
	u32 mbus_format;
	u8  mode_out;
	bool fb4_bus;
	bool is_bayer_chip;
} cmosis_formats[] ={
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGB,
		.mode_out = MODE_DECIM,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_RGGB,
		.mode_out = MODE_SINGLE,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_GRBG,
		.mode_out = MODE_SINGLE,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_GBRG,
		.mode_out = MODE_SINGLE,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_BGGR,
		.mode_out = MODE_SINGLE,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_COMPACT_MONO,
		.mode_out = MODE_COMPACT,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_COMPACT_RGGB,
		.mode_out = MODE_COMPACT,
		.is_bayer_chip = true,
	},
	{

		.mbus_format= MEDIA_BUS_FMT_QTEC_COMPACT_GBRG,
		.mode_out = MODE_COMPACT,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_COMPACT_GRBG,
		.mode_out = MODE_COMPACT,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_COMPACT_BGGR,
		.mode_out = MODE_COMPACT,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_LEGACY_MONO,
		.mode_out = MODE_SINGLE,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_MONO,
		.mode_out = MODE_MONO,
		.fb4_bus = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_RGBX,
		.mode_out = MODE_BAYER,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_RGGB,
		.mode_out = MODE_MONO,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_GRBG,
		.mode_out = MODE_MONO,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_GBRG,
		.mode_out = MODE_MONO,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_BGGR,
		.mode_out = MODE_MONO,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_GREEN,
		.mode_out = MODE_BAYER,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
	{
		.mbus_format= MEDIA_BUS_FMT_QTEC_FB4_GREY,
		.mode_out = MODE_BAYER,
		.fb4_bus = true,
		.is_bayer_chip = true,
	},
};

static inline const struct qtec_cmosis_format *get_format(u32 mbus_format){
	int i;

	for (i=0;i<ARRAY_SIZE(cmosis_formats);i++)
		if (mbus_format==cmosis_formats[i].mbus_format)
			return &cmosis_formats[i];

	return NULL;
}

static inline int qtec_cmosis_write_bram(struct qtec_cmosis *priv, uint16_t offset, uint16_t value){
	uint32_t aux;

	dev_dbg(&priv->pdev->dev, "BRAM W:0x%.4x 0x%.4x\n",offset,value);

	aux = offset<<BRAM_ADDR;
	aux |= BIT(BRAM_WE);
	aux |= value << BRAM_DATA;
	iowrite32(aux,priv->iomem+((priv->quirk & QUIRK_FB4)?BRAM_FB4:BRAM_FB1));
	/*dev_err(&priv->pdev->dev, "`AXI_OP(`OP_WRITE, 32'h%.8x, 32'h%.8x)",0x300C0000+BRAM,aux);*/

	return 0;
}

static inline int qtec_cmosis_read_bram(struct qtec_cmosis *priv, uint16_t offset, uint16_t *value){
	uint32_t aux;


	aux = offset<<BRAM_ADDR;
	iowrite32(aux,priv->iomem+((priv->quirk & QUIRK_FB4)?BRAM_FB4:BRAM_FB1));
	aux = ioread32(priv->iomem+((priv->quirk & QUIRK_FB4)?BRAM_FB4:BRAM_FB1));

	aux >>= BRAM_DATA;
	aux &= 0xffff;

	*value=aux;
	dev_dbg(&priv->pdev->dev, "BRAM R:0x%.4x 0x%.4x\n",offset,*value);
	return 0;
}

static inline int qtec_cmosis_read(struct qtec_cmosis *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "fg R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static inline int qtec_cmosis_write(struct qtec_cmosis *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "fg W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem+offset);

	/*if (offset!= SPI_CTRL && offset!= SPI_DATA)
		dev_err(&priv->pdev->dev, "`AXI_OP(`OP_WRITE, 32'h%.8x, 32'h%.8x)",0x300C0000+offset,value);*/
	return 0;
}

static inline int qtec_cmosis_write_pll(struct qtec_cmosis *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "pll W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem_pll+offset);
	return 0;
}

static inline int qtec_cmosis_read_pll(struct qtec_cmosis *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem_pll+offset);
	dev_dbg(&priv->pdev->dev, "pll R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static inline int qtec_cmosis_fill_spi_buffer(struct qtec_cmosis *priv, uint8_t len, uint8_t *spi_buffer, uint8_t *addr, uint16_t *data, bool do_write){
	int idx = 0;
	int i;

	for(i=0;i<len;i++){
		if (addr[i] >= 0x80)
			return -EINVAL;

		spi_buffer[idx] = addr [i];

		if (!do_write){
			if (priv->quirk & QUIRK_SPI_16_BITS)
				idx +=3;
			else
				idx +=2;
			continue;
		}

		spi_buffer[idx++] |= BIT(7);

		if (priv->quirk & QUIRK_SPI_16_BITS)
			spi_buffer[idx++] = data[i] >>8;
		spi_buffer[idx++] = data[i];

	}

	return idx;
}

static inline int qtec_cmosis_read_spi_buffer(struct qtec_cmosis *priv, uint8_t len, uint8_t *spi_buffer, uint16_t *data){
	int i, idx = 0;

	for (i=0;i<len;i++){
		idx++;

		data [i] = 0;
		if (priv->quirk & QUIRK_SPI_16_BITS)
			data[i] |= spi_buffer[idx++] << 8;
		data [i]  |= spi_buffer[idx++];
	}

	return 0;
}

static int qtec_cmosis_spi_internal_timeout_wait(struct qtec_cmosis *priv, bool do_write, uint8_t len, uint8_t *addr, uint16_t *data,long timeout, bool wait){
	uint32_t reg_spi;
	unsigned long expiration=jiffies+timeout;
	int i;
	int idx;
	uint8_t spi_buffer[MAX_SPI_CMD*2 +1] = {};

	if ((len > MAX_SPI_CMD) || ( (priv->quirk & QUIRK_SPI_16_BITS) && len > ((MAX_SPI_CMD*16)/24)))
		return -ENOMEM;

	if (len < 1)
		return -EINVAL;

	timeout = max(timeout, (long)100);

	/*Reset core*/
	reg_spi = (priv->quirk & QUIRK_SPI_IDLE)?BIT(SPI_V7_RST)|BIT(SPI_V7_OVERRUN):BIT(SPI_RST);
	qtec_cmosis_write(priv,SPI_CTRL,reg_spi);
	reg_spi &= ~((priv->quirk & QUIRK_SPI_IDLE)?BIT(SPI_V7_RST)|BIT(SPI_V7_OVERRUN):BIT(SPI_RST));
	qtec_cmosis_write(priv,SPI_CTRL,reg_spi);

	qtec_cmosis_read(priv,SPI_CTRL,&reg_spi);
	if (!(reg_spi & (priv->quirk & QUIRK_SPI_IDLE)?BIT(SPI_V7_RDY):BIT(SPI_RDY))){
		v4l2_err(&priv->sd, "Stalled SPI core\n");
		return -1;
	}

	/*Init buffer*/
	idx = qtec_cmosis_fill_spi_buffer(priv, len, spi_buffer, addr, data, do_write);

	/*Send buffer*/
	for (i=0; i<idx;){
		qtec_cmosis_write(priv,SPI_DATA,spi_buffer[i]<<8|spi_buffer[i+1]);
		i+=2;
	}


	/*Launch*/
	reg_spi =  (priv->quirk & QUIRK_SPI_IDLE)?BIT(SPI_V7_EN):BIT(SPI_EN);

	if (!wait)
		reg_spi |=  BIT(SPI_V7_IDLERUN);
	else
		reg_spi |=  BIT(SPI_EN_FOT);

	if ((priv->streaming) && (priv->quirk & QUIRK_SPI_IDLE) && (priv->quirk & QUIRK_SPI_NO_FOT) )
		reg_spi |=  BIT(SPI_V7_DLY); //Workaround for #497

	qtec_cmosis_write(priv,SPI_CTRL,reg_spi);

	expiration=jiffies+timeout;
	/*Wait end*/
	do {
		qtec_cmosis_read(priv,SPI_CTRL,&reg_spi);
		if (priv->quirk & QUIRK_SPI_IDLE){
			if ((reg_spi & BIT(SPI_V7_OVERRUN)) && do_write){
				v4l2_err(&priv->sd, "Overrun for for spi command: %s 0x%x 0x%x\n",do_write?"W":"R",addr[0],data[0]);
				return -1;
			}
			if ((reg_spi & BIT(SPI_V7_RDY)))
				break;
		}
		else if ((reg_spi & BIT(SPI_RDY)))
			break;
	}while(time_before(jiffies,expiration));
	if (!time_before(jiffies,expiration)){
		qtec_cmosis_write(priv,SPI_CTRL,0);
		return -ETIMEDOUT;
	}

	/*Get result*/
	if (!do_write){
		for(i=0;i<idx;){
			uint32_t aux;
			qtec_cmosis_read(priv,SPI_DATA,&aux);
			spi_buffer[i++] = aux>>8;
			spi_buffer[i++] = aux;
		}
		qtec_cmosis_read_spi_buffer(priv, len, spi_buffer, data);
	}

	qtec_cmosis_write(priv,SPI_CTRL,0);
	return 0;
}

static int qtec_cmosis_spi_internal(struct qtec_cmosis *priv, bool do_write, uint8_t len, uint8_t *addr, uint16_t *data){
	uint64_t aux;
	long timeout;
	int ret;

	if (!(priv->quirk & QUIRK_SPI_IDLE))
		return qtec_cmosis_spi_internal_timeout_wait(priv, do_write, len, addr, data, HZ, true);

	if (!priv->streaming || !do_write)
		return qtec_cmosis_spi_internal_timeout_wait(priv, do_write, len, addr, data, HZ/100, false);

	//3 frames of timeout
	aux = (uint64_t) priv->fival.numerator * (uint64_t) HZ * 3;
	do_div(aux,priv->fival.denominator);
	timeout = aux;

	//3 exposure time
	aux = (uint64_t) priv->exp->val * (uint32_t)HZ * 3;
	do_div(aux,1000000);

	timeout = max_t(long, aux, timeout);

	//min timeout 1/10 sec
	timeout = max_t(long, HZ/10, timeout);

	ret = qtec_cmosis_spi_internal_timeout_wait(priv, do_write, len, addr, data, timeout, true);
	if (!ret)
		return 0;

	if (!(priv->mode_trig->val==SELF_TIMED))
		ret = qtec_cmosis_spi_internal_timeout_wait(priv, do_write, len, addr, data, HZ/100, false);

	if (ret)
		v4l2_err(&priv->sd, "Error %d for spi command: %s 0x%x 0x%x\n",ret,do_write?"W":"R",addr[0],do_write?data[0]:0);

	return ret;
}

static int qtec_cmosis_spi_external(struct qtec_cmosis *priv, bool do_write, uint8_t len, uint8_t *addr, uint16_t *data){
	uint8_t txbuffer[(3*MAX_SPI_CMD)/2] = {};
	uint8_t rxbuffer[(3*MAX_SPI_CMD)/2];
	struct spi_transfer transfer = {
		.tx_buf = txbuffer,
		.rx_buf = rxbuffer,
		.bits_per_word=8,
		.len=(3*len)/2,
	};
	struct spi_message message;
	int ret;

	if (len > MAX_SPI_CMD)
		return -ENOMEM;

	if (len < 1)
		return -EINVAL;

	transfer.len =  qtec_cmosis_fill_spi_buffer(priv, len, txbuffer, addr, data, do_write);

	spi_message_init(&message);
	spi_message_add_tail(&transfer,&message);

	/* Send Message*/
	ret=spi_sync(priv->cmosis_spi,&message);

	if (!do_write)
		qtec_cmosis_read_spi_buffer(priv, len, rxbuffer, data);
	return ret;
}

static int _qtec_cmosis_spi_array(struct qtec_cmosis *priv, bool do_write, uint8_t len, uint8_t *addr, uint16_t *data){
	int ret;
	int i;

	if (priv->quirk & QUIRK_SPI){
		if (priv->spi_mutex)
			mutex_lock(&spi_mutex);
		ret = qtec_cmosis_spi_internal(priv, do_write, len, addr, data);
		if (priv->spi_mutex)
			mutex_unlock(&spi_mutex);
	}
	else
		ret = qtec_cmosis_spi_external(priv, do_write, len, addr, data);

	dev_dbg(&priv->pdev->dev, "SPI %c operation ret = %d\n",do_write?'W':'R',ret);

	for (i=0;i<len;i++)
		dev_dbg(&priv->pdev->dev, "0x%.2x (%3d) -> 0x%.4x (%.5d)\n",addr[i],addr[i],data[i],data[i]);

	if (ret){
		v4l2_err(&priv->sd, "SPI %c Error = %d\n",do_write?'W':'R',ret);
		for (i=0;i<len;i++)
			v4l2_err(&priv->sd, "0x%.2x (%3d) -> 0x%.4x (%.5d)\n",addr[i],addr[i],data[i],data[i]);
	}

	return ret;
}

static int qtec_cmosis_spi_array(struct qtec_cmosis *priv, bool do_write, bool online, uint8_t len, uint8_t *addr, uint16_t *data){
	int i, ret;
	uint32_t old_ctrl;
	unsigned long expiration;
	uint64_t aux;

	if (priv->chip_id == CMV8000v1)
		for (i=0;i<len;i++)
			if ((addr[i] ==  60) && (data[i] &BIT(3) && do_write)){
				dev_err(&priv->pdev->dev, "SPI WRITE ABORTED. Reg 60[3] can't be set\n");
				return -EINVAL;
			}
	//FIXME Workaround for #691
	if (!priv->streaming ||	((!(priv->quirk & QUIRK_SPI_NO_FOT)) && (priv->quirk & QUIRK_SPI_IDLE)))
		return  _qtec_cmosis_spi_array(priv, do_write, len, addr, data);

	if (priv->exp)
		aux = (uint64_t) priv->exp->val * (uint32_t)HZ * 3;
	else
		aux = HZ;
	do_div(aux,1000000);
	expiration = aux;

	aux = (uint64_t) priv->fival.numerator * (uint64_t) HZ * 3;
	do_div(aux,priv->fival.denominator);

	expiration = max_t(unsigned long, aux, expiration);
	expiration = max_t(unsigned long, HZ/10, expiration);
	expiration += jiffies;

	qtec_cmosis_read(priv,CONTROL,&old_ctrl);
	if (old_ctrl&BIT(WSLOCK)){
		uint32_t aux,framcnt;
		aux=old_ctrl&~(BIT(TRIG_ENABLE));
		qtec_cmosis_write(priv,CONTROL,aux);
		qtec_cmosis_read(priv,FRAMECNT,&framcnt);
		do {
			qtec_cmosis_read(priv,FRAMECNT,&aux);
			if ((aux&1) != (framcnt&1))
				break;
		}while(time_before(jiffies,expiration));
		if (!time_before(jiffies,expiration))
			v4l2_err(&priv->sd, "Timeout waiting for cmosis idle \n");
	}

	ret = _qtec_cmosis_spi_array(priv, do_write, len, addr, data);

	qtec_cmosis_write(priv,CONTROL,old_ctrl);

	return ret;
}

static int qtec_cmosis_spi_cmd(struct qtec_cmosis *priv, bool do_write, bool online, uint8_t reg, uint8_t len, uint64_t *value){
	uint16_t data[8];
	uint8_t addr[8];
	int i;
	int ret;

	if ((len > 8 ) || ((priv->quirk & QUIRK_SPI_16_BITS) && len >4))
		return -EINVAL;

	for (i=0;i<len;i++){
		addr[i]=reg+i;
		if (do_write){
			if (priv->quirk & QUIRK_SPI_16_BITS) {
				data [i] = *value & 0xffff;
				*value >>= 16;
			}
			else {
				data [i] = *value & 0xff;
				*value >>= 8;
			}
		}
	}

	ret = qtec_cmosis_spi_array(priv, do_write, online, len, addr, data);

	if (!do_write){
		*value = 0;
		for (i=0;i<len;i++){
			if (priv->quirk & QUIRK_SPI_16_BITS)
				*value |= (data[i] & 0xffff) << 16*i;
			else
				*value |= (data[i] & 0xff) << 8*i;
		}
	}

	return ret;
}

static inline int qtec_cmosis_spi_read(struct qtec_cmosis *priv, uint8_t reg, uint8_t len, uint64_t *value){
	return qtec_cmosis_spi_cmd(priv,false,false,reg,len,value);
}

static inline int qtec_cmosis_spi_write(struct qtec_cmosis *priv, bool online, uint8_t reg, uint8_t len, uint64_t value){
	return qtec_cmosis_spi_cmd(priv,true,online,reg,len, &value);
}

static inline int qtec_cmosis_spi_write_mask(struct qtec_cmosis *priv, bool online, uint8_t reg, uint16_t mask, uint16_t value){
	uint64_t aux;

	if (!mask)
		return 0;

	if (( (priv->quirk & QUIRK_SPI_16_BITS) && (mask==0xffff)) ||
		( !(priv->quirk & QUIRK_SPI_16_BITS) && (mask==0xff))){
			aux = value;
			return qtec_cmosis_spi_cmd(priv,true,online,reg,1, &aux);
		}

	qtec_cmosis_spi_cmd(priv,false,online,reg,1, &aux);
	aux&=~mask;
	aux|=value&mask;
	return qtec_cmosis_spi_cmd(priv,true,online,reg,1, &aux);
}

static int qtec_cmosis_set_vramp(struct qtec_cmosis *priv, int val){
	int ret;

	switch (priv->chip_id){
		case CMV12000v2:
			if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12)
				val = min(val+8,127);
			return qtec_cmosis_spi_write(priv,true,REG_VRAMP_12M,1,val | (val<<7));
		case CMV8000v1:
			if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12)
				val = min(val+6,127);
			ret = qtec_cmosis_spi_write(priv,true,REG_VRAMP1_8M,1,val);
			return ret | qtec_cmosis_spi_write(priv,true,REG_VRAMP2_8M,1,val);
		default:
			ret = qtec_cmosis_spi_write(priv,true,REG_VRAMP1,1,val);
			return ret | qtec_cmosis_spi_write(priv,true,REG_VRAMP2,1,val);
	}

	//Code never reached
	return 0;
}

static int qtec_cmosis_set_sensor_channels(struct qtec_cmosis *priv,int channels){
	uint32_t aux;

	if (priv->chip_id == CMV12000v2){
		switch (channels){
			case 0:
				aux = 0x0;
				break;
			case 2:
				aux = 0x00000001;
				break;
			case 4:
				aux = 0x00010001;
				break;
			case 8:
				aux = 0x01010101;
				break;
			case 16:
			default:
				aux = 0x11111111;
				break;
		}
		qtec_cmosis_spi_write(priv,false,90,2,aux);
		qtec_cmosis_spi_write(priv,false,92,2,aux);
		if (channels)
			qtec_cmosis_spi_write_mask(priv,false,94,0x7,(priv->quirk & QUIRK_NO_PHASE_IDELAY)?0x7:0x6);
		else
			qtec_cmosis_spi_write_mask(priv,false,94,0x7,0);
	} else if (priv->chip_id==CMV8000v1){
		switch (channels){
			case 0:
				aux=31;
				break;
			case 2:
				aux=15;
				break;
			case 4:
				aux=7;
				break;
			case 8:
				aux=5;
				break;
			case 16:
			default:
				aux=1;
				break;
		}
		qtec_cmosis_spi_write_mask(priv,false,59,0x1f,aux);

		if (channels){
			qtec_cmosis_spi_write_mask(priv,false,60,0xf,(priv->quirk & QUIRK_NO_PHASE_IDELAY)?0x7:0x6);
			qtec_cmosis_spi_write_mask(priv,false,121,BIT(4),BIT(4));
		} else {
			qtec_cmosis_spi_write_mask(priv,false,60,0xf,0);
			qtec_cmosis_spi_write_mask(priv,false,121,BIT(4),0);
		}

	} else {
		switch (channels){
			case 0:
				aux=0x00;
				break;
			case 2:
				aux=0x01;
				break;
			case 4:
				aux=0x11;
				break;
			case 8:
				aux=0x55;
				break;
			case 16:
			default:
				aux=0xff;
				break;
		}
		qtec_cmosis_spi_write(priv,false,80,1,aux);
		qtec_cmosis_spi_write(priv,false,81,1,aux);
		if (channels){
			qtec_cmosis_spi_write_mask(priv,false,82,0x3,(priv->quirk & QUIRK_NO_PHASE_IDELAY)?3:2);
			qtec_cmosis_spi_write_mask(priv,false,74,0xf,8);
		} else {
			qtec_cmosis_spi_write_mask(priv,false,82,0x3,0);
			qtec_cmosis_spi_write_mask(priv,false,74,0xf,0);
		}
	}

	return 0;
}

static int qtec_cmosis_init_cmosis_small(struct qtec_cmosis *priv){
	int i;
	static const uint8_t reg_fixedv2[][3]={
		{84,4,0xff},
		{85,1,0xff},
		{88,64,0xff},
		{91,64,0xff},
		{94,101,0xff},
		{95,106,0xff},
		{102,2,0xff}, //PGA
		{103,44,0xff},
		{115,1,0xff},
		{117,1,0xff},
		{55,2,0xff}, //AN11
	};
	static const uint8_t reg_fixedv3[][3]={
		{41,4,0x4},// inte_sync
		{73,20,0xff},
		{74,8,0xff}, //Mail from Pieterjan
		{77,0,0xff},
		{84,4,0xff},
		{85,1,0xff},
		{86,14,0xff},
		{87,12,0xff},
		{88,64,0xff},
		{91,64,0xff},
		{94,101,0xff},
		{95,106,0xff},
		{102,1,0xff},
		{103,40,0xff},
		{123,98,0xff},
		{113,0,0x1},// Mail from Pieterjan
		{115,1,0x1},// Bypass PLL
		{118,0,0xff},// 0 dummy lines. Hack, check mail from Maxim #498
		{123,98,0xff},
		//{117,1,0xff},//Different on datasheet 3.2
		{55,2,0xff}, //AN11
	};

	switch (priv->chip_id){
		case CMV2000v2:
		case CMV4000v2:
			for (i=0;i<(sizeof(reg_fixedv2)/sizeof(reg_fixedv2[0]));i++)
				qtec_cmosis_spi_write_mask(priv,false,reg_fixedv2[i][0],reg_fixedv2[i][2],reg_fixedv2[i][1]);
			break;
		default:
			for (i=0;i<(sizeof(reg_fixedv3)/sizeof(reg_fixedv3[0]));i++)
				qtec_cmosis_spi_write_mask(priv,false,reg_fixedv3[i][0],reg_fixedv3[i][2],reg_fixedv3[i][1]);
			break;
	}

	//Enabled channels
	qtec_cmosis_spi_write_mask(priv,false,82,0x07,0x7); //FIXME Needed?

	//10 or 12 bit
	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12){
		qtec_cmosis_spi_write(priv,false,111,1,0);
		qtec_cmosis_spi_write(priv,false,112,1,2);
		qtec_cmosis_spi_write(priv,false,117,1,4);//Different on datasheet 3.2
		qtec_cmosis_spi_write(priv,false,116,1,11);//Different on datasheet 3.2
	}
	else{
		qtec_cmosis_spi_write(priv,false,111,1,1);
		qtec_cmosis_spi_write(priv,false,112,1,0);
		qtec_cmosis_spi_write(priv,false,117,1,8);//Different on datasheet 3.2
		qtec_cmosis_spi_write(priv,false,116,1,9);//Different on datasheet 3.2
	}

	//training word
	qtec_cmosis_spi_write(priv,false,78,2,priv->sync_word);

	return 0;
}

static inline int qtec_cmosis_row_length(struct qtec_cmosis *priv, int nchans){
	int aux;
	switch (priv->nchans){
		case 2:
			aux=16;
			break;
		case 4:
			aux=8;
			break;
		case 8:
			aux=4;
			break;
		case 16:
		default:
			aux=2;
			break;
	}
	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12)
		aux*=2;

	return aux;

}

static int qtec_cmosis_init_cmosis_big(struct qtec_cmosis *priv){
	int i;
	static const uint8_t reg_fixed[][2]={
		{58,0}, //Workaround for #519
		{68,12},
		{71,29},
		{73,19},
		{74,120},
		{76,8},
		{77,50},
		{91,200},
		{92,240},
		{97,102},
		{98,102},
		{99,8},
		{103,101}, //#545 Black sun
		{106,12},
		{118,15},
	};

	for (i=0;i<(sizeof(reg_fixed)/sizeof(reg_fixed[0]));i++)
		qtec_cmosis_spi_write(priv,false,reg_fixed[i][0],1,reg_fixed[i][1]);

	qtec_cmosis_spi_write_mask(priv,false,62,0x7f,qtec_cmosis_row_length(priv,priv->nchans));
	qtec_cmosis_spi_write_mask(priv,false,77,0xff,priv->fot_len);

	//PLL
	qtec_cmosis_spi_write_mask(priv,false,123,0x1,1); //Gate parallel clock
	qtec_cmosis_spi_write_mask(priv,false,85,0xc,0x80);

	//10 or 12 bit
	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 10){
		priv->slot_len = 112;
		qtec_cmosis_spi_write_mask(priv,false,85,0x3f,41);
		qtec_cmosis_spi_write_mask(priv,false,65,0xff,96);
		qtec_cmosis_spi_write_mask(priv,false,66,0x1,0);
		qtec_cmosis_spi_write_mask(priv,false,71,0xff,29);
		qtec_cmosis_spi_write_mask(priv,false,72,0x1,0);
		qtec_cmosis_spi_write_mask(priv,false,73,0xff,19);
		qtec_cmosis_spi_write_mask(priv,false,74,0xff,120);
		qtec_cmosis_spi_write_mask(priv,false,75,0x3,0);
		qtec_cmosis_spi_write_mask(priv,false,78,0x3,0);
		qtec_cmosis_spi_write_mask(priv,false,82,0x1f,4);
		qtec_cmosis_spi_write_mask(priv,false,83,0x2,2);
		qtec_cmosis_spi_write_mask(priv,false,84,0x1f,4);
		qtec_cmosis_spi_write_mask(priv,false,117,0xc0,0);
	} else{
		priv->slot_len = 122;
		qtec_cmosis_spi_write_mask(priv,false,85,0x3f,43);
		qtec_cmosis_spi_write_mask(priv,false,65,0xff,231&0xff);
		qtec_cmosis_spi_write_mask(priv,false,66,0x1,231>>8);
		qtec_cmosis_spi_write_mask(priv,false,71,0xff,54);
		qtec_cmosis_spi_write_mask(priv,false,72,0x1,0);
		qtec_cmosis_spi_write_mask(priv,false,73,0xff,1);
		qtec_cmosis_spi_write_mask(priv,false,74,0xff,397&0xff);
		qtec_cmosis_spi_write_mask(priv,false,75,0x3,397>>8);
		qtec_cmosis_spi_write_mask(priv,false,78,0x3,3);
		qtec_cmosis_spi_write_mask(priv,false,82,0x1f,1);
		qtec_cmosis_spi_write_mask(priv,false,83,0x2,0);
		qtec_cmosis_spi_write_mask(priv,false,84,0x1f,5);
		qtec_cmosis_spi_write_mask(priv,false,117,0xc0,0xc0);
	}
	qtec_cmosis_spi_write_mask(priv,false,61,0xff,priv->slot_len);
	qtec_cmosis_spi_write_mask(priv,false,123,0x1,0); //Gate parallel clock

	//training word
	qtec_cmosis_spi_write(priv,false,56,1,priv->sync_word);
	qtec_cmosis_spi_write_mask(priv,false,57,0xf,(priv->sync_word>>8));

	return 0;
}

static int qtec_cmosis_reg_timing(int reg, int chans, int bitmode){
	int row = (reg==82)?0:1;
	int idx;
	static const uint16_t values[][8] = {
		{283,	283,	539,	795,	286,	286,	542,	1054}, //82
		{4127,	2063,	1031,	515,	4127,	2063,	1031,	515}, //85
	};

	switch (chans){
		case 2:
			idx = 0;
			break;
		case 4:
			idx = 1;
			break;
		case 8:
			idx = 2;
			break;
		default:
		case 16:
			idx = 3;
			break;
	}

	if (bitmode == 12)
		idx += 4;

	return values[row][idx];
}

static int qtec_cmosis_init_cmosis_huge(struct qtec_cmosis *priv){
	int i;
	int idx;

	static const uint16_t reg_fixed[][9]={
	//      reg	10 2	10 4	10 8	10 16	12 2	12 4	12 8	12 16
		{81,	31,	15,	7,	3,	31,	15,	7,	3},
		{83,	12805,	12805,	12805,	12805,	5897,	5897,	5897,	5897},
		{84,	128,	128,	128,	128,	257,	257,	257,	257},
		{86,	4127,	2063,	1031,	515,	4127,	2063,	1031,	515},
		{87,	540,	540,	540,	540,	1910,	1910,	1910,	1910},
		{88,	524,	524,	524,	524,	1910,	1910,	1910,	1910},
		{98,	44812,	44812,	44812,	44812,	39433,	39433,	39433,	39433},
		{102,	8302,	8302,	8302,	8302,	8302,	8302,	8302,	8302},
		{107,	11614,	11614,	11614,	11614,	11102,	11102,	11102,	11102},
		{108,	12381,	12381,	12381,	12381,	12381,	12381,	12381,	12381},
		{110,	12368,	12368,	12368,	12368,	12368,	12368,	12368,	12368},
		{112,	277,	277,	277,	277,	277,	277,	277,	277},
		{113,	789,	789,	789,	789,	542,	542,	542,	542},
		{114,	84,	84,	84,	84,	200,	200,	200,	200},
		{115,	0,	0,	0,	0,	0,	0,	0,	0},
		{117,	4,	4,	4,	4,	1,	1,	1,	1},
		{118,	1,	1,	1,	1,	0,	0,	0,	0},
		{124,	15,	15,	15,	15,	15,	15,	15,	15},
	};

	switch (priv->nchans){
		case 2:
			idx = 1;
			break;
		case 4:
			idx = 2;
			break;
		case 8:
			idx = 3;
			break;
		default:
		case 16:
			idx = 4;
			break;
	}

	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12)
		idx += 4;

	for (i=0;i<(sizeof(reg_fixed)/sizeof(reg_fixed[0]));i++)
		qtec_cmosis_spi_write(priv,false,reg_fixed[i][0],1,reg_fixed[i][idx]);

	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 10)
		qtec_cmosis_spi_write_mask(priv,true,107,0x7f80,82<<7);
	else
		qtec_cmosis_spi_write_mask(priv,true,107,0x7f80,85<<7);

	qtec_cmosis_spi_write(priv,false,82,1,qtec_cmosis_reg_timing(82,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]));
	qtec_cmosis_spi_write(priv,false,85,1,qtec_cmosis_reg_timing(85,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]));

	//training word
	qtec_cmosis_spi_write_mask(priv,false,89,0xfff,priv->sync_word);

	return 0;
}

static int qtec_cmosis_init_cmosis(struct qtec_cmosis *priv){

	//Prepare for later find_idelay, final value is set by setup_cmosis
	qtec_cmosis_set_sensor_channels(priv,priv->max_channels);

	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_init_cmosis_huge(priv);
	case CMV8000v1:
		return qtec_cmosis_init_cmosis_big(priv);
	default:
		return qtec_cmosis_init_cmosis_small(priv);
	}
	return 0;
}

static int qtec_cmosis_stop_sensor(struct qtec_cmosis *priv){

	qtec_cmosis_set_sensor_channels(priv,0);
	/*Do not set CMV_CTRL, or the registers will have their default values, this is, 16 channels 190 mA!*/

	return 0;
}

static int qtec_cmosis_reset_sensor(struct qtec_cmosis *priv, bool wait_pll){
	uint32_t aux;

	qtec_cmosis_read(priv,CMV_CTRL,&aux);
	aux |= BIT(RESET);
	qtec_cmosis_write(priv,CMV_CTRL,aux);
	msleep(1);
	aux &= ~BIT(RESET);

	if (wait_pll && (priv->quirk & QUIRK_NO_PHASE_IDELAY)){
		unsigned long expiration;
		uint32_t lock;

		qtec_cmosis_write(priv,CMV_CTRL,aux);
		expiration=jiffies+HZ;
		do{
			qtec_cmosis_read(priv,CMV_CTRL,&lock);
			if (lock & BIT(CMV_LOCK_BIT))
					break;
		}while(time_before(jiffies,expiration));

		if (!time_before(jiffies,expiration))
			v4l2_err(&priv->sd, "Timeout waiting for MMCM 0x%.8x\n", lock);
	}

	if (priv->quirk & QUIRK_FREQ_DELAY){
		if (((priv->chip_id==CMV2000v2)||(priv->chip_id==CMV4000v2))&&(priv->mode_trig->val != SELF_TIMED))
			aux &= ~BIT(CMV_FREQ_DELAY);
		else
			aux |= BIT(CMV_FREQ_DELAY);
	}

	qtec_cmosis_write(priv,CMV_CTRL,aux);
	msleep(1);//Required 1usec


	return 0;
}

static inline uint32_t qtec_cmosis_time2lines_huge(struct qtec_cmosis *priv, uint32_t time){
	uint64_t int_lines;
	uint64_t sub_lines;
	int aux;

	int_lines = (uint64_t)time * (uint64_t) priv->pixel_clk;
	int_lines += 1000000/2;
	do_div(int_lines,1000000);

	sub_lines = 1;
	sub_lines += 34 * (qtec_cmosis_reg_timing(82,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) & 0xff);

	if (sub_lines > int_lines)
		return 0;

	int_lines -= sub_lines;

	aux = qtec_cmosis_reg_timing(85,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val])+1;
	int_lines += aux/2;
	do_div(int_lines, aux);
	int_lines +=1;

	return int_lines;
}

static inline uint32_t qtec_cmosis_time2lines_big(struct qtec_cmosis *priv, uint32_t time){
	uint64_t int_lines;
	uint64_t sub_lines;

	int_lines = (uint64_t)time * (uint64_t) priv->pixel_clk;
	int_lines += 1000000/2;
	do_div(int_lines,1000000);

	sub_lines = 1;
	sub_lines += 148 * priv->fot_len;

	if (sub_lines > int_lines)
		return 0;

	int_lines -= sub_lines;
	int_lines += (priv->slot_len +1)/2;
	do_div(int_lines,(priv->slot_len +1));

	return int_lines;
}

static inline uint32_t qtec_cmosis_time2lines_small(struct qtec_cmosis *priv, uint32_t time){
	uint64_t int_lines;
	int aux;

	int_lines = (uint64_t) 100 * (uint64_t)time*(uint64_t) priv->pixel_clk;
	aux = 129*1000000;
	int_lines += aux/2;
	do_div(int_lines,aux);

	int_lines -= (uint64_t)43 * (uint64_t)priv->fot_len;

	int_lines +=50;
	do_div(int_lines,100);

	return int_lines;
}

static inline uint32_t qtec_cmosis_time2lines(struct qtec_cmosis *priv, uint32_t time){
	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_time2lines_huge(priv, time);
	case CMV8000v1:
		return qtec_cmosis_time2lines_big(priv, time);
	default:
		return qtec_cmosis_time2lines_small(priv, time);
	}

}

static int qtec_cmosis_set_flash_width(struct qtec_cmosis *priv){
	uint64_t exposure_clk;

	exposure_clk = (uint64_t)priv->exp->val * (uint64_t)priv->bus_clk;
	exposure_clk -= 1000000/2;
	do_div(exposure_clk, 1000000);

	qtec_cmosis_write(priv, FLASH_TIME, exposure_clk);

	return 0;
}

static int qtec_cmosis_set_exposure_huge(struct qtec_cmosis *priv, uint32_t lines){
	uint8_t addr[15];
	uint16_t data[15];
	uint32_t aux_lines;

	addr[0] = REG_EXPOSURE_12M;
	data[0] = lines & 0xffff;
	addr[1] = REG_EXPOSURE_12M+1;
	data[1] = (lines >> 16) & 0xff;

	addr[2] = 79;

	if (priv->resp_curve->val == 0){
		data[2] = 1;
		addr[3] = 70;
		data[3] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;
		return qtec_cmosis_spi_array(priv, true, true, 4, addr, data);
	}

	//HDR
	if (priv->resp_curve->val == 3){
		data[2] = 2;
		aux_lines = (lines * 9) / 100;
		addr[3] = 75;
		data[3] = aux_lines & 0xffff;
		addr[4] = 76;
		data[4] = (aux_lines >> 16) & 0xff;
		addr[5] = 106;
		data[5] =  96 | (96 << 7);
		addr[6] = 70;
		data[6] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;
		return qtec_cmosis_spi_array(priv,true,true, 7, addr, data);
	}

	//DUAL EXPOSURE
	if (priv->resp_curve->val == 4){
		uint32_t dual_lines = qtec_cmosis_time2lines(priv,priv->dual_exp->val);

		data[2] = 1;
		dual_lines = clamp(dual_lines, (uint32_t) 1, lines);
		addr[3] = REG_EXPOSURE_12M+2;
		data[3] = dual_lines & 0xffff;
		addr[4] = REG_EXPOSURE_12M+3;
		data[4] = (dual_lines >> 16) & 0xff;
		addr[5] = 70;
		data[5] = (priv->mode_trig->val == EXT_EXPOSURE) ? 3 : 2;
		return qtec_cmosis_spi_array(priv,true,true, 6, addr, data);
	}


	data[2] = 3;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?11:15))/100;
	addr[3] = 75;
	data[3] = aux_lines & 0xffff;
	addr[4] = 76;
	data[4] = (aux_lines >> 16) & 0xff;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?24:44))/100;
	addr[5] = 77;
	data[5] = aux_lines & 0xffff;
	addr[6] = 78;
	data[6] = (aux_lines >> 16) & 0xff;

	aux_lines = (priv->resp_curve->val == 1)?89:80;
	aux_lines |= ((priv->resp_curve->val == 1)?105:102) << 7;
	addr[7] = 106;
	data[7] = aux_lines;
	addr[8] = 70;
	data[8] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;

	return qtec_cmosis_spi_array(priv,true,true, 9, addr, data);
}

static int qtec_cmosis_set_exposure_big(struct qtec_cmosis *priv, uint32_t lines){
	uint8_t addr[15];
	uint16_t data[15];
	uint32_t aux_lines;

	addr[0] = REG_EXPOSURE_8M;
	data[0] = lines & 0xff;
	addr[1] = REG_EXPOSURE_8M+1;
	data[1] = (lines >> 8) & 0xff;
	addr[2] = REG_EXPOSURE_8M+2;
	data[2] = (lines >> 16) & 0xff;

	addr[3] = 47;

	if (priv->resp_curve->val == 0){
		data[3] = 1;
		addr[4] = 40;
		data[4] = (priv->mode_trig->val == EXT_EXPOSURE) ? 5 : 4;
		return qtec_cmosis_spi_array(priv, true, true, 5, addr, data);
	}

	//DUAL EXPOSURE
	if (priv->resp_curve->val == 4){
		uint32_t dual_lines = qtec_cmosis_time2lines(priv,priv->dual_exp->val);

		data[3] = 1;
		dual_lines = clamp(dual_lines, (uint32_t) 1, lines);
		addr[4] = 40;
		data[4] = (priv->mode_trig->val == EXT_EXPOSURE) ? 7 : 6;
		addr[5] = REG_EXPOSURE_8M+3;
		data[5] = dual_lines & 0xff;
		addr[6] = REG_EXPOSURE_8M+4;
		data[6] = (dual_lines >> 8) & 0xff;
		addr[7] = REG_EXPOSURE_8M+5;
		data[7] = (dual_lines >> 16) & 0xff;
		return qtec_cmosis_spi_array(priv, true, true, 8, addr, data);
	}

	//HDR
	if (priv->resp_curve->val == 3){
		data[3] = 2;
		aux_lines = (lines * 9)/100;
		addr[4] = 48;
		data[4] = aux_lines & 0xff;
		addr[5] = 49;
		data[5] = (aux_lines >> 8) & 0xff;
		addr[6] = 50;
		data[6] = (aux_lines >> 16) & 0xff;
		addr[7] = 95;
		data[7] = 96;
		addr[8] = 40;
		data[8] = 4;

		return qtec_cmosis_spi_array(priv,true,true, 9, addr, data);
	}


	data[3] = 3;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?11:15))/100;
	addr[4] = 48;
	data[4] = aux_lines & 0xff;
	addr[5] = 49;
	data[5] = (aux_lines >> 8) & 0xff;
	addr[6] = 50;
	data[6] = (aux_lines >> 16) & 0xff;

	addr[7] = 95;
	data[7] = (priv->resp_curve->val == 1)?89:80;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?24:44))/100;
	addr[8] = 51;
	data[8] = aux_lines & 0xff;
	addr[9] = 52;
	data[9] = (aux_lines >> 8) & 0xff;
	addr[10] = 53;
	data[10] = (aux_lines >> 16) & 0xff;

	addr[11] = 96;
	data[11] = (priv->resp_curve->val == 1)?105:102;
	addr[12] = 40;
	data[12] = (priv->mode_trig->val == EXT_EXPOSURE) ? 5 : 4;

	return qtec_cmosis_spi_array(priv,true,true, 13, addr, data);
}


static int qtec_cmosis_set_exposure_small(struct qtec_cmosis *priv,uint32_t lines){
	uint32_t aux_lines;
	uint8_t addr[16];
	uint16_t data[16];

	addr[0] = REG_EXPOSURE;
	data[0] = lines & 0xff;
	addr[1] = REG_EXPOSURE+1;
	data[1] = (lines >> 8) & 0xff;
	addr[2] = REG_EXPOSURE+2;
	data[2] = (lines >> 16) & 0xff;

	aux_lines =  (0x1000000-lines) & 0xffffff;

	addr[3] = REG_EXPOSURE+3;
	data[3] = aux_lines & 0xff;
	addr[4] = REG_EXPOSURE+4;
	data[4] = (aux_lines >> 8) & 0xff;
	addr[5] = REG_EXPOSURE+5;
	data[5] = (aux_lines >> 16) & 0xff;

	addr[6] = REG_NSLOPES;

	if (priv->resp_curve->val == 0){
		data[6] = 1;
		addr[7] = 41;
		data[7] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;
		return qtec_cmosis_spi_array(priv, true, true,8, addr, data);
	}

	//DUAL EXPOSURE
	if (priv->resp_curve->val == 4){
		uint32_t dual_lines = qtec_cmosis_time2lines(priv,priv->dual_exp->val);

		data[6] = 1;
		dual_lines = clamp(dual_lines, (uint32_t) 1, lines);
		addr[7] = 56;
		data[7] = dual_lines & 0xff;
		addr[8] = 57;
		data[8] = (dual_lines >> 8) & 0xff;
		addr[9] = 58;
		data[9] = (dual_lines >> 16) & 0xff;
		addr[10] = 41;
		data[10] = (priv->mode_trig->val == EXT_EXPOSURE) ? 3 : 2;
		return qtec_cmosis_spi_array(priv, true, true,11, addr, data);
	}

	//HDR
	if (priv->resp_curve->val == 3){
		data[6] = 2;
		aux_lines =  (lines * 9)/100;
		addr[7] = 48;
		data[7] = aux_lines & 0xff;
		addr[8] = 49;
		data[8] = (aux_lines >> 8) & 0xff;
		addr[9] = 50;
		data[9] = (aux_lines >> 16) & 0xff;
		addr[10] = 90;
		data[10] = 96;
		addr[11] = 41;
		data[11] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;
		return qtec_cmosis_spi_array(priv,true,true, 12, addr, data);
	}

	data[6] = 3;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?11:15))/100;
	addr[7] = 48;
	data[7] = aux_lines & 0xff;
	addr[8] = 49;
	data[8] = (aux_lines >> 8) & 0xff;
	addr[9] = 50;
	data[9] = (aux_lines >> 16) & 0xff;

	addr[10] = 89;
	data[10] = (priv->resp_curve->val == 1)?89:80;

	aux_lines = (lines * ((priv->resp_curve->val == 1)?24:44))/100;
	addr[11] = 51;
	data[11] = aux_lines & 0xff;
	addr[12] = 52;
	data[12] = (aux_lines >> 8) & 0xff;
	addr[13] = 53;
	data[13] = (aux_lines >> 16) & 0xff;

	addr[14] = 90;
	data[14] = (priv->resp_curve->val == 1)?105:102;

	addr[15] = 41;
	data[15] = (priv->mode_trig->val == EXT_EXPOSURE) ? 1 : 0;

	return qtec_cmosis_spi_array(priv,true,true, 16, addr, data);
}

static inline uint32_t _qtec_cmosis_set_exposure(struct qtec_cmosis *priv){
	uint32_t lines= qtec_cmosis_time2lines(priv,priv->exp->val);

#ifdef DEBUG_CLKS
	qtec_cmosis_horiz_line_dbg(priv);
#endif
	if (priv->quirk & QUIRK_PROG_FLASH_WIDTH)
		qtec_cmosis_set_flash_width(priv);

	lines=clamp(lines,(uint32_t)1,(uint32_t)0xffffff);

	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_set_exposure_huge(priv, lines);
	case CMV8000v1:
		return qtec_cmosis_set_exposure_big(priv, lines);
	default:
		return qtec_cmosis_set_exposure_small(priv, lines);
	}
}


static inline uint32_t qtec_cmosis_lines2time(struct qtec_cmosis *priv, uint32_t time);
static int qtec_cmosis_set_exposure(struct qtec_cmosis *priv){
	int ret = _qtec_cmosis_set_exposure(priv);
	if (!ret)
		priv->exp->cur.val = priv->exp->val = qtec_cmosis_lines2time(priv,qtec_cmosis_time2lines(priv,priv->exp->val));

	return ret;
}

static int qtec_cmosis_set_pga_small(struct qtec_cmosis *priv,int *req_val, bool set){
	int val=0;
	int pgadb=*req_val;

	if (pgadb > 9542){ //3.0
		val=7;//3.2
		*req_val=10103;
	} else if (pgadb > 8299){ //2.6
		val=6;//2.8
		*req_val=8493;
	} else if (pgadb > 6848) { //2.2
		val=5;//2.4
		*req_val=7604;
	} else if (pgadb > 5105){ //1.8
		val=4;//2.0
		*req_val=6021;
	} else if (pgadb > 3522){//1.5
		val=3; //1.6
		*req_val=4082;
	} else if (pgadb > 2279){//1.3
		val=2; //1.4
		*req_val=2923;
	} else if (pgadb > 828){ //1.1
		val=1; //1.2
		*req_val=1584;
	} else{
		val=0; //1.0
		*req_val=0;
	}

	if (!set)
		return 0;

	qtec_cmosis_spi_write_mask(priv,true,REG_PGA10,REG_PGA10_MASK,val&0x3);
	return qtec_cmosis_spi_write_mask(priv,true,REG_PGA2,REG_PGA2_MASK,(BIT(2)&val)?1:0);
}

static int qtec_cmosis_set_pga_big(struct qtec_cmosis *priv,int *req_val, bool set){
	int val=0;
	int pgadb=*req_val;

	if (pgadb > 10881){//3.5
		val=13; //4
		*req_val=12041;
	} else if (pgadb > 7958){//2.5
		val=5; //3
		*req_val=9542;
	} else if (pgadb > 3521){ //1.5
		val=14; //2
		*req_val=6021;
	} else if (pgadb > 13637){ //1.17
		val=15; //1.33 #518
		*req_val=2477;
	} else {
		val=7; //1.0
		*req_val=0;
	}

	if (!set)
		return 0;

	return qtec_cmosis_spi_write_mask(priv,true,REG_PGA_8M,0xf,val);
}

static int qtec_cmosis_set_pga_huge(struct qtec_cmosis *priv,int *req_val, bool set){
	int val=0;
	int pgadb=*req_val;

	if (pgadb > 10881){//3.5
		val=7; //4
		*req_val=12041;
	} else if (pgadb > 7958){//2.5
		val=3; //3
		*req_val=9542;
	} else if (pgadb > 3521){ //1.5
		val=1; //2
		*req_val=6021;
	} else {
		val=0; //1.0
		*req_val=0;
	}

	if (!set)
		return 0;

	return qtec_cmosis_spi_write_mask(priv,true,REG_PGA_12M,0xf,val);
}

static int qtec_cmosis_set_pga(struct qtec_cmosis *priv,int *req_val, bool set){

	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_set_pga_huge(priv, req_val, set);
	case CMV8000v1:
		return qtec_cmosis_set_pga_big(priv, req_val, set);
	default:
		return qtec_cmosis_set_pga_small(priv, req_val, set);
	}
}

static int qtec_cmosis_crop_height(struct qtec_cmosis *priv){
	int height=0,i;

	for (i=0;i<priv->n_crop;i++)
		if (priv->crop[i].height<1)
			return 0;
		else
			height+=priv->crop[i].height;

	return height;
}

#define REG_TOTAL_LINES 1
#define REG_N_LINES1 19
#define REG_START1 3

static bool qtec_cmosis_is_vdecim_format(u32 format){
	switch(format){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGB:
		case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
		case MEDIA_BUS_FMT_QTEC_FB4_GREEN:
		case MEDIA_BUS_FMT_QTEC_FB4_GREY:
			return true;
	}

	return false;
}
static inline int qtec_cmosis_vparam_calc(struct qtec_cmosis *priv,int val,int vskip){

	if (qtec_cmosis_is_vdecim_format(priv->format.code))
		val*=2;

	val = priv->v_binning->val*val/vskip;

	return val;
}

static inline int qtec_cmosis_readout_lines(struct qtec_cmosis *priv,struct v4l2_mbus_framefmt format){
	return qtec_cmosis_vparam_calc(priv,format.height,1);
}

static int qtec_cmosis_set_offset(struct qtec_cmosis *priv, int val){
	int ret;
	switch (priv->chip_id){
	case CMV12000v2:
		if (priv->bitmode->qmenu_int[priv->bitmode->val] == 12){
			val = val * 3736; //Found via experimentation, datasheet uses 3.5588
			val /= 1000;
			val = clamp (val, -2048, 2047);
		}
		ret = qtec_cmosis_spi_write_mask(priv,true,REG_OFFSET_12M_BOT,0xfff, val);
		return ret | qtec_cmosis_spi_write_mask(priv,true,REG_OFFSET_12M_TOP,0xfff, val);
	case CMV8000v1:
		return qtec_cmosis_spi_write(priv,true,REG_OFFSET_8M,2, val);
	default:
		return qtec_cmosis_spi_write(priv,true,REG_OFFSET,2, val);
	}

	return 0;
}

static int qtec_cmosis_set_adc_gain_huge(struct qtec_cmosis *priv,int val){
	int offset;
	int new_offset;
	int multiplier;

	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 10){
		offset = 165;
		multiplier = 0;
	} else{
		offset = 120;
		multiplier = 3;
	}

	new_offset = offset + val;
	new_offset = clamp(new_offset,0,255);

	qtec_cmosis_spi_write_mask(priv,true,116,0x3ff,(3<<8)|new_offset);
	return qtec_cmosis_spi_write_mask(priv,true,100,0x3,multiplier);
}

static int qtec_cmosis_set_adc_gain(struct qtec_cmosis *priv, int val){

	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_set_adc_gain_huge(priv, val);
		return 0;
	case CMV8000v1:
		return qtec_cmosis_spi_write_mask(priv,true,REG_GAIN_8M,0x3f, val);
	default:
		return qtec_cmosis_spi_write(priv,true,REG_GAIN,1, val);
	}

	return 0;
}

static int qtec_cmosis_vskip(struct qtec_cmosis *priv){
	int vskip;

	if (!priv->format.height)
		return 1;

	vskip = qtec_cmosis_crop_height(priv);
	vskip/=priv->format.height;
	if (vskip<1)
		vskip=1;

	return vskip;
}


static bool qtec_cmosis_yflip_huge(struct qtec_cmosis *priv){
	uint32_t yflip;

	yflip = (qtec_cmosis_vparam_calc(priv,priv->crop[0].top,1) >> 1) & 1;
	yflip ^= v4l2_ctrl_g_ctrl(priv->hflip);

	if (qtec_cmosis_is_vdecim_format(priv->format.code))
		yflip ^= v4l2_ctrl_g_ctrl(priv->vflip);

	yflip = !!yflip;

	return yflip;
}

static int qtec_cmosis_set_yflip_huge(struct qtec_cmosis *priv){
	uint32_t aux;
	bool yflip = qtec_cmosis_yflip_huge(priv);

	qtec_cmosis_read(priv,CONTROL,&aux);
	if (yflip)
		aux|=BIT(YFLIP);
	else
		aux&=~BIT(YFLIP);
	qtec_cmosis_write(priv,CONTROL,aux);

	return 0;
}

static int qtec_cmosis_set_crop_top_sensor(struct qtec_cmosis *priv, int vskip){
	int i;

	if (priv->chip_id == CMV12000v2)
		for (i=0;i<priv->n_crop;i++)
			qtec_cmosis_spi_write(priv,false,2+i,1, qtec_cmosis_vparam_calc(priv,priv->crop[i].top,vskip));
	else
		for (i=0;i<priv->n_crop;i++)
			qtec_cmosis_spi_write(priv,false,3+2*i,2, qtec_cmosis_vparam_calc(priv,priv->crop[i].top,vskip));

	return 0;
}

static int qtec_cmosis_setup_cmosis_huge(struct qtec_cmosis *priv, int vskip){
	int i;

	qtec_cmosis_spi_write(priv,false,REG_TOTAL_LINES,1,qtec_cmosis_readout_lines(priv, priv->format));
	for (i=0;i<priv->n_crop;i++)
		qtec_cmosis_spi_write(priv,false,34+i,1, qtec_cmosis_vparam_calc(priv,priv->crop[i].height,vskip));

	qtec_cmosis_spi_write(priv,false,REG_FLIP_12M,1,
			(v4l2_ctrl_g_ctrl(priv->hflip)<<HFLIP_IMG)|
			(v4l2_ctrl_g_ctrl(priv->vflip)<<VFLIP_IMG));

	if (v4l2_ctrl_g_ctrl(priv->mode_trig) == EXT_EXPOSURE)
		qtec_cmosis_spi_write_mask(priv,false,70,0x01,1);
	else
		qtec_cmosis_spi_write_mask(priv,false,70,0x01,0);

	if (v4l2_ctrl_g_ctrl(priv->bayer_skipping)){
		qtec_cmosis_spi_write(priv,false,66,1, 0);
		qtec_cmosis_spi_write(priv,false,67,1, vskip);
		qtec_cmosis_spi_write(priv,false,68,1, 0);
	} else {
		qtec_cmosis_spi_write(priv,false,66,1, vskip/2);
		qtec_cmosis_spi_write(priv,false,67,1, vskip);
		qtec_cmosis_spi_write(priv,false,68,1, 9);
	}

	msleep(1);//Required by 2.9.7 doc (page 18)

	return 0;
}

static int qtec_cmosis_setup_cmosis_big(struct qtec_cmosis *priv, int vskip){
	int i;
	bool is_12bit = priv->bitmode->qmenu_int[priv->bitmode->val]==12;


	switch (priv->nchans){
		case 16:
			qtec_cmosis_spi_write_mask(priv,false,59,0x1f,1);
			qtec_cmosis_spi_write_mask(priv,false,62,0x7f,is_12bit?4:2);
			break;
		case 8:
			qtec_cmosis_spi_write_mask(priv,false,59,0x1f,3);
			qtec_cmosis_spi_write_mask(priv,false,62,0x7f,is_12bit?8:4);
			break;
		case 4:
			qtec_cmosis_spi_write_mask(priv,false,59,0x1f,7);
			qtec_cmosis_spi_write_mask(priv,false,62,0x7f,is_12bit?16:8);
			break;
		default:
		case 2:
			qtec_cmosis_spi_write_mask(priv,false,59,0x1f,15);
			qtec_cmosis_spi_write_mask(priv,false,62,0x7f,is_12bit?32:16);
			break;
	}

	qtec_cmosis_spi_write(priv,false,REG_TOTAL_LINES,2,qtec_cmosis_readout_lines(priv, priv->format));
	for (i=0;i<priv->n_crop;i++)
		qtec_cmosis_spi_write(priv,false,REG_N_LINES1+2*i,2, qtec_cmosis_vparam_calc(priv,priv->crop[i].height,vskip));

	qtec_cmosis_spi_write(priv,false,REG_FLIP_8M,1,
			(v4l2_ctrl_g_ctrl(priv->hflip)<<HFLIP_IMG)|
			(v4l2_ctrl_g_ctrl(priv->vflip)<<VFLIP_IMG));

	if (v4l2_ctrl_g_ctrl(priv->mode_trig) == EXT_EXPOSURE)
		qtec_cmosis_spi_write_mask(priv,false,40,0x01,1);
	else
		qtec_cmosis_spi_write_mask(priv,false,40,0x01,0);

	vskip--;
	qtec_cmosis_spi_write_mask(priv,false,121,0x1,(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?0:1);
	qtec_cmosis_spi_write(priv,false,35,2,
			(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?0:vskip);
	qtec_cmosis_spi_write(priv,false,37,2,
			(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?vskip*2:vskip);

	msleep(1);//Required by 2.9.7 doc (page 18)

	return 0;
}

static int qtec_cmosis_setup_cmosis_small(struct qtec_cmosis *priv,int vskip){
	int i;

	switch (priv->nchans){
		case 16:
			qtec_cmosis_spi_write(priv,false,72,1,0);
			break;
		case 8:
			qtec_cmosis_spi_write(priv,false,72,1,1);
			break;
		case 4:
			qtec_cmosis_spi_write(priv,false,72,1,2);
			break;
		default:
		case 2:
			qtec_cmosis_spi_write(priv,false,72,1,3);
			break;
	}

	qtec_cmosis_spi_write(priv,false,REG_TOTAL_LINES,2,qtec_cmosis_readout_lines(priv, priv->format));
	for (i=0;i<priv->n_crop;i++)
		qtec_cmosis_spi_write(priv,false,REG_N_LINES1+2*i,2, qtec_cmosis_vparam_calc(priv,priv->crop[i].height,vskip));

	qtec_cmosis_spi_write(priv,false,REG_FLIP,1,
			(v4l2_ctrl_g_ctrl(priv->hflip)<<HFLIP_IMG)|
			(v4l2_ctrl_g_ctrl(priv->vflip)<<VFLIP_IMG));

	if (v4l2_ctrl_g_ctrl(priv->mode_trig) == EXT_EXPOSURE)
		qtec_cmosis_spi_write_mask(priv,false,41,0x01,1);
	else
		qtec_cmosis_spi_write_mask(priv,false,41,0x01,0);

	vskip--;
	qtec_cmosis_spi_write_mask(priv,false,39,0x1,(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?0:1);
	qtec_cmosis_spi_write(priv,false,35,2,
			(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?0:vskip);
	qtec_cmosis_spi_write(priv,false,37,2,
			(v4l2_ctrl_g_ctrl(priv->bayer_skipping))?vskip*2:vskip);

	msleep(1);//Required by 2.9.7 doc (page 18)

	return 0;
}

static int qtec_cmosis_setup_cmosis(struct qtec_cmosis *priv){
	int vskip = qtec_cmosis_vskip(priv);

	qtec_cmosis_set_sensor_channels(priv,priv->nchans);
	qtec_cmosis_set_pga(priv,&priv->pga_gain->val, true);
	qtec_cmosis_set_exposure(priv);
	qtec_cmosis_set_offset(priv,v4l2_ctrl_g_ctrl(priv->offset));
	qtec_cmosis_set_adc_gain(priv,priv->adc_gain->val);
	qtec_cmosis_set_vramp(priv, priv->vramp->val);
	qtec_cmosis_set_crop_top_sensor(priv, vskip);

	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_setup_cmosis_huge(priv, vskip);
	case CMV8000v1:
		return qtec_cmosis_setup_cmosis_big(priv, vskip);
	default:
		return qtec_cmosis_setup_cmosis_small(priv, vskip);
	}

}

static int qtec_cmosis_max_chans_width_small(struct qtec_cmosis *priv, const struct qtec_cmosis_format *qtec_format, int width){

	switch (qtec_format->mode_out){
		case MODE_SINGLE: //Not tested Check #456
			switch (width){
				case 0 ... 125:
					return 16;
				case 126 ... 254:
					return 8;
				case 255 ... 512:
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_DECIM:
			switch (width){
				case 0 ... 256:
					return 16;
				case 257 ... 512:
					return 8;
				case 513 ... 1024:
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_COMPACT:
		default:
			switch (width){
				case 0 ... 540:
					return 16;
				case 541 ... 1060:
					return 8;
				default:
					return 4;
			}
			break;
	}

	return -1;
}

static int qtec_cmosis_max_chans_width_big(struct qtec_cmosis *priv, const struct qtec_cmosis_format *qtec_format, int width){

	switch (qtec_format->mode_out){
		case MODE_SINGLE: //Not tested Check #456
			switch (width){
				case 0 ... 225:
					return 16;
				case 226 ... 451:
					return 8;
				case 452 ... 903:
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_DECIM:
			switch (width){
				case 0 ... 451:
					return 16;
				case 452 ... 903:
					return 8;
				case 904 ... 1971:
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_COMPACT:
		default:
			switch (width){
				case 0 ... 1125:
					return 16;
				case 1126 ... 2255:
					return 8;
				default:
					return 4;
			}
			break;
	}

	return -1;
}

static int qtec_cmosis_max_chans_width_huge(struct qtec_cmosis *priv, const struct qtec_cmosis_format *qtec_format, int width){

	switch (qtec_format->mode_out){
		case MODE_SINGLE: //Not tested Check #456
			switch (width){
				case 0 ... 257:
					return 16;
				case 258 ... 515:
					return 8;
				case 516 ... 1031:
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_DECIM:
			switch (width){
				case 0 ... 515:
					return 16;
				case 516 ... 1031:
					return 8;
				case 1032 ... 2048: //Finn calc 2047, but 2048 also works
					return 4;
				default:
					return 2;
			}
			break;
		case MODE_COMPACT:
		default:
			switch (width){
				case 0 ... 1260:
					return 16;
				case 1261 ... 2540:
					return 8;
				default:
					return 4;
			}
			break;
	}

	return -1;
}

static int qtec_cmosis_max_nchans_fmt(struct qtec_cmosis *priv, struct v4l2_mbus_framefmt format){
	const struct qtec_cmosis_format *qtec_format=get_format(format.code);
	int nchans;
	int width;

	if (priv->quirk & QUIRK_FB4)
		nchans = 16;
	else {

		if (!qtec_format)
			return -EINVAL;

		width=format.width * priv->h_binning->val;
		if (priv->quirk & QUIRK_FAST_BINNING)
			width /= priv->v_binning->val;

		switch (priv->chip_id){
			case CMV12000v2:
				nchans = qtec_cmosis_max_chans_width_huge(priv, qtec_format, width);
				break;
			case CMV8000v1:
				nchans = qtec_cmosis_max_chans_width_big(priv, qtec_format, width);
				break;
			default:
				nchans = qtec_cmosis_max_chans_width_small(priv, qtec_format, width);
				break;
		}

		if (nchans < 2)
			nchans = 2;
	}

	switch (priv->chip_id){
		case CMV12000v2:
		case CMV8000v1:
			break;
		default:
			// On 12 bit mode max channels is 4
			if ((priv->bitmode->qmenu_int[priv->bitmode->val]==12) && (nchans > 4))
				nchans=4;
	}

	if (nchans>priv->sync_channels)
		nchans=priv->sync_channels;

	return nchans;
}

static inline uint32_t qtec_cmosis_readout_clks(struct qtec_cmosis *priv,struct v4l2_mbus_framefmt format,bool extra_line){
	uint32_t clks;
	int nchans = qtec_cmosis_max_nchans_fmt(priv,format);

	clks = qtec_cmosis_readout_lines(priv,format);
	if (extra_line)
		clks++;

	if (priv->chip_id == CMV12000v2){
		clks = (clks +1)/2; //Dual side
		clks *= qtec_cmosis_reg_timing(85,nchans,priv->bitmode->qmenu_int[priv->bitmode->val])  +1;
		return clks;
	}

	if ((priv->chip_id == CMV8000v1) && (priv->bitmode->qmenu_int[priv->bitmode->val]==12) && (nchans>4)) //#518
		clks*= 427;
	else{
		unsigned int k1,k2;
		if (priv->chip_id == CMV8000v1){
			k1=113;
			k2=32;
		} else {
			k1=129;
			k2=16;
		}

		clks*= k1*k2;
		clks/=nchans;
	}

	return clks;
}

static inline uint32_t qtec_cmosis_fot_clks(struct qtec_cmosis *priv,struct v4l2_mbus_framefmt format){
	int nchans= qtec_cmosis_max_nchans_fmt(priv,format);
	uint32_t fot;

	if (priv->chip_id == CMV12000v2){
		fot = (qtec_cmosis_reg_timing(82,nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) >> 8) & 0xff;
		fot += 2;
		fot *= qtec_cmosis_reg_timing(85,nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) +1;
		return fot;
	}

	//#522
	if (priv->chip_id == CMV8000v1){
		fot = 1 + ((priv->bitmode->qmenu_int[priv->bitmode->val]==10)?112:122);
		fot *= 2 * qtec_cmosis_row_length(priv,nchans);
		fot += priv->fot_len * 224 +3 ;

		return fot;
	}

	return (priv->fot_len + (32/nchans))*129;
}

static inline uint32_t qtec_cmosis_exp_clks(struct qtec_cmosis *priv){
	uint64_t aux;

	aux =(uint64_t)  priv->exp->val * (uint64_t) priv->pixel_clk;

	if (priv->chip_id == CMV12000v2){
		do_div(aux,1000000);
		aux -= 34 * (qtec_cmosis_reg_timing(82,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) & 0xff);
		return aux;
	}

	do_div(aux,10000);
	aux-= 43 * priv->fot_len * 129; //Meassured
	do_div(aux,100);

	return aux;
}

static inline uint64_t qtec_cmosis_frame_clks(struct qtec_cmosis *priv){
	uint64_t aux;

	aux = (uint64_t) priv->fival.numerator * (uint64_t) priv->pixel_clk;
	do_div(aux,priv->fival.denominator);

	return aux;
}

#ifdef DEBUG_CLKS
static void qtec_cmosis_horiz_line_dbg(struct qtec_cmosis *priv){
	uint64_t aux;
	uint32_t fot,read,exp,frame,over;

	fot = qtec_cmosis_fot_clks(priv,priv->format);
	printk(KERN_ERR "FOT=%d pixel clks",fot);

	read= qtec_cmosis_readout_clks(priv,priv->format,false);
	printk(KERN_ERR "Readout=%d pixel clks\n",read);

	exp = qtec_cmosis_exp_clks(priv);
	printk(KERN_ERR "Exposure=%d pixel clks\n",exp);

	frame = qtec_cmosis_frame_clks(priv);
	printk(KERN_ERR "Frame time=%d pixel clks\n",frame);

	aux=exp+fot+read;
	if (aux<=frame)
		return;

	over=aux-frame;
	printk(KERN_ERR "over time=%lld pixel clks\n",aux);

	aux=read-over;
	printk(KERN_ERR "error time=%lld pixel clks\n",aux);

	aux*=qtec_cmosis_readout_lines(priv,priv->format);
	aux+=(read>>1);
	do_div(aux,read);
	printk(KERN_ERR "error line=%lld line\n",aux);
	return;
}
#endif

static inline int qtec_cmosis_error_line(struct qtec_cmosis *priv){
	uint64_t aux;
	uint32_t read=qtec_cmosis_readout_clks(priv,priv->format,false);
	uint64_t frame=qtec_cmosis_frame_clks(priv);


	aux=qtec_cmosis_exp_clks(priv) + qtec_cmosis_fot_clks(priv,priv->format) + read;

	if (aux<=frame)
		return -1;

	aux = aux-frame;
	aux*= (uint64_t)qtec_cmosis_readout_lines(priv,priv->format);
	do_div(aux,read);
	return aux;
}

static inline uint32_t qtec_cmosis_lines2time_huge(struct qtec_cmosis *priv, uint32_t lines){
	int64_t time;

	lines = max((uint32_t)1,lines);
	time = (uint64_t) (lines - 1) * (uint64_t) (qtec_cmosis_reg_timing(85,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) +1);
	time += 1 + 34 * (qtec_cmosis_reg_timing(82,priv->nchans,priv->bitmode->qmenu_int[priv->bitmode->val]) & 0xff);

	time *= (uint64_t) 1000000;
	do_div(time,priv->pixel_clk);

	return time;
}

static inline uint32_t qtec_cmosis_lines2time_big(struct qtec_cmosis *priv, uint32_t lines){
	int64_t time;

	time = (uint64_t)lines * (uint64_t) (priv->slot_len +1);
	time += 148 * priv->fot_len;
	time += 1;
	time *= (uint64_t)1000000;
	time += priv->pixel_clk/2;
	do_div(time,priv->pixel_clk);

	return time;
}

static inline uint32_t qtec_cmosis_lines2time_small(struct qtec_cmosis *priv, uint32_t lines){
	int64_t time;

	time = (uint64_t) lines * (uint64_t) 100;
	time += (uint64_t)43 * (uint64_t)priv->fot_len;
	time *= (uint64_t)129*(uint64_t)1000000;
	do_div(time,priv->pixel_clk);

	time+=50;
	do_div(time,100);

	return time;
}

static inline uint32_t qtec_cmosis_lines2time(struct qtec_cmosis *priv, uint32_t time){
	switch (priv->chip_id){
	case CMV12000v2:
		return qtec_cmosis_lines2time_huge(priv, time);
	case CMV8000v1:
		return qtec_cmosis_lines2time_big(priv, time);
	default:
		return qtec_cmosis_lines2time_small(priv, time);
	}
}

static int qtec_cmosis_stop(struct qtec_cmosis *priv){
	//FB4 can launch spureous irqs on stop
	if ((priv->quirk & QUIRK_IRQ_OVERFLOW) || (priv->quirk & QUIRK_IRQ_SYNC_ERROR))
		qtec_cmosis_write(priv, TRC_OVERFLOW, 0);

	qtec_cmosis_write(priv,CONTROL,0);
	//FIXME #272
	//Reset framegen
	qtec_cmosis_write(priv,CONTROL,BIT(FB_RESET));
	msleep(1);
	qtec_cmosis_write(priv,CONTROL,0);
	msleep(1);
	qtec_cmosis_stop_sensor(priv);


	return 0;
}

#define MIN_FRAME_DELAY 2
static int qtec_cmosis_set_ext_trig_delay(struct qtec_cmosis *priv,uint32_t time){
	uint64_t delay64;

	delay64=(uint64_t )time * (uint64_t)priv->bus_clk;
	do_div(delay64,1000000);
	delay64=clamp(delay64,(uint64_t)MIN_FRAME_DELAY,(uint64_t)0xffffffff);

	qtec_cmosis_write(priv,FRAME_DELAY,delay64);
	return 0;
}

static int qtec_cmosis_set_frame_delay(struct qtec_cmosis *priv){
	uint64_t delay64;

	if ((v4l2_ctrl_g_ctrl(priv->mode_trig) == SELF_TIMED)||
			(v4l2_ctrl_g_ctrl(priv->mode_trig) == IDLE)){
		delay64=(uint64_t) priv->fival.numerator * (uint64_t) priv->bus_clk;
		do_div(delay64,priv->fival.denominator);
		delay64=clamp(delay64,(uint64_t)MIN_FRAME_DELAY,(uint64_t)0xffffffff);
		qtec_cmosis_write(priv,FRAME_DELAY,delay64&0xffffffff);
	}
	else
		qtec_cmosis_set_ext_trig_delay(priv,v4l2_ctrl_g_ctrl(priv->ext_trig_delay));
	return 0;
}

static uint32_t qtec_cmosis_max_delay_ext_trig(struct qtec_cmosis *priv){
	uint64_t delay=0xffffffff;

	delay *= (uint64_t) 1000000;

	do_div(delay,priv->bus_clk);

	return delay & 0x7fffffff;
}

static int qtec_cmosis_temperature_huge(struct qtec_cmosis *priv, int32_t *temperature){
	uint64_t aux;
	int64_t zero;
	int64_t ret;
	bool is_neg;

	//Calculate typical zero degrees
	zero= 825;
	zero*= priv->pixel_clk;
	do_div(zero,30000000);

	//Read value
	if (qtec_cmosis_spi_read(priv,REG_TEMP_12M,1,&aux)){
		*temperature=0;
		return -EBUSY;
	}

	//Calc temperature
	ret=aux-zero;

	is_neg = ret < 0;
	if (is_neg)
		ret *= -1;

	ret*=30000000;
	//Milidegrees
	ret*=1000 * 10;
	do_div(ret,priv->pixel_clk*35);

	if (is_neg)
		ret *= -1;

	*temperature=ret;
	return 0;
}

//This is not calibrated, please read #199
static int qtec_cmosis_temperature_small_big(struct qtec_cmosis *priv, int32_t *temperature){
	uint64_t aux;
	int64_t zero;
	int64_t ret;
	bool is_neg;

	//Calculate typical zero degrees
	zero= 1000;
	zero*= priv->pixel_clk;
	do_div(zero,40000000);

	//Read value
	if (qtec_cmosis_spi_read(priv,(priv->chip_id == CMV8000v1)?REG_TEMP_8M:REG_TEMP,2,&aux)){
		*temperature=0;
		return -EBUSY;
	}

	//Calc temperature
	ret=aux-zero;
	is_neg = ret < 0;
	if (is_neg)
		ret *= -1;
	ret*=12; //(0.3*40)
	ret*=1000000;
	//Milidegrees
	ret*=1000;
	do_div(ret,priv->pixel_clk);

	if (is_neg)
		ret *= -1;
	*temperature=ret;
	return 0;
}

static int qtec_cmosis_temperature(struct qtec_cmosis *priv, int32_t *temperature){
	if (priv->chip_id == CMV12000v2)
		return qtec_cmosis_temperature_huge(priv, temperature);

	return qtec_cmosis_temperature_small_big(priv, temperature);
}

static int qtec_cmosis_min_tpf(struct qtec_cmosis *priv,
		struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){

	fival->numerator=qtec_cmosis_readout_clks(priv,format,true) + qtec_cmosis_fot_clks(priv,format);
	fival->denominator=priv->pixel_clk;
	return 0;
}

int v4l2_ctrl_modify_range_cond(struct v4l2_ctrl *ctrl,
			     s64 min, s64 max, u64 step, s64 def)
{
	if ((ctrl->minimum == min) &&	(ctrl->maximum == max) &&
		(ctrl->step == step) &&	(ctrl->default_value == def))
		return 0;

	return v4l2_ctrl_modify_range(ctrl,min,max,step,def);
}

static int qtec_cmosis_update_exposure_range(struct qtec_cmosis *priv){
	uint32_t min;
	uint32_t max;
	uint32_t def;
	bool is_locked;
	uint64_t fot;

	min=qtec_cmosis_lines2time(priv,1);
	def=priv->exp->default_value;
	max=qtec_cmosis_lines2time(priv,0xffffff);

	if ((priv->mode_trig->val == SELF_TIMED) || (priv->mode_trig->val == IDLE)){
		uint64_t max64;

		//Frame time
		max64=(uint64_t) priv->fival.numerator * (uint64_t) 1000000; //usec
		do_div(max64,priv->fival.denominator);

		//FOT
		fot =  qtec_cmosis_fot_clks(priv,priv->format);
		fot +=5000; //2*Found by experience //FIXME
		fot*=100000;//100.000
		fot/=(priv->pixel_clk/10);

		if (fot>max64)
			max64=0;
		else
			max64 -=fot;

		if (max>max64)
			max=max64&0xffffffff;
	}

	if (max > 0x7fffffff)
		max = 0x7fffffff;

	if (max<min)
		max=min;

	if (def>max)
		def=max;

	is_locked=mutex_is_locked(priv->exp->handler->lock);
	if (is_locked)
		mutex_unlock(priv->exp->handler->lock);
	v4l2_ctrl_modify_range_cond(priv->exp,min,max,1,def);
	if (is_locked)
		mutex_lock(priv->exp->handler->lock);

	return 0;
}

static inline int qtec_cmosis_hparam_calc(struct qtec_cmosis *priv,int val, bool hend){
	uint32_t aux = val;

	if (priv->format.code == MEDIA_BUS_FMT_QTEC_LEGACY_RGB ||
		priv->format.code == MEDIA_BUS_FMT_QTEC_FB4_RGBX ||
		priv->format.code == MEDIA_BUS_FMT_QTEC_FB4_GREY)
		aux *=2;

	aux *= priv->h_binning->val;

	if (hend){
		//const struct qtec_cmosis_format *qtec_format=get_format(priv->format.code);
		aux -= 1;
		//Fixes #564
		/*if  ((!(priv->quirk & QUIRK_FB4)) && (qtec_format->mode_out == MODE_COMPACT) &&
				(!(priv->quirk & QUIRK_BINNING)))
			aux -= 5;*/
	}

	if (priv->chip_id == CMV8000v1)
		aux = ((aux/224)<<8)+(aux%224);

	return aux;
}

#define PLL_CTRL (0x1f*4)
#define PLL_CTRL7 (0xff*4)
#define PLL_RESET (BIT(31)|BIT(0))
#define PLL_LOCK 31
#define PLL_AUX_LOCK 30
#define PLL_PS_INCDEC 3
#define PLL_PS_BUSY 4
static int qtec_cmosis_start_pll(struct qtec_cmosis *priv){
	unsigned long expiration;
	uint32_t aux;

	qtec_cmosis_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,0);

	expiration=jiffies+HZ;
	do{
		qtec_cmosis_read_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,&aux);
		if ((aux&BIT(PLL_LOCK))&&(aux&BIT(PLL_AUX_LOCK)))
			return 0;
	}while(time_before(jiffies,expiration));

	v4l2_err(&priv->sd, "Timeout waiting for pll lock 0%x\n", aux);
	return -EIO;
}

//#define KINTEX_TEMP_WORKAROUND

#ifdef KINTEX_TEMP_WORKAROUND
static int qtec_cmosis_divide_pixel_clock(struct qtec_cmosis *priv){
	uint32_t aux;

	if (priv->chip_id != CMV8000v1){
		v4l2_err(&priv->sd, "This sensor does not support, temperature divide. Expect data errors at high temperatures\n");
		return 0;
	}

	v4l2_err(&priv->sd, "Dividing pixel clock, from %dHz to %d Hz\n", priv->pixel_clk, priv->pixel_clk/2);
	priv->pixel_clk/=2;

	qtec_cmosis_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);

	//CLKOUT0
	qtec_cmosis_read_pll(priv,0x8*4,&aux);
	aux &= ~0xfff;
	aux |= 2 | 2<<6;
	qtec_cmosis_write_pll(priv,0x8*4,aux);

	//CLKOUT1
	qtec_cmosis_read_pll(priv,0xA*4,&aux);
	aux &= ~0xfff;
	if (priv->chip_id == CMV8000v1)
		aux |= 4 | 4<<6;
	else
		aux |= 2 | 2<<6;
	qtec_cmosis_write_pll(priv,0xA*4,aux);

	//CLKOUT3
	qtec_cmosis_read_pll(priv,0xe*4,&aux);
	aux &= ~0xfff;
	aux |= 8 | 8<<6;
	qtec_cmosis_write_pll(priv,0xe*4,aux);

	return qtec_cmosis_start_pll(priv);
}
#endif

static int qtec_cmosis_divide_clock_big(struct qtec_cmosis *priv){
	uint32_t aux;

	qtec_cmosis_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);

	if (priv->pll_7series){
		//Change CLKOUT1 divider 2 to 4 xapp888
		qtec_cmosis_read_pll(priv,0xA*4,&aux);
		aux &= ~0xfff;
		aux |= 2 | 2<<6;
		qtec_cmosis_write_pll(priv,0xA*4,aux);
	}
	else{
		//Change divider 2 to 4
		qtec_cmosis_read_pll(priv,0x6*4,&aux);
		aux|=BIT(11);
		qtec_cmosis_write_pll(priv,0x6*4,aux);
		qtec_cmosis_read_pll(priv,0x7*4,&aux);
		aux|=BIT(8);
		aux&=~BIT(7);
		qtec_cmosis_write_pll(priv,0x7*4,aux);
		qtec_cmosis_read_pll(priv,0x11*4,&aux);
		aux&=~BIT(4);
		qtec_cmosis_write_pll(priv,0x11*4,aux);
	}

	return qtec_cmosis_start_pll(priv);
}

static int qtec_cmosis_bitmode_dump(struct qtec_cmosis *priv){
	int i;
	uint32_t aux;
	qtec_cmosis_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);
	for (i=0;i<((priv->pll_7series)?(0x50*4):PLL_CTRL);i+=4){
		qtec_cmosis_read_pll(priv,i,&aux);
		v4l2_err(&priv->sd, "0x%.2x 0x%.4x\n",i/4,aux);
	}
	return qtec_cmosis_start_pll(priv);
}

struct pll_script{
	uint8_t reg;
	uint16_t val;
};

static int qtec_cmosis_run_pll_script(struct qtec_cmosis *priv,int length, const struct pll_script *to, const struct pll_script *from){
	int i;
	uint32_t aux,diff;

	qtec_cmosis_write_pll(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);

	for (i=0;i<length;i++){
		diff = from[i].val ^ to[i].val;
		if (!diff)
			continue;
		qtec_cmosis_read_pll(priv,from[i].reg*4,&aux);
		aux &=~diff;
		aux|=to[i].val&diff;
		qtec_cmosis_write_pll(priv,from[i].reg*4,aux);
	}

	return qtec_cmosis_start_pll(priv);
}

static int qtec_cmosis_set_bitmode(struct qtec_cmosis *priv){
	uint32_t aux;
	//Obtained via reverse engineer
	static const struct pll_script k7_bit10_30[]={{0x0c, 0x128a}, {0x16, 0x00c3}};
	static const struct pll_script  k7_bit12_30[]={{0x0c, 0x1186}, {0x16, 0x2083}};
	static const struct pll_script k7_bit10_47[]={
		{0x0c, 0x1145},
		{0x10, 2| 3<<6},
		{0x13, 0x0040},
		{0x14, 0x124a},
		{0x15, 0x0080},
		{0x16, 0x0082},
		{0x18, 0x020d},
		{0x28, 0x0100},
		{0x4e, 0x9008},
		{0x4f, 0x0100}};
	static const struct pll_script k7_bit12_47[]={
		{0x0c, 0x1186},
		{0x10, 3| 3<<6},
		{0x13, 0x1440},
		{0x14, 0x134d},
		{0x15, 0x4c00},
		{0x16, 0x2083},
		{0x18, 0x015e},
		{0x28, 0x9900},
		{0x4e, 0x0908},
		{0x4f, 0x1000}};
	static const struct pll_script s6_bit10_30[]={{0x08, 0x4181}, {0x09, 0x0d60}, {0x0f, 0x2e00}, {0x10, 0x03c8}, {0x11, 0x8018}, {0x17, 0x0b21}, {0x18, 0x3fdf}, {0x19, 0x02eb}};
	static const struct pll_script s6_bit12_30[]={{0x08, 0x4101}, {0x09, 0x0d50}, {0x0f, 0x8800}, {0x10, 0x0888}, {0x11, 0x8058}, {0x17, 0x0b20}, {0x18, 0x7a1f}, {0x19, 0xa2eb}};
	static const struct pll_script s6_bit10_25[]={{0x08, 0x4181}, {0x09, 0x0d60}, {0x11, 0x8018}, {0x13, 0x0a28}, {0x16, 0xfca1}};
	static const struct pll_script s6_bit12_25[]={{0x08, 0x4101}, {0x09, 0x0d50}, {0x11, 0x8058}, {0x13, 0x0a38}, {0x16, 0xfc81}};
	static const struct pll_script *bit_10,*bit_12;
	int length;

	if (priv->pll_7series)
		switch(priv->pixel_clk){
			case 30000000:
				length = ARRAY_SIZE(k7_bit10_30);
				bit_10 = k7_bit10_30;
				bit_12 = k7_bit12_30;
				break;
			case 47500000/2:
			case 47500000:
				length = ARRAY_SIZE(k7_bit10_47);
				bit_10 = k7_bit10_47;
				bit_12 = k7_bit12_47;
				break;
			default:
				v4l2_err(&priv->sd, "I do not know how change the bitmode for this pixel_clk %d!!!\n", priv->pixel_clk);
				qtec_cmosis_bitmode_dump(priv);
				return -1;
		}
	else
		switch(priv->pixel_clk){
			case 25000000:
				length = ARRAY_SIZE(s6_bit10_25);
				bit_12 = s6_bit12_25;
				bit_10 = s6_bit10_25;
				break;
			case 30000000:
				length = ARRAY_SIZE(s6_bit10_30);
				bit_12 = s6_bit12_30;
				bit_10 = s6_bit10_30;
				break;
			default:
				v4l2_err(&priv->sd, "I do not know how change the bitmode for this pixel_clk %d!!!\n", priv->pixel_clk);
				qtec_cmosis_bitmode_dump(priv);
				return -1;
		}

	qtec_cmosis_read(priv,CMV_CTRL,&aux);

	if (priv->bitmode->qmenu_int[priv->bitmode->val]==12)
		aux |= BIT(NUM_BITS);
	else
		aux &= ~BIT(NUM_BITS);

	qtec_cmosis_write(priv,CMV_CTRL,aux);

	if (priv->quirk & QUIRK_PROG_BITMODE_IGNORE)
		return 0;

	if (priv->bitmode->qmenu_int[priv->bitmode->val]==12)
		return qtec_cmosis_run_pll_script(priv,length,bit_12,bit_10);
	else
		return qtec_cmosis_run_pll_script(priv,length,bit_10,bit_12);
}

static int qtec_cmosis_idelay_busywait(struct qtec_cmosis *priv, uint32_t bit){
	unsigned long expiration = jiffies + HZ;
	uint32_t aux = 0;
	do {
		if (!time_before(jiffies,expiration)){
			qtec_cmosis_write(priv,CONTROL,0);
			v4l2_err(&priv->sd, "Timeout waiting for CH_CAL 0x%.8x\n",aux);
			return -1;
		}
		qtec_cmosis_read(priv,CMV_CTRL,&aux);
	}while(aux & bit);
	return 0;
}

static int qtec_cmosis_data_changed(struct qtec_cmosis *priv, int chan, bool ignore_data){
	uint32_t aux;
	uint32_t ref_data;

	qtec_cmosis_read(priv,CMV_CTRL,&aux);
	aux |= BIT(DLY_CLR_CHG);
	qtec_cmosis_write(priv,CMV_CTRL, aux);
	aux &= ~BIT(DLY_CLR_CHG);
	qtec_cmosis_write(priv,CMV_CTRL, aux);

	msleep(2);
#ifdef TEST_IDELAY_SYNC
	msleep(8);
#endif
	qtec_cmosis_read(priv,CMV_CTRL,&aux);
	if (aux & BIT(DLY_DATA_CHG))
		return -1;

	if (ignore_data)
		return 0;

	ref_data = chan ? priv->sync_word : 1;
	qtec_cmosis_read(priv,IDELAY_DATA,&aux);
	if (hweight32(aux) != hweight32(ref_data))
		return -1;

	return aux;
}

#define MAX_STEPS_WHITE 77

static int qtec_cmosis_max_steps(struct qtec_cmosis *priv)
{
	if (priv->pll_7series)
		return 32; /*period for 10 bits: 27 steps, for 12 bits: 23 steps*/
	//S-o-b: Finn and Ricardo
	//MORE than 63/53 corrupts the data!!!! DO NOT TRY
	if (priv->quirk & QUIRK_FB4)
		return (priv->bitmode->qmenu_int[priv->bitmode->val]==10)?76:63;
	return (priv->bitmode->qmenu_int[priv->bitmode->val]==10)?63:53;
}

static int qtec_cmosis_idelay_window(struct qtec_cmosis *priv, int channel, bool slave,char *sync_window, uint32_t *sync_val)
{
	int max_steps = qtec_cmosis_max_steps(priv);
	int i;
	uint32_t base;

	qtec_cmosis_read(priv,CMV_CTRL,&base);
	base &= 0x3ff;
	base |= channel<<DLY_CH;
	if (slave)
		base |= BIT(DLY_MS);

	//CAL needs to be done or the idelay does not work at all!
	//S-o-b: Finn and Ricardo
	qtec_cmosis_write(priv,CMV_CTRL, base | BIT(CH_CAL));
	qtec_cmosis_write(priv,CMV_CTRL, base);
	if (qtec_cmosis_idelay_busywait(priv, BIT(CH_BUSY)))
		return -1;
	qtec_cmosis_write(priv,CMV_CTRL, base | BIT(CH_RST));
	qtec_cmosis_write(priv,CMV_CTRL, base);
	if (qtec_cmosis_idelay_busywait(priv, BIT(CH_BUSY)))
		return -1;

	msleep(2); //Wait one burst after changing channel

	for (i=0 ; i<max_steps ; i++){
		int ret;

		ret = qtec_cmosis_data_changed(priv, channel, slave);
		sync_window[i] = (ret < 0) ? ' ':'X';
		if (sync_val)
			sync_val[i] = ret;

		if (i==(max_steps-1))
			break;

		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | BIT(DLY_RUN) | BIT(DLY_CNT));
		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | BIT(DLY_CNT));
		if (qtec_cmosis_idelay_busywait(priv, BIT(DLY_BUSY)))
			return -1;
	}

	sync_window[max_steps] = '\0';

	return 0;
}

static int qtec_cmosis_idelay_realign(struct qtec_cmosis *priv, int chan, char *data_locked){
	int i,j;
	int max_step = qtec_cmosis_max_steps(priv);
	int len,best_len =0, best_step =0;
	uint32_t aux;

	for (i =0; i< max_step;i++){
		len = 0;
		for (j=i;j<max_step;j++){
			if (data_locked[j] != 'X')
				break;
			if (len++ > best_len){
				best_len = len;
				best_step = i;
			}
		}
	}

	i = best_step+best_len/2;
	if (abs(i-priv->idelay[chan]) <= 2)
		return 0;

	dev_dbg(&priv->pdev->dev, "Temperature offset channel %2d: %d\n",chan, i-priv->idelay[chan]);

	qtec_cmosis_read(priv,CMV_CTRL,&aux);
	aux &= 0x3ff;
	aux |= chan<<DLY_CH;
	if (i > priv->idelay[chan])
		aux |= BIT(DLY_DIR);

	aux |= BIT(DLY_RUN) | (abs(i-priv->idelay[chan]) << DLY_CNT);
	qtec_cmosis_write(priv,CMV_CTRL,aux);
	aux &= ~BIT(DLY_RUN);
	qtec_cmosis_write(priv,CMV_CTRL,aux);
	if (qtec_cmosis_idelay_busywait(priv, BIT(DLY_BUSY)))
		return -1;

	priv->idelay[chan] = i;
	return 0;
}

static int qtec_cmosis_idelay_monitor(void *data)
{
	struct qtec_cmosis *priv = data;
	int i;
	static char data_locked[MAX_STEPS_WHITE];
	int chan_step = 16/priv->max_channels;

	while (true) {

		msleep_interruptible(10 * 1000);
		if (kthread_should_stop())
			break;

		if (!priv->streaming)
			continue;

		if (priv->stop_idelay_monitor)
			continue;

		mutex_lock(&priv->idelay_mutex);
		for (i=0;i<17;i++){

			if (i && (!((i-1)%chan_step ==0)))
				continue;

			qtec_cmosis_idelay_window(priv, i, true, data_locked, NULL);
			dev_dbg(&priv->pdev->dev, "channel %2d [%s]\n", i, data_locked);
			qtec_cmosis_idelay_realign(priv, i, data_locked);
		}
		mutex_unlock(&priv->idelay_mutex);
	}

	return 0;
}

static int qtec_cmosis_set_idelay(struct qtec_cmosis *priv){
	int chan_step = 16/priv->max_channels;
	int i;

	if (priv->quirk & QUIRK_NO_PHASE_IDELAY)
		return 0;

	for (i=0;i<17;i++){
		uint32_t base;

		if (i && (!((i-1)%chan_step ==0)))
			continue;

		qtec_cmosis_read(priv,CMV_CTRL,&base);
		base &= 0x3ff;
		base |= i<<DLY_CH;

		//CAL needs to be done or the idelay does not work at all!
		//S-o-b: Finn and Ricardo
		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(CH_CAL));
		qtec_cmosis_write(priv,CMV_CTRL, base);
		if (qtec_cmosis_idelay_busywait(priv, BIT(CH_BUSY)))
			return -1;
		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(CH_RST));
		qtec_cmosis_write(priv,CMV_CTRL, base);
		if (qtec_cmosis_idelay_busywait(priv, BIT(CH_BUSY)))
			return -1;

		msleep(2); //Wait one burst after changing channel

		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | BIT(DLY_RUN) | (priv->idelay[i]<<DLY_CNT));
		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | (priv->idelay[i]<<DLY_CNT));
		if (qtec_cmosis_idelay_busywait(priv, BIT(DLY_BUSY)))
			return -1;
	}

	return 0;
}

static int _qtec_cmosis_find_idelay(struct qtec_cmosis *priv){
	int max_step = qtec_cmosis_max_steps(priv);
	int i;
	static char data_locked[MAX_STEPS_WHITE];
	static uint32_t data_locked_val[MAX_STEPS_WHITE];
	int chan_step = 16/priv->max_channels;

#ifdef TEST_IDELAY_SYNC //Check ticket #661 for results of test
	static char data_locked_init[17][MAX_STEPS_WHITE];
	static char data_locked_last[17][MAX_STEPS_WHITE];
	static int32_t temperature_init[17];
	static int32_t temperature_last[17];
#endif

	for (i=0;i<17;i++){
		int max_len, best_step, step;
		int32_t temperature = 0;

		if (i && (!((i-1)%chan_step ==0)))
			continue;

		qtec_cmosis_idelay_window(priv, i, false, data_locked, data_locked_val);

		max_len = 0;
		best_step = 0;
		for (step=0;step<max_step;step++){
			int end;
			int len;
			for (end = step ; end < max_step; end++)
				if (data_locked[end] != 'X' || (data_locked_val[end]!=data_locked_val[step]))
					break;
			len = end - step;
			if (len >max_len){
				best_step = step;
				max_len = len;
			}
		}

		best_step += max_len/2;
		data_locked[best_step] = '0';
		temperature = 0;

		//Temperature calibration for 12M and 8M
		if (!(priv->quirk & QUIRK_IDELAY_MONITOR) &&
		    !priv->pll_7series &&
		    max_len > 5 &&
		    (priv->chip_id == CMV12000v2 || priv->chip_id == CMV8000v1)){
			qtec_cmosis_temperature(priv, &temperature);
			temperature -= 50000; //target temperature
			temperature = temperature/((priv->chip_id == CMV12000v2)?1732:1520);

			best_step = clamp(best_step+temperature, best_step-(max_len/2)+2,best_step+(max_len/2)-2);
			best_step = clamp(best_step,0,max_step-1);
			data_locked[best_step] = 'T';
		}
		priv->idelay[i] = best_step;

		v4l2_info(&priv->sd, "idelay calibration: channel %2d window %2d step %.2d data 0x%.3x(%2d) temp_offset %d\n",i , max_len, best_step, data_locked_val[best_step],hweight32(data_locked_val[best_step]), temperature);
		v4l2_info(&priv->sd, "[%s]\n",data_locked);

#ifdef TEST_IDELAY_SYNC
		data_locked[best_step] = 'X';
		memcpy(data_locked_init[i],data_locked,sizeof(data_locked));
		memcpy(data_locked_last[i],data_locked,sizeof(data_locked));
		qtec_cmosis_temperature(priv, &temperature_init[i]);
		temperature_last[i] = temperature_init[i];
#endif
	}

#ifdef TEST_IDELAY_SYNC
	v4l2_err(&priv->sd, "Running IDELAY SYNC TEST\n");
	while (1){
	for (i=0;i<17;i++){
		int k;
		int changed,step;
		uint32_t base;
		int32_t temp;

		if (!(i==0 || ((i-1)%chan_step ==0)))
			continue;

		qtec_cmosis_read(priv,CMV_CTRL,&base);
		base &=0x3ff;
		base |= i <<DLY_CH;
		qtec_cmosis_write(priv,CMV_CTRL, base);

		qtec_cmosis_write(priv,CMV_CTRL, base | BIT(CH_RST));
		qtec_cmosis_write(priv,CMV_CTRL, base);
		if (qtec_cmosis_idelay_busywait(priv, BIT(CH_BUSY)))
			return -1;

		for (step = 0; step < max_step; step++){

			data_locked[step] = (qtec_cmosis_data_changed(priv,i,slave)<0) ?' ':'X';

			if (step==(max_step-1))
				break;

			qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | BIT(DLY_RUN) | BIT(DLY_CNT));
			qtec_cmosis_write(priv,CMV_CTRL, base | BIT(DLY_DIR) | BIT(DLY_CNT));
			if (qtec_cmosis_idelay_busywait(priv, BIT(DLY_BUSY)))
				return -1;
		}
		data_locked[max_step] = '\0';

		changed = 0;
		for (k=0;k<sizeof(data_locked);k++)
			if (data_locked_last[i][k] != data_locked[k])
				changed ++;

		if (changed > 2){
			v4l2_err(&priv->sd, "obs: idelay changed on channel %2d (physical chan %2d)\n",i,i);
			qtec_cmosis_temperature(priv, &temp);
			v4l2_err(&priv->sd, "fst: %5d C [%s]\n",temperature_init[i], data_locked_init[i]);
			v4l2_err(&priv->sd, "old: %5d C [%s]\n",temperature_last[i], data_locked_last[i]);
			v4l2_err(&priv->sd, "new: %5d C [%s]\n",temp,data_locked);
			memcpy(data_locked_last[i],data_locked,sizeof(data_locked));
			temperature_last[i] = temp;
		}
	}
	msleep(1000);
	}
#endif

	return 0;
}

static int qtec_cmosis_find_idelay(struct qtec_cmosis *priv){
	int ret;
	mutex_lock(&priv->idelay_mutex);
	ret = _qtec_cmosis_find_idelay(priv);
	mutex_unlock(&priv->idelay_mutex);
	return ret;
}

static int qtec_cmosis_sync(struct qtec_cmosis *priv, int nchans, bool do_wlock){
	uint32_t aux;
	unsigned long expiration;

	//Reset control register
	qtec_cmosis_write(priv,CONTROL,0);

	//Reset cmosis chip
	qtec_cmosis_reset_sensor(priv, true);

	//Init cmosis chip
	qtec_cmosis_init_cmosis(priv);

	aux=BIT(RUN);
	//FIXME Ask Finn about BURST_224 and FLASH_DISABLE
	if ((priv->chip_id == CMV8000v1) && !(priv->quirk & QUIRK_VERSION_SENSOR))
		aux|=BIT(BURST_224)|BIT(CH_SWAP);

	switch(nchans){
		case 16:
			aux|=IF_16CH<<IF_CH;
			break;
		case 8:
			aux|=IF_8CH<<IF_CH;
			break;
		case 4:
			aux|=IF_4CH<<IF_CH;
			break;
		case 2:
			aux|=IF_2CH<<IF_CH;
			break;
	}
	qtec_cmosis_write(priv,CONTROL,aux);

	if (!do_wlock)
		return 0;

	if ((priv->quirk & QUIRK_IDELAY))
		if (qtec_cmosis_set_idelay(priv))
			return -1;

	aux|=BIT(WSYNCRUN);
	qtec_cmosis_write(priv,CONTROL,aux);

	expiration = jiffies + HZ/50; //20 msec
	do {
		qtec_cmosis_read(priv,CONTROL,&aux);
		if (aux&BIT(WSLOCK))
			return 0;
	}while(time_before(jiffies,expiration));

	dev_dbg(&priv->pdev->dev, "Timeout syncing BLCH=0x%lx (0x%.8x)\n", (aux >> BLCH) & MASK(BLCH_LEN),aux);

	return -1;
}


#define MAX_PHASE_K7 16
#define MAX_PHASE_S6 16
#define MAX_PHASE_K7_INC 112
#define MAX_PHASE MAX_PHASE_K7_INC

static int qtec_cmosis_max_phase(struct qtec_cmosis *priv){
	if (!priv->pll_7series)
		return MAX_PHASE_S6;
	if (priv->quirk & QUIRK_PHASE_INC)
		return MAX_PHASE_K7_INC;
	return MAX_PHASE_K7;
}

static int qtec_cmosis_set_phase_inc(struct qtec_cmosis *priv, unsigned int phase){
	uint32_t aux;
	int i;

	if (!priv->pll_7series)
		return 0;

	if (phase>=MAX_PHASE_K7_INC)
		return -EINVAL;

	//Initial restart
	qtec_cmosis_write_pll(priv,PLL_CTRL7,PLL_RESET);
	if (qtec_cmosis_start_pll(priv))
		return -EIO;

	for (i=0;i<phase;i++){
		qtec_cmosis_write_pll(priv,PLL_CTRL7,BIT(PLL_PS_BUSY));
		qtec_cmosis_write_pll(priv,PLL_CTRL7,0);
		qtec_cmosis_read_pll(priv,PLL_CTRL7,&aux);
		if (aux & BIT(PLL_PS_BUSY)){
			v4l2_err(&priv->sd, "Timeout setting pll phase inc 0x%.8x\n", aux);
			return -EIO;
		}
	}
	return 0;

}

static int qtec_cmosis_set_phase_k7(struct qtec_cmosis *priv, unsigned int phase){
	uint32_t aux;

	if (phase>=MAX_PHASE_K7)
		return -EINVAL;

	qtec_cmosis_write_pll(priv,PLL_CTRL7,PLL_RESET);
	//With the divider configuration, there are only 16
	//valid phases per cycle
	//Setting clk0 mux phase
	qtec_cmosis_read_pll(priv,0x8*4,&aux);
	aux&=~(BIT(13)|BIT(14)|BIT(15));
	aux|=(phase&0x7)<<13;
	qtec_cmosis_write_pll(priv,0x8*4,aux);
	//Set clk0 delay
	qtec_cmosis_read_pll(priv,0x9*4,&aux);
	aux&=~(0x1f|BIT(8)|BIT(9));
	aux|=phase>>3;
	qtec_cmosis_write_pll(priv,0x9*4,aux);

	return qtec_cmosis_start_pll(priv);
}

static int qtec_cmosis_set_phase_s6(struct qtec_cmosis *priv, unsigned int phase){
	uint32_t aux;

	if (phase>=MAX_PHASE_S6)
		return -EINVAL;
	qtec_cmosis_write_pll(priv,PLL_CTRL,PLL_RESET);
	//Setting clk0 mux phase
	qtec_cmosis_read_pll(priv,0x11*4,&aux);
	if(phase&BIT(0))
		aux|=BIT(2);
	else
		aux&=~BIT(2);
	qtec_cmosis_write_pll(priv,0x11*4,aux);

	qtec_cmosis_read_pll(priv,0x9*4,&aux);
	if(phase&BIT(1))
		aux|=BIT(13);
	else
		aux&=~BIT(13);
	if(phase&BIT(2))
		aux|=BIT(12);
	else
		aux&=~BIT(12);
	qtec_cmosis_write_pll(priv,0x9*4,aux);

	//Set clk0 delay
	qtec_cmosis_read_pll(priv,0x5*4,&aux);
	if(phase&BIT(3))
		aux|=BIT(8);
	else
		aux&=~BIT(8);
	qtec_cmosis_write_pll(priv,0x5*4,aux);
	return qtec_cmosis_start_pll(priv);

}

static int qtec_cmosis_set_phase(struct qtec_cmosis *priv, unsigned int phase){
	if (priv->quirk & QUIRK_NO_PHASE_IDELAY)
		return 0;
	if (!priv->pll_7series)
		return qtec_cmosis_set_phase_s6(priv,phase);
	if (priv->quirk & QUIRK_PHASE_INC)
		return qtec_cmosis_set_phase_inc(priv,phase);
	return qtec_cmosis_set_phase_k7(priv,phase);
}

static int qtec_cmosis_find_eye_idelay(struct qtec_cmosis *priv)
{
	int i;
	static char data_locked[MAX_STEPS_WHITE];
	static char ctrl_locked[MAX_STEPS_WHITE];
	int max_phase = qtec_cmosis_max_phase(priv);
	int best_phase = 0;
	int best_phase_size = -1;
	uint32_t old_sync_word;
	int max_step = qtec_cmosis_max_steps(priv);

	old_sync_word = priv->sync_word;

	priv->sync_word = 0x555; //Recommended by Alex

	for (i=0;i<max_phase;i++){
		int len,j;

		/*
		 * Some phases produce bad data
		 * Experimentally found 27-31
		 */
		if (priv->pll_7series)
			if (i >= 25 && i<= 33)
				continue;

		qtec_cmosis_set_phase(priv,i);
		qtec_cmosis_sync(priv,priv->max_channels,false);
		qtec_cmosis_idelay_window(priv, 1, false, data_locked, NULL);
		v4l2_info(&priv->sd, "Phase %.3d Data: [%s]\n",i,data_locked);
		qtec_cmosis_idelay_window(priv, 0, false,ctrl_locked, NULL);
		v4l2_info(&priv->sd, "Phase %.3d Ctrl: [%s]\n",i,ctrl_locked);

		len = 0;
		for (j=0;j<max_step;j++){
			if (data_locked[j] == 'X' && ctrl_locked[j] == 'X')
				len ++;
			else
				len = 0;
			if (len >best_phase_size){
				best_phase_size = len;
				best_phase = i;
			}
		}

		//Speedup PHASE_INC
		if (max_phase > 64)
			i++;
	}

	v4l2_info(&priv->sd, "Using Phase %3d, with window size %2d\n",best_phase,best_phase_size);

	priv->sync_word = old_sync_word;
	qtec_cmosis_write(priv,CONTROL,0);
	qtec_cmosis_stop_sensor(priv);
	priv->calibration_done = true;

	return best_phase;
}

#ifdef FIND_BAD_PHASES

/*
 * This experiment was run after the salsichon crisis :P.
 * It tries to figure out if some pll phases provides bad results.
 * On the kintex7 with a cmosis 8, we got the following bad phases: 28-31
 */

static int qtec_cmosis_find_bad_phases(struct qtec_cmosis *priv)
{
	int i;
	static uint32_t data_values1[MAX_STEPS_WHITE];
	static uint32_t data_valuesN[MAX_STEPS_WHITE];
	static char data_locked[MAX_STEPS_WHITE];
	int max_phase = qtec_cmosis_max_phase(priv);
	int max_step = qtec_cmosis_max_steps(priv);

	for (i=0;i<max_phase;i++){
		int j,diff_chan=0;
		qtec_cmosis_set_phase(priv,i);
		qtec_cmosis_sync(priv,priv->max_channels,false);

		qtec_cmosis_idelay_window(priv, 1, false, data_locked, data_values1);
		for (j=2;j<=priv->max_channels;j++){
			int k,bad_steps=0;;
			qtec_cmosis_idelay_window(priv, j, false, data_locked, data_valuesN);
			for (k=0;k<max_step;k++)
				if (data_locked[k] && (data_valuesN[k]!=data_values1[k]))
					bad_steps ++;
			if (bad_steps > (max_step/2))
				diff_chan ++;
		}

		if (diff_chan)
			v4l2_info(&priv->sd, "Phase %.3d DiffChan: [%d]\n",i,diff_chan);
	}

	qtec_cmosis_write(priv,CONTROL,0);
	qtec_cmosis_stop_sensor(priv);

	return  0;
}
#endif

#define MAX_TRIES_SYNC 4
static int qtec_cmosis_find_eye_phase(struct qtec_cmosis *priv, int nchans, int *eye_len){
	char do_sync[MAX_PHASE+1]={};
	int best_len=-1;
	int best_phase=-1;
	int n_tries;
	int phase;
	int i;
	int max_phase = qtec_cmosis_max_phase(priv);

	for (n_tries=0;n_tries<MAX_TRIES_SYNC;n_tries++)
		for (phase=0;phase<max_phase;phase++){
			//If it failed to sync before,
			//it is not a good candidate.
			if (do_sync[phase]!=n_tries)
				continue;
			//Set phase
			if (qtec_cmosis_set_phase(priv,phase))
				continue;
			if (qtec_cmosis_sync(priv, nchans, false))
				continue;

			do_sync[phase]++;
		}

	//Stop
	qtec_cmosis_write(priv,CONTROL,0);
	qtec_cmosis_stop_sensor(priv);

	n_tries=1;
	for (phase=0;phase<max_phase;phase++){
		int i;
		int len=1;
		//We only want the position that has worked the most tries
		if (do_sync[phase]<n_tries)
			continue;
		for (i=1;i<max_phase;i++){
			if (do_sync[(phase+i)%max_phase]==do_sync[phase])
				len++;
			else
				break;
		}
		if ((len>best_len)||(do_sync[phase]>n_tries)){
			best_len=len;
			best_phase=phase;
			n_tries=do_sync[phase];
		}
	}

	for (i=0;i<max_phase;i++)
		do_sync[i]+='0';
	do_sync[max_phase]='\0';

	*eye_len = best_len;

	//Save eye
	phase=(best_phase+(best_len/2))%max_phase;
	if (best_len<1){
		v4l2_err(&priv->sd, "%.2d chans: Could not sync after %d tries [%s]\n", nchans, MAX_TRIES_SYNC, do_sync);
		priv->calibration_done = false;
		return -1;
	}
	else{
		v4l2_info(&priv->sd, "%.2d chans: Found a %.2d units length eye with center in %.2d [%s]\n", nchans, best_len,phase,do_sync);
		priv->calibration_done = true;
	}

	return phase;
}

static int qtec_cmosis_max_width_sensor(struct qtec_cmosis *priv, bool only_active){
	switch(priv->chip_id){
		case CMV12000v2:
			if (only_active)
				return 4096 - 756;
			else
				return 4096;
		case CMV8000v1:
			if (only_active)
				return 3584 - 244;
			else
				return 3584;
		default:
			return 2048;
	}

	return 2048;
}

static int qtec_cmosis_max_size_sensor(struct qtec_cmosis *priv,u32 format,
		unsigned int *max_width, unsigned int *max_height, bool only_active){

	*max_width = qtec_cmosis_max_width_sensor(priv, only_active);

	switch(priv->chip_id){
		case CMV12000v2:
			if (only_active) //Pretend you are a 8M
				*max_height = 3072-576;
			else
				*max_height = 3072;
			break;
		case CMV8000v1:
			if (only_active)
				*max_height = 2528 - 32;
			else
				*max_height = 2528;
			break;
		case CMV4000v2:
		case CMV4000v3:
			*max_height = 2048;
			break;
		case CMV2000v2:
		case CMV2000v3:
		default:
			*max_height = 1088;
			break;
	}

	if (only_active && (priv->mosaic_mode==MOSAIC5x5))
		*max_height -= 5;

	switch(format){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGB:
		case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
		case MEDIA_BUS_FMT_QTEC_FB4_GREY:
			*max_width/=2;
			*max_height /=2;
			break;
		case MEDIA_BUS_FMT_QTEC_FB4_GREEN:
			*max_height /=2;
			break;
	}

	*max_height /= priv->v_binning->val;
	*max_width /= priv->h_binning->val;

	return 0;
}

static int qtec_cmosis_format_even(struct qtec_cmosis *priv, u32 format, bool *h_even, bool *v_even){
	if (	(format == MEDIA_BUS_FMT_QTEC_LEGACY_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_LEGACY_GBRG) ||
		(format == MEDIA_BUS_FMT_QTEC_COMPACT_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_COMPACT_GBRG) ||
		(format == MEDIA_BUS_FMT_QTEC_FB4_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_FB4_GBRG))
		*h_even=false;
	else
		*h_even=true;

	if (
		(format == MEDIA_BUS_FMT_QTEC_LEGACY_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_LEGACY_GRBG) ||
		(format == MEDIA_BUS_FMT_QTEC_COMPACT_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_COMPACT_GRBG) ||
		(format == MEDIA_BUS_FMT_QTEC_FB4_RGGB) ||
		(format == MEDIA_BUS_FMT_QTEC_FB4_GRBG))
		*v_even=false;
	else
		*v_even=true;

	return 0;
}

static bool qtec_cmosis_is_bayer_format(u32 format){
	switch(format){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
		case MEDIA_BUS_FMT_QTEC_FB4_RGGB:
		case MEDIA_BUS_FMT_QTEC_FB4_GRBG:
		case MEDIA_BUS_FMT_QTEC_FB4_GBRG:
		case MEDIA_BUS_FMT_QTEC_FB4_BGGR:
			return true;
		default:
			return false;
	}
}

static int qtec_cmosis_start_align(struct qtec_cmosis *priv, u32 format,int *hstart, int *vstart, int only_active){

	*hstart=0;
	*vstart =0;

	if (qtec_cmosis_is_bayer_format(format)){
		bool hstart_even,vstart_even;
		qtec_cmosis_format_even(priv,format,&hstart_even,&vstart_even);
		if (hstart_even != priv->hflip->val)
			*hstart = 1;
		if (vstart_even != priv->vflip->val)
			*vstart = 1;
	}

	if (only_active && (priv->chip_id==CMV8000v1 || priv->chip_id==CMV12000v2) ){
		int h_off, v_off;

		if (priv->chip_id==CMV12000v2){
			h_off = 756/2;
			v_off = 576/2;
		} else {
			h_off = 244/2;
			v_off = 32;
		}

		if (qtec_cmosis_is_vdecim_format(format))
			v_off/=2;

		if (format == MEDIA_BUS_FMT_QTEC_LEGACY_RGB ||
			format == MEDIA_BUS_FMT_QTEC_FB4_RGBX)
			h_off/=2;

		v_off /= priv->v_binning->val;
		h_off /= priv->h_binning->val;

		*hstart += h_off;
		*vstart += v_off;
	}
	return 0;
}

static int qtec_cmosis_max_size(struct qtec_cmosis *priv,u32 format,
		unsigned int *max_width, unsigned int *max_height, bool only_active){
	int hstart,vstart;
	const struct qtec_cmosis_format *qtec_format=get_format(format);
	if (!qtec_format)
		return -EINVAL;

	qtec_cmosis_max_size_sensor(priv,format,max_width,max_height,only_active);

	qtec_cmosis_start_align(priv,format,&hstart,&vstart,false);
	if (hstart)
		*max_width -= 2;
	if (vstart)
		*max_height -= 2;

	if (!(priv->quirk & QUIRK_FB4)){
		if (qtec_format->mode_out==MODE_COMPACT)
			*max_width-=*max_width%5;

		if (qtec_format->mode_out==MODE_SINGLE)
			*max_width=512; //Workaround for #456
	}

	return  0;
}

static int qtec_cmosis_def_crop(struct qtec_cmosis *priv, struct v4l2_rect *r){
	int32_t max_width,max_height,min_width,min_height;
	if (qtec_cmosis_max_size(priv,priv->format.code,&max_width,&max_height,true))
		return -EINVAL;
	qtec_cmosis_start_align(priv,priv->format.code,&min_width,&min_height,true);
	r->left = min_width;
	r->top = min_height;
	qtec_cmosis_start_align(priv,priv->format.code,&min_width,&min_height,false);
	r->width = max_width+min_width;
	r->height = max_height+min_height;
	return 0;
}

static int qtec_cmosis_enum_mbus_code(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);
	int i,index=-1;
	bool is_bayer = (priv->mosaic_mode==BAYER)?true:false;

	if (code->pad)
		return -EINVAL;

	for (i=0;i<ARRAY_SIZE(cmosis_formats);i++){
		if (is_bayer!=cmosis_formats[i].is_bayer_chip)
			continue;
		if (!!(priv->quirk & QUIRK_FB4) != !!cmosis_formats[i].fb4_bus)
			continue;
		index++;
		if (index==code->index){
			code->code=cmosis_formats[i].mbus_format;
			return 0;
		}
	}

	return -EINVAL;
}

static int qtec_cmosis_min_size(struct qtec_cmosis *priv,u32 format,
		unsigned int *min_width, unsigned int *min_height){

	*min_width=1;
	*min_height=1;

	if (qtec_cmosis_is_bayer_format(format)){
		*min_width =2;
		*min_height =2;
	}

	if ((format == MEDIA_BUS_FMT_QTEC_FB4_GREEN) ||
		(format == MEDIA_BUS_FMT_QTEC_FB4_GREY))
		*min_width = 4;

	if (!(priv->quirk & QUIRK_FB4)){
		const struct qtec_cmosis_format *qtec_format=get_format(format);

		if (!qtec_format)
			return -EINVAL;
		if (qtec_format->mode_out==MODE_COMPACT)
			*min_width=5;
	}

	if (priv->mosaic_mode==MOSAIC5x5){
		*min_width = max_t(unsigned int, 5, *min_width);
		*min_height = max_t(unsigned int, 5, *min_height);
	}

	return 0;
}

static u32 qtec_cmosis_try_mbus_code(struct qtec_cmosis *priv, u32 code_req){
	const struct qtec_cmosis_format *qtec_format=get_format(code_req);
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = 0,
	};

	//If invalid format use the first one
	if (qtec_format)
		return code_req;

	qtec_cmosis_enum_mbus_code(&priv->sd,NULL,&code);
	return code.code;
}

static int qtec_cmosis_try_fmt(struct v4l2_subdev *subdev,struct v4l2_mbus_framefmt *fmt){
	unsigned int max_width,max_height;
	unsigned int min_width,min_height;
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	fmt->code = qtec_cmosis_try_mbus_code(priv,fmt->code);

	qtec_cmosis_max_size(priv,fmt->code,&max_width,&max_height,false);
	qtec_cmosis_min_size(priv,fmt->code,&min_width,&min_height);

	if (priv->mosaic_mode==MOSAIC5x5){
		fmt->width -= fmt->width%5;
		fmt->height -= fmt->height%5;
	}

	if (min_height == 2)
		fmt->height -= fmt->height%2;

	v4l_bound_align_image(&fmt->width, min_width, max_width, 0,
			      &fmt->height, min_height, max_height, 0, 0);
	fmt->colorspace=V4L2_COLORSPACE_SRGB;
	fmt->field=V4L2_FIELD_NONE;

	return 0;
}

static int qtec_cmosis_max_nchans(struct qtec_cmosis *priv){
	return qtec_cmosis_max_nchans_fmt(priv,priv->format);
}

static int qtec_cmosis_max_tpf(struct qtec_cmosis *priv,
		struct v4l2_mbus_framefmt format, struct v4l2_fract *fival){
	fival->numerator=0x7fffffff;
	if ((priv->mode_trig->val == SELF_TIMED) || (priv->mode_trig->val == IDLE))
		fival->denominator=priv->bus_clk;
	else
		fival->denominator=1;
	return 0;
}

#define FRACT_CMP(a, OP, b)	\
	((u64)(a).numerator * (b).denominator  OP  (u64)(b).numerator * (a).denominator)
static int qtec_cmosis_clamp_interval(struct qtec_cmosis *priv,struct v4l2_fract *fival){
	struct v4l2_fract interval;
	int ret;

	ret=qtec_cmosis_min_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,<,interval))
		*fival=interval;

	ret=qtec_cmosis_max_tpf(priv, priv->format, &interval);
	if (ret)
		return ret;
	if (FRACT_CMP(*fival,>,interval))
		*fival=interval;

	return 0;
}

static bool qtec_cmosis_is_bayer(struct qtec_cmosis *priv){
	return qtec_cmosis_is_bayer_format(priv->format.code);
}

static int qtec_cmosis_crop_sort_check(struct qtec_cmosis *priv){
	struct v4l2_rect aux;
	int i,j;

	//bubble sort
	for (i=0;i<priv->n_crop-1;i++){
		if (priv->crop[i].height < 1)
			return -1;
		for (j=i;j<priv->n_crop;j++)
			if (priv->crop[i].top>priv->crop[j].top){
				aux=priv->crop[i];
				priv->crop[i]=priv->crop[j];
				priv->crop[j]=aux;
			}
	}

	for (i=1;i<priv->n_crop;i++)
		if ((priv->crop[i].top < (priv->crop[i-1].top+priv->crop[i-1].height)))
			return -1; //selection overlap

	return 0;
}

static int qtec_cmosis_update_crop(struct qtec_cmosis *priv){
	int max_left,max_top,min_left,min_top;
	int i;
	int crop_height;
	bool is_bayer=qtec_cmosis_is_bayer(priv);
	bool is_vdecim = qtec_cmosis_is_vdecim_format(priv->format.code);

	//calc borders
	qtec_cmosis_start_align(priv,priv->format.code,&min_left,&min_top,false);
	qtec_cmosis_max_size_sensor(priv,priv->format.code,&max_left,&max_top,false);

	//width
	for (i=0;i<priv->n_crop;i++)
		priv->crop[i].width=priv->format.width;
	max_left-=priv->format.width;
	priv->crop[0].left=clamp(priv->crop[0].left,min_left,max_left);
	if (((min_left&1)!=(priv->crop[0].left&1)) && is_bayer)
		priv->crop[0].left++;
	if (priv->crop[0].left>max_left)
		priv->crop[0].left-=2;

	//height
	for (i=0;i<priv->n_crop;i++){
		priv->crop[i].left=priv->crop[0].left;
		if (priv->mosaic_mode==MOSAIC5x5){
			priv->crop[i].left-= priv->crop[i].left%5;
			priv->crop[i].top-= priv->crop[i].top%5;
			priv->crop[i].height-= priv->crop[i].height%5;
		}

		if (is_bayer)
			priv->crop[i].height-= priv->crop[i].height%2;

		if (priv->chip_id == CMV12000v2){
			int offset;

			if (is_vdecim)
				offset = 2;
			else{
				offset = 4;
				priv->crop[i].top-= priv->crop[i].top%2;
			}

			if (((priv->crop[i].top % offset) != (priv->crop[0].top % offset))){
				priv->crop[i].top-=priv->crop[i].top % offset;
				priv->crop[i].top+=priv->crop[0].top % offset;
			}
		}
	}

	//Height
	if (priv->n_crop > MAX_CROP)
		priv->n_crop=1;
	crop_height=qtec_cmosis_crop_height(priv);
	//No skipping on mosaic
	if ((priv->mosaic_mode==MOSAIC5x5) && (crop_height != priv->format.height))
		crop_height = 0;
	if ((crop_height==0) || (crop_height % priv->format.height) || (crop_height>max_top) ||
		qtec_cmosis_crop_sort_check(priv) ||((priv->chip_id == CMV12000v2) && crop_height != priv->format.height) ){
		//FIXME We dont support line skipping in CMV12K
		priv->crop[0].height=priv->format.height;
		priv->n_crop=1;
		crop_height=priv->crop[0].height;
	}

	max_top-=crop_height - priv->n_crop +1;
	for (i=0;i<priv->n_crop;i++){
		priv->crop[i].top=clamp(priv->crop[i].top,min_top,max_top);
		if (((min_top&1)!=(priv->crop[i].top&1)) && is_bayer)
			priv->crop[i].top++;
		if (priv->crop[i].top>max_top)
			priv->crop[i].top-=2;
		max_top+=priv->crop[i].height +1;
		min_top=priv->crop[i].height+priv->crop[i].top;
	}

	return 0;
}

static int qtec_cmosis_update_vflip_cmv12(struct qtec_cmosis *priv){
	int hstart,vstart;

	if (priv->chip_id != CMV12000v2)
		return 0;

	priv->vflip->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;

	if (qtec_cmosis_is_vdecim_format(priv->format.code))
		return 0;

	qtec_cmosis_start_align(priv,priv->format.code,&hstart,&vstart,false);
	if ((vstart % 2) == 1)
		v4l2_ctrl_s_ctrl(priv->vflip,!priv->vflip->val);
	priv->vflip->flags |= V4L2_CTRL_FLAG_READ_ONLY ;

	return 0;
}

static int qtec_cmosis_update_adc_gain_range(struct qtec_cmosis *priv){
	int offset;
	int min,max,def;

	if (priv->bitmode->qmenu_int[priv->bitmode->val] == 10)
		offset = 165;
	else
		offset = 120;

	min = -offset;
	max = 255-offset;
	def = clamp((int)priv->adc_gain->default_value,min,max);

	v4l2_ctrl_modify_range_cond(priv->adc_gain,min,max,1,def);
	return 0;
}

static int qtec_cmosis_set_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	if (format->pad)
		return -EINVAL;

	qtec_cmosis_try_fmt(subdev,fmt);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY){
		if (cfg)
			cfg->try_fmt = *fmt;
		return 0;
	}

	if (priv->streaming)
		return -EBUSY;

	priv->format=*fmt;
	priv->nchans=qtec_cmosis_max_nchans(priv);
	qtec_cmosis_clamp_interval(priv,&priv->fival);
	qtec_cmosis_update_exposure_range(priv);
	qtec_cmosis_update_crop(priv);
	if (priv->chip_id == CMV12000v2)
		qtec_cmosis_update_vflip_cmv12(priv);
	qtec_cmosis_update_adc_gain_range(priv);

	return 0;
}

static int qtec_cmosis_set_sensor_type(struct qtec_cmosis *priv){
	uint32_t aux = 0;


	if (priv->quirk & QUIRK_VERSION_SENSOR)
		switch (priv->chip_id){
		case CMV2000v2:
		case CMV2000v3:
			aux |= 0 << SENSOR_TYPE;
			break;
		case CMV4000v2:
		case CMV4000v3:
			aux |= 1 << SENSOR_TYPE;
			break;
		case CMV8000v1:
			aux |= 2 << SENSOR_TYPE;
			break;
		case CMV12000v2:
		default:
			aux |= 3 << SENSOR_TYPE;
			aux |= 1 << DATA_MODE;
			break;
		}

	qtec_cmosis_write(priv,CMV_CTRL,aux);

	return 0;
}

#define MIN_SANITY_WINDOW 6
static int qtec_cmosis_calibrate(struct qtec_cmosis *priv){
	int ret=0;
	int len, chans=priv->max_channels;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	qtec_cmosis_set_sensor_type(priv);

	if (priv->quirk & QUIRK_PROG_BITMODE)
		qtec_cmosis_set_bitmode(priv);

#ifdef FIND_BAD_PHASES
	while(1)
		qtec_cmosis_find_bad_phases(priv);
#endif
	if (priv->quirk & QUIRK_NO_PHASE_IDELAY)
		priv->calibration_done = true;
	else if (priv->quirk & QUIRK_IDELAY) {
		ret = qtec_cmosis_find_eye_idelay(priv);
		if (ret >= 0){
			qtec_cmosis_set_phase(priv,ret);
			qtec_cmosis_sync(priv,priv->max_channels,false);
			qtec_cmosis_find_idelay(priv);
		}
	} else {

		for (; chans>=2 ; chans>>=1){
			ret=qtec_cmosis_find_eye_phase(priv, chans, &len);

			if (!(priv->quirk & QUIRK_SYNC_ONLY_USED_CHANS) || len>=MIN_SANITY_WINDOW)
				break;
		}
	}

	//Set phase
	if (ret>=0){
		priv->phase->flags&=~V4L2_CTRL_FLAG_READ_ONLY ;
		v4l2_ctrl_s_ctrl(priv->phase,ret);
		priv->phase->default_value=ret;
		priv->phase->flags|=V4L2_CTRL_FLAG_READ_ONLY ; //Make it read only (the user can crash the camera)
		ret=0;
	}

	priv->sync_channels = max(2,chans);
	format.format = priv->format;
	//Recalculate nchans et al
	qtec_cmosis_set_fmt(&priv->sd, NULL, &format);
	qtec_cmosis_temperature(priv, &priv->calibration_temperature);

	return ret;
}

static int qtec_cmosis_init_sync_err(struct qtec_cmosis *priv){
	uint32_t aux;

	qtec_cmosis_read(priv, TRC_OVERFLOW, &aux);

	aux |= BIT(SYNC_ERR_STATUS) | BIT(SYNC_ERR_EN);

	qtec_cmosis_write(priv, TRC_OVERFLOW, aux);

	return 0;
}

#define OVERFLOW_FILTER 30
static int qtec_cmosis_init_overflow(struct qtec_cmosis *priv){
	uint32_t aux;

	qtec_cmosis_read(priv, TRC_OVERFLOW, &aux);

	priv->trig_overflow_v32 = false;

	qtec_cmosis_write(priv,GLOBAL_TIMER,0);

	qtec_cmosis_write(priv, TRIG_FILTER, OVERFLOW_FILTER);

	if ((priv->mode_trig->val==SELF_TIMED) || (priv->mode_trig->val == IDLE))
		aux &= ~BIT(OVERFLOW_EN);
	else
		aux |= BIT(OVERFLOW_EN);

	aux |= BIT(OVERFLOW_STATUS);

	qtec_cmosis_write(priv, TRC_OVERFLOW, aux);

	return 0;
}

static inline u32 b_l(u32 in, int cmv8k_mode){
	switch(cmv8k_mode){
		case 0:
			return in;
		case 1:
			return ((in/56)<<6)+(in%56);
		case 2:
			return ((in/112)<<7)+(in%112);
	}
	return in;
}

static inline uint32_t scalegrey(uint16_t val, uint32_t scaler){
	uint64_t aux = val;

	aux *= scaler;
	do_div(aux, 0x4000);
	return aux;
}

static int qtec_cmosis_set_fb4_output_scaler(struct qtec_cmosis *priv, uint32_t value){
	int i;

	if (priv->format.code == MEDIA_BUS_FMT_QTEC_FB4_GREY){
		for (i=0;i<16;i++)
			switch (i%4){
				case 0:
					qtec_cmosis_write_bram(priv,COEFF+i, scalegrey(0x1323,value));
					break;
				case 1:
					qtec_cmosis_write_bram(priv,COEFF+i, scalegrey(0x2591/2+1,value));
					break;
				case 2:
					qtec_cmosis_write_bram(priv,COEFF+i, scalegrey(0x2591/2,value));
					break;
				default:
					qtec_cmosis_write_bram(priv,COEFF+i, scalegrey(0x74c,value));
			}

		return 0;
	}

	//COEFFICIENTS
	for (i=0;i<16;i++){
		if ((((i%4) == 1) || (i%4) == 2)  &&
			(priv->format.code == MEDIA_BUS_FMT_QTEC_FB4_RGBX))
			qtec_cmosis_write_bram(priv,COEFF+i, value/2);
		else
			qtec_cmosis_write_bram(priv,COEFF+i, value);
	}

	return 0;
}

#define FB4_COLS 8192
static int qtec_cmosis_clear_fb4_border(struct qtec_cmosis *priv, int start, int end, int c8){
	int pix[2];
	int i;

	pix[0] = start-1;
	pix[1] = end;

	for (i=0;i<ARRAY_SIZE(pix);i++){
		int aux;

		if (pix[i]<0)
			continue;
		aux = b_l(pix[i],c8);
		if (aux>=FB4_COLS/4)
			continue;
		qtec_cmosis_write_bram(priv,LINCOMB+aux*2,0);
		qtec_cmosis_write_bram(priv,LINCOMB+aux*2+1,0);
	}

	return 0;
}

static int qtec_cmosis_set_fb4_hbinning(struct qtec_cmosis *priv){
	int i,j;
	int hbin = priv->h_binning->val;
	int left = priv->crop[0].left;
	int right = priv->crop[0].left + priv->crop[0].width;
	int q,startcol,endcol;
	int c8 = (priv->chip_id == CMV8000v1)?1:0;
	int pix = 0;

	//Dont redo if not needed
	if ((priv->fb4_config.hbin == hbin) && (priv->fb4_config.code == priv->format.code) && (left == priv->fb4_config.left) && (right==priv->fb4_config.right))
		return 0;

	startcol = left * hbin;
	endcol = right * hbin - 1;

	//LINCOMB
	switch(priv->format.code){
	case MEDIA_BUS_FMT_QTEC_FB4_GREY:
		c8 *= 2;
		for (i=startcol;i<=endcol;i++){
			int q_pix = (pix/hbin)%4;
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2, 0xf << (4*q_pix));
			if ((pix%hbin) == (hbin-1))
				qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1, 0x1 << q_pix);
			else
				qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1, 0);

			pix ++;
		}
		qtec_cmosis_clear_fb4_border(priv,startcol,i,c8);
		break;
	case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
		c8 *= 2;
		//RGBx does not need to rotate, data will always be quad aligned
		for (i=startcol;i<=endcol;i++){
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2, 0x2861);
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1,((pix%hbin) == (hbin-1))?0xf:0);
			pix ++;
		}
		qtec_cmosis_clear_fb4_border(priv,startcol,i,c8);
		break;
	case MEDIA_BUS_FMT_QTEC_FB4_GREEN:
		//TODO Check why 24 looks better than 42
		c8 *= 2;
		q = startcol & 0x1;
		for (i=startcol/2;i<=(endcol+1)/2;i++){
			uint16_t write_en = 0;
			uint16_t coeff_en = 0;

			for (j=0;j<2;j++){
				int col = pix*2 +j -q;
				int out_col = (col/hbin) %4;
				bool do_write = (col%hbin) == (hbin -1);
				int j_g = j?2:4;

				if (col<0)
					continue;
				if (col >= priv->crop[0].width *hbin)
					continue;

				write_en |= do_write << out_col;
				coeff_en |= j_g << (out_col*4);
			}
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2,coeff_en);
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1,write_en);
			pix ++;
		}
		qtec_cmosis_clear_fb4_border(priv,startcol/2,i,c8);

		break;
	case MEDIA_BUS_FMT_QTEC_FB4_MONO:
		q = startcol & 0x3;
		for (i=startcol/4;i<=(endcol+3)/4;i++){
			uint16_t write_en = 0;
			uint16_t coeff_en = 0;
			for (j=0;j<4;j++){
				int col = pix*4+j -q;
				int out_col = (col/hbin) % 4;
				bool do_write = (col % hbin) == (hbin -1);

				if (col <0)
					continue;
				if (col >= priv->crop[0].width *hbin)
					continue;

				write_en |= do_write << out_col;
				coeff_en |= (1<<j) << (out_col*4);
			}
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2,coeff_en);
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1,write_en);
			pix ++;
		}
		qtec_cmosis_clear_fb4_border(priv,startcol/4,i,c8);
		break;
	default:
		startcol =  ((priv->crop[0].left & ~1) * hbin) + (priv->crop[0].left & 1);
		endcol = (startcol + priv->crop[0].width*hbin) -1;
		q = startcol & 0x3;
		//printk(KERN_ERR "startcol=%d endcol=%d q=%d\n",startcol,endcol,q);
		for (i=startcol/4;i<=(endcol+3)/4;i++){
			uint16_t write_en = 0;
			uint16_t coeff_en = 0;
			for (j=0;j<4;j++){
				int col =  pix*4 +j - q;
				int out_col = ((((col/2)/hbin)&1)<<1) + (col&1);
				bool do_write = ((col/2) % hbin) == (hbin -1);
				//printk(KERN_ERR "j=%d col=%d out_col=%d do_write=%d\n",j,col,out_col,do_write);

				if (col <0)
					continue;
				if (col >= priv->crop[0].width *hbin)
					continue;

				write_en |= do_write << out_col;
				coeff_en |= (1<<j) << (out_col*4);
			}
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2,coeff_en);
			qtec_cmosis_write_bram(priv,LINCOMB+b_l(i,c8)*2+1,write_en);
			pix ++;
		}
		qtec_cmosis_clear_fb4_border(priv,startcol/4,i,c8);
		break;
	}

	priv->fb4_config.hbin = hbin;
	priv->fb4_config.code = priv->format.code;
	priv->fb4_config.left = left;
	priv->fb4_config.right = right;

	return 0;
}

#define MAX_SYNC_TRIES 8
static int qtec_cmosis_start(struct qtec_cmosis *priv){
	uint32_t aux;
	const struct qtec_cmosis_format *qtec_format=get_format(priv->format.code);
	int max_tries = MAX_SYNC_TRIES; //Maximum meassured errors is 4. Workaround fo #682

	if (!qtec_format)
		return -EINVAL;

	/* Check if calibration is still valid */
	if (priv->calibration_done && !(priv->quirk & QUIRK_NO_PHASE_IDELAY)){
		int32_t temperature;
		qtec_cmosis_temperature(priv, &temperature);
		if (ABS(temperature-priv->calibration_temperature) > 10000){
			v4l2_err(&priv->sd, "Triggering recalibration due to big temperature change.\n");
			priv->calibration_done = false;
		}
	}

	if (!priv->calibration_done){
		if (qtec_cmosis_calibrate(priv)){
			v4l2_err(&priv->sd, "Unable to calibrate to cmosis. Abort\n");
			return -EIO;
		}
		qtec_cmosis_stop(priv);
		priv->streaming = false;
		v4l2_info(&priv->sd, "Forcing pipeline reset due to recalibration\n");
		return -EAGAIN;
	}

	if (priv->quirk & QUIRK_PHASE_INC)
		qtec_cmosis_set_phase(priv,priv->phase->cur.val);

	while (qtec_cmosis_sync(priv, priv->nchans, true)){
		v4l2_warn(&priv->sd, "Cmosis failed to sync. %d attempts left\n",--max_tries);
		if (max_tries <= 0){
			v4l2_err(&priv->sd, "Cmosis failed to sync. Aborting\n");
			return -EIO;
		}
	}

	if (max_tries != MAX_SYNC_TRIES)
		v4l2_info(&priv->sd, "Cmosis managed to sync after retrying. Continuing\n");

	//Clamp frame_interval
	qtec_cmosis_clamp_interval(priv,&priv->fival);

	//Set max and min of exposure
	qtec_cmosis_update_exposure_range(priv);

	qtec_cmosis_setup_cmosis(priv);
	//Set HPARAM
	aux = qtec_cmosis_hparam_calc(priv,priv->crop[0].left,false) <<HSTART;
	aux |=	qtec_cmosis_hparam_calc(priv,priv->crop[0].left+priv->crop[0].width,true) <<HEND;
	qtec_cmosis_write(priv,HPARAM,aux);

	qtec_cmosis_set_frame_delay(priv);

	if ((priv->quirk & QUIRK_BINNING) && !(priv->quirk & QUIRK_FB4)) {
		aux =  v4l2_ctrl_g_ctrl(priv->output_scaler)<< (BINSCALE+1);
		aux |= (priv->h_binning->val-1) << H_BINNING;
		aux |= (priv->v_binning->val-1) << V_BINNING;
		qtec_cmosis_write(priv,BINNING,aux);
	}

	if (priv->quirk & QUIRK_FB4){
		qtec_cmosis_write(priv,BINNING_FB4, priv->v_binning->val-1);
		qtec_cmosis_set_fb4_output_scaler(priv, v4l2_ctrl_g_ctrl(priv->output_scaler));
		qtec_cmosis_set_fb4_hbinning(priv);
		qtec_cmosis_write(priv,LINE_SKIP,0x0000ffff);
	}

	qtec_cmosis_read(priv,CONTROL,&aux);

	aux&=~(((1<<MODE_OUT_LEN)-1)<<MODE_OUT);
	aux|=qtec_format->mode_out<<MODE_OUT;
	aux&=~(((1<<MODE_TRIG_LEN)-1)<<MODE_TRIG);
	switch (priv->mode_trig->val){
		case SELF_TIMED:
		case IDLE:
			aux|= MODE_SELF_TIMED << MODE_TRIG;
			break;
		case EXT_TRIG:
			aux|= MODE_TRIG_DELAY << MODE_TRIG;
			break;
		case EXT_EXPOSURE:
			aux|= MODE_EXT_EXPOSURE << MODE_TRIG;
			break;
	}

	if (v4l2_ctrl_g_ctrl(priv->hflip))
		aux|=BIT(XFLIP);
	else
		aux&=~BIT(XFLIP);

	if (priv->chip_id == CMV12000v2){
		if (qtec_cmosis_yflip_huge(priv))
			aux|=BIT(YFLIP);
		else
			aux&=~BIT(YFLIP);
	}else{
		if (v4l2_ctrl_g_ctrl(priv->vflip) && qtec_cmosis_is_vdecim_format(priv->format.code))
			aux|=BIT(YFLIP);
		else
			aux&=~BIT(YFLIP);
	}

	if (v4l2_ctrl_g_ctrl(priv->flash_pol))
		aux|=BIT(FLASH_POL);
	else
		aux&=~BIT(FLASH_POL);

	if ((priv->quirk & QUIRK_FLASH_DISABLE) && v4l2_ctrl_g_ctrl(priv->flash_disable))
		aux|=BIT(FLASH_DISABLE);
	else
		aux&=~BIT(FLASH_DISABLE);

	if (v4l2_ctrl_g_ctrl(priv->trig_pol))
		aux|=BIT(TRIG_POL);
	else
		aux&=~BIT(TRIG_POL);


	if (priv->mode_trig->val != IDLE)
		aux|=BIT(TRIG_ENABLE);

	qtec_cmosis_write(priv,CONTROL,aux);

	if (priv->quirk & QUIRK_IRQ_OVERFLOW)
		qtec_cmosis_init_overflow(priv);

	if (priv->quirk & QUIRK_IRQ_SYNC_ERROR)
		qtec_cmosis_init_sync_err(priv);

	return 0;
}

static int qtec_cmosis_flip_set(struct qtec_cmosis *priv,bool hflip,bool vflip){
	int max_width,max_height;

	switch(priv->format.code){
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			break;
		default:
			return 0;
	}

	if (qtec_cmosis_max_size(priv,priv->format.code,&max_width,&max_height,false))
		return -EINVAL;

	if ((priv->format.width == max_width) && (priv->hflip->val != hflip)){
		bool hstart_even,vstart_even;
		qtec_cmosis_format_even(priv,priv->format.code,&hstart_even,&vstart_even);

		//New max size is smaller
		if (hstart_even != hflip)
			return -EINVAL;
	}

	if ((priv->format.height == max_height) && (priv->vflip->val != vflip)){
		bool hstart_even,vstart_even;
		qtec_cmosis_format_even(priv,priv->format.code,&hstart_even,&vstart_even);

		//New max size is smaller
		if (vstart_even != vflip)
			return -EINVAL;
	}

	qtec_cmosis_update_crop(priv);
	return 0;
}

static int qtec_cmosis_get_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	if (format->pad)
		return -EINVAL;

	format->format=priv->format;

	return 0;
}

static int qtec_cmosis_init_fmt(struct qtec_cmosis *priv){
	unsigned int max_width,max_height;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = 0,
	};

	qtec_cmosis_enum_mbus_code(&priv->sd,NULL,&code);
	format.format.code = code.code;
	qtec_cmosis_max_size(priv,format.format.code,&max_width,&max_height,true);
	format.format.width=max_width;
	format.format.height=max_height;
	qtec_cmosis_set_fmt(&priv->sd,NULL ,&format);
	qtec_cmosis_def_crop(priv,&priv->crop[0]);
	priv->n_crop=1;
	qtec_cmosis_update_crop(priv);
	return 0;
}

static int qtec_cmosis_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
                            struct v4l2_subdev_selection *s){
	struct qtec_cmosis *priv = container_of(sd, struct qtec_cmosis, sd);
	int i;

	if (s->target == V4L2_SEL_TGT_COMPOSE_ACTIVE)
		return 0;

	if (s->target != V4L2_SEL_TGT_CROP_ACTIVE)
		return -EINVAL;

	if (s->rectangles> MAX_CROP)
		return -EINVAL;

	if (priv->streaming){
		struct v4l2_rect crop;
		switch (s->rectangles){
		case 0:
			crop = s->r;
			break;
		case 1:
			crop = s->pr[0].r;
			break;
		default:
			v4l2_warn(&priv->sd, "Online change of cropping area only supported for one rectangle\n");
			return -EBUSY;
		}

		if ((crop.left != priv->crop[0].left) ||
			(crop.height != priv->crop[0].height) ||
			(crop.width != priv->crop[0].width)){
			v4l2_warn(&priv->sd, "Online change of cropping are only supported for top field\n");
			return -EBUSY;
		}

		if (crop.top == priv->crop[0].top) //Nothing to do
			return 0;
	}

	if (s->rectangles){
		for (i=0;i<s->rectangles;i++)
			priv->crop[i]=s->pr[i].r;
		priv->n_crop=s->rectangles;
	}
	else{
		priv->crop[0]=s->r;
		priv->n_crop=1;
	}
	qtec_cmosis_update_crop(priv);

	if (priv->streaming){
		qtec_cmosis_set_crop_top_sensor(priv, qtec_cmosis_vskip(priv));
		if (priv->chip_id == CMV12000v2)
			qtec_cmosis_set_yflip_huge(priv);
	}

	return 0;
}

static int qtec_cmosis_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
                            struct v4l2_subdev_selection *s){
	struct qtec_cmosis *priv = container_of(sd, struct qtec_cmosis, sd);
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
			s->rectangles = 0;
			return qtec_cmosis_def_crop(priv, &s->r);
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		case V4L2_SEL_TGT_CROP_BOUNDS:
			if (qtec_cmosis_max_size_sensor(priv,priv->format.code,&max_width,&max_height,false))
				return -EINVAL;
			s->r.width = max_width;
			s->r.height = max_height;
			s->r.left = 0;
			s->r.top = 0;
			s->rectangles = 0;
			return 0;
		default:
			return -EINVAL;
	}
	return 0;
}

static int qtec_cmosis_enum_fsize(struct v4l2_subdev *subdev,struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);
	unsigned int max_width,max_height;
	unsigned int min_width,min_height;

	if (fse->index!=0)
		return -EINVAL;

	if (qtec_cmosis_max_size(priv,fse->code,&max_width,&max_height,false))
		return -EINVAL;

	if (qtec_cmosis_min_size(priv,fse->code,&min_width,&min_height))
		return -EINVAL;

	fse->min_width=min_width;
	fse->max_width=max_width;
	fse->min_height=min_height;
	fse->max_height=max_height;

	return 0;
}

static int qtec_cmosis_enum_frameintervals(struct v4l2_subdev *subdev, struct v4l2_frmivalenum *fival){
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);
	const struct qtec_cmosis_format *qtec_format=get_format(fival->pixel_format);
	int ret;
	struct v4l2_mbus_framefmt format;

	if (fival->index!=0)
		return -EINVAL;

	if (!qtec_format)
		return -EINVAL;

	format.code=fival->pixel_format;
	format.width=fival->width;
	format.height=fival->height;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;

	ret= qtec_cmosis_min_tpf(priv, format, &fival->stepwise.min);
	if (ret)
		return ret;
	ret= qtec_cmosis_max_tpf(priv, format, &fival->stepwise.max);
	if (ret)
		return ret;

	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1; //compliance
	//fival->stepwise.step.denominator = priv->bus_clk;

	return 0;
}

static int qtec_cmosis_g_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	fival->pad=0;
	fival->interval=priv->fival;

	return 0;
}

static int qtec_cmosis_s_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_frame_interval *fival){
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	fival->pad=0;
	if (fival->interval.denominator==0)
		fival->interval=(struct v4l2_fract) DEF_FIVAL;
	qtec_cmosis_clamp_interval(priv,&fival->interval);
	priv->fival=fival->interval;
	qtec_cmosis_update_exposure_range(priv);
	qtec_cmosis_set_frame_delay(priv);

	return 0;
}

static int qtec_cmosis_s_stream(struct v4l2_subdev *subdev, int enable){
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);
	int ret;

	if (enable){
		ret = qtec_cmosis_start(priv);
		if (!ret)
			priv->streaming=1;
		return ret;
	}

	ret=qtec_cmosis_stop(priv);
	if (!ret)
		priv->streaming=0;
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_cmosis_s_register(struct v4l2_subdev *subdev,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	if (reg->reg == 0x10001){
		priv->stop_idelay_monitor = reg->val;
		v4l2_err(&priv->sd, "Stopping resync loop = %d\n",priv->stop_idelay_monitor);
	} else if (reg->reg == 0x10000){
		v4l2_err(&priv->sd, "Fake Frame error detect via debug interface\n");
		schedule_work(&priv->async_idelay);
	} else if (reg->reg >= 0x3000)
		qtec_cmosis_write_bram(priv,reg->reg-0x3000,reg->val);
	else if (reg->reg >= 0x2000)
		qtec_cmosis_write_pll(priv,reg->reg-0x2000,reg->val);
	else if (reg->reg >= 0x1000)
		qtec_cmosis_write(priv,reg->reg-0x1000,reg->val);
	else
		qtec_cmosis_spi_write(priv,false,reg->reg,1,reg->val&0xffff);
	return 0;
}

static int qtec_cmosis_g_register(struct v4l2_subdev *subdev,
			      struct v4l2_dbg_register *reg)
{
	struct qtec_cmosis *priv = container_of(subdev, struct qtec_cmosis, sd);

	if (reg->reg >= 0x3000){
		uint16_t val;
		qtec_cmosis_read_bram(priv,reg->reg-0x3000,&val);
		reg->val=val;
		reg->size=4;
	}
	else if (reg->reg >= 0x2000){
		uint32_t val;
		qtec_cmosis_read_pll(priv,reg->reg-0x2000,&val);
		reg->val=val;
		reg->size=4;
	}
	else if (reg->reg >= 0x1000){
		uint32_t val;
		qtec_cmosis_read(priv,reg->reg-0x1000,&val);
		reg->val=val;
		reg->size=4;
	}
	else
	{
		uint64_t val;
		qtec_cmosis_spi_read(priv,reg->reg,1,&val);
		reg->val=val;
		reg->size=2;
	}

	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_cmosis_pad_ops = {
	.get_selection = qtec_cmosis_get_selection,
	.set_selection = qtec_cmosis_set_selection,
	.enum_frame_size = qtec_cmosis_enum_fsize,
	.set_fmt = qtec_cmosis_set_fmt,
	.get_fmt = qtec_cmosis_get_fmt,
	.enum_mbus_code = qtec_cmosis_enum_mbus_code,
};

static const struct v4l2_subdev_video_ops qtec_cmosis_video_ops = {
	.s_stream = qtec_cmosis_s_stream,
	.g_frame_interval = qtec_cmosis_g_frame_interval,
	.s_frame_interval = qtec_cmosis_s_frame_interval,
	.enum_frameintervals = qtec_cmosis_enum_frameintervals,
};

static const struct v4l2_subdev_core_ops qtec_cmosis_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_cmosis_g_register,
	.s_register = qtec_cmosis_s_register,
	#endif
};
static const struct v4l2_subdev_ops qtec_cmosis_ops = {
	.core = &qtec_cmosis_core_ops,
	.video = &qtec_cmosis_video_ops,
	.pad = &qtec_cmosis_pad_ops,
};

static int qtec_cmosis_set_binning(struct qtec_cmosis *priv,struct v4l2_ctrl *ctrl){
	struct v4l2_mbus_framefmt new_fmt;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	if (!priv->initialized)
		return 0;

	memcpy(&new_fmt,&priv->format,sizeof(new_fmt));
	qtec_cmosis_try_fmt(&priv->sd,&new_fmt);

	if (memcmp(&new_fmt,&priv->format,sizeof(new_fmt))){
		v4l2_err(&priv->sd, "Invalid combination of binning and format\n");
		v4l2_err(&priv->sd, "old_fmt=%dx%d new_fmt=%dx%d\n",priv->format.width, priv->format.height,new_fmt.width,new_fmt.height);
		return -EIO;
	}

	format.format = new_fmt;
	qtec_cmosis_set_fmt(&priv->sd, NULL, &format);

	return 0;
}

static int qtec_cmosis_set_fpn(struct qtec_cmosis *priv,struct v4l2_ctrl *ctrl){
	int i;

	for (i=0; i< ctrl->dims[0];i++){
		int32_t val=ctrl->p_new.p_s32[i];

		if (val >= 16384)
			val= ((val-16384)/2) & 0xff;
		else
			val = 0x100 | (((val-(16384 - 256*2))/2) & 0xff);

		qtec_cmosis_write_bram(priv,FPN+i,val);
	}

	return 0;
}

/*
 * Controls
 */
static int qtec_cmosis_s_ctrl_aux(struct v4l2_ctrl *ctrl)
{
	struct qtec_cmosis *priv = container_of(ctrl->handler, struct qtec_cmosis, ctrl_handler);
	uint32_t aux;

	switch (ctrl->id){
		case QTEC_CMOSIS_CID_MODE_TRIG:
			if (((ctrl->cur.val == SELF_TIMED) && (ctrl->val == IDLE)) ||
				((ctrl->cur.val == IDLE) && (ctrl->val == SELF_TIMED)))
				break;
		case QTEC_CMOSIS_CID_PHASE:
		case QTEC_CMOSIS_CID_TRIG_POL:
		case V4L2_CID_HFLIP:
		case V4L2_CID_VFLIP:
		case QTEC_CMOSIS_CID_H_BINNING:
		case QTEC_CMOSIS_CID_V_BINNING:
		case QTEC_CMOSIS_CID_BAYER_SKIPPING:
		case QTEC_CMOSIS_CID_BITMODE:
			if (priv->streaming)
				return -EBUSY;
	};

	if (!priv->streaming)
		switch (ctrl->id){ //Do not do spi writes on not streaming sensor #521
			case V4L2_CID_GAIN:
				return qtec_cmosis_set_pga(priv,&ctrl->val, false);
			case QTEC_CMOSIS_CID_ADC_GAIN:
			case QTEC_CMOSIS_CID_OFFSET:
			case QTEC_CMOSIS_CID_VRAMP:
			case QTEC_CMOSIS_CID_DUAL_EXPOSURE:
			case V4L2_CID_EXPOSURE_ABSOLUTE:
			case QTEC_CMOSIS_CID_TRACE_TRIGGER:
			case QTEC_CMOSIS_CID_TRACE_FRAME:
			case QTEC_CMOSIS_CID_TRACE_READOUT:
			case QTEC_CMOSIS_CID_RESPONSE_CURVE:
			case QTEC_CMOSIS_CID_MODE_TRIG:
			case QTEC_CMOSIS_CID_FLASH_DISABLE:
				return 0;
		};

	switch (ctrl->id){
		case QTEC_CMOSIS_CID_PHASE:
			return qtec_cmosis_set_phase(priv,ctrl->val);
		case QTEC_CMOSIS_CID_ADC_GAIN:
			return qtec_cmosis_set_adc_gain(priv,ctrl->val);
		case QTEC_CMOSIS_CID_OFFSET:
			return qtec_cmosis_set_offset(priv,ctrl->val);
		case V4L2_CID_GAIN:
			return qtec_cmosis_set_pga(priv,&ctrl->val, true);
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			priv->exp->val = ctrl->val;
			//Passthrough
		case QTEC_CMOSIS_CID_DUAL_EXPOSURE:
			return qtec_cmosis_set_exposure(priv);
		case QTEC_CMOSIS_CID_RESPONSE_CURVE:
			return qtec_cmosis_set_exposure(priv);
		case V4L2_CID_HFLIP:
			return qtec_cmosis_flip_set(priv,ctrl->val,priv->vflip->val);
		case V4L2_CID_VFLIP:
			return qtec_cmosis_flip_set(priv,priv->hflip->val,ctrl->val);
		case QTEC_CMOSIS_CID_MANUAL_TRIGGER:
			qtec_cmosis_read(priv,CONTROL,&aux);
			aux&=~BIT(TRIG_OVERFLOW);
			aux|=BIT(TRIG_SW);
			qtec_cmosis_write(priv,CONTROL,aux);
			aux&=~BIT(TRIG_SW);
			qtec_cmosis_write(priv,CONTROL,aux);
			return 0;
		case QTEC_CMOSIS_CID_EXT_TRIG_DELAY:
			if (priv->mode_trig->val == EXT_TRIG)
				return qtec_cmosis_set_ext_trig_delay(priv,ctrl->val);
			else
				return 0;
		case QTEC_CMOSIS_CID_EXT_TRIG_OVERFLOW:
			if (priv->quirk & QUIRK_IRQ_OVERFLOW)
				priv->trig_overflow_v32 = false;
			else{
				qtec_cmosis_read(priv,CONTROL,&aux);
				aux |= BIT(TRIG_OVERFLOW);
				qtec_cmosis_write(priv,CONTROL,aux);
			}
			return 0;
		case QTEC_CMOSIS_CID_VRAMP:
			return qtec_cmosis_set_vramp(priv, ctrl->val);
		case QTEC_CMOSIS_CID_H_BINNING:
		case QTEC_CMOSIS_CID_V_BINNING:
			return qtec_cmosis_set_binning(priv, ctrl);
		case QTEC_CMOSIS_CID_OUTPUT_SCALER:
			if (priv->quirk & QUIRK_FB4)
				return qtec_cmosis_set_fb4_output_scaler(priv,ctrl->val);
			qtec_cmosis_read(priv,BINNING,&aux);
			aux &= 0xffff;
			aux |= ctrl->val << (BINSCALE +1);
			qtec_cmosis_write(priv,BINNING,aux);
			return 0;
		case QTEC_CMOSIS_CID_FPN:
			return qtec_cmosis_set_fpn(priv,ctrl);
		case QTEC_CMOSIS_CID_BITMODE:
			priv->calibration_done = false;
			return 0;
		case QTEC_CMOSIS_CID_MODE_TRIG:
			qtec_cmosis_read(priv,CONTROL,&aux);
			if (ctrl->val == IDLE)
				aux &=~BIT(TRIG_ENABLE);
			else
				aux|=BIT(TRIG_ENABLE);
			qtec_cmosis_write(priv,CONTROL,aux);
			return 0;
		case QTEC_CMOSIS_CID_FLASH_DISABLE:
			qtec_cmosis_read(priv,CONTROL,&aux);
			if (ctrl->val)
				aux|=BIT(FLASH_DISABLE);
			else
				aux &=~BIT(FLASH_DISABLE);
			qtec_cmosis_write(priv,CONTROL,aux);
			return 0;
		case QTEC_CMOSIS_CID_FLASH_POL:
			qtec_cmosis_read(priv,CONTROL,&aux);
			if (ctrl->val)
				aux|=BIT(FLASH_POL);
			else
				aux &=~BIT(FLASH_POL);
			qtec_cmosis_write(priv,CONTROL,aux);
			return 0;
	}

	return 0;
}

static int qtec_cmosis_s_ctrl(struct v4l2_ctrl *ctrl){
	int ret;

	ret=qtec_cmosis_s_ctrl_aux(ctrl);

	if (ret)
		ctrl->val=ctrl->cur.val;

	return ret;
}

static int qtec_cmosis_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	struct qtec_cmosis *priv = container_of(ctrl->handler, struct qtec_cmosis, ctrl_handler);
	uint32_t aux;
	const struct qtec_cmosis_format *qtec_format=get_format(priv->format.code);
	int htotal;
	uint32_t lines,diff,orig;

	switch (ctrl->id) {
		case QTEC_CMOSIS_CID_EXPOSURE_STEP:
			lines = qtec_cmosis_time2lines(priv,priv->exp->val);
			lines =  max_t(int,1,lines);
			orig = qtec_cmosis_lines2time(priv,lines);
			aux = qtec_cmosis_lines2time(priv,lines-1);
			diff = orig-aux;
			aux = qtec_cmosis_lines2time(priv,lines+1);
			aux = aux-orig;
			ctrl->val = max(aux,diff);

			return 0;
		case QTEC_CMOSIS_CID_N_CHANNELS:
			ctrl->val=priv->nchans;
			return 0;
		case QTEC_CMOSIS_CID_EXT_TRIG_OVERFLOW:
			if (priv->quirk & QUIRK_IRQ_OVERFLOW)
				ctrl->val = priv->trig_overflow_v32;
			else {
				qtec_cmosis_read(priv,CONTROL,&aux);
				ctrl->val=!!(aux&BIT(TRIG_OVERFLOW));
			}
			return 0;
		case QTEC_CMOSIS_CID_TEMPERATURE:
			if (priv->streaming)
				return qtec_cmosis_temperature(priv,&ctrl->val);

			dev_err(&priv->pdev->dev, "Sensor is powered off, returning invalid temperature\n");
			return 0;
		case V4L2_CID_HBLANK:
			//FIXME
			if (priv->quirk & QUIRK_FB4){
				ctrl->val = 2;
				return 0;
			}

			switch (priv->chip_id){
				case CMV12000v2:
					htotal = 257;
					break;
				case CMV8000v1:
					htotal = 226;
					break;
				default:
					htotal = 129;
					break;
			}
			switch(priv->nchans){
				case 16:
					break;
				case 8:
					htotal *= 2;
					break;
				case 4:
					htotal *= 4;
					break;
				case 2:
					htotal *= 8;
					break;
				default:
					return -EINVAL;
			}
			switch(qtec_format->mode_out){
				case MODE_SINGLE:
					htotal *= 1;
					break;
				case MODE_DECIM:
					htotal *= 2;
					break;
				case MODE_COMPACT:
					htotal *= 5;
					break;
				default:
					return -EINVAL;
			}

			ctrl->val = htotal - priv->format.width;
			return 0;

		case V4L2_CID_VBLANK:
			ctrl->val=1;
			return 0;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops qtec_cmosis_ctrl_ops = {
	.g_volatile_ctrl = qtec_cmosis_g_volatile_ctrl,
	.s_ctrl = qtec_cmosis_s_ctrl,
};

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_bitmode(struct qtec_cmosis *priv, int bitmode){
	static const s64 bitmode_values[] = { 10, 12 };
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.menu_skip_mask = 0,
		.min = 0,
		.max = 1,
		.name = "Sensor Bit Mode",
		.id = QTEC_CMOSIS_CID_BITMODE,
		.qmenu_int = bitmode_values,
		.def = 0,
	};
	ctrl.def=(bitmode==12)?1:0;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_trig(struct qtec_cmosis *priv, char *name, int id,int def){
	static const char * const mode_trig_menu[] = {
		"Self Timed",
		"External Trigger",
		"External Exposure",
		"Idle",
		NULL
	};
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_MENU,
		.qmenu = mode_trig_menu,
		.menu_skip_mask = 0,
		.min = 0,
		.max = 3,
	};
	ctrl.name=name,
	ctrl.id=id;
	ctrl.def=def;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_curve(struct qtec_cmosis *priv, char *name, int id,int def){
	static const char * const mode_trig_menu[] = {
		"Linear",
		"Gamma 2.2",
		"Gamma Newtec",
		"HDR",
		"HDR Dual Exposure",
		NULL
	};
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_MENU,
		.qmenu = mode_trig_menu,
		.menu_skip_mask = 0,
		.min = 0,
		.max = 4,
	};
	ctrl.name=name,
	ctrl.id=id;
	ctrl.def=def;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_bool(struct qtec_cmosis *priv, char *name, int id,int def){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
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

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_bool_volatile(struct qtec_cmosis *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
	};

	ctrl.id=id;
	ctrl.name=name;
	ctrl.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control(struct qtec_cmosis *priv, char *name, int id,int min, int max,int def, int flags){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
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

static char * mosaic_mode2_str(unsigned int bayer){
	switch (bayer){
		case MONO:
			return "Mono";
		case BAYER:
			return "Bayer";
		case MOSAIC5x5:
			return "5x5-Mosaic-IR";
		case BAND100:
			return "100-Band-LS-IR";
	}
	return "Unknown";
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_sensor_type(struct qtec_cmosis *priv){
	int version=0;
	int size=0;
	struct v4l2_ctrl *c;
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
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

	switch(priv->chip_id){
		case CMV2000v2:
			version=2;
			size=2;
			break;
		case CMV4000v2:
			version=2;
			size=4;
			break;
		case CMV2000v3:
			version=3;
			size=2;
			break;
		case CMV4000v3:
			version=3;
			size=4;
			break;
		case CMV8000v1:
			version=1;
			size=8;
			break;
		case CMV12000v2:
			version=2;
			size=12;
			break;
	}
	snprintf(c->p_cur.p_char,32,"CMV%d000v%d %s",size,version,mosaic_mode2_str(priv->mosaic_mode));

	return c;
}

static struct v4l2_ctrl *qtec_cmosis_add_button_control(struct qtec_cmosis *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BUTTON,
	};
	ctrl.name=name,
	ctrl.id=id;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_cmosis_add_custom_control_fpn(struct qtec_cmosis *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_cmosis_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = 16384 - (256*2),
		.max = 16384 + (255*2),
		.def = 16384,
		.id  = QTEC_CMOSIS_CID_FPN,
		.name = "Fixed Pattern Noise Correction",
	};

	ctrl.dims[0] = qtec_cmosis_max_width_sensor(priv, false);

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static inline uint32_t bus2usec(struct qtec_cmosis *priv, uint32_t bus){
	uint64_t aux = bus;

	aux *= (uint64_t) 1000000;
	do_div(aux, priv->bus_clk);

	return aux;
}

static void qt5023_async_retrain_delay(struct work_struct *work){
	struct qtec_cmosis *priv = container_of(work, struct qtec_cmosis , async_idelay);

	if (!(priv->quirk & QUIRK_IDELAY))
		return;
	/*
	 * Do not re-enable Frame-error detect.
	 * Up to date, we do not have an online way to recalibrate
	 * and we will keep getting IRQ errors, which will increase
	 * the power usage, that will create more Frame-erros, that
	 * will increase the power usage....
	 */
	return;
}

static irqreturn_t qtec_cmosis_irq(int irq, void *data){
	struct qtec_cmosis *priv = data;
	uint32_t over;

	qtec_cmosis_read(priv, TRC_OVERFLOW, &over);
	if (!(over & (BIT(OVERFLOW_STATUS)|BIT(SYNC_ERR_STATUS)))){
		v4l2_err(&priv->sd, "Spureous IRQ 0x%.8x\n", over);
	}

	if (over & BIT(SYNC_ERR_STATUS)){
		uint32_t aux;

		over &= ~BIT(SYNC_ERR_EN); //Disable irq until retrain

		qtec_cmosis_read(priv, FRAME_ERROR_STATUS, &aux);
		if (aux){
			v4l2_err(&priv->sd, "Frame error detected on channel(s) 0x%.4x\n", aux);
			schedule_work(&priv->async_idelay);
		}
		else
			v4l2_err(&priv->sd, "Spureous Frame error IRQ 0x%.8x 0x%.8x\n", over, aux);
	}

	if (over & BIT(OVERFLOW_STATUS)){
		priv->trig_overflow_v32 = true;
		v4l2_err(&priv->sd, "%d Trigger overflow event(s) detected\n", over&0xfff);
	}

	over &= BIT(SYNC_ERR_EN) | BIT(SYNC_ERR_STATUS) | BIT(OVERFLOW_EN) | BIT(OVERFLOW_STATUS);
	qtec_cmosis_write(priv, TRC_OVERFLOW, over);

	return IRQ_HANDLED;
}

//Probe
static int qtec_cmosis_init_quirks(struct qtec_cmosis *priv){
	uint32_t version;

	qtec_cmosis_read(priv,CONTROL,&version);

	priv->version =  (version >> VERSION)&0xff;

	switch (priv->version){
	case 0x12:
	case 0x13:
		break;
	case 0x15:
		priv->quirk = QUIRK_BINNING;
		priv->quirk |= QUIRK_FPN;
		break;
	case 0x16:
		priv->quirk = QUIRK_BINNING;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_SPI;
		break;
	case 0x17:
		priv->quirk = QUIRK_BINNING;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FAST_BINNING;
		priv->quirk |= QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		break;
	case 0x18:
		priv->quirk = QUIRK_BINNING;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FAST_BINNING;
		priv->quirk |= QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_PROG_BITMODE;
		break;
	case 0x19:
		priv->quirk = QUIRK_BINNING;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FAST_BINNING;
		priv->quirk |= QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_PROG_FLASH_WIDTH;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		break;
	case 0x30:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		break;
	case 0x31:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		break;
	case 0x32:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		break;
	case 0x33:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		priv->quirk |= QUIRK_PROG_FLASH_WIDTH;
		break;
	case 0x40:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		priv->quirk |= QUIRK_FB4;
		break;
	case 0x41:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		priv->quirk |= QUIRK_FB4;
		priv->quirk |= QUIRK_PROG_FLASH_WIDTH;
		priv->quirk |= QUIRK_PHASE_INC;
		break;
	case 0x42:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		priv->quirk |= QUIRK_FB4;
		priv->quirk |= QUIRK_PROG_FLASH_WIDTH;
		priv->quirk |= QUIRK_NO_PHASE_IDELAY;
		break;
	case 0x43:
		priv->quirk = QUIRK_SPI;
		priv->quirk |= QUIRK_SPI_IDLE;
		priv->quirk |= QUIRK_BURST_224;
		priv->quirk |= QUIRK_PROG_BITMODE;
		priv->quirk |= QUIRK_IRQ_OVERFLOW;
		priv->quirk |= QUIRK_FREQ_DELAY;
		priv->quirk |= QUIRK_VERSION_SENSOR;
		priv->quirk |= QUIRK_SYNC_ONLY_USED_CHANS;
		priv->quirk |= QUIRK_IDELAY;
		priv->quirk |= QUIRK_IRQ_SYNC_ERROR;
		priv->quirk |= QUIRK_FPN;
		priv->quirk |= QUIRK_FLASH_DISABLE;
		priv->quirk |= QUIRK_FB4;
		priv->quirk |= QUIRK_PROG_FLASH_WIDTH;
		priv->quirk |= QUIRK_PHASE_INC;
		priv->quirk |= QUIRK_IDELAY_MONITOR;
		break;
	default:
		dev_err(&priv->pdev->dev, "Unknown core version 0x%.2x. Aborting\n",priv->version);
		return -1;
	}

	return 0;

}

static int qtec_cmosis_probe_sensor(struct qtec_cmosis *priv){
	uint64_t aux;

	//disable tristate
	qtec_cmosis_write(priv,CONTROL,0);

	//force pll start
	if (qtec_cmosis_start_pll(priv))
		return -EIO;

	//Reset cmosis
	qtec_cmosis_reset_sensor(priv, false);

	qtec_cmosis_spi_read(priv,REG_ID,1,&aux);
	if (aux==0){
		qtec_cmosis_spi_read(priv,REG_ID_8M,1,&aux);
		if (aux!=CMV8000v1){
			//Force word read
			priv->quirk |= QUIRK_SPI_16_BITS;
			qtec_cmosis_spi_read(priv,REG_ID,1,&aux);
			if (aux!=CMV12000v2){

				dev_err(&priv->pdev->dev,"qtec_cmosis:  Error reading chip ID 0x%0llx, check SPI speed\n",aux);
				return -EINVAL;
			}
		}
	}

	//Probe spi chip
	switch (aux){
		case CMV4000v2:
		case CMV2000v2:
			priv->quirk |= QUIRK_SPI_NO_FOT;
			//fall through
		case CMV2000v3:
		case CMV4000v3:
		case CMV8000v1:
		case CMV12000v2:
			break;
		default:
			dev_err(&priv->pdev->dev,"qtec_cmosis: ERROR Unknown Chip 0x%llx\n",aux);
			return -EINVAL;
	}

	priv->chip_id=aux;

	if (((aux==CMV8000v1) && !(priv->quirk&QUIRK_BURST_224)) ||
		((aux==CMV12000v2) && !(priv->quirk & QUIRK_VERSION_SENSOR)))
	{
		dev_err(&priv->pdev->dev,"qtec_cmosis: ERROR This core does not support this chip\n");
		return -EINVAL;
	}

	if ((aux==CMV8000v1) && (priv->quirk & QUIRK_BINNING)){ //Workaround for #588
		dev_err(&priv->pdev->dev,"qtec_cmosis: ERROR Binning bitstreams do not support CMV8000\n");
		return -EINVAL;
	}

	//Save fot_len (needed for FOT calculations)
	switch (aux){
		case CMV2000v2:
		case CMV2000v3:
			qtec_cmosis_spi_read(priv,73,1,&aux);
			break;
		case CMV4000v2:
		case CMV4000v3:
			aux=20;//Force to 20 (acording to datasheet 3.5)
			break;
		case CMV8000v1:
			aux=50;//Set to 50 (acording to datasheet 1.1)
			break;
	}

	priv->fot_len=aux;

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

static atomic_t qtec_cmosis_instance = ATOMIC_INIT(0);
static int qtec_cmosis_probe(struct platform_device *pdev){
	struct qtec_cmosis *priv;
	struct resource res;
	int ret;
	struct device_node *node;
	uint32_t aux,bitmode;
	const char *pll_name;

	priv=(struct qtec_cmosis *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

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

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bayer_chip",&aux);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse is_bayer_chip property\n");
		return -EIO;
	}
	priv->mosaic_mode=aux;
	if (priv->mosaic_mode > BAND100){
		dev_err(&pdev->dev, "Invalid bayer mode %d\n",aux);
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bit_mode",&bitmode);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse bit_mode property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,sync_word",&priv->sync_word);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse sync_word property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,bus_clk",&priv->bus_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse bus_clk property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,pixel_clk",&priv->pixel_clk);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse pixel_clk property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,sync_channels",&priv->max_channels);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse sync_channels property\n");
		return -EIO;
	}
	priv->sync_channels = priv->max_channels;
	priv->nchans=priv->sync_channels;

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,offset",&priv->of_offset);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse offset property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,adc_gain",&priv->of_adc_gain);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse adc_gain property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,vramp",&priv->of_vramp);
	if (ret){
		dev_err(&pdev->dev, "Unable to parse vramp property\n");
		return -EIO;
	}

	ret=of_property_read_u32(priv->pdev->dev.of_node,"qtec,mutex_spi",&aux);
	priv->spi_mutex = !ret;

	ret=qtec_cmosis_init_quirks(priv);
	if (ret<0)
		return ret;

	if (!(priv->quirk & QUIRK_SPI)){
		node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,cmosis_device",0);
		if (!node){
			dev_err(&pdev->dev, "Unable to parse cmosis_device phandle\n");
			return -EIO;
		}
		priv->cmosis_spi=of_find_spi_device_by_node(node);
		if (!priv->cmosis_spi){
			dev_err(&pdev->dev, "Unable to find spi cmosis dev\n");
			return -EIO;
		}
		of_node_put(node);

		ret=spi_setup(priv->cmosis_spi);
		if (ret){
			dev_err(&pdev->dev, "Unable to setup spi cmosis dev %d\n",ret);
			return ret;
		}
	}

	if (priv->quirk & QUIRK_IRQ_OVERFLOW){
		ret=of_irq_to_resource(pdev->dev.of_node,0,&res);
		if(!ret){
			dev_err(&pdev->dev, "Unable to get overflow irq\n");
			return -EIO;
		}
		ret = devm_request_irq(&pdev->dev,res.start,qtec_cmosis_irq,0,DRIVER_NAME,priv);
		if(ret){
			dev_err(&pdev->dev, "Unable to request overflow irq\n");
			return -EIO;
		}
	}

	ret=qtec_cmosis_probe_sensor(priv);
	if (ret<0)
		return ret;

	//If there are only 4 channels, use 12 bit mode and disable programmable bitmode.
	//Dual eye does not support programmable bitmode due to lack of clock nets
	if ((priv->sync_channels == 4) && (priv->chip_id != CMV8000v1) && priv->spi_mutex){
		dev_info(&pdev->dev, "Dual eye does not support programmable bitmode\n");
		bitmode = 12;
		priv->quirk |= QUIRK_PROG_BITMODE_IGNORE;
		priv->dual_eye = true;
	}

	priv->initialized=false;
	priv->trig_overflow_v32 = false;
	mutex_init(&priv->idelay_mutex);

	//Subdev
	v4l2_subdev_init(&priv->sd, &qtec_cmosis_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_cmosis_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	//Controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 32); //32 is just a guess for the hash table

	priv->mode_trig = qtec_cmosis_add_custom_control_trig(priv,"Trigger Mode",QTEC_CMOSIS_CID_MODE_TRIG,MODE_SELF_TIMED);
	priv->phase = qtec_cmosis_add_custom_control(priv,"Sync Phase",QTEC_CMOSIS_CID_PHASE,0,qtec_cmosis_max_phase(priv),qtec_cmosis_max_phase(priv)/2, V4L2_CTRL_FLAG_READ_ONLY);
	priv->flash_pol = qtec_cmosis_add_custom_control_bool(priv,"Invert Flash Polarity",QTEC_CMOSIS_CID_FLASH_POL,0);
	priv->trig_pol = qtec_cmosis_add_custom_control_bool(priv,"Invert Trigger Polarity",QTEC_CMOSIS_CID_TRIG_POL,0);
	if (priv->quirk & QUIRK_FLASH_DISABLE)
		priv->flash_disable = qtec_cmosis_add_custom_control_bool(priv,"Disable Flash",QTEC_CMOSIS_CID_FLASH_DISABLE,0);
	if (priv->chip_id == CMV12000v2)
		priv->adc_gain = qtec_cmosis_add_custom_control(priv,"ADC Gain",QTEC_CMOSIS_CID_ADC_GAIN,-255,255,priv->of_adc_gain,V4L2_CTRL_FLAG_SLIDER);
	else
		priv->adc_gain = qtec_cmosis_add_custom_control(priv,"ADC Gain",QTEC_CMOSIS_CID_ADC_GAIN,0,63,priv->of_adc_gain,V4L2_CTRL_FLAG_SLIDER);//#532

	switch (priv->chip_id){
		case CMV8000v1:
		case CMV12000v2:
			priv->vramp = qtec_cmosis_add_custom_control(priv,"V Ramp",QTEC_CMOSIS_CID_VRAMP,0,127,priv->of_vramp&0x7f, V4L2_CTRL_FLAG_SLIDER);
			priv->vramp->flags |= V4L2_CTRL_FLAG_READ_ONLY;
			break;
		default:
			priv->vramp = qtec_cmosis_add_custom_control(priv,"V Ramp",QTEC_CMOSIS_CID_VRAMP,
					min(102,priv->of_vramp&0x7f),max(115,priv->of_vramp&0x7f),priv->of_vramp&0x7f, V4L2_CTRL_FLAG_SLIDER);
			break;
	}
	priv->exp = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE, 0, 0xffffff, 1,50000);
	priv->dual_exp = qtec_cmosis_add_custom_control(priv,"Dual Exposure",QTEC_CMOSIS_CID_DUAL_EXPOSURE, 0, 0xffffff, 50000, 0);
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,
			V4L2_CID_HBLANK, 0, 0xffffff, 1,0);
	priv->hblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY ;
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,
			V4L2_CID_VBLANK, 0, 0xffffff, 1,0);
	priv->vblank->flags|=V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_READ_ONLY;
	priv->hflip = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1,0);
	priv->vflip = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, priv->dual_eye ? 0 : 1);
	//Max pga depends on chip
	switch (priv->chip_id){
	case CMV2000v2:
	case CMV4000v2:
		aux=4082;
		break;
	case CMV2000v3:
	case CMV4000v3:
		aux=10103;
		break;
	case CMV8000v1:
	case CMV12000v2:
	default:
		aux=12042;
		break;

	}
	priv->pga_gain = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_cmosis_ctrl_ops,V4L2_CID_GAIN, 0, aux,1,0);
	if (priv->chip_id==CMV8000v1 && bitmode==12)
		priv->of_offset  = clamp(priv->of_offset *4,-2048,2047); //#518
	priv->offset = qtec_cmosis_add_custom_control(priv,"Offset",QTEC_CMOSIS_CID_OFFSET,(priv->chip_id == CMV8000v1 || priv->chip_id == CMV12000v2 )?-2048:-8192,
								(priv->chip_id == CMV8000v1 || priv->chip_id == CMV12000v2)?2047:800,priv->of_offset, V4L2_CTRL_FLAG_SLIDER); //check #285
	priv->manual_trigger = qtec_cmosis_add_button_control(priv,"Manual Trigger",QTEC_CMOSIS_CID_MANUAL_TRIGGER);
	priv->ext_trig_delay = qtec_cmosis_add_custom_control(priv,"External Trigger Delay",QTEC_CMOSIS_CID_EXT_TRIG_DELAY,0x0,
			qtec_cmosis_max_delay_ext_trig(priv),0,V4L2_CTRL_FLAG_SLIDER);
	priv->trigger_overflow = qtec_cmosis_add_custom_control_bool_volatile(priv,"External Trigger Overflow",QTEC_CMOSIS_CID_EXT_TRIG_OVERFLOW);

	priv->temperature = qtec_cmosis_add_custom_control(priv,"Sensor Temperature",QTEC_CMOSIS_CID_TEMPERATURE,-200000,200000,20000, V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);
	priv->nchans_ctrl = qtec_cmosis_add_custom_control(priv,"Number of Channels",QTEC_CMOSIS_CID_N_CHANNELS,1,16,16, V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);

	priv->sensor_type = qtec_cmosis_add_custom_control_sensor_type(priv);

	priv->h_binning = qtec_cmosis_add_custom_control(priv,"Horizontal Binning",QTEC_CMOSIS_CID_H_BINNING,1,16,1, V4L2_CTRL_FLAG_SLIDER);
	priv->v_binning = qtec_cmosis_add_custom_control(priv,"Vertical Binning",QTEC_CMOSIS_CID_V_BINNING,1,16,1, V4L2_CTRL_FLAG_SLIDER);
	priv->bayer_skipping = qtec_cmosis_add_custom_control_bool(priv,"Bayer Skipping",QTEC_CMOSIS_CID_BAYER_SKIPPING,priv->mosaic_mode==BAYER?1:0);
	priv->exp_step = qtec_cmosis_add_custom_control(priv,"Exposure Step",QTEC_CMOSIS_CID_EXPOSURE_STEP,0,1000000,1, V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY);

	if ((priv->quirk & QUIRK_BINNING) || (priv->quirk & QUIRK_FB4))
		priv->output_scaler = qtec_cmosis_add_custom_control(priv,"Output Scaler",QTEC_CMOSIS_CID_OUTPUT_SCALER,0/2,0xffff/2,BIT(15)/2, V4L2_CTRL_FLAG_SLIDER); //*2 to Match white balance format
	if (priv->quirk & QUIRK_FPN)
		priv->fpn = qtec_cmosis_add_custom_control_fpn(priv);

	priv->bitmode = qtec_cmosis_add_custom_control_bitmode(priv,bitmode);

	priv->resp_curve = qtec_cmosis_add_custom_control_curve(priv,"Response Curve",QTEC_CMOSIS_CID_RESPONSE_CURVE,0);

	//Init controls
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	if ((!(priv->quirk & QUIRK_PROG_BITMODE)) || (priv->quirk &QUIRK_PROG_BITMODE_IGNORE))
		priv->bitmode->flags|=V4L2_CTRL_FLAG_READ_ONLY;

	priv->v_binning->flags|=V4L2_CTRL_FLAG_READ_ONLY ;
	priv->h_binning->flags|=V4L2_CTRL_FLAG_READ_ONLY ;

	if ( (((priv->mosaic_mode==MONO) && (priv->quirk & QUIRK_BINNING))) ||
		(((priv->mosaic_mode==MONO) ||  (priv->mosaic_mode==BAYER))  && (priv->quirk & QUIRK_FB4))) {
		priv->v_binning->flags &= ~V4L2_CTRL_FLAG_READ_ONLY ;
		priv->h_binning->flags &= ~V4L2_CTRL_FLAG_READ_ONLY ;
	}

	INIT_WORK(&priv->async_idelay, qt5023_async_retrain_delay);

	//Connect to subdev
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	//Init video infrasctuctures
	priv->fival=(struct v4l2_fract) DEF_FIVAL;
	qtec_cmosis_init_fmt(priv);
	qtec_cmosis_update_exposure_range(priv);
	priv->streaming=0;

	//CMV8000 needs clk1 to be setup differently
	if (priv->chip_id == CMV8000v1)
		qtec_cmosis_divide_clock_big(priv);

#ifdef KINTEX_TEMP_WORKAROUND
	if (priv->pll_7series && priv->pixel_clk==47500000)
		qtec_cmosis_divide_pixel_clock(priv);
#endif

	qtec_cmosis_calibrate(priv);

	/*Save some power by stopping the lvds channels*/
	qtec_cmosis_stop_sensor(priv);

	if (priv->quirk & QUIRK_IDELAY_MONITOR){
		priv->kthread = kthread_run(qtec_cmosis_idelay_monitor,priv,"idelay monitor");
		if (IS_ERR(priv->kthread))
			v4l2_err(&priv->sd, "Error starting idelay monitorthread. Continuing  with limited functionality\n");
	}

	priv->initialized=true;

	v4l2_info(&priv->sd, "qtec_cmosis fg core version 0x%.2x (%s) V4L2 subdevice registered as %s\n", priv->version ,priv->sensor_type->p_cur.p_char, priv->sd.name);

	return 0;
}

static int __exit qtec_cmosis_remove(struct platform_device *pdev){
	struct qtec_cmosis *priv=platform_get_drvdata(pdev);

	if (priv->quirk & QUIRK_IDELAY_MONITOR)
		kthread_stop(priv->kthread);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static struct of_device_id qtec_cmosis_of_match[] = {
	{ .compatible = "qtec,axi_cmosis_if-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_cmosis_of_match);

static struct platform_driver qtec_cmosis_plat_driver = {
	.probe		= qtec_cmosis_probe,
	.remove		= qtec_cmosis_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_cmosis_of_match,
	},
};

module_platform_driver(qtec_cmosis_plat_driver);

MODULE_DESCRIPTION("CMOSIS module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
