/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/lcm.h>
#include <linux/leds.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-sg.h>
#include <linux/sched.h>

#define DRIVER_NAME "qt5023_video"

//#define BANDWITH_TEST 16
//#define INTERVAL_TEST

static uint32_t (*qt5023_video_encoder_provider)(void) = NULL;
EXPORT_SYMBOL(qt5023_video_encoder_provider);

static struct v4l2_device *qt5023_v4l2_device = NULL;
EXPORT_SYMBOL(qt5023_v4l2_device);

#define QUIRK_BYTESWAP (BIT(0))
#define QUIRK_TIMESTAMP (BIT(1))
#define QUIRK_FB4 (BIT(2))
#define QUIRK_HSV (BIT(3))

#define MAX_APERTURES 6
struct qt5023_video_buf{
	struct vb2_v4l2_buffer vb2_buf; //Must be first in struct
	struct list_head list;
	int n_desc;
	struct compact_desc{
		unsigned long length;
		unsigned long addr;
	}*compact_desc;
	uint64_t ap_addr[MAX_APERTURES];
	int n_ap;
	struct qt5023_frame *frame;
};

#define APERTURE_REG_LEN 8
struct qt5023_video_pcie_aperture{
	int index;
	uint32_t fpga_addr;
	unsigned long length;
};

#define CHANNELS_PER_FRAMEBUS 5
#define CHANNELS_PER_FB4 4
#define LEDNAME 32
struct qt5023_video{
	struct v4l2_device v4l2_dev; //NEEDs to be first!!!!
	struct vb2_queue vb2_vidq;
	struct v4l2_ctrl_handler ctrl_handler;
	struct video_device	   vdev;
	struct mutex vdev_mutex;
	int num;
	const char *sensor_serial;
	int bitstream_version;
	uint32_t i2c_addr_val;

	/*Led triggers*/
	struct led_trigger *led_img;
	char    led_img_name[LEDNAME];
	struct led_trigger *led_drop;
	char    led_drop_name[LEDNAME];

	/*Submodules*/
	struct v4l2_subdev *sd_sensor;
	struct v4l2_subdev *sd_white;
	struct v4l2_subdev *sd_testgen;
	struct v4l2_subdev *sd_xform;
	struct v4l2_subdev *sd_encoder;
	struct v4l2_subdev *sd_m43;
	struct work_struct init_subdev_wk;
	struct work_struct reset_pipeline_wk;

	/*Controls*/
	struct v4l2_ctrl	   *drop_frames;
	struct v4l2_ctrl	   *buffer_size;
	struct v4l2_ctrl	   *active_frames_ctrl;
	struct v4l2_ctrl	   *sensor_serial_ctrl;
	struct v4l2_ctrl	   *bitstream_ctrl;
	struct v4l2_ctrl	   *reset_pipeline;
	struct v4l2_ctrl	   *head_i2c_addr;
	struct v4l2_ctrl	   *head_i2c_bus;

	struct i2c_client *i2c_m43;

	/*io*/
	void __iomem *iomem;
	struct platform_device *pdev;

	spinlock_t slock;
	struct task_struct *kthread;
	struct list_head buffer_list;
	struct completion thread_comp;
	struct list_head frame_list;

	/*Formats*/
	struct qt5023_video_fmt *formats;
	int n_formats;

	//packer
	void __iomem *packer_iomem;
	unsigned long active_frames;
	unsigned long frame_seq;
	unsigned long frame_drop;
	unsigned long do_reset_pipeline;
	unsigned long must_stop;

	//cdma
	void __iomem *cdma_iomem;
	int cdma_n_desc;
	void __iomem *cdma_desc_iomem;
	uint32_t cdma_desc_base_addr;
	struct qt5023_video_buf *cdma_current_buf;
	bool cdma_ready;

	//dma
	void __iomem *dma_iomem;
	int dma_n_desc;
	int dma_n_desc_preloaded;
	void __iomem *dma_desc_iomem;
	uint32_t dma_desc_base_addr;
	int dma_running_desc;
	uint32_t dma_last_circular;

	//apertures
	void __iomem *aperture_iomem;
	struct qt5023_video_pcie_aperture apertures[MAX_APERTURES];
	int n_apertures;

	//pll
	void __iomem *pll_iomem;
	bool pll_7series;

	//encoder
	void __iomem *encoder_iomem;

	//circular buffer
	uint32_t circular_address;
	unsigned long circular_length;
	unsigned long circular_nbuff;

	struct v4l2_format format;
	struct v4l2_fract timeperframe;

	int i2c_adapter;

	uint16_t version;
	uint32_t quirk;

#ifdef INTERVAL_TEST
	s64 last_interval;
	s64 last_timestamp;
#endif
};

static const struct v4l2_fract tpf_default = {.numerator = 1,       .denominator = 1}; //1fps

struct qt5023_frame{
	struct list_head list;
	unsigned long size;
	unsigned long seq;
	unsigned long circular_address;
	u64 timestamp;
	struct v4l2_timecode timecode;
};

#define MAX_DESC_SIZE ((8*1024*1024)-1)
struct qt5023_video_axi_cdma_desc{
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

#define N_COEFF 3
struct fb_matrix_coeff{
	uint16_t rgbmult[N_COEFF][N_COEFF];
	uint16_t offset[N_COEFF];
	uint32_t hsv[N_COEFF];
};

enum packer_mode {FB_FIXED, FB_DYNAMIC, FB_DYNAMIC_HSV ,FB4};

enum fb4_pack_mode {MONO16=0, MONO8, RGB16, RGB8};

struct qt5023_video_fmt {
	const char *name;
	uint32_t   fourcc;          /* v4l2 format id */
	u32 mbus_formats[3];
	uint8_t    depth;
	enum packer_mode packer_mode;
	u32 n_encoding;
	u32 encoding[2];
	union {
		struct packer_framebus_dynamic_config {
			struct fb_matrix_coeff coeff[2];
			int bits_per_channel;
			int channels_per_pixel;
			int map[CHANNELS_PER_FRAMEBUS];
			int ram_select[CHANNELS_PER_FRAMEBUS];
		} fb_dinst;
		struct packer_framebus_fixed_config {
			u32 encoding;
			struct fb_matrix_coeff coeff[2];
			int n_inst;
			const uint32_t *inst;
		} fb_finst;
		struct packer_fb4_config {
			enum fb4_pack_mode mode;
			int map[CHANNELS_PER_FB4];
		} fb4_config;
	};

	bool little_endian;
	unsigned int packer_pixels_per_sensor_pixels;
};

static const uint32_t uyvy_inst[] = {
			0x0000e1c0,
			0x0000a030,
			0x0000620c,
			0x00022503,
			0x0000ebc0,
			0x0000aa30,
			0x00006c0c,
			0x00022f03,
			};
static const uint32_t yuyv_inst[] = {
			0x0000e0c0,
			0x0000a130,
			0x0000650c,
			0x00022203,
			0x0000eac0,
			0x0000ab30,
			0x00006f0c,
			0x00022c03,
			};

#define COEFF_GRAY {{ .rgbmult = {{0x1323,0x2591,0x74c},{0,0,0},{0,0,0}}, .offset = {0,0,0}, .hsv = {0,0,0} }} /*rec601 luma*/
#define COEFF_YUV  {{ .rgbmult = {{4211,8257,1605},{0xffff-2425,0xffff-4768,7193},{7193,0xffff-6029,0xffff-1163}}, .offset={0x1000,0x8000,0x8000}, .hsv = {0,0,0} }} //16386*http://www.fourcc.org/fccyvrgb.php
#define COEFF_BYPASS {{ .rgbmult = {{0x4000,0,0},{0,0x4000,0},{0,0,0x4000}}, .offset = {0,0,0}, .hsv = {0,0,0}} }
#define COEFF_HSV_180 { .rgbmult = {{0x4000,0,0},{0,0x4000,0},{0,0,0x4000}}, .offset = {0,0,0}, .hsv = {0xf000,0xff00,0x8000}}
#define COEFF_HSV_256 { .rgbmult = {{0x4000,0,0},{0,0x4000,0},{0,0,0x4000}}, .offset = {0,0,0}, .hsv = {0x15000,0xff00,0x8000}}

static const struct qt5023_video_fmt def_formats[] = {
	{
		.name     = "RGB 24bits",
		.fourcc   = V4L2_PIX_FMT_RGB24,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 24,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 3,
			.map = {0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "RGB 24bits",
		.fourcc   = V4L2_PIX_FMT_RGB24,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_RGBX},
		.depth    = 24,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = RGB8,
			.map = {0,1,2,-1},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "RGB 32bits",
		.fourcc   = V4L2_PIX_FMT_RGB32,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {3,0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "RGB 32bits",
		.fourcc   = V4L2_PIX_FMT_RGB32,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_RGBX},
		.depth    = 32,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {-1,0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGR 24bits",
		.fourcc   = V4L2_PIX_FMT_BGR24,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 24,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 3,
			.map = {2,1,0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGR 24bits",
		.fourcc   = V4L2_PIX_FMT_BGR24,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_RGBX},
		.depth    = 24,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = RGB8,
			.map = {2,1,0,-1},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGR 32bits",
		.fourcc   = V4L2_PIX_FMT_BGR32,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {2,1,0,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGR 32bits",
		.fourcc   = V4L2_PIX_FMT_BGR32,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_RGBX},
		.depth    = 32,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {2,1,0,-1},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 8bits",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_GRAY,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 16bits",
		.fourcc   = V4L2_PIX_FMT_Y16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 16,
		.packer_mode = FB_DYNAMIC,
		.little_endian = true,
		.fb_dinst ={
			.coeff    = COEFF_GRAY,
			.bits_per_channel = 16,
			.channels_per_pixel = 1,
			.map = {0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 16bits BE",
		.fourcc   = V4L2_PIX_FMT_Y16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 16,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_GRAY,
			.bits_per_channel = 16,
			.channels_per_pixel = 1,
			.map = {0,1,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 8bits",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_MONO,MEDIA_BUS_FMT_QTEC_LEGACY_MONO},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 8bits",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_MONO},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 8bits",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREY},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 16bits",
		.fourcc   = V4L2_PIX_FMT_Y16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_MONO,MEDIA_BUS_FMT_QTEC_LEGACY_MONO},
		.depth    = 16,
		.packer_mode = FB_DYNAMIC,
		.little_endian = true,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 16,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 16bits",
		.fourcc   = V4L2_PIX_FMT_Y16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_MONO},
		.depth    = 16,
		//.little_endian = true, //This is inverted in the core
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 16bits",
		.fourcc   = V4L2_PIX_FMT_Y16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREY},
		.depth    = 16,
		//.little_endian = true, //This is inverted in the core
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 16bits BE",
		.fourcc   = V4L2_PIX_FMT_Y16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_MONO,MEDIA_BUS_FMT_QTEC_LEGACY_MONO},
		.depth    = 16,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 16,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Mono 16bits BE",
		.fourcc   = V4L2_PIX_FMT_Y16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_MONO},
		.depth    = 16,
		.little_endian = true, //This is inverted in the core
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Grey 16bits BE",
		.fourcc   = V4L2_PIX_FMT_Y16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREY},
		.depth    = 16,
		.little_endian = true, //This is inverted in the core
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer BGGR 8bits",
		.fourcc   = V4L2_PIX_FMT_SBGGR8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_BGGR,MEDIA_BUS_FMT_QTEC_LEGACY_BGGR},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer BGGR 8bits",
		.fourcc   = V4L2_PIX_FMT_SBGGR8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_BGGR},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer GBRG 8bits",
		.fourcc   = V4L2_PIX_FMT_SGBRG8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_GBRG,MEDIA_BUS_FMT_QTEC_LEGACY_GBRG},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer GBRG 8bits",
		.fourcc   = V4L2_PIX_FMT_SGBRG8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GBRG},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer GRBG 8bits",
		.fourcc   = V4L2_PIX_FMT_SGRBG8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_GRBG,MEDIA_BUS_FMT_QTEC_LEGACY_GRBG},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer GRBG 8bits",
		.fourcc   = V4L2_PIX_FMT_SGRBG8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GRBG},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer RGGB 8bits",
		.fourcc   = V4L2_PIX_FMT_SRGGB8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_COMPACT_RGGB,MEDIA_BUS_FMT_QTEC_LEGACY_RGGB},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 1,
			.map = {0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Bayer RGGB 8bits",
		.fourcc   = V4L2_PIX_FMT_SRGGB8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_RGGB},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "UYVY 4:2:2",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 16,
		.packer_mode = FB_FIXED,
		.fb_finst ={
			.coeff    = COEFF_YUV,
			.n_inst = ARRAY_SIZE(uyvy_inst),
			.inst = uyvy_inst,
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "YUYV 4:2:2",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 16,
		.packer_mode = FB_FIXED,
		.fb_finst ={
			.coeff    = COEFF_YUV,
			.n_inst = ARRAY_SIZE(yuyv_inst),
			.inst = yuyv_inst,
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "RGB++ 40bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_RGBPP40,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP,MEDIA_BUS_FMT_QTEC_LEGACY_RGB,MEDIA_BUS_FMT_QTEC_LEGACY_MONO},
		.depth    = 40,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 5,
			.map = {0,1,2,3,4},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	/*{
		.name     = "RGB++ 80bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_RGBPP80,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP,MEDIA_BUS_FMT_QTEC_LEGACY_RGB,MEDIA_BUS_FMT_QTEC_LEGACY_MONO},
		.depth    = 80,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 16,
			.channels_per_pixel = 5,
			.map = {0,1,2,3,4},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},*/
	{
		.name     = "Green 8bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB},
		.depth    = 8,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 8,
			.channels_per_pixel = 2,
			.map = {3,4},
		},
		.packer_pixels_per_sensor_pixels = 2,
	},
	{
		.name     = "Green 8bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN8,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREEN},
		.depth    = 8,
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO8,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Green 16bits BE",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB},
		.depth    = 16,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 16,
			.channels_per_pixel = 2,
			.map = {3,4},
		},
		.packer_pixels_per_sensor_pixels = 2,
	},
	{
		.name     = "Green 16bits BE",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN16_BE,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREEN},
		.depth    = 16,
		.packer_mode = FB4,
		.little_endian = true, // this is inverted in the core
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "Green 16bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB},
		.depth    = 16,
		.little_endian = true,
		.packer_mode = FB_DYNAMIC,
		.fb_dinst ={
			.coeff    = COEFF_BYPASS,
			.bits_per_channel = 16,
			.channels_per_pixel = 2,
			.map = {3,4},
		},
		.packer_pixels_per_sensor_pixels = 2,
	},
	{
		.name     = "Green 16bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_GREEN16,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_FB4_GREEN},
		.depth    = 16,
		//.little_endian = true, //this is inverted in the core
		.packer_mode = FB4,
		.fb4_config ={
			.mode = MONO16,
			.map = {0,1,2,3},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "xHSV 32bits",
		.fourcc   = V4L2_PIX_FMT_HSV32,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC_HSV,
		.n_encoding = 2,
		.encoding = { V4L2_HSV_ENC_180, V4L2_HSV_ENC_256},
		.fb_dinst ={
			.coeff    = {COEFF_HSV_180, COEFF_HSV_256},
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {3,0,1,2},
			.ram_select = {0,2,2,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "HSV 24bits",
		.fourcc   = V4L2_PIX_FMT_HSV24,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 24,
		.packer_mode = FB_DYNAMIC_HSV,
		.n_encoding = 2,
		.encoding = { V4L2_HSV_ENC_180, V4L2_HSV_ENC_256},
		.fb_dinst ={
			.coeff    = {COEFF_HSV_180, COEFF_HSV_256},
			.bits_per_channel = 8,
			.channels_per_pixel = 3,
			.map = {0,1,2},
			.ram_select = {2,2,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "HRGB 32bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_HRGB,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC_HSV,
		.n_encoding = 2,
		.encoding = { V4L2_HSV_ENC_180, V4L2_HSV_ENC_256},
		.fb_dinst ={
			.coeff    = {COEFF_HSV_180, COEFF_HSV_256},
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {0,0,1,2},
			.ram_select = {2,0,0,0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "YRGB 32bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_YRGB,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC_HSV,
		.fb_dinst ={
			.coeff    = COEFF_GRAY,
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {0,0,1,2},
			.ram_select = {0,1,1,1},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGRH 32bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_BGRH,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC_HSV,
		.n_encoding = 2,
		.encoding = { V4L2_HSV_ENC_180, V4L2_HSV_ENC_256},
		.fb_dinst ={
			.coeff    = {COEFF_HSV_180, COEFF_HSV_256},
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {2,1,0,0},
			.ram_select = {0,0,0,2},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},
	{
		.name     = "BGRY 32bits",
		.fourcc   = V4L2_PIX_FMT_QTEC_BGRY,
		.mbus_formats =  {MEDIA_BUS_FMT_QTEC_LEGACY_RGB, MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP},
		.depth    = 32,
		.packer_mode = FB_DYNAMIC_HSV,
		.fb_dinst ={
			.coeff    = COEFF_GRAY,
			.bits_per_channel = 8,
			.channels_per_pixel = 4,
			.map = {2,1,0,0},
			.ram_select = {1,1,1,0},
		},
		.packer_pixels_per_sensor_pixels = 1,
	},

};

static const struct qt5023_video_fmt *get_format_fourcc(struct qt5023_video *priv, uint32_t pixelformat)
{
	int i;
	uint32_t fourcc=pixelformat;

	//Simple translation for legacy apps. Will be removed
	if ((pixelformat==v4l2_fourcc('Y', '8', ' ', ' ')) ||
			(pixelformat==v4l2_fourcc('Y', '8', '0', '0')) )
		fourcc=V4L2_PIX_FMT_GREY;

	for (i = 0; i < priv->n_formats; i++)
		if (priv->formats[i].fourcc == fourcc)
			return &priv->formats[i];

	return NULL;
}

static const struct qt5023_video_fmt *get_format_mbus_format(struct qt5023_video *priv, uint32_t format)
{
	int i;

	for (i = 0; i < priv->n_formats; i++)
		if (priv->formats[i].mbus_formats[0] == format)
			return &priv->formats[i];

	return NULL;
}

/*
 * Controls
 */
static int qt5023_video_s_ctrl(struct v4l2_ctrl *ctrl){
	struct qt5023_video *priv = container_of(ctrl->handler, struct qt5023_video, ctrl_handler);
	switch (ctrl->id){
		case QTEC_VIDEO_CID_RESET_PIPELINE:
			schedule_work(&priv->reset_pipeline_wk);
			break;
		default:
			return -EINVAL;
	};

	return 0;
}

static int qt5023_video_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	struct qt5023_video *priv = container_of(ctrl->handler, struct qt5023_video, ctrl_handler);
	switch (ctrl->id) {
		case QTEC_VIDEO_CID_DROP_FRAMES:
			ctrl->val=priv->frame_drop;
			break;
		case QTEC_VIDEO_CID_ACTIVE_FRAMES:
			ctrl->val=priv->active_frames;
			break;
		case QTEC_VIDEO_CID_BUFFER_SIZE:
			ctrl->val=priv->circular_nbuff;
			break;
	}

	return 0;
}

static struct v4l2_ctrl *qt5023_video_add_custom_control_head_i2c_addr(struct qt5023_video *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = NULL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.id = QTEC_VIDEO_CID_HEAD_I2C_ADDR,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Head I2C Address",
	};

	ctrl.min=0;
	ctrl.max=256;
	ctrl.def=priv->i2c_addr_val;
	ctrl.step=1;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qt5023_video_add_custom_control_head_i2c_bus(struct qt5023_video *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = NULL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.id = QTEC_VIDEO_CID_HEAD_I2C_BUS,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Head I2C Bus",
	};

	ctrl.min=0;
	ctrl.max=256;
	ctrl.def=priv->i2c_adapter;
	ctrl.step=1;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qt5023_video_add_custom_control_bitstream(struct qt5023_video *priv){
	static struct v4l2_ctrl_config ctrl = {
		.ops = NULL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.id = QTEC_VIDEO_CID_BITSTREAM_VERSION,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Bitstream Version",
	};

	ctrl.min=0;
	ctrl.max=ctrl.def=priv->bitstream_version<0?0xffffffff:priv->bitstream_version;
	ctrl.step=1;
	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qt5023_video_add_custom_control_sensor_serial(struct qt5023_video *priv){
	struct v4l2_ctrl *c;
	static struct v4l2_ctrl_config ctrl = {
		.ops = NULL,
		.type = V4L2_CTRL_TYPE_STRING,
		.step = 1,
		.min = 1,
		.max = 32,
		.id = QTEC_VIDEO_CID_SENSOR_SERIAL,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.name = "Sensor Serial",
	};

	c=v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
	if (!c)
		return NULL;

	strncpy(c->p_cur.p_char,priv->sensor_serial,32);

	return c;
}

static const struct v4l2_ctrl_ops qt5023_video_ctrl_ops_volatile = {
	.g_volatile_ctrl = qt5023_video_g_volatile_ctrl,
};

static const struct v4l2_ctrl_ops qt5023_video_ctrl_ops_write_only = {
	.s_ctrl = qt5023_video_s_ctrl,
};

static struct v4l2_ctrl *qt5023_video_add_custom_control_volatile(struct qt5023_video *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qt5023_video_ctrl_ops_volatile,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x7fffffff,
		.step = 1,
		.def=0,
	};

	ctrl.id=id;
	ctrl.name=name;
	ctrl.flags |= V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qt5023_video_add_custom_control_button(struct qt5023_video *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qt5023_video_ctrl_ops_write_only,
		.type = V4L2_CTRL_TYPE_BUTTON,
	};

	ctrl.id=id;
	ctrl.name=name;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

/*PLL*/
static inline int qt5023_video_pll_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "pll W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->pll_iomem+offset);
	return 0;
}

static inline int qt5023_video_pll_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->pll_iomem+offset);
	dev_dbg(&priv->pdev->dev, "pll R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#define PLL_CTRL (0x1f*4)
#define PLL_CTRL7 (0xff*4)
#define PLL_RESET (BIT(31)|BIT(0))
#define PLL_LOCK 31
#define PLL_AUX_LOCK 30
static int qt5023_video_pll_stop(struct qt5023_video *priv){

	qt5023_video_pll_write(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,PLL_RESET);

	return 0;
}

static int qt5023_video_pll_start(struct qt5023_video *priv){
	unsigned long expiration;
	uint32_t aux;

	qt5023_video_pll_write(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,0);
	expiration=jiffies+HZ;
	do{
		qt5023_video_pll_read(priv,(priv->pll_7series)?PLL_CTRL7:PLL_CTRL,&aux);
		if ((aux&BIT(PLL_LOCK))&&(aux&BIT(PLL_AUX_LOCK)))
			return 0;
	}while(time_before(jiffies,expiration));

	dev_err(&priv->pdev->dev, "Timeout waiting for pll lock 0%x\n", aux);

	return -EIO;
}

/*ENCODER*/
static inline int qt5023_video_encoder_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "encoder W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->encoder_iomem+offset);
	return 0;
}

static inline int qt5023_video_encoder_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->encoder_iomem+offset);
	dev_dbg(&priv->pdev->dev, "encoder R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#define ENC_CTRL 0x0
#define ENC_EN (BIT(0)|BIT(1))
#define ENC_VALID BIT(5)
#define ENC_OVERFLOW BIT(4)
#define ENC_VAL 0x4
#define ENC_VAL_MASK 0xffff
#define ENC_VALID_COPY BIT(25)
#define ENC_OVERFLOW_COPY BIT(24)

static int qt5023_video_encoder_end(struct qt5023_video *priv){
	if (priv->encoder_iomem)
		qt5023_video_encoder_write(priv,ENC_CTRL,0);
	return 0;
}

static int qt5023_video_encoder_init(struct qt5023_video *priv){
	if (priv->encoder_iomem)
		qt5023_video_encoder_write(priv,ENC_CTRL,ENC_EN);

	return 0;
}

#define PACKER_IRQ_ALL (BIT(PACKER_IRQIMG) | BIT(PACKER_BUSERROR) | BIT(PACKER_OVERFLOW))

#define PACKERCOEFFOFFSET 0x4

#define PACKERHSVFACTOR 0x54

#define PACKERBASEADDR 0x10

#define PACKERLASTBASE 0x14

#define PACKERDMA_LENGTH 0x18

#define PACKERDMA_STEP 0x1c

#define PACKERCOEFFRGBMULT 0x20
#define PACKERCOEFFRGBMULT_OFFSET 0x20

#define PACKERTIMESTAMP 0x38

#define PACKERINST 0x100
#define INST_MASK 0
#define INST_PIXEL_ADDR 8
#define INST_REG_SHIFT 13
#define INST_FIFO_WE 17
#define INST_RAM_SELECT 18

static inline int qt5023_video_packer_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "packer W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->packer_iomem+offset);
	//dev_err(&priv->pdev->dev, "`AXI_OP(`OP_WRITE, 32'h%.8x, 32'h%.8x)",0x300e0000+offset,value);
	return 0;
}

static inline int qt5023_video_packer_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->packer_iomem+offset);
	dev_dbg(&priv->pdev->dev, "packer R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static int qt5023_video_encoder_value(struct qt5023_video *priv, uint32_t *val){
	uint32_t aux;

	if (priv->encoder_iomem){

		qt5023_video_encoder_read(priv,ENC_VAL,&aux);
		if (!(aux&ENC_VALID_COPY))
			return -1;

		*val = aux&ENC_VAL_MASK;

		return 0;
	}

	if (priv->quirk & QUIRK_TIMESTAMP)
		return qt5023_video_packer_read(priv, PACKERTIMESTAMP,val);

	*val = 0;

	return 0;
}

static int qt5023_video_packer_irq_handler_common(struct qt5023_video *priv, u32 circular_address){
	struct qt5023_frame *frame;
	unsigned long flags=0;
	uint32_t aux=0;

	frame=kmalloc(sizeof(*frame),GFP_ATOMIC);
	if (!frame){
		priv->frame_seq++;
		priv->frame_drop++;
		v4l2_err(&priv->v4l2_dev, "Out of memory for frame %ld \n",priv->frame_drop);
		return -ENOMEM;
	}
	frame->size=priv->format.fmt.pix.sizeimage;
	frame->seq=priv->frame_seq++;
	frame->circular_address=circular_address;
	frame->timestamp = ktime_get_ns();

	qt5023_video_encoder_value(priv,&aux);
	if (qt5023_video_encoder_provider)
		aux = (qt5023_video_encoder_provider)();
	memset(&frame->timecode,0,sizeof(frame->timecode));
	memcpy(frame->timecode.userbits,&aux,sizeof(aux));

	spin_lock_irqsave(&priv->slock,flags);
	//Check if there is an available spot
	if( priv->active_frames > (priv->circular_nbuff/2)){
		struct qt5023_frame *old_frame = list_first_entry_or_null(&priv->frame_list, struct qt5023_frame, list);
		if (old_frame){
			list_del(&old_frame->list);
			if (priv->active_frames)
				priv->active_frames--;
			else
				dev_warn(&priv->pdev->dev,"Unbalanced active frames\n");
			priv->frame_drop++;
			kfree(old_frame);
			dev_dbg(&priv->pdev->dev, "Frame: %ld dropped.\n",old_frame->seq);
		}
	}

	priv->active_frames++;
	list_add_tail(&frame->list, &priv->frame_list);

	spin_unlock_irqrestore(&priv->slock,flags);

	//Notify bh
	complete(&priv->thread_comp);

	return 0;
}

/*FB4 DMA */
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

static inline int qt5023_video_dma_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "dma W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->dma_iomem+offset);
	return 0;
}

static inline int qt5023_video_dma_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->dma_iomem+offset);
	dev_dbg(&priv->pdev->dev, "dma R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#define S_RXEOF 26
#define S_RXSOF 27
#define S_DMAINTERR 28
#define S_DMASLVERR 29
#define S_DMADECERR 30
#define S_CMPLT 31
#define DMA_IRQ_ALL (BIT(ERR_IRQ)|BIT(DLY_IRQ)|BIT(IOC_IRQ))
static irqreturn_t qt5023_video_dma_irq_handler(int irq, void *data){
	struct qt5023_video *priv = data;
	uint32_t aux=0;

	qt5023_video_dma_read(priv,DMASR,&aux);
	qt5023_video_dma_write(priv,DMASR,aux);

	if (unlikely((aux & DMA_IRQ_ALL)==0)){
		v4l2_err(&priv->v4l2_dev, "DMA Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(aux & (BIT(ERR_IRQ) | BIT(DLY_IRQ)))){
		uint32_t curdesc;
		qt5023_video_dma_read(priv,CURDESC,&curdesc);

		v4l2_err(&priv->v4l2_dev, "DMA: Error processing descriptor 0x%.8x (0x%8x)\n", curdesc, aux);

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
		priv->do_reset_pipeline=1;
		complete(&priv->thread_comp);
	}

	if (likely(aux & IOC_IRQ)){
		uint32_t status;
		struct qt5023_video_axi_cdma_desc __iomem *desc=priv->dma_desc_iomem;

		while ((status = ioread32(&desc[priv->dma_running_desc].status)) & BIT(S_CMPLT)){

			if (status & BIT(S_RXSOF))
				priv->dma_last_circular = ioread32(&desc[priv->dma_running_desc].source);

			if (status & BIT(S_RXEOF))
				qt5023_video_packer_irq_handler_common(priv, priv->dma_last_circular);

			if (status & BIT(S_DMAINTERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Internal Error\n",priv->dma_running_desc);
			if (status & BIT(S_DMASLVERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Slave Error\n",priv->dma_running_desc);
			if (status & BIT(S_DMADECERR))
				v4l2_err(&priv->v4l2_dev, "DMA Descriptor %.2d: Decode Error\n",priv->dma_running_desc);

			iowrite32(0,&desc[priv->dma_running_desc].status);
			qt5023_video_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+priv->dma_running_desc*sizeof(*desc));
			priv->dma_running_desc ++;
			if (priv->dma_running_desc == priv->dma_n_desc_preloaded)
				priv->dma_running_desc = 0;
		}
	}

	return IRQ_HANDLED;
}

/*FB4 PACKER*/
#define FB4CTRL 0
#define ENABLE 0
#define IRQ_OFLW_ENA 1
#define IRQ_STOP_ENA 2
#define IRQ_STOP_MODE 3
#define FB4_MODE 4
#define FB4_BYTE_SWAP 8
#define HARD_STOP 9
#define STAT_ACTIVE 16
#define STAT_EMPTY 17
#define STAT_OFLW 18
#define STAT_IRQ 19
#define VERSION 24

static irqreturn_t qt5023_video_packer_fb4_irq_handler(int irq, void *data){
	uint32_t aux, aux2;
	struct qt5023_video *priv = data;

	qt5023_video_packer_read(priv,FB4CTRL,&aux);
	qt5023_video_packer_write(priv,FB4CTRL,aux);

	if (unlikely((aux & STAT_IRQ)==0)){
		v4l2_err(&priv->v4l2_dev, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (aux & STAT_OFLW)
		v4l2_err(&priv->v4l2_dev, "FIFO Overflow 0x%.8x\n",aux);
	else
		v4l2_err(&priv->v4l2_dev, "Other IRQ 0x%.8x\n",aux);

	qt5023_video_dma_read(priv,DMACR,&aux);
	v4l2_err(&priv->v4l2_dev, "DMACR  0x%.8x\n",aux);
	qt5023_video_dma_read(priv,DMASR,&aux);
	v4l2_err(&priv->v4l2_dev, "DMASR  0x%.8x\n",aux);
	qt5023_video_dma_read(priv,CURDESC,&aux);
	v4l2_err(&priv->v4l2_dev, "CURDESC  0x%.8x\n",aux);
	qt5023_video_dma_read(priv,TAILDESC,&aux2);
	v4l2_err(&priv->v4l2_dev, "TAILDESC  0x%.8x\n",aux2);

	if (aux == aux2)
		v4l2_err(&priv->v4l2_dev, "Run out of descriptors: TAILDESC == CURDESC 0x%.8x\n",aux2);

	priv->do_reset_pipeline=1;
	complete(&priv->thread_comp);

	return IRQ_HANDLED;
}

static int qt5023_video_packer_stop_fb4(struct qt5023_video *priv, bool do_wait){
	unsigned long expiration;
	uint32_t aux;

	qt5023_video_packer_read(priv,FB4CTRL,&aux);
	aux&=~BIT(ENABLE);
	aux&=~BIT(IRQ_OFLW_ENA);
	aux&=~BIT(IRQ_STOP_ENA);
	qt5023_video_packer_write(priv,FB4CTRL,aux);
	aux |= BIT(STAT_EMPTY);
	aux |= BIT(STAT_OFLW);
	aux |= BIT(STAT_IRQ);
	qt5023_video_packer_write(priv,FB4CTRL,aux);

	if (!do_wait)
		return 0;

	expiration=jiffies+2*HZ;
	do {
		qt5023_video_packer_read(priv,FB4CTRL,&aux);
		if (!(aux & BIT(STAT_ACTIVE)))
			return 0;
	}while(time_before(expiration,jiffies)==0);

	dev_err(&priv->pdev->dev, "Timeout while stopping\n");
	return 1;
}

#define REG_OFF 0x4
#define REG_COEFF 0x40
static int qt5023_video_packer_start_fb4(struct qt5023_video *priv){
	const struct qt5023_video_fmt *fmt;
	int i,j;
	uint32_t aux;
	uint32_t last_address;
	struct qt5023_video_axi_cdma_desc __iomem *desc=priv->dma_desc_iomem;
	uint32_t imagesize_padding = (priv->format.fmt.pix.sizeimage + 0xf) &~0xf;
	int desc_per_frame = (imagesize_padding+(MAX_DESC_SIZE-1))/MAX_DESC_SIZE;
	int frame_per_n_desc = priv->dma_n_desc / desc_per_frame;
	int frame_per_circular = priv->circular_length / imagesize_padding;

	priv->circular_nbuff = min(frame_per_n_desc,frame_per_circular);

	j=0;
	last_address = priv->circular_address;
	for (i=0;i<priv->circular_nbuff;i++){
		int frame_size = priv->format.fmt.pix.sizeimage;
		//avoid CDMA Internal Error IRQ
		if (last_address & 0xf)
			last_address =  (last_address + 0x10) & ~0xf;
		while (frame_size) {
			int len= (frame_size >MAX_DESC_SIZE) ? MAX_DESC_SIZE:frame_size;
			memset_io(&desc[j],0,sizeof(*desc));
			iowrite32(priv->dma_desc_base_addr+((j+1)*sizeof(*desc)),&desc[j].next);
			iowrite32(last_address,&desc[j].source);
			iowrite32(len,&desc[j].length);
			frame_size -=len;
			last_address += len;
			dev_dbg(&priv->pdev->dev, "DMA Descriptor %.2d Addr:0x%.8x Next: 0x%.8x len:0x%x\n",j,desc[j].source,desc[j].next,desc[j].length);
			j ++;
		}
	}

	priv->dma_n_desc_preloaded =  j;
	iowrite32(priv->dma_desc_base_addr,&desc[priv->dma_n_desc_preloaded-1].next);
	dev_dbg(&priv->pdev->dev, "DMA Descriptor %.2d Addr:0x%.8x Next: 0x%.8x len:0x%x\n",priv->dma_n_desc_preloaded-1,desc[priv->dma_n_desc_preloaded-1].source,desc[priv->dma_n_desc_preloaded-1].next,desc[priv->dma_n_desc_preloaded-1].length);

	priv->dma_running_desc = 0;

	//reset dma engine
	qt5023_video_dma_write(priv,DMACR,BIT(RESET));
	while(1){
		qt5023_video_dma_read(priv,DMACR,&aux);
		if (!(aux & BIT(RESET)))
			break;
	}

	//program dma engine
	qt5023_video_dma_write(priv,CURDESC,priv->dma_desc_base_addr);
	qt5023_video_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+((priv->dma_n_desc_preloaded-1)*sizeof(*desc)));

	aux = BIT(IOC_IRQEN) | BIT(ERR_IRQEN);
	aux |= 1<< IRQTHRESHOLD;
	aux |= BIT(RS);
	qt5023_video_dma_write(priv,DMACR,aux);

	//FIXME, check if it is needed
	qt5023_video_dma_write(priv,TAILDESC,priv->dma_desc_base_addr+((priv->dma_n_desc_preloaded-1)*sizeof(*desc)));

	/*Flush fifo*/
	qt5023_video_packer_write(priv,FB4CTRL,BIT(HARD_STOP));
	qt5023_video_packer_write(priv,FB4CTRL,0);

	/*Offsets and coeffs*/
	fmt = get_format_fourcc(priv,priv->format.fmt.pix.pixelformat);
	if (!fmt) {
		v4l2_err(&priv->v4l2_dev, "Fourcc format (0x%08x) unknown.\n",
				priv->format.fmt.pix.pixelformat);
		return -EINVAL;
	}

	for (i=0;i<CHANNELS_PER_FB4;i++){
		qt5023_video_packer_write(priv,REG_OFF+i*4,0);
		for (j=0;j<CHANNELS_PER_FB4;j++)
			qt5023_video_packer_write(priv,REG_COEFF+i*16+j*4,fmt->fb4_config.map[i]==j?0x4000:0);
	}

	aux=BIT(ENABLE);
	aux|=BIT(IRQ_OFLW_ENA);
	if (fmt->little_endian)
		aux|=BIT(FB4_BYTE_SWAP);
	aux|=fmt->fb4_config.mode << FB4_MODE;
	qt5023_video_packer_write(priv,FB4CTRL,aux);

	return 0;
}

/*PACKER*/
#define PACKERRUN 0
#define PACKER_RUN 0
#define PACKER_IRQENA 2
#define PACKER_IRQIMG 3
#define PACKER_BUSERROR 4
#define PACKER_NOTIDLE 5
#define PACKER_OVERFLOW 7
#define PACKER_FRAMECNT 8
#define PACKER_FRAMECNT_LEN 4
#define BYTE_SWAP 18
#define VERSION 24

static irqreturn_t qt5023_video_packer_legacy_irq_handler(int irq, void *data){
	struct qt5023_video *priv = data;
	uint32_t aux=0;

	qt5023_video_packer_read(priv,PACKERRUN,&aux);
	qt5023_video_packer_write(priv,PACKERRUN,aux);//ACK irq asap

	if (unlikely((aux & PACKER_IRQ_ALL)==0)){
		v4l2_err(&priv->v4l2_dev, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(BIT(PACKER_BUSERROR)&aux)){
		v4l2_err(&priv->v4l2_dev, "Packer Bus Error 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(BIT(PACKER_OVERFLOW)&aux)){
		aux&=~BIT(PACKER_RUN);
		aux&=~BIT(PACKER_IRQENA);
		qt5023_video_packer_write(priv,PACKERRUN,aux);//ACK irq asap
		v4l2_err(&priv->v4l2_dev, "Overflow Error 0x%.8x\n",aux);
		priv->do_reset_pipeline=1;
		complete(&priv->thread_comp);
		return IRQ_HANDLED;
	}

	qt5023_video_packer_read(priv,PACKERLASTBASE,&aux);
	qt5023_video_packer_irq_handler_common(priv, aux);

	return IRQ_HANDLED;
}

static int qt5023_video_packer_stop_legacy(struct qt5023_video *priv, bool do_wait){
	unsigned long expiration;
	uint32_t aux;

	qt5023_video_packer_read(priv,PACKERRUN,&aux);
	aux&=~BIT(PACKER_RUN);
	aux&=~BIT(PACKER_IRQENA);
	qt5023_video_packer_write(priv,PACKERRUN,aux);
	aux |= BIT(PACKER_IRQIMG);
	aux |= BIT(PACKER_BUSERROR);
	aux |= BIT(PACKER_OVERFLOW);
	qt5023_video_packer_write(priv,PACKERRUN,aux);

	if (!do_wait)
		return 0;

	expiration=jiffies+2*HZ;
	do {
		qt5023_video_packer_read(priv,PACKERRUN,&aux);
		if (!(aux & BIT(PACKER_NOTIDLE)))
			return 0;
	}while(time_before(expiration,jiffies)==0);

	dev_err(&priv->pdev->dev, "Timeout while stopping\n");

	return 1;
}

static int qt5023_video_packer_stop(struct qt5023_video *priv, bool do_wait){
	if (priv->quirk & QUIRK_FB4)
		return qt5023_video_packer_stop_fb4(priv,do_wait);
	return qt5023_video_packer_stop_legacy(priv,do_wait);
}

#define PACKERINST_NINST 20
#define PACKER_MATRIX_NINPUT 20
static int qt5023_video_packer_start_legacy(struct qt5023_video *priv){
	uint32_t aux=0;
	const struct qt5023_video_fmt *fmt;
	int i,j;
	int n_inst,channel_out,pixel_lsb;
	int input_base;
	const struct fb_matrix_coeff *coeff;
	//avoid CDMA internal Error IRQ
	uint32_t imagesize_padding = (priv->format.fmt.pix.sizeimage + 0xf) & ~0xf;
	int enc = 0;

	fmt = get_format_fourcc(priv,priv->format.fmt.pix.pixelformat);
	if (!fmt) {
		v4l2_err(&priv->v4l2_dev, "Fourcc format (0x%08x) unknown.\n",
				priv->format.fmt.pix.pixelformat);
		return -EINVAL;
	}

	qt5023_video_packer_read(priv,PACKERRUN,&aux);
	if (aux & PACKER_IRQ_ALL)
		v4l2_err(&priv->v4l2_dev, "Pending IRQ at start: 0x%.8x\n",aux);

	//Find number of frames and clear old irqs (if any)
	priv->circular_nbuff = priv->circular_length/imagesize_padding;
	if (priv->circular_nbuff> ((1<<PACKER_FRAMECNT_LEN)-1))
		priv->circular_nbuff= (1<<PACKER_FRAMECNT_LEN)-1;
	aux &= ~(((1<<PACKER_FRAMECNT_LEN)-1)<< PACKER_FRAMECNT);
	aux |= priv->circular_nbuff<< PACKER_FRAMECNT;
	aux |= BIT(PACKER_IRQIMG);
	aux |= BIT(PACKER_BUSERROR);
	aux |= BIT(PACKER_OVERFLOW);
	qt5023_video_packer_write(priv,PACKERRUN,aux);
	aux |= BIT(PACKER_IRQENA);
	qt5023_video_packer_write(priv,PACKERRUN,aux);

	if (fmt->n_encoding){
		int i;

		for (i = 0 ; i < fmt->n_encoding ; i++)
			if (fmt->encoding[i] == priv->format.fmt.pix.ycbcr_enc)
				break;

		if (i == fmt->n_encoding)
			i = 0;

		enc = i;
	}
	/*Setup packer*/
	if (fmt->packer_mode == FB_DYNAMIC || fmt->packer_mode == FB_DYNAMIC_HSV){
		int pixels_per_framebus=1;

		coeff = &fmt->fb_dinst.coeff[enc];
		pixel_lsb=32-fmt->fb_dinst.bits_per_channel;
		n_inst=0;

		switch(fmt->mbus_formats[0]){
			case MEDIA_BUS_FMT_QTEC_LEGACY_MONO:
			case MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP:
			case MEDIA_BUS_FMT_QTEC_LEGACY_RGB:
			case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
			case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
			case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
			case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
				pixels_per_framebus=1;
				break;
			case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
			case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
			case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
			case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
			case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
				pixels_per_framebus=5;
				break;
			default:
				v4l2_err(&priv->v4l2_dev, "Invalid mbus format (0x%08x) unknown.\n",
					fmt->mbus_formats[0]);
				return -EINVAL;
		}

		for (input_base=0; input_base < PACKER_MATRIX_NINPUT ; input_base += (CHANNELS_PER_FRAMEBUS/pixels_per_framebus)){
			for (channel_out=0;channel_out<fmt->fb_dinst.channels_per_pixel;channel_out++){
				uint32_t instruction = ((1<<(fmt->fb_dinst.bits_per_channel/4))-1) << (INST_MASK + (pixel_lsb/4));

				instruction |= (input_base + fmt->fb_dinst.map[channel_out]) << INST_PIXEL_ADDR;

				instruction |= (((pixel_lsb+fmt->fb_dinst.bits_per_channel)/4)-1) << INST_REG_SHIFT; // (MSB/4)

				if (pixel_lsb==0){
					instruction |= BIT(INST_FIFO_WE);
					pixel_lsb=32-fmt->fb_dinst.bits_per_channel;
				}
				else
					pixel_lsb-=fmt->fb_dinst.bits_per_channel;

				if (fmt->packer_mode == FB_DYNAMIC_HSV)
					instruction |= fmt->fb_dinst.ram_select[channel_out] << INST_RAM_SELECT;

				qt5023_video_packer_write(priv,PACKERINST+4*n_inst,instruction);
				n_inst++;
			}
		}
	} else {
		coeff = &fmt->fb_finst.coeff[enc];
		for (n_inst=0;n_inst<fmt->fb_finst.n_inst;n_inst++)
			qt5023_video_packer_write(priv,PACKERINST+4*n_inst,fmt->fb_finst.inst[n_inst]);
	}

	//Clean unused inst
	for (;n_inst<PACKERINST_NINST;n_inst++)
		qt5023_video_packer_write(priv,PACKERINST+4*n_inst,0);

	/*Setup matrix*/
	for(i=0;i<N_COEFF;i++){
		if (priv->quirk & QUIRK_HSV)
			qt5023_video_packer_write(priv,
				PACKERHSVFACTOR + 4*i, coeff->hsv[i]);
		qt5023_video_packer_write(priv,
			PACKERCOEFFOFFSET + 4*i, coeff->offset[i]);
		for (j=0;j<N_COEFF;j++)
			qt5023_video_packer_write(priv,
				PACKERCOEFFRGBMULT+(i*PACKERCOEFFRGBMULT_OFFSET)+j*4, coeff->rgbmult[i][j]);
	}

	//Setup dma
	qt5023_video_packer_write(priv,PACKERBASEADDR,priv->circular_address);
	qt5023_video_packer_write(priv,PACKERDMA_LENGTH,priv->format.fmt.pix.sizeimage);
	qt5023_video_packer_write(priv,PACKERDMA_STEP,imagesize_padding);

	if (fmt->little_endian)
		aux |= BIT(BYTE_SWAP);
	else
		aux &= ~BIT(BYTE_SWAP);

	//Start core
	aux|=BIT(PACKER_RUN);
	qt5023_video_packer_write(priv,PACKERRUN,aux);

	return 0;
}

static int qt5023_video_packer_start(struct qt5023_video *priv){
	if (priv->quirk & QUIRK_FB4)
		return qt5023_video_packer_start_fb4(priv);

	return qt5023_video_packer_start_legacy(priv);
}

/*CDMA*/
#define CDMACR 0x0
#define CDMA_TAILPTREN 1
#define CDMA_RESET 2
#define CDMA_SGMODE 3
#define CDMA_ERRIRQEN 14
#define CDMA_IOCIRQEN 12
#define CDMA_IRQTHRESHOLD 16
#define CDMA_IRQTHRESHOLD_LEN 8

#define CDMASR 0x4
#define CDMA_IDLE 1
#define CDMA_IRQDMAINTERR  4
#define CDMA_IRQSLVERR  5
#define CDMA_IRQDECERR  6
#define CDMA_IRQSGINTERR  8
#define CDMA_IRQSGSLVERR 9
#define CDMA_IRQSGDECERR 10
#define CDMA_IRQIOC 12
#define CDMA_IRQERR 14

#define CDMACURDESC 0x8

#define CDMATAILDESC 0x10

static inline int qt5023_video_cdma_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "cdma W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->cdma_iomem+offset);
	return 0;
}

static inline int qt5023_video_cdma_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->cdma_iomem+offset);
	dev_dbg(&priv->pdev->dev, "cdma R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static int qt5023_video_cdma_init(struct qt5023_video *priv){
	uint32_t aux=BIT(CDMA_RESET);
	unsigned long timeout = jiffies + HZ;

	qt5023_video_cdma_write(priv,CDMACR,aux);

	while (BIT(CDMA_RESET)&aux){
		if (time_before(timeout,jiffies)){
			dev_err(&priv->pdev->dev, "Timeout while reseting cdma 0x%x\n",aux);
			return -1;
		}
		qt5023_video_cdma_read(priv,CDMACR,&aux);
	}

	return 0;
}

#define CDMA_IRQ_ALL (BIT(CDMA_IRQERR)|BIT(CDMA_IRQIOC))
static irqreturn_t qt5023_video_cdma_irq_handler(int irq, void *data){
	struct qt5023_video *priv=data;
	uint32_t aux=0;
	int res=VB2_BUF_STATE_DONE;

	qt5023_video_cdma_read(priv,CDMASR,&aux);
	qt5023_video_cdma_write(priv,CDMASR,aux); //ACK irq ASAP
	if (unlikely((aux & CDMA_IRQ_ALL)==0)){
		v4l2_err(&priv->v4l2_dev, "CDMA Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (unlikely(BIT(CDMA_IRQERR)&aux)){
		uint32_t curdesc;
		qt5023_video_cdma_read(priv,CDMACURDESC,&curdesc);
		v4l2_err(&priv->v4l2_dev, "Frame %ld: Error processing descriptor 0x%.8x (0x%.8x)\n",priv->cdma_current_buf->frame->seq,curdesc,aux);

		if (BIT(CDMA_IRQDMAINTERR)&aux)
			v4l2_err(&priv->v4l2_dev, "DMA Internal Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSGDECERR)&aux)
			v4l2_err(&priv->v4l2_dev, "Scatter Gather Decode Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSGSLVERR)&aux)
			v4l2_err(&priv->v4l2_dev, "Scatter Gather Slave Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSGINTERR)&aux)
			v4l2_err(&priv->v4l2_dev, "Scatter Gather Internal Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQDECERR)&aux)
			v4l2_err(&priv->v4l2_dev, "Decode Error IRQ 0x%.8x\n",aux);
		if (BIT(CDMA_IRQSLVERR)&aux)
			v4l2_err(&priv->v4l2_dev, "Slave Error IRQ 0x%.8x\n",aux);

		//Reset core
		qt5023_video_cdma_init(priv);

		res=VB2_BUF_STATE_ERROR;
	}

	//Clear irq and save status
	vb2_buffer_done(&priv->cdma_current_buf->vb2_buf.vb2_buf, res);
	priv->cdma_current_buf=NULL;

	priv->cdma_ready = true;
	complete(&priv->thread_comp);
	return IRQ_HANDLED;
}

#define APERTURE_OFFSET 0x208
static inline int qt5023_video_aperture_write(struct qt5023_video *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "aperture W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->aperture_iomem+offset+APERTURE_OFFSET);
	return 0;
}

static inline int qt5023_video_aperture_read(struct qt5023_video *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->aperture_iomem+offset+APERTURE_OFFSET);
	dev_dbg(&priv->pdev->dev, "aperture R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static int qt5023_cdma_frame(struct qt5023_video *priv, struct qt5023_video_buf *buf){
	struct qt5023_video_axi_cdma_desc __iomem *desc=priv->cdma_desc_iomem;
	int ndesc;
	int ap;
	unsigned long last_address,length;
	uint32_t aux;
	struct vb2_v4l2_buffer *vbuf = &buf->vb2_buf;

	//Fill struct
	vb2_set_plane_payload(&buf->vb2_buf.vb2_buf, 0, buf->frame->size);
	vbuf->sequence = buf->frame->seq;
	vbuf->vb2_buf.timestamp = buf->frame->timestamp;
	vbuf->field = V4L2_FIELD_NONE;
	vbuf->flags = V4L2_BUF_FLAG_TIMECODE;
	vbuf->timecode = buf->frame->timecode;

	//gain lock

	priv->cdma_current_buf=buf;

	//config pcie bridge
	for (ap=0;ap<buf->n_ap;ap++){
		qt5023_video_aperture_write(priv,(priv->apertures[ap].index*APERTURE_REG_LEN)+4,buf->ap_addr[ap]&0xffffffff);
		qt5023_video_aperture_write(priv,(priv->apertures[ap].index*APERTURE_REG_LEN)+0,(buf->ap_addr[ap]>>32)&0xffffffff);
	}

	//check if dma is idle
	qt5023_video_cdma_read(priv,CDMASR,&aux);
	if (!(BIT(CDMA_IDLE)&aux)){
		v4l2_err(&priv->v4l2_dev, "CMDA is not IDLE. ERROR? 0x%.8x\n",aux);
		return -1;
	}

	//setup desc
	last_address=buf->frame->circular_address;
	length=buf->frame->size;
	for(ndesc=0;(ndesc<buf->n_desc)&&(length>0);ndesc++){
		unsigned long desc_len=buf->compact_desc[ndesc].length;
		if (desc_len>length)
			desc_len=length;
		iowrite32(priv->cdma_desc_base_addr+((ndesc+1)*sizeof(*desc)),&desc[ndesc].next);
		iowrite32(last_address,&desc[ndesc].source);
		iowrite32(buf->compact_desc[ndesc].addr,&desc[ndesc].dest);
		iowrite32(desc_len,&desc[ndesc].length);
		iowrite32(0,&desc[ndesc].status);
		length-=desc_len;
		last_address+=desc_len;
		dev_dbg(&priv->pdev->dev, "%d: 0x%.8x 0x%.8x 0x%.8x 0x%.8x 0x%.8x\n",
				ndesc,desc[ndesc].next,desc[ndesc].source,desc[ndesc].dest,desc[ndesc].length,desc[ndesc].status);
	}

	//circular descriptors
	if (ndesc)
		iowrite32(priv->cdma_desc_base_addr,&desc[ndesc-1].next);

	if (length>0){
		v4l2_err(&priv->v4l2_dev, "Frame bigger than buffer size (%ld bytes too big)\n",length);
		return -1;
	}

	//Clear SG (Otherwise curdesc is ignored)
	aux= ndesc << CDMA_IRQTHRESHOLD;
	aux|=BIT(CDMA_ERRIRQEN)|BIT(CDMA_IOCIRQEN)|BIT(CDMA_TAILPTREN)|BIT(CDMA_SGMODE);
	qt5023_video_cdma_write(priv,CDMACR,aux);

	qt5023_video_cdma_write(priv,CDMACURDESC,priv->cdma_desc_base_addr);

	//Launch dma
	qt5023_video_cdma_write(priv,CDMATAILDESC,priv->cdma_desc_base_addr+(ndesc-1)*sizeof(*desc));
	priv->cdma_ready = false;
	return 0;
}

/*
 * Kthread
 */
static int qt5023_video_thread(void *data){
	struct qt5023_video *priv = data;
	unsigned long flags=0;
	struct qt5023_video_buf *video_buf;
	struct qt5023_frame *frame;
	struct sched_param param = { .sched_priority = 2 };

	//Attempt to reduce the frames jitter/delay
	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()){
#ifdef INTERVAL_TEST
		s64 interval;
		s64 interval_diff;
		s64 timestamp;
#endif

		wait_for_completion_interruptible(&priv->thread_comp);
		//Check if we should stop
		if(unlikely(kthread_should_stop())||priv->must_stop)
			break;

		//Check for overflow errors
		if (priv->do_reset_pipeline){
			v4l2_err(&priv->v4l2_dev, "Not Restarting system\n");
			v4l2_subdev_call(priv->sd_xform,video,s_stream,0);
			v4l2_subdev_call(priv->sd_sensor,video,s_stream,0);
			v4l2_subdev_call(priv->sd_testgen,video,s_stream,0);
			qt5023_video_packer_stop(priv,false);
			qt5023_video_pll_stop(priv);
			priv->frame_seq=0;
			priv->frame_drop=0;
			qt5023_video_pll_start(priv);
			qt5023_video_packer_start(priv);
			v4l2_subdev_call(priv->sd_xform,video,s_stream,1);
			v4l2_subdev_call(priv->sd_testgen,video,s_stream,1);
			v4l2_subdev_call(priv->sd_sensor,video,s_stream,1);
			priv->do_reset_pipeline=0;
		}

		//Update leds
		led_trigger_event(priv->led_img, (priv->frame_seq&1)?LED_FULL:LED_OFF);
		led_trigger_event(priv->led_drop, (priv->frame_drop&1)?LED_FULL:LED_OFF);

		//Check if there are buffers ready from camera and from user
		if (list_empty(&priv->buffer_list)||list_empty(&priv->frame_list)|| !priv->cdma_ready)
			continue;

		//Get user buffer and camera frame
		spin_lock_irqsave(&priv->slock,flags);
		if (list_empty(&priv->buffer_list)||list_empty(&priv->frame_list)){
			spin_unlock_irqrestore(&priv->slock,flags);
			continue;
		}
		video_buf = list_first_entry_or_null(&priv->buffer_list, struct qt5023_video_buf, list);
		frame = list_first_entry_or_null(&priv->frame_list, struct qt5023_frame, list);
		list_del(&video_buf->list);
		list_del(&frame->list);
		if (priv->active_frames)
			priv->active_frames--;
		else
			dev_warn(&priv->pdev->dev,"Unbalanced active frames\n");
		spin_unlock_irqrestore(&priv->slock,flags);

#ifdef INTERVAL_TEST
		//interval test
		timestamp = timeval_to_ns(&frame->timestamp);
		interval = timestamp - priv->last_timestamp;
		interval_diff = interval - priv->last_interval;
		if (interval_diff < 0)
			interval_diff = -interval_diff;
		if ((interval_diff>1000000) && (frame->seq>1))
			dev_err(&priv->pdev->dev, "Frame: %ld Interval diff %lld nsec\n",frame->seq,interval_diff);
		priv->last_interval = interval;
		priv->last_timestamp = timestamp;
#endif

		//cdma frame
		video_buf->frame=frame;
		qt5023_cdma_frame(priv,video_buf);
	}

	led_trigger_event(priv->led_img, LED_OFF);
	led_trigger_event(priv->led_drop, LED_OFF);

	return 0;
}

static int qt5023_video_update_xform_fmt(struct qt5023_video *priv){
	const struct qt5023_video_fmt *fmt;
	struct v4l2_format *f = &priv->format;
	int ret;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	ret= v4l2_subdev_call(priv->sd_xform,pad,get_fmt,NULL,&format);
	if (ret==-ENODEV)
		return 0;
	if (ret)
		return ret;

	fmt = get_format_fourcc(priv,f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	f->fmt.pix.width = format.format.width * fmt->packer_pixels_per_sensor_pixels;
	f->fmt.pix.height = format.format.height;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

/*
 * VB2 Buffer
 */
static int qt5023_video_vb2_queue_setup(struct vb2_queue *vq,
		unsigned int *nbufs,
		unsigned int *num_planes, unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct qt5023_video *priv = vb2_get_drv_priv(vq);

	qt5023_video_update_xform_fmt(priv);
	sizes[0] = priv->format.fmt.pix.sizeimage;

	if (!sizes[0]){
		dev_err(&priv->pdev->dev, "Invalid size for the format\n");
		return -EINVAL;
	}

	if (*nbufs < 1){
		dev_err(&priv->pdev->dev, "Invalid number of buffers %d\n",*nbufs);
		return -EINVAL;
	}

	*num_planes = 1;

	return 0;
}

static int qt5023_video_vb2_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qt5023_video_buf *buf = container_of(vvb, struct qt5023_video_buf, vb2_buf);
	struct qt5023_video *priv = vb2_get_drv_priv(vb->vb2_queue);
	struct sg_table *sg_table = vb2_dma_sg_plane_desc(vb, 0);
	struct scatterlist *real_sg;
	int i;
	unsigned int last_desc=-1;
	unsigned long last_address=-1;
	void *next_compact_desc;
	struct compact_desc *compact_desc=NULL;

	if (!sg_table){
		v4l2_err(&priv->v4l2_dev, "Missing sg_table\n");
		return -EIO;
	}

	//Find minimun number of dma descriptors compatible with cdma
	buf->compact_desc=NULL;
	for_each_sg(sg_table->sgl, real_sg, sg_table->nents, i) {
		unsigned long dma_len;
		unsigned long dma_addr=sg_dma_address(real_sg);
		unsigned long unalloc_len=sg_dma_len(real_sg);
		while(unalloc_len){
			dma_len=unalloc_len;

			//Split if crosses apertures
			if ((dma_addr&(~(priv->apertures[0].length-1))) != ((dma_addr+dma_len)&(~(priv->apertures[0].length-1))) )
				dma_len=priv->apertures[0].length-(dma_addr&(priv->apertures[0].length-1));

			//Add to latest descriptor if possible
			if (dma_addr==last_address){
				unsigned long extra_size;
				extra_size=priv->apertures[0].length-(compact_desc[last_desc].addr&(priv->apertures[0].length-1));
				if (extra_size>(MAX_DESC_SIZE-compact_desc[last_desc].length))
					extra_size=MAX_DESC_SIZE-compact_desc[last_desc].length;

				if (extra_size> dma_len)
					extra_size= dma_len;

				compact_desc[last_desc].length+=extra_size;
				last_address += extra_size;
				unalloc_len -= extra_size;
				dma_len -= extra_size;
				dma_addr += extra_size;
			}

			if (dma_len){
				//New desc
				if (dma_len>MAX_DESC_SIZE)
					dma_len=MAX_DESC_SIZE;
				last_desc++;
				next_compact_desc=krealloc(compact_desc,(last_desc+1)*sizeof(compact_desc[0]),GFP_KERNEL);
				if (!next_compact_desc){
					v4l2_err(&priv->v4l2_dev, "Unable to allocate compact descriptors\n");
					kfree(compact_desc);
					return -ENOMEM;
				}
				compact_desc=next_compact_desc;
				compact_desc[last_desc].length=dma_len;
				compact_desc[last_desc].addr=dma_addr;
				last_address=dma_len+dma_addr;
			}
			dma_addr+=dma_len;
			unalloc_len-=dma_len;
		}
	}
	buf->n_desc=last_desc+1;

#ifdef BANDWITH_TEST
	v4l2_err(&priv->v4l2_dev, "This is a Bandwith x %d test. DO NOT USE IN PRODUCTION!!!!\n", BANDWITH_TEST);
	next_compact_desc=krealloc(compact_desc,buf->n_desc*BANDWITH_TEST*sizeof(compact_desc[0]),GFP_KERNEL);
	if (!next_compact_desc){
		v4l2_err(&priv->v4l2_dev, "Unable to allocate compact descriptors\n");
		kfree(compact_desc);
		return -ENOMEM;
	}
	compact_desc=next_compact_desc;
	for (i=1;i<BANDWITH_TEST;i++){
		int  j;
		for (j=0;j<buf->n_desc;j++)
			compact_desc[i*buf->n_desc+j] = compact_desc[j];
	}
	buf->n_desc *= BANDWITH_TEST;
#endif

	if (buf->n_desc>priv->cdma_n_desc){
		v4l2_err(&priv->v4l2_dev, "Too many cdma descriptors are requiered (%d>%d) \n",buf->n_desc,priv->cdma_n_desc);
		kfree(compact_desc);
		return -EIO;
	}

	//Setup apertures
	buf->n_ap=0;
	for(i=0;i<buf->n_desc;i++){
		//check cross apertures
		int ap;
		unsigned long base_addr=compact_desc[i].addr&(~(priv->apertures[0].length-1));
		dev_dbg(&priv->pdev->dev, "Descriptor %d Addr:0x%.10lx  len:0x%lx\n",i,compact_desc[i].addr,compact_desc[i].length);

		for(ap=0;ap<buf->n_ap;ap++)
			if (base_addr==buf->ap_addr[ap])
				break;

		if (ap==buf->n_ap){
			if (ap==priv->n_apertures){
				v4l2_err(&priv->v4l2_dev, "Not enough apertures for video buffers...\n");
				kfree(compact_desc);
				return -EIO;
			}

			//Create aperture
			buf->ap_addr[ap]=base_addr;
			buf->n_ap++;
		}

		compact_desc[i].addr-=base_addr;
		compact_desc[i].addr+=priv->apertures[ap].fpga_addr;
	}

	buf->compact_desc=compact_desc;

	return 0;
}

static int qt5023_video_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qt5023_video_buf *buf = container_of(vvb, struct qt5023_video_buf, vb2_buf);
	buf->frame=NULL;

	return 0;
}

static void qt5023_video_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qt5023_video_buf *video_buf = container_of(vvb, struct qt5023_video_buf, vb2_buf);
	struct qt5023_video *priv = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags;

	//Add buffer
	spin_lock_irqsave(&priv->slock,flags);
	list_add_tail(&video_buf->list, &priv->buffer_list);
	spin_unlock_irqrestore(&priv->slock,flags);

	//Notify kthread of new usr buffer
	complete(&priv->thread_comp);

	return;
}

static void qt5023_video_vb2_buf_finish(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qt5023_video_buf *buf = container_of(vvb, struct qt5023_video_buf, vb2_buf);

	if (buf->frame)
		kfree(buf->frame);

	return;
}

static void qt5023_video_vb2_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qt5023_video_buf *buf = container_of(vvb, struct qt5023_video_buf, vb2_buf);

	if (buf->compact_desc)
		kfree(buf->compact_desc);

	return;
}

static int qt5023_video_kill_thread(struct qt5023_video *priv){
	unsigned long flags=0;
	struct qt5023_video_buf *video_buf;
	priv->must_stop=1;
	complete(&priv->thread_comp);
	if (IS_ERR(priv->kthread))
		v4l2_err(&priv->v4l2_dev, "Capture thread was not initiated. Continue cleaning\n");
	else
		kthread_stop(priv->kthread);

	priv->kthread = ERR_PTR(-EINVAL);

	//Remove pending frames
	spin_lock_irqsave(&priv->slock,flags);

	while(!list_empty(&priv->frame_list)){
		struct qt5023_frame *frame = list_first_entry_or_null(&priv->frame_list, struct qt5023_frame, list);
		if (!frame)
			break;
		list_del(&frame->list);
		kfree(frame);
	}
	while(!list_empty(&priv->buffer_list)){
		video_buf = list_first_entry_or_null(&priv->buffer_list, struct qt5023_video_buf, list);
		if (!video_buf)
			continue;
		list_del(&video_buf->list);
		if (video_buf->vb2_buf.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&video_buf->vb2_buf.vb2_buf, VB2_BUF_STATE_QUEUED);
		else
			v4l2_err(&priv->v4l2_dev, "Ignoring inactive video buf\n");
	}
	spin_unlock_irqrestore(&priv->slock,flags);

	qt5023_video_cdma_init(priv);
	if (priv->cdma_current_buf){
		if (priv->cdma_current_buf->vb2_buf.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&priv->cdma_current_buf->vb2_buf.vb2_buf, VB2_BUF_STATE_QUEUED);
		else
			v4l2_err(&priv->v4l2_dev, "Ignoring Inactive current buf\n");
	}
	return 0;
}

static int qt5023_video_vb2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct qt5023_video *priv = vb2_get_drv_priv(vq);
	int ret;

	if (!vq->num_buffers){
		v4l2_err(&priv->v4l2_dev, "Starting stream with no buffers. Abort\n");
		return -EINVAL;
	}

	qt5023_video_update_xform_fmt(priv);

	priv->active_frames=0;
	priv->frame_seq=0;
	priv->frame_drop=0;
	priv->do_reset_pipeline=0;
	priv->must_stop=0;
	priv->cdma_ready = true;

	priv->kthread=kthread_run(qt5023_video_thread,priv,priv->v4l2_dev.name);
	if (IS_ERR(priv->kthread)){
		v4l2_err(&priv->v4l2_dev, "Error starting capture thread, aborting\n");
		qt5023_video_kill_thread(priv);
		return PTR_ERR(priv->kthread);
	}
retry:

	ret=qt5023_video_pll_start(priv);
	if (ret){
		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start PLL\n");
		return ret;
	}

	ret = qt5023_video_encoder_init(priv);
	if (ret){
		qt5023_video_pll_stop(priv);
		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start encoder\n");
		return ret;
	}

	ret = qt5023_video_packer_start(priv);
	if (ret){
		qt5023_video_encoder_end(priv);
		qt5023_video_pll_stop(priv);
		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start packer\n");
	}

	ret=v4l2_subdev_call(priv->sd_xform,video,s_stream,1);
	if ((ret!=0)&&(ret!=-ENODEV)){
		qt5023_video_packer_stop(priv,true);
		qt5023_video_encoder_end(priv);
		qt5023_video_pll_stop(priv);
		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start xform\n");
		return ret;
	}

	ret=v4l2_subdev_call(priv->sd_testgen,video,s_stream,1);
	if ((ret!=0)&&(ret!=-ENODEV)){
		v4l2_subdev_call(priv->sd_xform,video,s_stream,0);
		qt5023_video_packer_stop(priv,true);
		qt5023_video_encoder_end(priv);
		qt5023_video_pll_stop(priv);
		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start testgen\n");
		return ret;
	}

	ret=v4l2_subdev_call(priv->sd_sensor,video,s_stream,1);
	if (ret){
		v4l2_subdev_call(priv->sd_testgen,video,s_stream,0);
		v4l2_subdev_call(priv->sd_xform,video,s_stream,0);
		qt5023_video_packer_stop(priv,true);
		qt5023_video_encoder_end(priv);
		qt5023_video_pll_stop(priv);
		if (ret == -EAGAIN){
			v4l2_info(&priv->v4l2_dev, "Sensor requested pipeline reset. Retrying\n");
			goto retry;
		}

		qt5023_video_kill_thread(priv);
		v4l2_err(&priv->v4l2_dev, "Unable to start sensor\n");
		return ret;
	}

	return 0;
}

static int qt5023_wait_for_frames(struct qt5023_video *priv){
	unsigned long expiration = jiffies +HZ;

	do{
		if (!atomic_read(&priv->vb2_vidq.owned_by_drv_count))
			return 0;
	}while(time_before(jiffies,expiration));

	dev_err(&priv->pdev->dev, "Timeout waiting for %d frames owned by the driver\n", atomic_read(&priv->vb2_vidq.owned_by_drv_count) );

	return -EIO;

}

static void qt5023_video_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct qt5023_video *priv = vb2_get_drv_priv(vq);

	qt5023_wait_for_frames(priv);
	qt5023_video_packer_stop(priv,true);
	qt5023_video_kill_thread(priv);
	v4l2_subdev_call(priv->sd_xform,video,s_stream,0);
	v4l2_subdev_call(priv->sd_sensor,video,s_stream,0);
	v4l2_subdev_call(priv->sd_testgen,video,s_stream,0);
	qt5023_video_cdma_init(priv);
	qt5023_video_encoder_end(priv);
	qt5023_video_pll_stop(priv);

	return;
}

static const struct vb2_ops qt5023_video_vb2_ops = {
	.queue_setup		= qt5023_video_vb2_queue_setup,
	.buf_init		= qt5023_video_vb2_buf_init,
	.buf_prepare		= qt5023_video_vb2_buf_prepare,
	.buf_queue		= qt5023_video_vb2_buf_queue,
	.buf_finish		= qt5023_video_vb2_buf_finish,
	.buf_cleanup		= qt5023_video_vb2_buf_cleanup,
	.start_streaming	= qt5023_video_vb2_start_streaming,
	.stop_streaming		= qt5023_video_vb2_stop_streaming,
};

/*
 * IOCTLS
 */
static int qt5023_video_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	sprintf(inp->name, "Input %u", inp->index);
	return 0;
}

static int qt5023_video_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int qt5023_video_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i>0)
		return -EINVAL;
	return 0;
}

static int qt5023_video_try_fmt_priv(struct qt5023_video *priv,
			struct v4l2_format *f, bool check_xform);
static int qt5023_video_enum_frameintervals(struct file *file, void *pr,
					     struct v4l2_frmivalenum *fival)
{
	struct qt5023_video *priv=video_drvdata(file);
	const struct qt5023_video_fmt *fmt;
	struct v4l2_format f;
	int ret;

	fmt = get_format_fourcc(priv,fival->pixel_format);
	if (!fmt)
		return -EINVAL;

	f.fmt.pix.pixelformat=fival->pixel_format;
	f.fmt.pix.width=fival->width;
	f.fmt.pix.height=fival->height;
	qt5023_video_try_fmt_priv(priv,&f,false);

	if (f.fmt.pix.pixelformat!=fival->pixel_format ||
		f.fmt.pix.width!=fival->width ||
		f.fmt.pix.height!=fival->height)
		return -EINVAL; //Compliance test

	fival->width /= fmt->packer_pixels_per_sensor_pixels;

	fival->pixel_format=fmt->mbus_formats[0];
	ret=v4l2_subdev_call(priv->sd_sensor,video,enum_frameintervals,fival);
	fival->width *= fmt->packer_pixels_per_sensor_pixels;
	fival->pixel_format=f.fmt.pix.pixelformat;

	return ret;
}

static int qt5023_video_g_parm(struct file *file, void *pr,
			  struct v4l2_streamparm *parm)
{
	struct v4l2_subdev_frame_interval fival;
	struct qt5023_video *priv=video_drvdata(file);
	int ret;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret=v4l2_subdev_call(priv->sd_sensor,video,g_frame_interval,&fival);
	if (ret)
		return ret;

	parm->parm.capture.capability   = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe = fival.interval;
	parm->parm.capture.readbuffers  = 2;

	return 0;
}

#define FRACT_CMP(a, OP, b)	\
	((u64)(a).numerator * (b).denominator  OP  (u64)(b).numerator * (a).denominator)

static int qt5023_video_s_parm(struct file *file, void *pr,
			  struct v4l2_streamparm *parm)
{
	struct qt5023_video *priv=video_drvdata(file);
	struct v4l2_subdev_frame_interval fival;
	int ret;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fival.interval=parm->parm.capture.timeperframe;
	ret=v4l2_subdev_call(priv->sd_sensor,video,s_frame_interval,&fival);
	if (ret)
		return ret;

	ret=v4l2_subdev_call(priv->sd_sensor,video,g_frame_interval,&fival);
	if (ret)
		return ret;
	parm->parm.capture.capability   = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe = fival.interval;
	parm->parm.capture.readbuffers  = 2;

	return 0;
}

static int qt5023_video_enum_fmt_vid_cap(struct file *file, void  *pr,
					struct v4l2_fmtdesc *f)
{
	struct qt5023_video *priv=video_drvdata(file);
	const struct qt5023_video_fmt *fmt;

	if (f->index >= priv->n_formats)
		return -EINVAL;

	fmt = &priv->formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int qt5023_video_try_fmt_priv(struct qt5023_video *priv,
			struct v4l2_format *f, bool check_xform)
{
	const struct qt5023_video_fmt *fmt;
	int ret;
	int mult_size;
	int last_width;
	bool did_undeflow=false;
	int old_code;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	struct v4l2_subdev_pad_config cfg;
	u32 encoding;

	fmt = get_format_fourcc(priv,f->fmt.pix.pixelformat);
	if (!fmt) {
		v4l2_err(&priv->v4l2_dev, "Fourcc format (0x%08x) %c%c%c%c unknown.\n",
			f->fmt.pix.pixelformat,
			(f->fmt.pix.pixelformat>>0)&0xff,
			(f->fmt.pix.pixelformat>>8)&0xff,
			(f->fmt.pix.pixelformat>>16)&0xff,
			(f->fmt.pix.pixelformat>>24)&0xff
			);
		f->fmt.pix.pixelformat = priv->formats[0].fourcc;
		fmt = get_format_fourcc(priv,f->fmt.pix.pixelformat);
	}

	switch(fmt->mbus_formats[0]){
		case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
			mult_size=1;
			break;
		case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			mult_size=20;
			break;
		default:
			mult_size=4;
			break;
	}

	format.format.width = f->fmt.pix.width / fmt->packer_pixels_per_sensor_pixels ;
	format.format.width -= format.format.width%mult_size;
	if (format.format.width<mult_size)
		format.format.width=mult_size;
	format.format.height = f->fmt.pix.height;
	format.format.code = old_code = fmt->mbus_formats[0];
	encoding = f->fmt.pix.ycbcr_enc;

	last_width=format.format.width;
	//Find a width that satisfies the sensor and the packer
	while (1){
		ret = v4l2_subdev_call(priv->sd_sensor,pad, set_fmt, &cfg, &format);
		if (ret)
			return ret;
		format.format = cfg.try_fmt;
		if ((format.format.width % mult_size)==0) //We found it
			break;
		if (last_width == format.format.width){
			//Sensor cannot come with a better suggestion....
			if (did_undeflow)
				return -EINVAL;
			format.format.width=0xffff;
			did_undeflow=true;
		}
		format.format.width -= format.format.width%mult_size;
		if (format.format.width<mult_size){
			//No more image left
			if (did_undeflow)
				return -EINVAL;
			format.format.width=0xffff;
			did_undeflow=true;
		}
		last_width=format.format.width;
	}

	if (check_xform){
		ret = v4l2_subdev_call(priv->sd_xform,pad, set_fmt, NULL, &format);
		format.format = cfg.try_fmt;
	}

	//Maybe the sensor has changed the format (ie. cmv12k bayer)
	if (old_code != format.format.code)
		fmt = get_format_mbus_format(priv,format.format.code);

	memset(f,0,sizeof(*f));//Compliance test

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix.width = format.format.width * fmt->packer_pixels_per_sensor_pixels;
	f->fmt.pix.height = format.format.height;
	f->fmt.pix.pixelformat = fmt->fourcc;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	if ((f->fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)||(f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV))
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	else
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	f->fmt.pix.priv = 0;
	f->fmt.pix.ycbcr_enc = 0;

	if (fmt->n_encoding) {
		int i;

		for (i = 0 ; i < fmt->n_encoding ; i++)
			if (fmt->encoding[i] == encoding)
				break;

		if (i == fmt->n_encoding)
			i = 0;

		f->fmt.pix.ycbcr_enc = fmt->encoding[i];
	}


	return 0;
}

static int qt5023_video_try_fmt_vid_cap(struct file *file, void *pr,
			struct v4l2_format *f)
{
	struct qt5023_video *priv=video_drvdata(file);

	return qt5023_video_try_fmt_priv(priv,f,true);
}

static int qt5023_video_g_fmt_vid_cap(struct file *file, void *pr,
					struct v4l2_format *f)
{
	struct qt5023_video *priv=video_drvdata(file);

	qt5023_video_update_xform_fmt(priv);
	*f=priv->format;

	return  0;
}

static int qt5023_video_enum_framesizes(struct file *file, void *fh,
					 struct v4l2_frmsizeenum *fsize)
{
	struct qt5023_video *priv=video_drvdata(file);
	const struct qt5023_video_fmt *fmt;
	int ret;
	int min_size;
	struct v4l2_subdev_frame_size_enum fse;

	fmt = get_format_fourcc(priv,fsize->pixel_format);
	if (!fmt)
		return -EINVAL;
	fse.index = fsize->index;
	fse.code = fmt->mbus_formats[0];
	fse.which = V4L2_SUBDEV_FORMAT_TRY;
	ret = v4l2_subdev_call(priv->sd_sensor,pad,enum_frame_size,NULL,&fse);
	if (ret)
		return ret;

	fsize->stepwise.min_width = fse.min_width * fmt->packer_pixels_per_sensor_pixels;
	fsize->stepwise.max_width = fse.max_width * fmt->packer_pixels_per_sensor_pixels;
	fsize->stepwise.min_height = fse.min_height;
	fsize->stepwise.max_height = fse.max_height;

	switch(fmt->mbus_formats[0]){
		case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			min_size=20;
			break;
		case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
			min_size=1;
			break;
		default:
			min_size=4;
			break;
	}

	fsize->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	fsize->stepwise.step_width= 1;
	fsize->stepwise.step_height= 1;
	fsize->stepwise.min_width = lcm(fsize->stepwise.min_width,min_size);
	fsize->stepwise.max_width -= fsize->stepwise.max_width%min_size;
	if (fsize->stepwise.min_height==0)
		fsize->stepwise.min_height=1;

	memset(fsize->reserved,0,sizeof(fsize->reserved));

	return 0;
}

static int qt5023_video_s_fmt_priv(struct qt5023_video *priv, struct v4l2_format *f){
	struct vb2_queue *q = &priv->vb2_vidq;
	const struct qt5023_video_fmt *fmt;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	if (vb2_is_busy(q)) {
		v4l2_err(&priv->v4l2_dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	ret = qt5023_video_try_fmt_priv(priv, f,false);
	if (ret < 0)
		return ret;

	fmt = get_format_fourcc(priv,f->fmt.pix.pixelformat);
	format.format.code = fmt->mbus_formats[0];
	format.format.width = f->fmt.pix.width / fmt->packer_pixels_per_sensor_pixels;
	format.format.height = f->fmt.pix.height;
	ret= v4l2_subdev_call(priv->sd_sensor, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;
	ret= v4l2_subdev_call(priv->sd_white, pad, set_fmt, NULL, &format);
	if ((ret!=0)&&(ret!=-ENODEV))
		return ret;
	ret= v4l2_subdev_call(priv->sd_testgen, pad, set_fmt, NULL, &format);
	if ((ret!=0)&&(ret!=-ENODEV))
		return ret;
	ret= v4l2_subdev_call(priv->sd_xform, pad, set_fmt, NULL, &format);
	if ((ret!=0)&&(ret!=-ENODEV))
		return ret;

	priv->format = *f;

	qt5023_video_update_xform_fmt(priv);
	*f=priv->format;

	return 0;
}

static int qt5023_video_s_fmt_vid_cap(struct file *file, void *pr,
					struct v4l2_format *f)
{
	struct qt5023_video *priv=video_drvdata(file);

	return qt5023_video_s_fmt_priv(priv,f);
}

static int qt5023_video_vidioc_querycap(struct file *file, void *pr,
		struct v4l2_capability *cap)
{
	struct qt5023_video *priv=video_drvdata(file);
	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, DRIVER_NAME);
	snprintf(cap->bus_info, sizeof(cap->bus_info),"platform:%s",dev_name(&priv->pdev->dev));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING ;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static inline void selection_multiply(struct v4l2_selection *sel, unsigned int num){
	int i;

	if (!sel->rectangles){
		sel->r.width *= num;
		sel->r.left *= num;
		return;
	}

	for (i=0;i<sel->rectangles;i++){
		sel->pr[i].r.width *=num;
		sel->pr[i].r.left *=num;
	}

	return;
}

static inline void selection_divide(struct v4l2_selection *sel, unsigned int num){
	int i;

	if (!sel->rectangles){
		sel->r.width /= num;
		sel->r.left /= num;
		return;
	}

	for (i=0;i<sel->rectangles;i++){
		sel->pr[i].r.width /=num;
		sel->pr[i].r.left /=num;
	}

	return;
}

static void qt5023_video_try_selection(struct qt5023_video *priv, struct v4l2_selection *sel){
	struct v4l2_format f;

	if (sel->rectangles)
		return;

	f.fmt.pix.pixelformat = priv->format.fmt.pix.pixelformat;
	f.fmt.pix.width=sel->r.width;
	f.fmt.pix.height=sel->r.height;

	qt5023_video_try_fmt_priv(priv, &f, true);

	sel->r.width = f.fmt.pix.width;
	sel->r.height = f.fmt.pix.height;

	return;
}

static int qt5023_video_g_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
	struct qt5023_video *priv=video_drvdata(file);
	int ret;
	struct v4l2_subdev_selection s;
	const struct qt5023_video_fmt *fmt = get_format_fourcc(priv,priv->format.fmt.pix.pixelformat);

	if (!fmt)
		return -EINVAL;

	s.rectangles=sel->rectangles;
	s.target=sel->target;
	s.pr=sel->pr;
	ret=v4l2_subdev_call(priv->sd_sensor,pad,get_selection,NULL,&s);
	if (ret)
		return ret;

	sel->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sel->target=s.target;
	sel->pr=s.pr;
	sel->r=s.r;
	sel->rectangles=s.rectangles;
	selection_multiply(sel,fmt->packer_pixels_per_sensor_pixels);

	//Make sure you can use the default selections T#625
	if ((s.target == V4L2_SEL_TGT_COMPOSE_DEFAULT) ||
		(s.target == V4L2_SEL_TGT_CROP_DEFAULT))
		qt5023_video_try_selection(priv, sel);

	if (!sel->rectangles)
		memset(sel->reserved,0,sizeof(sel->reserved));
	return 0;
}

static int qt5023_video_s_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
	struct qt5023_video *priv=video_drvdata(file);
	struct v4l2_subdev_selection s;
	int ret;
	const struct qt5023_video_fmt *fmt = get_format_fourcc(priv,priv->format.fmt.pix.pixelformat);

	if (!fmt)
		return -EINVAL;

	if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	selection_divide(sel,fmt->packer_pixels_per_sensor_pixels);


	s.target=sel->target;
	s.r=sel->r;
	s.pr=sel->pr;
	s.rectangles=sel->rectangles;

	ret=v4l2_subdev_call(priv->sd_sensor,pad,set_selection,NULL,&s);
	v4l2_subdev_call(priv->sd_sensor,pad,get_selection,NULL,&s);

	sel->target = s.target;
	sel->r = s.r;
	sel->rectangles = s.rectangles;

	selection_multiply(sel,fmt->packer_pixels_per_sensor_pixels);

	if (!sel->rectangles)
		memset(sel->reserved,0,sizeof(sel->reserved));
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qt5023_video_s_register(struct file *file, void *fh,
					const struct v4l2_dbg_register *reg){
	struct qt5023_video *priv=video_drvdata(file);
	uint32_t val=reg->val;

	if (reg->reg >= 0x7000)
		iowrite32(val,priv->dma_desc_iomem+(reg->reg-0x7000));
	else if (reg->reg >= 0x6000)
		qt5023_video_dma_write(priv, reg->reg-0x6000, val);
	else if (reg->reg >=0x5000)
		iowrite32(val,priv->cdma_desc_iomem+(reg->reg-0x5000));
	else if (reg->reg >=0x4000)
		qt5023_video_encoder_write(priv, reg->reg-0x4000, val);
	else if (reg->reg>= 0x3000)
		qt5023_video_pll_write(priv, reg->reg-0x3000, val);
	else if (reg->reg >= 0x2000)
		qt5023_video_aperture_write(priv, reg->reg-0x2000, val);
	else if (reg->reg >= 0x1000)
		qt5023_video_cdma_write(priv, reg->reg-0x1000, val);
	else
		qt5023_video_packer_write(priv, reg->reg, val);

	return 0;
}

static int qt5023_video_g_register(struct file *file, void *fh,
					struct v4l2_dbg_register *reg){
	struct qt5023_video *priv=video_drvdata(file);
	uint32_t val;

	if (reg->reg >=0x7000)
		val=ioread32(priv->dma_desc_iomem+(reg->reg-0x7000));
	else if (reg->reg >=0x6000)
		qt5023_video_dma_read(priv, reg->reg-0x6000, &val);
	else if (reg->reg >=0x5000)
		val=ioread32(priv->cdma_desc_iomem+(reg->reg-0x5000));
	else if (reg->reg >=0x4000)
		qt5023_video_encoder_read(priv, reg->reg-0x4000, &val);
	else if (reg->reg>= 0x3000)
		qt5023_video_pll_read(priv, reg->reg-0x3000, &val);
	else if (reg->reg >=0x2000)
		qt5023_video_aperture_read(priv, reg->reg-0x2000, &val);
	else if (reg->reg >=0x1000)
		qt5023_video_cdma_read(priv, reg->reg-0x1000, &val);
	else
		qt5023_video_packer_read(priv, reg->reg, &val);

	reg->val=val;
	reg->size=4;
	return 0;
}
#endif

static const struct v4l2_ioctl_ops qt5023_video_ioctl_ops = {
	/*Driver specific*/
	.vidioc_querycap	= qt5023_video_vidioc_querycap,
	.vidioc_enum_frameintervals = qt5023_video_enum_frameintervals,
	.vidioc_g_parm        = qt5023_video_g_parm,
	.vidioc_s_parm        = qt5023_video_s_parm,
	.vidioc_enum_fmt_vid_cap  = qt5023_video_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = qt5023_video_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = qt5023_video_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = qt5023_video_s_fmt_vid_cap,
	.vidioc_enum_framesizes   = qt5023_video_enum_framesizes,
	.vidioc_enum_input         = qt5023_video_enum_input,
	.vidioc_g_input         = qt5023_video_g_input,
	.vidioc_s_input         = qt5023_video_s_input,
	.vidioc_g_selection	= qt5023_video_g_selection,
	.vidioc_s_selection	= qt5023_video_s_selection,

	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register      = qt5023_video_g_register,
	.vidioc_s_register      = qt5023_video_s_register,
	#endif

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
static const struct v4l2_file_operations qt5023_video_v4l_fops = {
	.owner = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static struct video_device qt5023_video_v4l_template = {
	.name = DRIVER_NAME,

	.fops = &qt5023_video_v4l_fops,
	.ioctl_ops = &qt5023_video_ioctl_ops,
	.release = video_device_release_empty,
};

/*
 * Probe/Remove
 */
static int match_of_node(struct device *dev, void *data)
{
	if (dev->of_node==data)
		return 1;
	return 0;
}

static  struct v4l2_subdev* qt5023_video_probe_subdevices_phandle(struct qt5023_video *priv, struct device_node *node){
	struct device *dev;
	struct v4l2_subdev *sd;
	int ret;
	const char *compatible = NULL;

	ret=of_property_read_string(node,"compatible",&compatible);
	if (ret)
		v4l2_err(&priv->v4l2_dev,"Unable to find compatible param for %s\n", node->name);
	else
		request_module("of:N*T*C%s",compatible); //Find with modinfo.

	dev=device_find_child(priv->pdev->dev.parent,node, match_of_node);
	if (!dev){
		v4l2_err(&priv->v4l2_dev,"Unable to find device\n");
		return NULL;
	}

	if (!dev->driver){
		v4l2_err(&priv->v4l2_dev,"Unable to get submodule driver\n");
		put_device(dev);
		return NULL;
	}
	put_device(dev);

	sd=dev_get_drvdata(dev);
	if (!sd){
		v4l2_err(&priv->v4l2_dev,"Unable to find drvdata\n");
		return NULL;
	}

	ret=v4l2_device_register_subdev(&priv->v4l2_dev, sd);
	if (ret){
		v4l2_err(&priv->v4l2_dev,"Unable to register subdev\n");
		return NULL;
	}

	return sd;
}


static int qt5023_default_format(struct qt5023_video *priv){
	struct v4l2_format f;
	struct v4l2_subdev_selection s;
	int ret;

	f.fmt.pix.pixelformat=priv->formats[0].fourcc;
	f.fmt.pix.width=0xffff;
	f.fmt.pix.height=0xffff;
	qt5023_video_s_fmt_priv(priv,&f);

	s.target=V4L2_SEL_TGT_COMPOSE_DEFAULT;
	ret=v4l2_subdev_call(priv->sd_sensor,pad,get_selection,NULL,&s);
	if (ret)
		return ret;

	f.fmt.pix.width=s.r.width;
	f.fmt.pix.height=s.r.height;
	ret=qt5023_video_s_fmt_priv(priv,&f);
	if (ret)
		return ret;

	s.target=V4L2_SEL_TGT_CROP_ACTIVE;
	ret=v4l2_subdev_call(priv->sd_sensor,pad,set_selection,NULL,&s);

	return ret;
}

static int qt5023_video_init_quirks(struct qt5023_video *priv){
	uint32_t version;

	qt5023_video_packer_read(priv,PACKERRUN,&version);
	version >>= VERSION;
	version &= 0xff;

	priv->version = version;

	switch (version){
	case 0x00:
		priv->quirk = 0;
		break;
	case 0x11:
		priv->quirk = QUIRK_BYTESWAP;
		priv->quirk |= QUIRK_TIMESTAMP;
		break;
	case 0x12:
		priv->quirk = QUIRK_BYTESWAP;
		priv->quirk |= QUIRK_TIMESTAMP;
		priv->quirk |= QUIRK_HSV;
		break;
	case 0x40:
		priv->quirk = QUIRK_BYTESWAP;
		priv->quirk |= QUIRK_TIMESTAMP;
		priv->quirk |= QUIRK_FB4;
		break;
	default:
		dev_err(&priv->pdev->dev, "Unknown core version 0x%.2x. Aborting\n",version);
		return -1;
	}

	return 0;
}

static int qt5023_video_dynamic_formats(struct qt5023_video *priv){
	int n_formats=0;
	int i;

	for(i=0;i<ARRAY_SIZE(def_formats);i++){
		struct qt5023_video_fmt *formats;
		int j;

		if (def_formats[i].little_endian && !(priv->quirk & QUIRK_BYTESWAP))
			continue;

		if (def_formats[i].packer_mode == FB_DYNAMIC_HSV && !(priv->quirk & QUIRK_HSV))
			continue;

		for(j=0;j<ARRAY_SIZE(def_formats[i].mbus_formats);j++){
			int ret;
			struct v4l2_subdev_mbus_code_enum code = {
				.which = V4L2_SUBDEV_FORMAT_ACTIVE,
				.index = 0,
			};

			if (def_formats[i].mbus_formats[j]==0)
				continue;
			while((ret=v4l2_subdev_call(priv->sd_sensor,pad,enum_mbus_code,NULL,&code))==0){
				if (code.code == def_formats[i].mbus_formats[j])
					break;
				code.index ++;
			}
			//Found match
			if (ret==0){
				formats=krealloc(priv->formats,(n_formats+1)*sizeof(priv->formats[0]),GFP_KERNEL);
				if (!formats)
					return -ENOMEM;
				priv->formats=formats;
				memcpy(&priv->formats[n_formats],&def_formats[i],sizeof(def_formats[i]));
				priv->formats[n_formats].mbus_formats[0]=def_formats[i].mbus_formats[j];
				n_formats++;
				break;
			}

		}
	}

	if(n_formats == 0){
		v4l2_err(&priv->v4l2_dev, "No formats is supported, unknown sensor\n");
		return -EINVAL;
	}

	v4l2_info(&priv->v4l2_dev, "%d formats supported\n",n_formats);
	priv->n_formats=n_formats;
	priv->timeperframe=tpf_default;
	return 0;
}

static void qt5023_video_reset_pipeline(struct work_struct *work){
	struct qt5023_video *priv=container_of(work, struct qt5023_video , reset_pipeline_wk);
	priv->do_reset_pipeline=1;
	complete(&priv->thread_comp);

}

static int qt5023_video_init_i2c_subdevices(struct qt5023_video *priv)
{
	struct v4l2_subdev *sd;
	struct i2c_adapter *i2c_adap;
	struct i2c_board_info i2c_info = {};
	static const unsigned short normal_i2c[] = { 0x74, I2C_CLIENT_END };
	char *m43_name = "qtec_m43";

	i2c_adap = i2c_get_adapter(0);//FIXME
	if (!i2c_adap){
		v4l2_err(&priv->v4l2_dev,"Unable to get i2c adapter\n");
		return -1;
	}
	strlcpy(i2c_info.type, m43_name, I2C_NAME_SIZE);
	priv->i2c_m43 = i2c_new_probed_device(i2c_adap, &i2c_info,
						   normal_i2c, NULL);
	i2c_put_adapter(i2c_adap);

	if (!priv->i2c_m43){
		v4l2_warn(&priv->v4l2_dev,"Unable to probe m43 controller. Continuing without it\n");
		return -1;
	}
	request_module("i2c:%s",m43_name);

	sd = i2c_get_clientdata(priv->i2c_m43);
	if (!sd){
		v4l2_err(&priv->v4l2_dev,"Unable to find m43 drvdata\n");
		goto err;
	}

	if (v4l2_device_register_subdev(&priv->v4l2_dev, sd)){
		v4l2_err(&priv->v4l2_dev,"Unable to register subdev\n");
		goto err;
	}

	priv->sd_m43 = sd;

	return 0;
err:
	i2c_unregister_device(priv->i2c_m43);
	priv->i2c_m43 = NULL;
	return -1;
}

static int qt5023_video_remove(struct platform_device *pdev);
static void qt5023_video_init_subdevices(struct work_struct *work){
	struct device_node *node;
	int ret;
	struct qt5023_video *priv=container_of(work, struct qt5023_video , init_subdev_wk);

	//Make sure pll is started
	qt5023_video_pll_start(priv);

	qt5023_video_init_i2c_subdevices(priv);

	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,sensor",0);
	if (!node){
		v4l2_err(&priv->v4l2_dev, "Unable to parse sensor phandle\n");
		goto submodules_err;
	}

	priv->sd_sensor=qt5023_video_probe_subdevices_phandle(priv,node);
	if (!priv->sd_sensor){
		v4l2_err(&priv->v4l2_dev, "Unable to init sensor submodule\n");
		goto submodules_err;
	}
	of_node_put(node);

	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,white_balance",0);
	if (node){
		priv->sd_white=qt5023_video_probe_subdevices_phandle(priv,node);
		if (!priv->sd_white){
			v4l2_err(&priv->v4l2_dev, "Unable to init white submodule\n");
			goto submodules_err;
		}
		of_node_put(node);
	} else
		v4l2_warn(&priv->v4l2_dev, "WARNING: Unable to parse white balance phandle, continuing without it\n");


	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,xform",0);
	if (node){
		priv->sd_xform=qt5023_video_probe_subdevices_phandle(priv,node);
		if (!priv->sd_xform){
			v4l2_err(&priv->v4l2_dev, "Unable to init xform submodule\n");
			goto submodules_err;
		}
		of_node_put(node);
	}
	else
		v4l2_warn(&priv->v4l2_dev, "WARNING: Unable to parse xform phandle, continuing without it\n");

	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,testgen",0);
	if (node){
		priv->sd_testgen=qt5023_video_probe_subdevices_phandle(priv,node);
		if (!priv->sd_testgen){
			v4l2_err(&priv->v4l2_dev, "Unable to init testgen submodule\n");
			goto submodules_err;
		}
		of_node_put(node);
	}

	node=of_parse_phandle(priv->pdev->dev.of_node,"qtec,encoder",0);
	if (node){
		priv->sd_encoder=qt5023_video_probe_subdevices_phandle(priv,node);
		if (!priv->sd_encoder){
			v4l2_err(&priv->v4l2_dev, "Unable to init encoder submodule\n");
			goto submodules_err;
		}
		of_node_put(node);
	}

	ret=v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
	if (ret){
		v4l2_err(&priv->v4l2_dev, "Unable to register subdev modules device\n");
		goto submodules_err;
	}

	ret=qt5023_video_dynamic_formats(priv);
	if (ret){
		v4l2_err(&priv->v4l2_dev, "Unable to obtain the dynamic list of formats\n");
		goto submodules_err;
	}

	//Set default format
	qt5023_default_format(priv);

	/*Save some power*/
	qt5023_video_pll_stop(priv);

	/*
	 * video dev
	 */
	mutex_init(&priv->vdev_mutex);
	priv->vdev=qt5023_video_v4l_template;
	priv->vdev.lock=&priv->vdev_mutex;
	priv->vdev.v4l2_dev=&priv->v4l2_dev;
	priv->vdev.queue=&priv->vb2_vidq;
	video_set_drvdata(&priv->vdev,priv);

	ret = video_register_device(&priv->vdev, VFL_TYPE_GRABBER, -1);
	if (ret){
		v4l2_err(&priv->v4l2_dev, "Unable to register video device\n");
		vb2_queue_release(&priv->vb2_vidq);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister(&priv->v4l2_dev);
		return;
	}

	v4l2_info(&priv->v4l2_dev, "qt5023_video V4L2 device version 0x%.2x registered as %s\n", priv->version, video_device_node_name(&priv->vdev));

	return;

submodules_err:
	/*Save some power*/
	qt5023_video_pll_stop(priv);
	qt5023_video_remove(priv->pdev);
	platform_set_drvdata(priv->pdev,NULL);
	return;
}

static void qt5023_video_load_subdevices_async(struct qt5023_video *priv)
{
	schedule_work(&priv->init_subdev_wk);
}

static int qt5032_parse_apertures(struct qt5023_video *priv, struct device_node *node){
	int n_ap;
	int max_aperture=MAX_APERTURES-1;
	int min_aperture=0;
	uint32_t apertures_val[MAX_APERTURES*2];
	int ret;

	ret=of_property_read_u32_array(priv->pdev->dev.of_node,"qtec,aperture-addr",apertures_val,2);
	if (ret==0){
		min_aperture=apertures_val[0];
		max_aperture=apertures_val[0]+apertures_val[1]-1;
		dev_info(&priv->pdev->dev, "Using apertures %d-%d\n",min_aperture,max_aperture);
	}

	if (of_property_read_u32_array(node,"qtec,apertures",apertures_val,(max_aperture+1)*2)!=0){
		dev_err(&priv->pdev->dev, "Unable to get aperture geometry\n");
		return -EIO;
	}

	priv->n_apertures = max_aperture-min_aperture+1;
	for (n_ap=0;n_ap<priv->n_apertures;n_ap++){
		priv->apertures[n_ap].fpga_addr=apertures_val[(min_aperture+n_ap)*2];
		priv->apertures[n_ap].length=apertures_val[((min_aperture+n_ap)*2)+1];
		priv->apertures[n_ap].index=min_aperture+n_ap;
		dev_dbg(&priv->pdev->dev, "Apertures %d 0x%x 0x%lx \n",priv->apertures[n_ap].index,priv->apertures[n_ap].fpga_addr,priv->apertures[n_ap].length);
	}
	return 0;
}

static atomic_t qt5023_video_instance = ATOMIC_INIT(0);
static int qt5023_video_probe(struct platform_device *pdev){
	struct qt5023_video *priv;
	struct resource res;
	int ret;
	struct device_node *node;
	const char *pll_name;
	u32 u32_arr[2];

	priv=(struct qt5023_video *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;
	INIT_WORK(&priv->init_subdev_wk, qt5023_video_init_subdevices);
	INIT_WORK(&priv->reset_pipeline_wk, qt5023_video_reset_pipeline);

	/*
	 * Device tree parsing
	 */
	/*packer*/
	ret=of_address_to_resource(pdev->dev.of_node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get packer address\n");
		return ret;
	}

	priv->packer_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->packer_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap packer\n");
		return PTR_ERR(priv->packer_iomem);
	}

	/*Init version quirks ASAP */
	ret=qt5023_video_init_quirks(priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to init quirks\n");
		return -EIO;
	}


	ret=of_irq_to_resource(pdev->dev.of_node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get packer irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,(priv->quirk & QUIRK_FB4)?qt5023_video_packer_fb4_irq_handler:qt5023_video_packer_legacy_irq_handler,
			0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request packer irq\n");
		return ret;
	}
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

		ret=devm_request_irq(&pdev->dev,res.start,qt5023_video_dma_irq_handler,0,DRIVER_NAME,priv);
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

		priv->dma_n_desc=resource_size(&res)/sizeof(struct qt5023_video_axi_cdma_desc);
		if (priv->dma_n_desc>((1<<CDMA_IRQTHRESHOLD_LEN)-1))
			priv->dma_n_desc=(1<<CDMA_IRQTHRESHOLD_LEN)-1;
		ret=of_property_read_u32(node,"reg-axi",&priv->dma_desc_base_addr);
		if (ret){
			dev_err(&pdev->dev, "Unable to get dma descriptors base address\n");
			return -EIO;
		}
		of_node_put(node);
	}

	/*cdma*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,cdma",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get cdma phandle\n");
		return -EIO;
	}
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma address\n");
		return ret;
	}

	priv->cdma_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->cdma_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap cdma\n");
		return PTR_ERR(priv->cdma_iomem);
	}

	ret=of_irq_to_resource(node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get cdma irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,qt5023_video_cdma_irq_handler,0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request cdma irq\n");
		return ret;
	}
	of_node_put(node);

	/*descriptors*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,desc_mem",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get cdma descriptor node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptor address\n");
		return ret;
	}

	priv->cdma_desc_iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->cdma_desc_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap descriptors\n");
		return PTR_ERR(priv->cdma_desc_iomem);
	}

	priv->cdma_n_desc=resource_size(&res)/sizeof(struct qt5023_video_axi_cdma_desc);
	if (priv->cdma_n_desc>((1<<CDMA_IRQTHRESHOLD_LEN)-1))
		priv->cdma_n_desc=(1<<CDMA_IRQTHRESHOLD_LEN)-1;
	ret=of_property_read_u32(node,"reg-axi",&priv->cdma_desc_base_addr);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptors base address\n");
		return -EIO;
	}
	of_node_put(node);

	/*video mem*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,circular_buffer",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get cdma descriptor node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptor address\n");
		return ret;
	}
	ret=of_property_read_u32_array(node,"reg-axi",u32_arr,2);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma descriptors base address\n");
		return -EIO;
	}
	priv->circular_address = u32_arr[0];
	priv->circular_length=u32_arr[1];
	of_node_put(node);

	/*aperture*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,pcie_bridge",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get pcie bridge node\n");
		return -EIO;
	}
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get aperture address\n");
		return ret;
	}

	//Dont request or qtec_pcie wont load
	priv->aperture_iomem  = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (IS_ERR(priv->aperture_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap pci bridge\n");
		return PTR_ERR(priv->aperture_iomem);
	}

	ret=qt5032_parse_apertures(priv,node);
	if (ret)
		return ret;

	of_node_put(node);

	/*sensor_pll*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,pll",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get pll node\n");
		return -EIO;
	}
	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get pll address\n");
		return ret;
	}

	//Dont request or sensors wont load
	priv->pll_iomem  = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (IS_ERR(priv->pll_iomem)){
		dev_err(&pdev->dev, "Unable to ioremap pll\n");
		return PTR_ERR(priv->pll_iomem);
	}

	ret=of_property_read_string(node, "qtec,pll_type",&pll_name);
	if (!ret && !strcmp(pll_name, "PLLE2_ADV"))
		priv->pll_7series = true;
	else
		priv->pll_7series = false;

	of_node_put(node);

	/*encoder*/
	node=of_parse_phandle(pdev->dev.of_node,"qtec,encoder",0);
	if (!node){
		dev_info(&pdev->dev, "Encoder not found, continuing...\n");
		priv->encoder_iomem  = NULL;
	}
	else{
		ret=of_address_to_resource(node,0,&res);
		if (ret){
			dev_err(&pdev->dev, "Unable to get pll address\n");
			return ret;
		}

		//Dont request or sensors wont load
		priv->encoder_iomem  = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
		if (IS_ERR(priv->encoder_iomem)){
			dev_err(&pdev->dev, "Unable to ioremap encoder\n");
			return PTR_ERR(priv->encoder_iomem);
		}

		of_node_put(node);
	}

	ret = of_property_read_string(pdev->dev.of_node,"qtec,serial_number",&priv->sensor_serial);
	if (ret){
		dev_err(&pdev->dev, "Unable to get sensor serial number\n");
		return -EIO;
	}

	ret = of_property_read_u32(pdev->dev.of_node,"qtec,bitstream_version",&priv->bitstream_version);
	if (ret){
		dev_err(&pdev->dev, "Unable to get bitstream_version\n");
		return -EIO;
	}

	ret = of_property_read_u32(pdev->dev.of_node,"qtec,head_i2c_address",&priv->i2c_addr_val);
	if (ret){
		dev_err(&pdev->dev, "Unable to get Sensor i2c addr\n");
		return -EIO;
	}

	node=of_parse_phandle(pdev->dev.of_node,"qtec,head_i2c_bus",0);
	if (!node){
		priv->i2c_adapter = 0;
	} else {
		ret = i2c_get_adapter_id_of_node(node);
		if (ret<0){
			dev_err(&pdev->dev, "Unable to find i2c bus address\n");
			priv->i2c_adapter = 0;
		} else
			priv->i2c_adapter = ret;

		of_node_put(node);
	}

	priv->n_formats=0;
	priv->formats=NULL;

	//Check if the device is dma capable
	//If not, it is very likely that we are using swiommu. Abort!
	//(Check if the sw-iommu fix is there)
	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(64))){
		dev_err(&pdev->dev, "Unable to set dma mask 64 bits, trying 32\n");
		if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))){
			dev_err(&pdev->dev, "Unable to set dma mask 32 bits. Abort\n");
			return -EIO;
		}
	}

	/*
	 * v4l2_dev
	 */

	priv->num=v4l2_device_set_name(&priv->v4l2_dev, DRIVER_NAME, &qt5023_video_instance);

	ret=v4l2_device_register(&pdev->dev,&priv->v4l2_dev);
	if (ret){
		dev_err(&pdev->dev, "Unable to register v4l2 device\n");
		return ret;
	}

	/*
	 * Controls
	 */
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 64); // 64 is n_controls guess
	priv->drop_frames=qt5023_video_add_custom_control_volatile(priv,"Dropped Frames",QTEC_VIDEO_CID_DROP_FRAMES);
	priv->active_frames_ctrl=qt5023_video_add_custom_control_volatile(priv,"Waiting Frames",QTEC_VIDEO_CID_ACTIVE_FRAMES);
	priv->buffer_size=qt5023_video_add_custom_control_volatile(priv,"Max Frame Queue",QTEC_VIDEO_CID_BUFFER_SIZE);
	priv->bitstream_ctrl=qt5023_video_add_custom_control_bitstream(priv);
	priv->sensor_serial_ctrl=qt5023_video_add_custom_control_sensor_serial(priv);
	priv->head_i2c_addr=qt5023_video_add_custom_control_head_i2c_addr(priv);
	priv->head_i2c_bus=qt5023_video_add_custom_control_head_i2c_bus(priv);
	priv->reset_pipeline=qt5023_video_add_custom_control_button(priv,"Reset Pipeline",QTEC_VIDEO_CID_RESET_PIPELINE);
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		v4l2_device_unregister(&priv->v4l2_dev);
		return priv->ctrl_handler.error;
	}
	priv->v4l2_dev.ctrl_handler=&priv->ctrl_handler;

	/*
	 * VB2 Queue
	 */
	priv->vb2_vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	priv->vb2_vidq.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	priv->vb2_vidq.dev = &pdev->dev;
	priv->vb2_vidq.drv_priv = priv;
	priv->vb2_vidq.buf_struct_size = sizeof (struct qt5023_video_buf);
	priv->vb2_vidq.mem_ops = &vb2_dma_sg_memops;
	priv->vb2_vidq.ops = &qt5023_video_vb2_ops;
	priv->vb2_vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(&priv->vb2_vidq);
	if (ret){
		dev_err(&pdev->dev, "Unable to init vb2 queue\n");
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister(&priv->v4l2_dev);
		return ret;
	}

	/*Work thread*/
	spin_lock_init(&priv->slock);
	init_completion(&priv->thread_comp);
	INIT_LIST_HEAD(&priv->buffer_list);
	INIT_LIST_HEAD(&priv->frame_list);

	/*cdma */
	if (qt5023_video_cdma_init(priv)){
		dev_err(&pdev->dev, "Unable to init cdma\n");
		vb2_queue_release(&priv->vb2_vidq);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister(&priv->v4l2_dev);
		return -EIO;
	}

	/*
	 * video dev
	 */
	mutex_init(&priv->vdev_mutex);
	priv->vdev=qt5023_video_v4l_template;
	priv->vdev.lock=&priv->vdev_mutex;
	priv->vdev.v4l2_dev=&priv->v4l2_dev;
	priv->vdev.queue=&priv->vb2_vidq;
	video_set_drvdata(&priv->vdev,priv);

	/*Led triggers */
	snprintf(priv->led_img_name,LEDNAME,"packer-%s",priv->v4l2_dev.name);
	led_trigger_register_simple(priv->led_img_name,&priv->led_img); //Cannot return err
	snprintf(priv->led_drop_name,LEDNAME,"drop-%s",priv->v4l2_dev.name);
	led_trigger_register_simple(priv->led_drop_name,&priv->led_drop); //Cannot return err
	led_trigger_event(priv->led_img, LED_OFF);
	led_trigger_event(priv->led_drop, LED_OFF);


	/*Subdevices*/
	qt5023_video_load_subdevices_async(priv);

	qt5023_v4l2_device = priv->vdev.v4l2_dev;

	return 0;
}

static int qt5023_video_remove(struct platform_device *pdev){
	struct qt5023_video *priv=platform_get_drvdata(pdev);

	if (priv==NULL){
		dev_err(&pdev->dev, "Already removed \n");
		return 0;
	}

	if(priv->formats){
		kfree(priv->formats);
		priv->formats=NULL;
	}

	led_trigger_unregister_simple(priv->led_img);
	led_trigger_unregister_simple(priv->led_drop);
	video_unregister_device(&priv->vdev);
	vb2_queue_release(&priv->vb2_vidq);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_device_unregister(&priv->v4l2_dev);
	if (priv->i2c_m43)
		i2c_unregister_device(priv->i2c_m43);
	platform_set_drvdata(pdev,NULL);

	qt5023_v4l2_device = NULL;
	return 0;
}

static struct of_device_id qt5023_video_of_match[] = {
	{ .compatible = "qtec,axi_matrix_packer-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qt5023_video_of_match);

static struct platform_driver qt5023_video_plat_driver = {
	.probe		= qt5023_video_probe,
	.remove		= qt5023_video_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qt5023_video_of_match,
	},
};

module_platform_driver(qt5023_video_plat_driver);

MODULE_DESCRIPTION("Qt5023 v4l2 module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
