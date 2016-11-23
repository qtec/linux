/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013-2014 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-vmalloc.h>

#define DRIVER_NAME "qtec_testgen"

struct qtec_testgen_pixel{
	uint8_t chan[5];
};

struct qtec_testgen_buf{
	struct vb2_v4l2_buffer vb2_buf; //Must be first in struct
	struct list_head list;
	uint32_t offset;
};

enum {LAST=0, CURRENT=1, NEXT=2};
enum {STOP=0, WAITING_FG, RUNNING, CLEAN};

#define LEDNAME 32
struct qtec_testgen{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct v4l2_ctrl_handler ctrl_handler;

	struct task_struct *kthread;
	struct completion thread_comp;
	struct mutex stop_mutex;

	int state;

	//vdev
	struct v4l2_device v4l2_dev;

	struct vb2_queue vb2_vidq;
	struct video_device	   vdev;
	struct mutex vdev_mutex;
	unsigned long usr_size;

	struct list_head buffer_list;
	struct qtec_testgen_buf *buffers_running[3];

	/*io*/
	void __iomem *iomem;
	struct platform_device *pdev;

	/*Controls*/
	struct v4l2_ctrl	   *missed_frames;

	/*Led triggers*/
	struct led_trigger *led_img;
	char led_img_name[LEDNAME];
	struct led_trigger *led_drop;
	char led_drop_name[LEDNAME];

	struct led_trigger *led_trig;
	char    led_name[LEDNAME];

	unsigned int missed_buffers;
	unsigned int n_buffers;

	bool fg_is_streaming;

	/*Test mem*/
	void __iomem *test_mem;
	uint32_t test_mem_base_addr;
	unsigned long test_mem_len;
	unsigned long test_mem_next_offset;

	struct v4l2_mbus_framefmt format_testgen;
	struct v4l2_mbus_framefmt format_fg;
};

static const struct qtec_testgen_compact_table {
	u32 mbus_code;
	uint32_t vdev_code;
} compact_table[]={
	{
		.mbus_code=MEDIA_BUS_FMT_QTEC_COMPACT_MONO,
		.vdev_code=V4L2_PIX_FMT_GREY,
	},
	{
		.mbus_code=MEDIA_BUS_FMT_QTEC_COMPACT_RGGB,
		.vdev_code=V4L2_PIX_FMT_SRGGB8,
	},
	{
		.mbus_code=MEDIA_BUS_FMT_QTEC_COMPACT_GBRG,
		.vdev_code=V4L2_PIX_FMT_SGBRG8,
	},
	{
		.mbus_code=MEDIA_BUS_FMT_QTEC_COMPACT_GRBG,
		.vdev_code=V4L2_PIX_FMT_SGRBG8,
	},
	{
		.mbus_code=MEDIA_BUS_FMT_QTEC_COMPACT_BGGR,
		.vdev_code=V4L2_PIX_FMT_SBGGR8,
	},
};

#define BASEADDR 0x0

#define LENGTH 0x4

#define H_PARAMS 0x8
#define V_PARAMS 0xc
#define ACTIVE 0
#define TOTAL 16

#define FRAMES 0x10
#define MAX_FRAMES 255

#define CTRL 0x14
#define IRQ_STATUS 15
#define IRQENA 3
#define RUN 0
#define STAT_RUN 14

static inline int qtec_testgen_write(struct qtec_testgen *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "testgen W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem+offset);
	return 0;
}

static inline int qtec_testgen_read(struct qtec_testgen *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "testgen R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static irqreturn_t qtec_testgen_irq_handler(int irq, void *data){
	struct qtec_testgen *priv=data;
	uint32_t aux;

	qtec_testgen_read(priv,CTRL,&aux);
	qtec_testgen_write(priv,CTRL,aux);//ACK irq asap

	if (unlikely(priv->state!=RUNNING))
		return IRQ_HANDLED;

	if (unlikely((aux & BIT(IRQ_STATUS))==0)){
		v4l2_err(&priv->sd, "Spureous IRQ 0x%.8x\n",aux);
		return IRQ_HANDLED;
	}

	if (priv->buffers_running[LAST]){
		struct vb2_v4l2_buffer *vbuf =
			to_vb2_v4l2_buffer(&priv->buffers_running[LAST]->vb2_buf.vb2_buf);
		vbuf->vb2_buf.timestamp = ktime_get_ns();
		vbuf->sequence=priv->n_buffers;
		vbuf->field=V4L2_FIELD_NONE;
		vb2_buffer_done(&priv->buffers_running[LAST]->vb2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	}
	else
		priv->missed_buffers++;

	priv->buffers_running[LAST]=priv->buffers_running[CURRENT];
	priv->buffers_running[CURRENT]=priv->buffers_running[NEXT];

	if (priv->buffers_running[NEXT]){
		qtec_testgen_write(priv,BASEADDR,priv->test_mem_base_addr+priv->buffers_running[NEXT]->offset);
		priv->buffers_running[NEXT]=NULL;
	}

	priv->n_buffers++;

	complete(&priv->thread_comp);
	return IRQ_HANDLED;
}

static int qtec_testgen_stop(struct qtec_testgen *priv){
	uint32_t aux;
	unsigned long expiration;

	//Stop HW
	qtec_testgen_read(priv,CTRL,&aux);
	aux&=~BIT(RUN);
	aux&=~BIT(IRQENA);
	aux|=BIT(IRQ_STATUS);
	qtec_testgen_write(priv,CTRL,aux);

	//Clear buffer list
	INIT_LIST_HEAD(&priv->buffer_list);

	//Check if hw has stooped
	expiration=jiffies+2*HZ;
	do {
		qtec_testgen_read(priv,CTRL,&aux);
		if ((aux & BIT(STAT_RUN))==0)
			return 0;
	}while(time_before(expiration,jiffies)==0);

	dev_err(&priv->pdev->dev, "Timeout while stopping\n");

	return 0;
}

static inline bool qtec_testgen_is_compact(u32 code){
	switch(code){
		case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			return true;
		default:
			return false;
	}
}

static int qtec_testgen_next_frame(struct qtec_testgen *priv){
	unsigned long offset,size;
	struct qtec_testgen_buf *video_buf;
	void *buf_addr;

	if (list_empty(&priv->buffer_list)|| priv->buffers_running[NEXT])
		return -1;

	mutex_lock(&priv->stop_mutex);
	video_buf = list_first_entry_or_null(&priv->buffer_list, struct qtec_testgen_buf, list);
	if (!video_buf){
		mutex_unlock(&priv->stop_mutex);
		return -1;
	}
	list_del(&video_buf->list);

	size=vb2_get_plane_payload(&video_buf->vb2_buf.vb2_buf,0);
	buf_addr=vb2_plane_vaddr(&video_buf->vb2_buf.vb2_buf,0);
	video_buf->offset=priv->test_mem_next_offset;
	if (!buf_addr || !size){
		mutex_unlock(&priv->stop_mutex);
		dev_err(&priv->pdev->dev, "Ignoring invalid buffer\n");
		return -1;
	}

	memcpy_toio(priv->test_mem+priv->test_mem_next_offset,buf_addr,size);
	mutex_unlock(&priv->stop_mutex);

	priv->buffers_running[NEXT]=video_buf;

	//calculate next offset
	offset=priv->test_mem_next_offset+size;
	if (offset&0x3){
		offset+=4;
		offset&=~0x3;
	}
	if ((offset+size)>priv->test_mem_len)
		offset=0;
	priv->test_mem_next_offset=offset;

	return 0;
}

static int qtec_testgen_thread(void *data){
	struct qtec_testgen *priv = data;

	while (!kthread_should_stop()){
		wait_for_completion_interruptible(&priv->thread_comp);
		if (unlikely(priv->state==CLEAN))
			return 0;
		if (list_empty(&priv->buffer_list)|| priv->buffers_running[NEXT])
			continue;
		led_trigger_event(priv->led_img, (priv->n_buffers&1)?LED_FULL:LED_OFF);
		led_trigger_event(priv->led_drop, (priv->missed_buffers&1)?LED_FULL:LED_OFF);

		qtec_testgen_next_frame(priv);
	}

	return 0;
}

static int qtec_testgen_start(struct qtec_testgen *priv){
	uint32_t aux;
	int hblank;
	int vblank;
	struct v4l2_ctrl *ctrl;
	int testgen_width;
	unsigned long offset=0;

	ctrl=v4l2_ctrl_find(priv->sd.v4l2_dev->ctrl_handler,V4L2_CID_HBLANK);
	if (ctrl)
		hblank=v4l2_ctrl_g_ctrl(ctrl);
	else
		hblank=1;

	ctrl=v4l2_ctrl_find(priv->sd.v4l2_dev->ctrl_handler,V4L2_CID_VBLANK);
	if (ctrl)
		vblank=v4l2_ctrl_g_ctrl(ctrl);
	else
		vblank=1;

	testgen_width=priv->format_testgen.width;

	if (qtec_testgen_is_compact(priv->format_testgen.code)){
		hblank/=5;
		testgen_width/=5;
	}

	//param_len
	qtec_testgen_write(priv,LENGTH,	testgen_width*priv->format_testgen.height
			*sizeof(struct qtec_testgen_pixel));
	//sizes
	aux=testgen_width<<ACTIVE;
	aux|=(testgen_width+hblank)<<TOTAL;
	qtec_testgen_write(priv,H_PARAMS,aux);

	aux=priv->format_testgen.height<<ACTIVE;
	aux|=(priv->format_testgen.height+vblank)<<TOTAL;
	qtec_testgen_write(priv,V_PARAMS,aux);

	//buff_params
	qtec_testgen_write(priv,FRAMES,1);

	//baseaddr
	if (priv->buffers_running[NEXT])
		offset=priv->buffers_running[NEXT]->offset;
	qtec_testgen_write(priv,BASEADDR,priv->test_mem_base_addr+offset);

	aux = BIT(RUN);
	aux |= BIT(IRQENA);
	qtec_testgen_write(priv,CTRL,aux);

	return 0;
}

/*Subdev*/
static int qtec_testgen_set_fmt(struct v4l2_subdev *subdev,
	struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct qtec_testgen *priv = container_of(subdev, struct qtec_testgen, sd);
	struct v4l2_mbus_framefmt new_fmt;

	memset(&new_fmt,0,sizeof(new_fmt));
	new_fmt.width=fmt->width;
	new_fmt.height=fmt->height;
	new_fmt.code=fmt->code;
	priv->format_fg=new_fmt;

	return 0;
}

static int qtec_testgen_s_stream(struct v4l2_subdev *subdev, int enable){
	struct qtec_testgen *priv = container_of(subdev, struct qtec_testgen, sd);

	if (enable){
		priv->fg_is_streaming=true;
		if (priv->state==WAITING_FG){
			if (memcmp(&priv->format_fg,&priv->format_testgen,sizeof(priv->format_testgen))){
				v4l2_err(&priv->sd, "Format missmatch. Cannot start streaming\n");
				return 0;
			}
			qtec_testgen_start(priv);
			priv->state=RUNNING;
		}
	}
	else{
		priv->fg_is_streaming=false;
		if (priv->state==RUNNING){
			qtec_testgen_stop(priv);
			priv->state=WAITING_FG;
		}
	}

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_testgen_s_register(struct v4l2_subdev *subdev,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_testgen *priv = container_of(subdev, struct qtec_testgen, sd);
	uint32_t val=reg->val;

	if (reg->reg<0x1000)
		qtec_testgen_write(priv, reg->reg, val);
	else
		iowrite32(val,priv->test_mem+reg->reg-0x1000);

	return 0;
}

static int qtec_testgen_g_register(struct v4l2_subdev *subdev,
			      struct v4l2_dbg_register *reg)
{
	struct qtec_testgen *priv = container_of(subdev, struct qtec_testgen, sd);
	uint32_t val;

	if (reg->reg<0x1000)
		qtec_testgen_read(priv, reg->reg, &val);
	else
		val=ioread32(priv->test_mem+reg->reg-0x1000);

	reg->val=val;
	reg->size=4;
	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_testgen_pad_ops = {
	.set_fmt = qtec_testgen_set_fmt,
};

static const struct v4l2_subdev_video_ops qtec_testgen_video_ops = {
	.s_stream = qtec_testgen_s_stream,
};

static const struct v4l2_subdev_core_ops qtec_testgen_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_testgen_g_register,
	.s_register = qtec_testgen_s_register,
	#endif
};

static const struct v4l2_subdev_ops qtec_testgen_ops = {
	.core = &qtec_testgen_core_ops,
	.video = &qtec_testgen_video_ops,
	.pad = &qtec_testgen_pad_ops,
};

/*Controls*/
static int qtec_testgen_g_ctrl(struct v4l2_ctrl *ctrl)
{
       struct qtec_testgen *priv = container_of(ctrl->handler, struct qtec_testgen, ctrl_handler);

       switch (ctrl->id){
               case QTEC_TESTGEN_MISSED_FRAMES:
                       ctrl->val=priv->missed_buffers;
                       break;
               default:
                       return -EINVAL;
       }

       return 0;
}

static const struct v4l2_ctrl_ops qtec_testgen_ctrl_ops_vol = {
       .g_volatile_ctrl = qtec_testgen_g_ctrl,
};

/*VB2*/
static int qtec_testgen_vb2_queue_setup(struct vb2_queue *vq,
		unsigned int *nbufs,
		unsigned int *num_planes, unsigned int sizes[],
		struct device *alloc_devs[])
{
	struct qtec_testgen *priv = vb2_get_drv_priv(vq);
	int i;

	memcpy(&priv->format_testgen,&priv->format_fg,sizeof(priv->format_fg));

	sizes[0] = priv->format_testgen.width;
	sizes[0] *= priv->format_testgen.height;
	if (!qtec_testgen_is_compact(priv->format_testgen.code))
		sizes[0] *= sizeof(struct qtec_testgen_pixel);

	if (priv->test_mem_len < (2*sizes[0])){
		v4l2_err(&priv->sd, "Test mem is too small... Cant setup buffers\n");
		return -ENOMEM;
	}

	*num_planes = 1;

	INIT_LIST_HEAD(&priv->buffer_list);
	for (i=LAST;i<=NEXT;i++)
		priv->buffers_running[i]=NULL;

	priv->state=STOP;

	priv->usr_size = sizes[0];

	return 0;
}

static void qtec_testgen_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct qtec_testgen_buf *video_buf = container_of(vvb, struct qtec_testgen_buf, vb2_buf);
	struct qtec_testgen *priv = vb2_get_drv_priv(vb->vb2_queue);

	//Add buffer
	list_add_tail(&video_buf->list, &priv->buffer_list);

	//Notify thread
	complete(&priv->thread_comp);

	return;
}

static int qtec_testgen_vb2_start_streaming(struct vb2_queue *vq, unsigned int count){
	struct qtec_testgen *priv = vb2_get_drv_priv(vq);

	if(memcmp(&priv->format_fg,&priv->format_testgen,sizeof(priv->format_testgen))){
		v4l2_err(&priv->sd, "Format missmatch. Cannot start streaming\n");
		return -1;
	}
	if (priv->fg_is_streaming){
		qtec_testgen_start(priv);
		priv->state=RUNNING;
		complete(&priv->thread_comp);
	}
	else
		priv->state=WAITING_FG;
	return 0;
}

static void qtec_testgen_vb2_stop_streaming(struct vb2_queue *vq){
	struct qtec_testgen *priv = vb2_get_drv_priv(vq);

	if (priv->state==RUNNING){
		mutex_lock(&priv->stop_mutex);
		qtec_testgen_stop(priv);
		mutex_unlock(&priv->stop_mutex);
	}
	priv->state=STOP;

	vb2_wait_for_all_buffers(vq);

	return;
}

static const struct vb2_ops qtec_testgen_vb2_ops = {
	.queue_setup		= qtec_testgen_vb2_queue_setup,
	.buf_queue		= qtec_testgen_vb2_buf_queue,
	.stop_streaming		= qtec_testgen_vb2_stop_streaming,
	.start_streaming	= qtec_testgen_vb2_start_streaming,
};

/*
 * IOCTLS
 */
static int qtec_testgen_vidioc_querycap(struct file *file, void *pr,
		struct v4l2_capability *cap)
{
	struct qtec_testgen *priv=video_drvdata(file);
	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, DRIVER_NAME);
	snprintf(cap->bus_info, sizeof(cap->bus_info),"platform:%s",dev_name(&priv->pdev->dev));
		cap->device_caps = V4L2_CAP_VIDEO_OUTPUT |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING ;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int qtec_testgen_enum_output(struct file *file, void *priv,
				struct v4l2_output *out)
{
	if (out->index > 0)
		return -EINVAL;

	out->type = V4L2_OUTPUT_TYPE_MODULATOR; //Why not?
	sprintf(out->name, "Output %u", out->index);
	return 0;
}

static int qtec_testgen_g_output(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int qtec_testgen_s_output(struct file *file, void *priv, unsigned int i)
{
	if (i>0)
		return -EINVAL;
	return 0;
}

static int qtec_testgen_g_fmt_vid_out(struct file *file, void *pr,
			struct v4l2_format *f)
{
	int i;
	struct qtec_testgen *priv=video_drvdata(file);

	memset(f,0,sizeof(*f));//Compliance test

	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	f->fmt.pix.width = priv->format_fg.width;
	f->fmt.pix.height = priv->format_fg.height;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	for (i=0;i<ARRAY_SIZE(compact_table);i++)
		if (compact_table[i].mbus_code==priv->format_fg.code)
			break;

	if (i<ARRAY_SIZE(compact_table)){
		f->fmt.pix.bytesperline =f->fmt.pix.width;
		f->fmt.pix.pixelformat = compact_table[i].vdev_code;
	}
	else{
		f->fmt.pix.bytesperline =f->fmt.pix.width*sizeof(struct qtec_testgen_pixel);
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_QTEC_RGBPP40;
	}

	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;

	f->fmt.pix.priv = 0;

	return 0;
}

static int qtec_testgen_enum_fmt_vid_out(struct file *file, void  *pr,
					struct v4l2_fmtdesc *f)
{
	struct v4l2_format fmt;

	if (f->index!=0)
		return -EINVAL;

	qtec_testgen_g_fmt_vid_out(file,NULL,&fmt);

	f->pixelformat = fmt.fmt.pix.pixelformat;

	if (f->pixelformat==V4L2_PIX_FMT_QTEC_RGBPP40)
		strlcpy(f->description, "Qtec Legacy Raw 40bits", sizeof(f->description));
	else
		strlcpy(f->description, "Qtec Compact Raw 8bits", sizeof(f->description));

	return 0;
}

static const struct v4l2_ioctl_ops qtec_testgen_ioctl_ops = {
	/*Driver specific*/
	.vidioc_querycap	= qtec_testgen_vidioc_querycap,
	.vidioc_enum_fmt_vid_out  = qtec_testgen_enum_fmt_vid_out,
	.vidioc_try_fmt_vid_out   = qtec_testgen_g_fmt_vid_out,
	.vidioc_g_fmt_vid_out     = qtec_testgen_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out     = qtec_testgen_g_fmt_vid_out,
	.vidioc_enum_output       = qtec_testgen_enum_output,
	.vidioc_g_output          = qtec_testgen_g_output,
	.vidioc_s_output          = qtec_testgen_s_output,

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

static int testgen_fop_release(struct file *file)
{
	struct qtec_testgen *priv=video_drvdata(file);

	if ( priv->usr_size && file->f_pos && (file->f_pos % priv->usr_size))
		dev_err(&priv->pdev->dev, "Invalid size of testgen file: %lld instead of %ld\n", file->f_pos % priv->usr_size, priv->usr_size);

	return vb2_fop_release(file);
}


static const struct v4l2_file_operations qtec_testgen_v4l_fops = {
	.owner = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = testgen_fop_release,
	.write          = vb2_fop_write,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static struct video_device qtec_testgen_v4l_template = {
	.name = DRIVER_NAME,

	.fops = &qtec_testgen_v4l_fops,
	.ioctl_ops = &qtec_testgen_ioctl_ops,
	.release = video_device_release_empty,
};

static struct v4l2_ctrl *qtec_testgen_add_custom_control_volatile(struct qtec_testgen *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_testgen_ctrl_ops_vol,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 255,
		.def = 0,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE |V4L2_CTRL_FLAG_READ_ONLY,
	};
	ctrl.name=name,
	ctrl.id=id;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

/*
 * Probe/Remove
 */
static atomic_t qtec_testgen_sub_instance = ATOMIC_INIT(0);
static atomic_t qtec_testgen_instance = ATOMIC_INIT(0);
static int qtec_testgen_probe(struct platform_device *pdev){
	struct qtec_testgen *priv;
	struct resource res;
	int ret;
	struct device_node *node;
	int i;

	priv=(struct qtec_testgen *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev,priv);
	priv->pdev=pdev;

	/*
	 * Device tree parsing
	 */
	ret=of_address_to_resource(pdev->dev.of_node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get cdma address\n");
		return ret;
	}

	priv->iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem)){
		dev_err(&pdev->dev, "Unable to ioremap packer\n");
		return PTR_ERR(priv->iomem);
	}

	ret=of_irq_to_resource(pdev->dev.of_node,0,&res);
	if(!ret){
		dev_err(&pdev->dev, "Unable to get packer irq\n");
		return -EIO;
	}

	ret=devm_request_irq(&pdev->dev,res.start,qtec_testgen_irq_handler,0,DRIVER_NAME,priv);
	if (ret){
		dev_err(&pdev->dev, "Unable to request packer irq\n");
		return ret;
	}

	node=of_parse_phandle(pdev->dev.of_node,"qtec,test_mem",0);
	if (!node){
		dev_err(&pdev->dev, "Unable to get test mem node\n");
		return -EIO;
	}

	ret=of_address_to_resource(node,0,&res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get test mem address\n");
		return ret;
	}
	priv->test_mem_len=resource_size(&res);

	priv->test_mem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->test_mem)){
		dev_err(&pdev->dev, "Unable to ioremap test mem\n");
		return PTR_ERR(priv->test_mem);
	}
	ret=of_property_read_u32(node,"reg-axi",&priv->test_mem_base_addr);
	if (ret){
		dev_err(&pdev->dev, "Unable to get test mem base address\n");
		return -EIO;
	}
	of_node_put(node);

	/*initial values*/
	mutex_init(&priv->stop_mutex);
	INIT_LIST_HEAD(&priv->buffer_list);
	init_completion(&priv->thread_comp);
	priv->fg_is_streaming = false;
	priv->state=STOP;
	priv->test_mem_next_offset=0;
	for (i=LAST;i<=NEXT;i++)
		priv->buffers_running[i]=NULL;

	priv->format_fg.width=1024;
	priv->format_fg.height=768;
	priv->format_fg.code=MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP;
	priv->format_fg.field=V4L2_FIELD_NONE;
	priv->format_fg.colorspace=V4L2_COLORSPACE_SRGB;
	//Host device shall set_fmt asap

	/*
	 * Subdev
	 */
	v4l2_subdev_init(&priv->sd, &qtec_testgen_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_testgen_sub_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	/*Controls*/
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 4); //4 is just a guess for the hash table

	priv->missed_frames=qtec_testgen_add_custom_control_volatile(priv,"Missed Frames Test Generator",QTEC_TESTGEN_MISSED_FRAMES);

	if (priv->ctrl_handler.error) {
		video_unregister_device(&priv->vdev);
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}
	//Init controls
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	/*v4l2 device*/
	v4l2_device_set_name(&priv->v4l2_dev, DRIVER_NAME, &qtec_testgen_instance);
	ret=v4l2_device_register(&pdev->dev,&priv->v4l2_dev);
	if (ret){
		dev_err(&pdev->dev, "Unable to register v4l2 device\n");
		return ret;
	}

	/*
	 * vdev
	 */
	priv->vb2_vidq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	priv->vb2_vidq.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_WRITE;
	priv->vb2_vidq.drv_priv = priv;
	priv->vb2_vidq.buf_struct_size = sizeof (struct qtec_testgen_buf);
	priv->vb2_vidq.mem_ops = &vb2_vmalloc_memops;
	priv->vb2_vidq.ops = &qtec_testgen_vb2_ops;
	priv->vb2_vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(&priv->vb2_vidq);
	if (ret){
		dev_err(&pdev->dev, "Unable to init vb2 queue\n");
		v4l2_device_unregister(&priv->v4l2_dev);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister_subdev(&priv->sd);
		return ret;
	}
	mutex_init(&priv->vdev_mutex);
	priv->vdev=qtec_testgen_v4l_template;
	priv->vdev.lock=&priv->vdev_mutex;
	priv->vdev.v4l2_dev=&priv->v4l2_dev;
	priv->vdev.queue=&priv->vb2_vidq;
	priv->vdev.vfl_dir=VFL_DIR_TX;
	video_set_drvdata(&priv->vdev,priv);

	ret = video_register_device(&priv->vdev, VFL_TYPE_GRABBER, -1);
	if (ret){
		dev_err(&pdev->dev, "Unable to register dist video device\n");
		vb2_queue_release(&priv->vb2_vidq);
		v4l2_device_unregister(&priv->v4l2_dev);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister_subdev(&priv->sd);
		return ret;
	}

	/*Led triggers */
	snprintf(priv->led_img_name,LEDNAME,"testgen-%s",priv->v4l2_dev.name);
	led_trigger_register_simple(priv->led_img_name,&priv->led_img); //Cannot return err
	snprintf(priv->led_drop_name,LEDNAME,"testgen-drop-%s",priv->v4l2_dev.name);
	led_trigger_register_simple(priv->led_drop_name,&priv->led_drop); //Cannot return err
	led_trigger_event(priv->led_img, LED_OFF);
	led_trigger_event(priv->led_drop, LED_OFF);

	/*Launch kthread*/
	priv->kthread=kthread_run(qtec_testgen_thread,priv,priv->v4l2_dev.name);
	if (!priv->kthread){
		dev_err(&pdev->dev, "Unable to launch thread\n");
		video_unregister_device(&priv->vdev);
		vb2_queue_release(&priv->vb2_vidq);
		v4l2_device_unregister(&priv->v4l2_dev);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
		v4l2_device_unregister_subdev(&priv->sd);
		return ret;
	}

	v4l2_info(&priv->sd, "qtec_testgen V4L2 subdevice registered as %s\n", priv->sd.name);

	return 0;
}

static int qtec_testgen_remove(struct platform_device *pdev){
	struct qtec_testgen *priv=platform_get_drvdata(pdev);

	priv->state=CLEAN;
	complete(&priv->thread_comp);
	kthread_stop(priv->kthread);

	led_trigger_unregister_simple(priv->led_img);
	led_trigger_unregister_simple(priv->led_drop);

	video_unregister_device(&priv->vdev);
	vb2_queue_release(&priv->vb2_vidq);
	v4l2_device_unregister(&priv->v4l2_dev);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static struct of_device_id qtec_testgen_of_match[] = {
	{ .compatible = "qtec,axi_fb_testgen-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_testgen_of_match);

static struct platform_driver qtec_testgen_plat_driver = {
	.probe		= qtec_testgen_probe,
	.remove		= qtec_testgen_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_testgen_of_match,
	},
};

module_platform_driver(qtec_testgen_plat_driver);

MODULE_DESCRIPTION("Qtec testgen v4l2 module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
