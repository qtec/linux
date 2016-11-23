/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define DRIVER_NAME "qtec_white"

enum {RED=0,GREEN,BLUE,IR1,IR2};
#define MAX_COLORS (IR2+1)

struct qtec_white{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct platform_device *pdev;
	void __iomem *iomem;

	u32 mbus_format;
	int n_colors;
	int n_sectors;
	int wb_offset;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	   *white_balance[MAX_COLORS];
	struct v4l2_ctrl	   *offset[MAX_COLORS];
	struct v4l2_ctrl	   *white_balance_compact;
	struct v4l2_ctrl	   *offset_compact;
};

#define SECTORSEL 0x0
#define VERSION 24
#define N_SECTORS_V0 8
#define N_SECTORS_V4 16

#define ENDLINE 0x4
#define WHITEBALANCE 0x20
#define WHITEBALANCE_V40 0x10

#define GAIN 0
#define GAIN_LEN 16
#define GAIN_MASK ((1<<GAIN_LEN)-1)
#define GAIN_DEF 0x4000

#define OFFSET 16
#define OFFSET_LEN 16
#define OFFSET_MASK ((1<<OFFSET_LEN)-1)
#define OFFSET_DEF 0
#define OFFSET_MIN_VALUE -0x7ffe
#define OFFSET_MAX_VALUE 0x7fff

static inline int qtec_white_write(struct qtec_white *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "white W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value,priv->iomem+offset);
	return 0;
}

static inline int qtec_white_read(struct qtec_white *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "white R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

static int qtec_white_s_ctrl_aux(struct v4l2_ctrl *ctrl)
{
	struct qtec_white *priv = container_of(ctrl->handler, struct qtec_white, ctrl_handler);
	uint32_t value;
	static const int qtec_v4l2[][3] ={
		{V4L2_CID_RED_BALANCE,RED,true},
		{QTEC_WHITE_CID_GREEN_BALANCE,GREEN,true},
		{V4L2_CID_BLUE_BALANCE,BLUE,true},
		{QTEC_WHITE_CID_IR1_BALANCE,IR1,true},
		{QTEC_WHITE_CID_IR2_BALANCE,IR2,true},
		{QTEC_WHITE_CID_RED_OFFSET,RED,false},
		{QTEC_WHITE_CID_GREEN_OFFSET,GREEN,false},
		{QTEC_WHITE_CID_BLUE_OFFSET,BLUE,false},
		{QTEC_WHITE_CID_IR1_OFFSET,IR1,false},
		{QTEC_WHITE_CID_IR2_OFFSET,IR2,false},
	};
	int i;

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	for (i=0; i<ARRAY_SIZE(qtec_v4l2);i++)
		if (ctrl->id == qtec_v4l2[i][0])
			break;

	if (i==ARRAY_SIZE(qtec_v4l2))
		return -EINVAL;

	qtec_white_read(priv, priv->wb_offset+4*qtec_v4l2[i][1],&value);

	if(qtec_v4l2[i][2]){
		//gain
		value&=~((GAIN_MASK)<<GAIN);
		value|=ctrl->val<<GAIN;
	}
	else{
		//offset
		value&=~((OFFSET_MASK)<<OFFSET);
		value|=ctrl->val<<OFFSET;
	}
	qtec_white_write(priv, priv->wb_offset+4*qtec_v4l2[i][1],value);

	return 0;
}

static int qtec_white_s_ctrl(struct v4l2_ctrl *ctrl){
	int ret;

	ret=qtec_white_s_ctrl_aux(ctrl);

	if (ret)
		ctrl->val=ctrl->cur.val;

	return ret;
}

static const struct v4l2_ctrl_ops qtec_white_ctrl_ops = {
	.s_ctrl = qtec_white_s_ctrl,
};

static int qtec_white_s_ctrl_compact(struct v4l2_ctrl *ctrl)
{
	struct qtec_white *priv = container_of(ctrl->handler, struct qtec_white, ctrl_handler);
	int i;
	uint32_t value;

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	for (i=0;i<priv->n_colors;i++){
		qtec_white_read(priv, priv->wb_offset+4*i,&value);
		if(ctrl->id==QTEC_WHITE_CID_COMPACT_BALANCE){
			//gain
			value&=~((GAIN_MASK)<<GAIN);
			value|=ctrl->val<<GAIN;
		}
		else{
			//offset
			value&=~((OFFSET_MASK)<<OFFSET);
			value|=ctrl->val<<OFFSET;
		}
		qtec_white_write(priv, priv->wb_offset+4*i,value);
	}

	return 0;
}
static const struct v4l2_ctrl_ops qtec_white_ctrl_ops_compact = {
	.s_ctrl = qtec_white_s_ctrl_compact,
};

static struct v4l2_ctrl *qtec_white_add_custom_control(struct qtec_white *priv, char *name, int id,int min, int max,int def){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_white_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
	};

	ctrl.max=max;
	ctrl.min=min;
	ctrl.id=id;
	ctrl.name=name;
	ctrl.def=def;
	ctrl.flags = V4L2_CTRL_FLAG_SLIDER;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *qtec_white_add_custom_control_compact(struct qtec_white *priv, char *name, int id, int min, int max,int def){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_white_ctrl_ops_compact,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
	};

	ctrl.max=max;
	ctrl.min=min;
	ctrl.id=id;
	ctrl.name=name;
	ctrl.def=def;
	ctrl.flags = V4L2_CTRL_FLAG_SLIDER;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static int qtec_white_set_fmt(struct v4l2_subdev *subdev,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int n_colors;
	int compact;
	int i;
	struct qtec_white *priv = container_of(subdev, struct qtec_white, sd);

	if (fmt->code==priv->mbus_format)
		return 0;

	switch(fmt->code){
		case MEDIA_BUS_FMT_QTEC_LEGACY_MONO:
			n_colors=1;
			compact=false;
			break;
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGBPP:
			n_colors=5;
			compact=false;
			break;
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGB:
		case MEDIA_BUS_FMT_QTEC_FB4_RGBX:
			n_colors=3;
			compact=false;
			break;
		case MEDIA_BUS_FMT_QTEC_COMPACT_MONO:
		case MEDIA_BUS_FMT_QTEC_LEGACY_RGGB:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GBRG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_GRBG:
		case MEDIA_BUS_FMT_QTEC_LEGACY_BGGR:
		case MEDIA_BUS_FMT_QTEC_COMPACT_RGGB:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GBRG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_GRBG:
		case MEDIA_BUS_FMT_QTEC_COMPACT_BGGR:
			n_colors=5;
			compact=true;
			break;
		case MEDIA_BUS_FMT_QTEC_FB4_MONO:
		case MEDIA_BUS_FMT_QTEC_FB4_RGGB:
		case MEDIA_BUS_FMT_QTEC_FB4_GRBG:
		case MEDIA_BUS_FMT_QTEC_FB4_GBRG:
		case MEDIA_BUS_FMT_QTEC_FB4_BGGR:
		case MEDIA_BUS_FMT_QTEC_FB4_GREEN:
		case MEDIA_BUS_FMT_QTEC_FB4_GREY:
			n_colors =4;
			compact=true;
			break;
		default:
			v4l2_err(&priv->sd, "Invalid mbus format (0x%08x) unknown.\n",
			fmt->code);
			fmt->code=MEDIA_BUS_FMT_QTEC_COMPACT_MONO;
			n_colors=priv->n_colors;
			compact=true;
	}

	priv->mbus_format=fmt->code;

	if ((compact==false)&&(n_colors>1)){
		for(i=0;i<n_colors;i++){
			qtec_white_s_ctrl(priv->white_balance[i]);
			priv->white_balance[i]->flags&=~V4L2_CTRL_FLAG_INACTIVE;
			qtec_white_s_ctrl(priv->offset[i]);
			priv->offset[i]->flags&=~V4L2_CTRL_FLAG_INACTIVE;
		}
		for(;i<priv->n_colors;i++){
			priv->white_balance[i]->flags|=V4L2_CTRL_FLAG_INACTIVE;
			priv->offset[i]->flags|=V4L2_CTRL_FLAG_INACTIVE;
		}
		priv->white_balance_compact->flags|=V4L2_CTRL_FLAG_INACTIVE;
		priv->offset_compact->flags|=V4L2_CTRL_FLAG_INACTIVE;
	}
	else{
		for(i=0;i<priv->n_colors;i++){
			priv->white_balance[i]->flags|=V4L2_CTRL_FLAG_INACTIVE;
			priv->offset[i]->flags|=V4L2_CTRL_FLAG_INACTIVE;
		}
		priv->white_balance_compact->flags&=~V4L2_CTRL_FLAG_INACTIVE;
		priv->offset_compact->flags&=~V4L2_CTRL_FLAG_INACTIVE;
		for(i=0;i<priv->n_sectors;i++){ //Workaround for #275
			qtec_white_write(priv,SECTORSEL,i);
			qtec_white_s_ctrl_compact(priv->white_balance_compact);
			qtec_white_s_ctrl_compact(priv->offset_compact);
			if (n_colors==1){ //Workaround for #321
				int chan;
				for (chan=1;chan<priv->n_colors;chan++)
					qtec_white_write(priv,priv->wb_offset+4*chan,0);
			}
		}
		qtec_white_write(priv,SECTORSEL,0);
	}

	return 0;
}

//Init sectors
static int qtec_white_init_sectors(struct qtec_white *priv){
	int i;

	for(i=0;i<priv->n_sectors;i++){
		qtec_white_write(priv,SECTORSEL,i);
		qtec_white_write(priv,ENDLINE,~0);
	}
	qtec_white_write(priv,SECTORSEL,0);

	return 0;
}

//Debug registers
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_white_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_white *priv = container_of(sd, struct qtec_white, sd);

	qtec_white_write(priv, reg->reg,reg->val);
	return 0;
}

static int qtec_white_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	uint32_t val;
	struct qtec_white *priv = container_of(sd, struct qtec_white, sd);

	qtec_white_read(priv, reg->reg,&val);
	reg->val=val;
	reg->size=4;
	return 0;
}
#endif

static const struct v4l2_subdev_pad_ops qtec_white_pad_ops = {
	.set_fmt = qtec_white_set_fmt,
};

static const struct v4l2_subdev_core_ops qtec_white_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_white_g_register,
	.s_register = qtec_white_s_register,
#endif
};

static const struct v4l2_subdev_ops qtec_white_ops = {

	.pad = &qtec_white_pad_ops,
	.core = &qtec_white_core_ops,
};

//Probe
static atomic_t qtec_white_instance = ATOMIC_INIT(0);
static int qtec_white_probe(struct platform_device *pdev){
	struct qtec_white *priv;
	struct resource res;
	int ret;
	u32 version;

	priv=(struct qtec_white *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
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

	qtec_white_read(priv, SECTORSEL, &version);
	version >>= VERSION;
	switch (version){
		case 0x0:
			priv->n_colors = 5;
			priv->n_sectors = N_SECTORS_V0;
			priv->wb_offset = WHITEBALANCE;
			break;
		case 0x40:
			priv->n_colors = 4;
			priv->n_sectors = N_SECTORS_V4;
			priv->wb_offset = WHITEBALANCE_V40;
			break;
		default:
			dev_err(&pdev->dev, "Unknown version 0x%x\n", version);
			return -EIO;
	}

	//Subdev
	v4l2_subdev_init(&priv->sd, &qtec_white_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&qtec_white_instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE ; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS ;

	//Controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, priv->n_colors*2);
	priv->white_balance[RED] = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_white_ctrl_ops,
			V4L2_CID_RED_BALANCE, 0,GAIN_MASK, 1, GAIN_DEF);
	priv->white_balance[BLUE] = v4l2_ctrl_new_std(&priv->ctrl_handler, &qtec_white_ctrl_ops,
			V4L2_CID_BLUE_BALANCE, 0,GAIN_MASK, 1, GAIN_DEF);
	priv->white_balance[GREEN] = qtec_white_add_custom_control(priv,
		"Green Balance", QTEC_WHITE_CID_GREEN_BALANCE,0, GAIN_MASK,GAIN_DEF);
	priv->white_balance[IR1] = qtec_white_add_custom_control(priv,
		"IR1 Balance", QTEC_WHITE_CID_IR1_BALANCE,0,GAIN_MASK,GAIN_DEF);
	if (priv->n_colors >4)
		priv->white_balance[IR2] = qtec_white_add_custom_control(priv,
				"IR2 Balance", QTEC_WHITE_CID_IR2_BALANCE,0,GAIN_MASK,GAIN_DEF);
	priv->offset[RED] = qtec_white_add_custom_control(priv,
		"Red Offset", QTEC_WHITE_CID_RED_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);
	priv->offset[GREEN] = qtec_white_add_custom_control(priv,
		"Green Offset", QTEC_WHITE_CID_GREEN_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);
	priv->offset[BLUE] = qtec_white_add_custom_control(priv,
		"Blue Offset", QTEC_WHITE_CID_BLUE_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);
	priv->offset[IR1] = qtec_white_add_custom_control(priv,
		"IR1 Offset", QTEC_WHITE_CID_IR1_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);
	if (priv->n_colors >4)
		priv->offset[IR2] = qtec_white_add_custom_control(priv,
			"IR2 Offset", QTEC_WHITE_CID_IR2_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);
	priv->white_balance_compact = qtec_white_add_custom_control_compact(priv,
		"Compact Balance", QTEC_WHITE_CID_COMPACT_BALANCE,0,GAIN_MASK,GAIN_DEF);
	priv->offset_compact = qtec_white_add_custom_control_compact(priv,
		"Compact Offset", QTEC_WHITE_CID_COMPACT_OFFSET,OFFSET_MIN_VALUE,OFFSET_MAX_VALUE,OFFSET_DEF);

	//Disable compact
	priv->white_balance_compact->flags|=V4L2_CTRL_FLAG_INACTIVE;
	priv->offset_compact->flags|=V4L2_CTRL_FLAG_INACTIVE;

	//Init hardware
	qtec_white_init_sectors(priv);
	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	//Connect to subdev
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	v4l2_info(&priv->sd, "qtec_white V4L2 subdevice version 0x%x registered as %s\n", version, priv->sd.name);

	return 0;
}

static int __exit qtec_white_remove(struct platform_device *pdev){
	struct qtec_white *priv=platform_get_drvdata(pdev);

	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static struct of_device_id qtec_white_of_match[] = {
	{ .compatible = "qtec,axi_white_balance-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_white_of_match);

static struct platform_driver qtec_white_plat_driver = {
	.probe		= qtec_white_probe,
	.remove		= qtec_white_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_white_of_match,
	},
};

module_platform_driver(qtec_white_plat_driver);

MODULE_DESCRIPTION("White balance module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");
