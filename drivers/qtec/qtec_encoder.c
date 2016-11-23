/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2016 Qtechnology/AS
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

#define DRIVER_NAME "qtec_encoder"

#define CONTROL 0x0
#define VERSION 24

#define CNT 0x4

struct qtec_encoder{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct platform_device *pdev;
	void __iomem *iomem;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *encoder_pos;
};

static inline int qtec_encoder_write(struct qtec_encoder *priv, uint32_t offset, uint32_t value){
	dev_dbg(&priv->pdev->dev, "encoder W:0x%.2x 0x%.8x\n",offset,value);
	iowrite32(value, priv->iomem+offset);
	return 0;
}

static inline int qtec_encoder_read(struct qtec_encoder *priv, uint32_t offset, uint32_t *value){
	*value=ioread32(priv->iomem+offset);
	dev_dbg(&priv->pdev->dev, "encoder R:0x%.2x 0x%.8x\n",offset,*value);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int qtec_encoder_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct qtec_encoder *priv = container_of(sd, struct qtec_encoder, sd);

	qtec_encoder_write(priv, reg->reg,reg->val);
	return 0;
}

static int qtec_encoder_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	uint32_t val;
	struct qtec_encoder *priv = container_of(sd, struct qtec_encoder, sd);

	qtec_encoder_read(priv, reg->reg,&val);
	reg->val=val;
	reg->size=4;
	return 0;
}
#endif


static const struct v4l2_subdev_core_ops qtec_encoder_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = qtec_encoder_g_register,
	.s_register = qtec_encoder_s_register,
#endif
};

static const struct v4l2_subdev_ops qtec_encoder_ops = {
	.core = &qtec_encoder_core_ops,
};

static int qtec_encoder_g_volatile_ctrl(struct v4l2_ctrl *ctrl){
	struct qtec_encoder *priv = container_of(ctrl->handler, struct qtec_encoder, ctrl_handler);
	uint32_t aux;

	switch (ctrl->id) {
		case QTEC_ENCODER_CID_ENCODER_POSITION:
			qtec_encoder_read(priv, CNT, &aux);
			ctrl->val =  aux & 0xffff;
			return 0;
		default:
			return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops qtec_encoder_ctrl_ops = {
	.g_volatile_ctrl = qtec_encoder_g_volatile_ctrl,
};

static atomic_t qtec_encoder_instance = ATOMIC_INIT(0);
static int qtec_encoder_probe(struct platform_device *pdev){
	struct qtec_encoder *priv;
	struct resource res;
	int ret;
	u32 version;
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_encoder_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = 0,
		.max = 0xffff,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.name = "Encoder Position",
		.id = QTEC_ENCODER_CID_ENCODER_POSITION,
	};

	priv = (struct qtec_encoder *)devm_kzalloc(&pdev->dev,sizeof(*priv),GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;

	//Dt parse
	ret = of_address_to_resource(pdev->dev.of_node, 0, &res);
	if (ret){
		dev_err(&pdev->dev, "Unable to get address\n");
		return ret;
	}

	priv->iomem  = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(priv->iomem)){
		dev_err(&pdev->dev, "Unable to ioremap memory\n");
		return PTR_ERR(priv->iomem);
	}

	qtec_encoder_read(priv, CONTROL, &version);
	version >>= VERSION;
	switch (version){
		case 0x1:
			break;
		default:
			dev_err(&pdev->dev, "Unknown version 0x%x\n", version);
			return -EIO;
	}

	//Subdev
	v4l2_subdev_init(&priv->sd, &qtec_encoder_ops);
	snprintf(priv->sd.name, sizeof(priv->sd.name), "%s-%d",DRIVER_NAME, atomic_inc_return(&qtec_encoder_instance) - 1 );
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE ; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS ;

	//Controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 8);
	priv->encoder_pos = v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);

	v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (priv->ctrl_handler.error) {
		dev_err(&pdev->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}

	//Connect to subdev
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	v4l2_info(&priv->sd, "qtec_encoder V4L2 subdevice version 0x%x registered as %s\n", version, priv->sd.name);

	return 0;
}

static int __exit qtec_encoder_remove(struct platform_device *pdev){
	struct qtec_encoder *priv=platform_get_drvdata(pdev);

	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static struct of_device_id qtec_encoder_of_match[] = {
	{ .compatible = "qtec,axi_encoder-1.00.a"},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, qtec_encoder_of_match);

static struct platform_driver qtec_encoder_plat_driver = {
	.probe		= qtec_encoder_probe,
	.remove		= qtec_encoder_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= qtec_encoder_of_match,
	},
};

module_platform_driver(qtec_encoder_plat_driver);

MODULE_DESCRIPTION("Encoder module");
MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_LICENSE("GPL");


