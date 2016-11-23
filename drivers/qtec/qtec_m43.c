/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2015 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/qtec/qtec_video.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <linux/kthread.h>

#define DRIVER_NAME "qtec_m43"

enum SMBUS_CMD {VERSION=0, STATUS, LENS_START, LENS_STOP, LEN, PARAMS, DO_CMD_READ, DO_CMD_WRITE, OFFSET_READ, RST_AVR=0xf0 ,BUFFER_SIZE=0xfd, BUFFER_ADDR, BUFFER};

struct zoom_step {
	uint32_t aperture_min;
	uint32_t aperture_max;
	uint32_t focus_min;
	uint32_t focal_length_middle;
};

struct qtec_m43{
	struct v4l2_subdev sd; //NEEDS to be first!!!!
	struct i2c_client *client;
	struct task_struct *kthread;
	wait_queue_head_t wq;
	struct mutex lock;
	int focal_length_val;

	bool motor_zoom;
	int n_steps;
	struct zoom_step *steps;
	uint16_t lens_vendor;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	   *lens_name;
	struct v4l2_ctrl	   *lens_version;
	struct v4l2_ctrl	   *lens_active;
	struct v4l2_ctrl	   *focus;
	struct v4l2_ctrl	   *focal_length;
	struct v4l2_ctrl	   *focal_length_non_block;
	struct v4l2_ctrl	   *aperture;
};

static int __m43_cmd(struct qtec_m43 *priv, uint8_t type, uint8_t code, uint8_t param1 , uint8_t param2, uint8_t *buffer, int maxlen){
	static uint8_t buff[32];
	int i,len,ret;
	unsigned long expiration;

	ret = i2c_smbus_write_word_data(priv->client, PARAMS, param2<<8|param1);
	if (ret<0)
		return ret;

	ret = i2c_smbus_write_word_data(priv->client, DO_CMD_READ, type<<8|code);
	if (ret<0)
		return ret;

	expiration=jiffies+2*HZ;
	do {
		if (!time_before(jiffies,expiration))
			return -ETIMEDOUT;
		ret=i2c_smbus_read_byte(priv->client);
		if (ret == 0 || ret ==0xfd)
			break;
	}while(1);

	if ((ret) && (ret!=0xfd))
		return -EIO;

	len = i2c_smbus_read_word_data(priv->client,LEN);
	if (len<1)
		return len;

	if (len > maxlen)
		return -ENOSPC;

	for (i=0;i<len && i<maxlen;){
		ret = i2c_smbus_read_block_data(priv->client, BUFFER, buff);
		if (ret < 0)
			return ret;
		if (!ret || (i+ret>maxlen))
			return -EIO;
		memcpy(buffer+i,buff,ret);
		i+=ret;
	}

	return i;
}

static int _m43_cmd(struct qtec_m43 *priv, uint8_t type, uint8_t code, uint8_t param1 , uint8_t param2, uint8_t *buffer, int maxlen){
	int ret;

	ret = __m43_cmd(priv,type,code,param1,param2,buffer,maxlen);
	dev_dbg(&priv->client->dev, "cmd type:%.2x code:%.2x p1: %.2x p2: %.2x RET=%d\n", type, code, param1, param2, ret);
	if (ret>0)
		print_hex_dump_bytes("data:",DUMP_PREFIX_OFFSET,buffer,ret);

	return ret;
}

static int m43_cmd(struct qtec_m43 *priv, uint8_t type, uint8_t code, uint8_t param1 , uint8_t param2, uint8_t *buffer, int maxlen){
	int ret;

	mutex_lock(&priv->lock);
	ret = _m43_cmd(priv,type,code,param1,param2,buffer,maxlen);
	mutex_unlock(&priv->lock);

	return ret;
}

static int m43_focal_length(struct qtec_m43 *priv, int val){
	static uint8_t buffer[256];
	unsigned long expiration;
	int ret=0;
	int motor_stopped = 10;

	//value is *10
	val /= 100;

	mutex_lock(&priv->lock);
	if (_m43_cmd(priv, 0xa0,0xd1,val,val>>8,NULL,0)){
		ret = -EIO;
		goto ex;
	}

	buffer[1] = 0x0;
	expiration=jiffies+10*HZ;
	do {
		if (!time_before(jiffies,expiration)){
			ret =ETIMEDOUT;
			goto ex;
		}
		if (priv->lens_vendor == 0x2) //LUMIX
			ret = _m43_cmd(priv, 0x81, 0xe3, 0x4, 0xb, buffer,sizeof(buffer));
		else
			ret = _m43_cmd(priv, 0xc1, 0x80, 0x4, 0xb, buffer,sizeof(buffer));
		if (ret<2){
			ret =EIO;
			goto ex;
		}

		if (!(buffer[1] & 0xf))
			motor_stopped --;
	}while(motor_stopped>0);
	ret = 0;

ex:
	msleep(1000);
	mutex_unlock(&priv->lock);
	return ret;
}

static s32 m43_fast_cmd(struct qtec_m43 *priv, const struct i2c_client *client, u8 command, u16 value){
	s32 ret;

	mutex_lock(&priv->lock);
	ret = i2c_smbus_write_word_data(client,command,value);
	mutex_unlock(&priv->lock);

	return ret;
}

static void m43_stoplens(struct qtec_m43 *priv){
	m43_fast_cmd(priv,priv->client, LENS_STOP, 0);

	__v4l2_ctrl_s_ctrl_string(priv->lens_name, "");
	__v4l2_ctrl_s_ctrl_string(priv->lens_version, "");
	v4l2_ctrl_activate(priv->lens_name, false);
	v4l2_ctrl_activate(priv->lens_version, false);
	v4l2_ctrl_activate(priv->focus, false);
	v4l2_ctrl_activate(priv->focal_length, false);
	v4l2_ctrl_activate(priv->focal_length_non_block, false);
	v4l2_ctrl_activate(priv->aperture, false);
	return;
}

static int v4l2_ctrl_modify_range_cond(struct v4l2_ctrl *ctrl,
                            s64 min, s64 max, u64 step, s64 def)
{
       if ((ctrl->minimum == min) &&   (ctrl->maximum == max) &&
               (ctrl->step == step) && (ctrl->default_value == def))
               return 0;

       return v4l2_ctrl_modify_range(ctrl,min,max,step,def);
}
static int __v4l2_ctrl_modify_range_cond(struct v4l2_ctrl *ctrl,
                            s64 min, s64 max, u64 step, s64 def)
{
       if ((ctrl->minimum == min) &&   (ctrl->maximum == max) &&
               (ctrl->step == step) && (ctrl->default_value == def))
               return 0;

       return __v4l2_ctrl_modify_range(ctrl,min,max,step,def);
}

static int update_aperture(struct qtec_m43 *priv){
	int app;
	static uint8_t buffer[2];

	if (m43_cmd(priv, 0xb0,0xdd,0,0,buffer,sizeof(buffer))!=2)
		return -1;

	app = buffer[0] + (buffer[1]<<8);
	app *= 1000;
	app /= 256;

	app = clamp_t(int,app, priv->aperture->minimum, priv->aperture->maximum);
	if (priv->aperture->cur.val != app)
		__v4l2_ctrl_s_ctrl(priv->aperture, app);

	return 0;
}

static int m43_startuplens(struct qtec_m43 *priv){
	static uint8_t buffer[2048];
	int ret;
	int i;
	unsigned long expiration;
	int min,max;
	char version[5];

	ret = m43_fast_cmd(priv,priv->client, LENS_START, 0);
	if (ret){
		dev_err(&priv->client->dev, "Error starting up lens, ret=%d\n", ret);
		return ret;
	}

	expiration=jiffies+5*HZ;
	do {
		if (!time_before(jiffies,expiration)){
			ret = m43_fast_cmd(priv,priv->client, LENS_STOP, 0);
			return -ETIMEDOUT;
		}
		ret=i2c_smbus_read_byte(priv->client);
		if (ret == 0 || ret ==0xfd)
			break;
	}while(1);

	ret = m43_cmd(priv, 0xc0, 0xf9, 0, 0, buffer,20);
	if (ret<20){
		dev_err(&priv->client->dev, "Error getting lens vendor, ret=%d\n", ret);
		return -EIO;
	}
	priv->lens_vendor = (buffer[1] <<8) | buffer[0];

	ret = m43_cmd(priv, 0xc3, 0xf0, 0, 0, buffer,sizeof(buffer));
	if (ret<66){
		dev_err(&priv->client->dev, "Error getting lens info, ret=%d\n", ret);
		return -EIO;
	}

	buffer[31+10] = '\0';
	__v4l2_ctrl_s_ctrl_string(priv->lens_name,buffer+10);
	snprintf(version, 5, "%.2X%.2X", buffer[64], buffer[65]);
	__v4l2_ctrl_s_ctrl_string(priv->lens_version, version);

	priv->n_steps = buffer[42];

	if (priv->steps)
		devm_kfree(&priv->client->dev,priv->steps);
	priv->steps = NULL;

	if ((66 + 7*(2*priv->n_steps) + (2*priv->n_steps-1) + 1)>ret)
		return -ENOMEM;

	priv->steps = kzalloc(priv->n_steps*sizeof(*priv->steps), GFP_KERNEL);
	if (!priv->steps)
		return -ENOMEM;

	for (i=0;i<priv->n_steps;i++){
		int idx = 66 + 0*(2*priv->n_steps) + 2*i;
		priv->steps[i].focal_length_middle = (buffer[idx+1]<<8) + buffer[idx];
		idx = 66 + 1*(2*priv->n_steps) + 2*i;
		priv->steps[i].aperture_min = (((buffer[idx+1]<<8) + buffer[idx])*1000)/256;
		idx = 66 + 2*(2*priv->n_steps) + 2*i;
		priv->steps[i].aperture_max = (((buffer[idx+1]<<8) + buffer[idx])*1000)/256;
		idx = 66 + 7*(2*priv->n_steps) + 2*i;
		priv->steps[i].focus_min = (buffer[idx+1]<<8) + buffer[idx];
		dev_dbg(&priv->client->dev, "%2d %.5d %.5d %.5d\n",i,priv->steps[i].aperture_min, priv->steps[i].aperture_max, priv->steps[i].focus_min);
	}

	priv->motor_zoom = !!(buffer[2] & 0x40);

	priv->focal_length->flags &= ~(V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |V4L2_CTRL_FLAG_READ_ONLY);
	priv->focal_length_non_block->flags &= ~(V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |V4L2_CTRL_FLAG_READ_ONLY);
	if (priv->motor_zoom){
		priv->focal_length->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		priv->focal_length_non_block->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	}
	else {
		priv->focal_length->flags |= V4L2_CTRL_FLAG_READ_ONLY;
		priv->focal_length_non_block->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (priv->n_steps){
		min = priv->steps[0].focal_length_middle *1000;
		max = priv->steps[priv->n_steps-1].focal_length_middle *1000;
		if (min>max){
			int aux = min;
			min = max;
			max = aux;
		}
		__v4l2_ctrl_s_ctrl(priv->focal_length,  min);
		__v4l2_ctrl_s_ctrl(priv->focal_length_non_block,  min);
		__v4l2_ctrl_modify_range_cond(priv->focal_length,min,max,1000,min);
		__v4l2_ctrl_modify_range_cond(priv->focal_length_non_block,min,max,1000,min);
	}

	v4l2_ctrl_activate(priv->lens_name, true);
	v4l2_ctrl_activate(priv->lens_version, true);
	v4l2_ctrl_activate(priv->focus, true);
	v4l2_ctrl_activate(priv->focal_length, true);
	v4l2_ctrl_activate(priv->focal_length_non_block, true);

	update_aperture(priv);
	v4l2_ctrl_activate(priv->aperture, true);

	return 0;
}

static int m43_thread(void *data)
{
	struct qtec_m43 *priv = data;
	int last_zoom = -1;
	static uint8_t buffer[256];
	int ret;
	int retries = 0;
	int lens_stop = 0;
	int app,focus;

	do{
		if (kthread_should_stop())
			break;

		if (!v4l2_ctrl_g_ctrl(priv->lens_active)){
			last_zoom = -1;
			retries  = 0;
			continue;
		}

		if (priv->focal_length_val != -1) {
			if (priv->focal_length->cur.val != priv->focal_length_val)
				__v4l2_ctrl_s_ctrl(priv->focal_length, priv->focal_length_val);
			priv->focal_length_val = -1;
		}

		//Poll lens status
		if (retries > 5) {
			v4l2_err(&priv->sd, "Too many retries while polling m43, stopping lens\n");
			__v4l2_ctrl_s_ctrl(priv->lens_active, false);
			m43_stoplens(priv);
			retries  = 0;
			last_zoom = -1;
			continue;
		}


		if (priv->lens_vendor == 0x2) //LUMIX
			ret = m43_cmd(priv, 0x81, 0xe3, 0x4, 0xb, buffer,sizeof(buffer));
		else
			ret = m43_cmd(priv, 0xc1, 0x80, 0x4, 0xb, buffer,sizeof(buffer));

		if (ret<5){
			retries ++;
			continue;
		}

		if (buffer[4] >= priv->n_steps){
			v4l2_err(&priv->sd, "Invalid step %d. Max step is %d. Retrying\n",buffer[4],priv->n_steps-1);
			retries ++;
			continue;
		}
		retries =0;


		//is lens moving?
		if (priv->motor_zoom && buffer[1]&0xf){
			lens_stop = 0;
			continue;
		}

		if (lens_stop < 4){
			lens_stop ++;
			continue;
		}
		//Poll aperture
		update_aperture(priv);

		//Poll focus
		if (m43_cmd(priv, 0xb0,0xad,0,0,buffer,sizeof(buffer))!=2)
			continue;
		focus = buffer[0] + (buffer[1]<<8);
		if (buffer[4]==last_zoom || buffer[4]>=priv->n_steps){
			if (priv->focus->cur.val != focus)
				v4l2_ctrl_s_ctrl(priv->focus, focus);
			continue;
		}

		last_zoom = buffer[4];
		/*if (priv->focal_length->cur.val != priv->steps[last_zoom].focal_length_middle*1000 )
			v4l2_ctrl_s_ctrl(priv->focal_length,  priv->steps[last_zoom].focal_length_middle*1000);
		if (priv->focal_length_non_block->cur.val != priv->steps[last_zoom].focal_length_middle*1000 )
			v4l2_ctrl_s_ctrl(priv->focal_length_non_block,  priv->steps[last_zoom].focal_length_middle*1000);*/
		app = priv->aperture->cur.val;
		app = clamp_t(int,app, priv->steps[last_zoom].aperture_min, priv->steps[last_zoom].aperture_max);
		v4l2_ctrl_modify_range_cond(priv->aperture,priv->steps[last_zoom].aperture_min,priv->steps[last_zoom].aperture_max,1,app);
		dev_dbg(&priv->client->dev, "%2d app\\%.5d app/%.5d focus:0x%.4x focal_length:0x%.4x\n",last_zoom,priv->steps[last_zoom].aperture_min, priv->steps[last_zoom].aperture_max, priv->steps[last_zoom].focus_min,priv->steps[last_zoom].focal_length_middle);
	} while(!wait_event_timeout(priv->wq, kthread_should_stop(), HZ/4));

	return 0;
}

/* device ID table */
static const struct i2c_device_id m43_id[] = {
	{ "qtec_m43", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m43_id);

#define FW_VER 0x4301
static int m43_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	s32 id=0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BLOCK_DATA |
				I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA  |
				I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	id = i2c_smbus_read_word_data(client,VERSION);
	if (id != FW_VER){
		dev_warn(&adapter->dev,
			"Error loading %s at %d, 0x%02x: Invalid M43 version 0x%.4x, expected 0x%.4x. Please update\n",
			client->name, i2c_adapter_id(client->adapter),
			client->addr,id,FW_VER);
		return -ENODEV;
	}

	if (info)
		strlcpy(info->type, m43_id[0].name, I2C_NAME_SIZE);

	return 0;
}

static const struct v4l2_subdev_core_ops core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
};

static int qtec_m43_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct qtec_m43 *priv = container_of(ctrl->handler, struct qtec_m43, ctrl_handler);
	static uint8_t buffer[2];
	int aux;

	if (ctrl->val == ctrl->cur.val)
		return 0;

	switch (ctrl->id){
	case QTEC_M43_CID_LENS_ACTIVE:
		if (ctrl->val)
			return m43_startuplens(priv);
		m43_stoplens(priv);
		return 0;
	case QTEC_M43_CID_FOCUS:
		aux = ctrl->val;
		aux = m43_cmd(priv, 0xb0,0xa2,aux,aux>>8,buffer,sizeof(buffer));
		dev_dbg(&priv->client->dev, "Focus %.4d = %.4d\n",ctrl->val, buffer[0] + (buffer[1]<<8));
		ctrl->val = buffer[0] + (buffer[1]<<8);
		return (aux!=2)?-EIO:0;
	case QTEC_M43_CID_FOCAL_LENGTH_NON_BLOCKING:
		priv->focal_length_val = ctrl->val;
		return 0;
	case QTEC_M43_CID_FOCAL_LENGTH:
		aux = m43_focal_length(priv,ctrl->val);
		__v4l2_ctrl_s_ctrl(priv->focal_length_non_block,  ctrl->val);
		return aux;
	case QTEC_M43_CID_APERTURE:
		aux = ctrl->val;
		aux *= 256;
		aux /= 1000;
		if (m43_cmd(priv, 0xb0,0xd1,aux,aux>>8,buffer,2)!=2)
			return -EIO;
		aux = buffer[0] + (buffer[1]<<8);
		aux *= 1000;
		aux /= 256;
		ctrl->val = aux;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops qtec_m43_ctrl_ops = {
	.s_ctrl = qtec_m43_s_ctrl,
};


static struct v4l2_ctrl *add_custom_control_text(struct qtec_m43 *priv,char *name, int id, int len){
	static struct v4l2_ctrl_config ctrl = {
		.type = V4L2_CTRL_TYPE_STRING,
		.step = 1,
		.min = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE,
	};
	ctrl.name = name;
	ctrl.id = id;
	ctrl.max = len;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *add_custom_control_val_max(struct qtec_m43 *priv, char *name, int id, int flags, uint32_t max){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_m43_ctrl_ops,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.step = 1,
		.min = 0,
		.def = 0,
	};
	ctrl.flags = flags  |V4L2_CTRL_FLAG_INACTIVE ;
	ctrl.name = name;
	ctrl.id = id;
	ctrl.max = max;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static struct v4l2_ctrl *add_custom_control_val(struct qtec_m43 *priv, char *name, int id, int flags){

	return add_custom_control_val_max(priv, name, id, flags, 0x7fffffff);
}

static struct v4l2_ctrl *add_custom_control_bool(struct qtec_m43 *priv, char *name, int id){
	static struct v4l2_ctrl_config ctrl = {
		.ops = &qtec_m43_ctrl_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.step = 1,
		.min = 0,
		.max = 1,
		.def = 0,
	};
	ctrl.name = name;
	ctrl.id = id;

	return v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl, NULL);
}

static atomic_t instance = ATOMIC_INIT(0);
static int m43_probe(struct i2c_client *client,
		      const struct i2c_device_id *id)
{
	struct qtec_m43 *priv;

	if (m43_detect(client,NULL))
		return -ENODEV;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(priv->client, priv);
	init_waitqueue_head(&priv->wq);
	mutex_init(&priv->lock);
	priv->focal_length_val = -1;

	//v4l2 subdev
	v4l2_subdev_init(&priv->sd, &subdev_ops);
	snprintf(priv->sd.name,sizeof(priv->sd.name),"%s-%d",DRIVER_NAME,atomic_inc_return(&instance)-1);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE ; //Allow independent access
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS ;

	//controls
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 8);
	priv->lens_active = add_custom_control_bool(priv,"Lens Active",QTEC_M43_CID_LENS_ACTIVE);
	priv->lens_name = add_custom_control_text(priv,"Lens Name",QTEC_M43_CID_LENS_NAME,33);
	priv->lens_version = add_custom_control_text(priv,"Lens Version",QTEC_M43_CID_LENS_VERSION,7);
	priv->focus = add_custom_control_val_max(priv, "Focus", QTEC_M43_CID_FOCUS, 0, 0x7fff);
	priv->focal_length = add_custom_control_val(priv, "Focal Length", QTEC_M43_CID_FOCAL_LENGTH, 0);
	priv->focal_length_non_block = add_custom_control_val(priv, "Focal Length Non Blocking", QTEC_M43_CID_FOCAL_LENGTH_NON_BLOCKING, 0);
	priv->aperture = add_custom_control_val(priv, "Aperture", QTEC_M43_CID_APERTURE, V4L2_CTRL_FLAG_SLIDER);
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error creating controls\n");
		return priv->ctrl_handler.error;
	}
	priv->sd.ctrl_handler = &priv->ctrl_handler;
	priv->sd.owner = THIS_MODULE;

	__v4l2_ctrl_s_ctrl(priv->lens_active, true);

	priv->kthread = kthread_run(m43_thread,priv,priv->sd.name);
	if (IS_ERR(priv->kthread))
		dev_err(&client->dev, "Error starting m43 thread. Continuing synchronously with limited functionality\n");

	v4l2_info(&priv->sd, "qtec_m43 subdevice registered as %s, @ %d:0x%02x\n", priv->sd.name,
			i2c_adapter_id(client->adapter), client->addr);

	return 0;
}

static int m43_remove(struct i2c_client *client)
{
	struct qtec_m43 *priv = i2c_get_clientdata(client);

	__v4l2_ctrl_s_ctrl(priv->lens_active, false);
	kthread_stop(priv->kthread);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return 0;
}

static const unsigned short m43_addrs[] = { (0x74), I2C_CLIENT_END };

static struct i2c_driver qtec_m43_i2c_driver = {
	.driver.name  = DRIVER_NAME,
	.probe        = m43_probe,
	.remove       = m43_remove,
	.id_table     = m43_id,
	.detect       = m43_detect,
	.address_list = m43_addrs,
};

module_i2c_driver(qtec_m43_i2c_driver);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("Qtechnology micro 4/3 controller");
MODULE_LICENSE("GPL");
