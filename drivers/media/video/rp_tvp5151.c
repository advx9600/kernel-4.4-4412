/*
 * driver for Fusitju RPDZKJ LS 8MP camera
 *
 * Copyright (c) 2010, Samsung Electronics. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <media/v4l2-device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include <linux/videodev2.h>
#include <linux/io.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include <linux/regulator/machine.h>

#include <media/tvp5150_platform.h>

#include "rp_tvp5151.h"


#include <mach/gpio.h>
#include <plat/gpio-cfg.h>


#define CONTINUOUS_FOCUS
#define RPTVP5150_DRIVER_NAME	"rptvp5150"
#define SDCARD_FW

#define RPTVP5150_I2C_RETRY		5

#define RPTVP5150_JPEG_MAXSIZE	0x3A0000
#define RPTVP5150_THUMB_MAXSIZE	0xFC00
#define RPTVP5150_POST_MAXSIZE	0xBB800


#define CHECK_ERR(x)	if ((x) < 0) { \
				cam_err("i2c failed, err %d\n", x); \
				return x; \
			}


static struct i2c_client *gclient;


#ifdef CONTINUOUS_FOCUS
static int g_focus_mode=0;
#endif

static const struct rptvp5150_frmsizeenum preview_frmsizes[] = {
	{ RPTVP5150_PREVIEW_QCIF,	176,	144,	0x05 },	/* 176 x 144 */
	{ RPTVP5150_PREVIEW_QCIF2,	528,	432,	0x2C },	/* 176 x 144 */
	{ RPTVP5150_PREVIEW_QVGA,	320,	240,	0x09 },
	{ RPTVP5150_PREVIEW_VGA,	640,	480,	0x17 },
	//{ RPTVP5150_PREVIEW_D1,	720,	480,	0x18 },
	//{ RPTVP5150_PREVIEW_WVGA,	800,	480,	0x1A },
	//{ RPTVP5150_PREVIEW_720P,	1280,	720,	0x21 },
	//{ RPTVP5150_PREVIEW_1080P,	1920,	1080,	0x28 },
	//{ RPTVP5150_PREVIEW_HDR,	3264,	2448,	0x27 },
};

static const struct rptvp5150_frmsizeenum capture_frmsizes[] = {
	{ RPTVP5150_CAPTURE_VGA,	640,	480,	0x09 },
#if 0
	{ RPTVP5150_CAPTURE_WVGA,	800,	480,	0x0A },
	{ RPTVP5150_CAPTURE_W1MP,	1600,	960,	0x2C },
	{ RPTVP5150_CAPTURE_2MP,	1600,	1200,	0x2C },
	{ RPTVP5150_CAPTURE_W2MP,	2048,	1232,	0x2C },
	{ RPTVP5150_CAPTURE_3MP,	2048,	1536,	0x1B },
	{ RPTVP5150_CAPTURE_W4MP,	2560,	1536,	0x1B },
	{ RPTVP5150_CAPTURE_5MP,	2592,	1944,	0x1B },
	{ RPTVP5150_CAPTURE_W6MP,	3072,	1856,	0x1B },	
	{ RPTVP5150_CAPTURE_7MP,	3072,	2304,	0x2D },
	{ RPTVP5150_CAPTURE_W7MP,	2560,	1536,	0x2D },
	{ RPTVP5150_CAPTURE_8MP,	3264,	2448,	0x25 },
#endif

};

static struct rptvp5150_control rptvp5150[] = {
	{
		.id = V4L2_CID_CAMERA_ISO,
		.minimum = ISO_AUTO,
		.maximum = ISO_800,
		.step = 1,
		.value = ISO_AUTO,
		.default_value = ISO_AUTO,
	}, {
		.id = V4L2_CID_CAMERA_BRIGHTNESS,
		.minimum = EV_MINUS_4,
		.maximum = EV_MAX - 1,
		.step = 1,
		.value = EV_DEFAULT,
		.default_value = EV_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_SATURATION,
		.minimum = SATURATION_MINUS_2,
		.maximum = SATURATION_MAX - 1,
		.step = 1,
		.value = SATURATION_DEFAULT,
		.default_value = SATURATION_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_SHARPNESS,
		.minimum = SHARPNESS_MINUS_2,
		.maximum = SHARPNESS_MAX - 1,
		.step = 1,
		.value = SHARPNESS_DEFAULT,
		.default_value = SHARPNESS_DEFAULT,
	}, {
		.id = V4L2_CID_CAMERA_ZOOM,
		.minimum = ZOOM_LEVEL_0,
		.maximum = ZOOM_LEVEL_MAX - 1,
		.step = 1,
		.value = ZOOM_LEVEL_0,
		.default_value = ZOOM_LEVEL_0,
	}, {
		.id = V4L2_CID_CAM_JPEG_QUALITY,
		.minimum = 1,
		.maximum = 100,
		.step = 1,
		.value = 100,
		.default_value = 100,
	},
};

static inline struct rptvp5150_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rptvp5150_state, sd);
}

static int rptvp5150_i2c_read(struct v4l2_subdev *sd, unsigned char reg)
{

	return i2c_smbus_read_byte_data(gclient, reg);
}



static int rptvp5150_i2c_write(struct v4l2_subdev *sd, unsigned char i2c_data[],
				unsigned char length)
{
#if 0
	int ret;
	ret = i2c_smbus_write_byte_data(gclient, i2c_data[0], i2c_data[1]);
	return ret;
#else
	struct i2c_msg msg = {gclient->addr, 0, length, i2c_data};
	return i2c_transfer(gclient->adapter, &msg, 1) == 1 ? 0 : -EIO; 
#endif

}



static int rptvp5150_set_mode(struct v4l2_subdev *sd, u32 mode)
{
	int i, err;
	u32 old_mode, val;
	cam_trace("E\n");
	
	switch (/*old_mode*/1) {
	case RPTVP5150_SYSINIT_MODE:
		cam_warn("sensor is initializing\n");
		err = -EBUSY;
		break;

	default:
		err = -EINVAL;
	}

	cam_trace("X\n");
	return 0;
}

/*
 * v4l2_subdev_core_ops
 */
 
static int rptvp5150_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rptvp5150); i++) {
		if (qc->id == rptvp5150[i].id) {
			qc->maximum = rptvp5150[i].maximum;
			qc->minimum = rptvp5150[i].minimum;
			qc->step = rptvp5150[i].step;
			qc->default_value = rptvp5150[i].default_value;
			return 0;
		}
	}

	return -EINVAL;
}

static int rptvp5150_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct rptvp5150_state *state = to_state(sd);
	int err = 0;
	#ifdef CONTINUOUS_FOCUS
	int data,af_pos_h,af_pos_l;
	#endif
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
			ctrl->value = 2;
		
		break;

	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = RPTVP5150_JPEG_MAXSIZE +
			RPTVP5150_THUMB_MAXSIZE + RPTVP5150_POST_MAXSIZE;
		break;

	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
		ctrl->value = state->jpeg.main_size;
		break;

	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
		ctrl->value = state->jpeg.main_offset;
		break;

	case V4L2_CID_CAM_JPEG_THUMB_SIZE:
		ctrl->value = state->jpeg.thumb_size;
		break;

	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
		ctrl->value = state->jpeg.thumb_offset;
		break;

	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
		ctrl->value = state->jpeg.postview_offset;
		break;

	case V4L2_CID_CAMERA_EXIF_FLASH:
		ctrl->value = state->exif.flash;
		break;

	case V4L2_CID_CAMERA_EXIF_ISO:
		ctrl->value = state->exif.iso;
		break;

	case V4L2_CID_CAMERA_EXIF_TV:
		ctrl->value = state->exif.tv;
		break;

	case V4L2_CID_CAMERA_EXIF_BV:
		ctrl->value = state->exif.bv;
		break;

	case V4L2_CID_CAMERA_EXIF_EBV:
		ctrl->value = state->exif.ebv;
		break;	

	case V4L2_CID_CAMERA_MODEL:
		break;

	default:
		cam_err("no such control id %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE);
		/*err = -ENOIOCTLCMD*/
		err = 0;
		break;
	}

	if (err < 0 && err != -ENOIOCTLCMD)
		cam_err("failed, id %d\n", ctrl->id - V4L2_CID_PRIVATE_BASE);

	return err;
}


static int rptvp5150_set_exposure(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int err, i;
	int num;
	int val = ctrl->value;
	gclient = v4l2_get_subdevdata(sd);

	return 0;
}

static int rptvp5150_set_whitebalance(struct v4l2_subdev *sd, int val)
{
	int err, i;
	int num;
	unsigned char ctrlreg[1][2];
	gclient = v4l2_get_subdevdata(sd);

	return 0;
}

static int rptvp5150_set_sharpness(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl qc = {0,};
	int val = ctrl->value, err;
	u32 sharpness[] = {0x03, 0x04, 0x05, 0x06, 0x07};
	cam_dbg("E, value %d\n", val);

	qc.id = ctrl->id;
	rptvp5150_queryctrl(sd, &qc);

	if (val < qc.minimum || val > qc.maximum) {
		cam_warn("invalied value, %d\n", val);
		val = qc.default_value;
	}

	val -= qc.minimum;


	cam_trace("X\n");
	return 0;
}

static int rptvp5150_set_saturation(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl qc = {0,};
	int val = ctrl->value, err;
	u32 saturation[] = {0x01, 0x02, 0x03, 0x04, 0x05};
	cam_dbg("E, value %d\n", val);

	qc.id = ctrl->id;
	rptvp5150_queryctrl(sd, &qc);

	if (val < qc.minimum || val > qc.maximum) {
		cam_warn("invalied value, %d\n", val);
		val = qc.default_value;
	}

	val -= qc.minimum;


	cam_trace("X\n");
	return 0;
}



static int rptvp5150_set_effect(struct v4l2_subdev *sd, int val)
{
	int err;
	cam_dbg("E, value %d\n", val);

retry:
	switch (val) {
	case IMAGE_EFFECT_NONE:
	case IMAGE_EFFECT_BNW:
	case IMAGE_EFFECT_SEPIA:
		break;

	case IMAGE_EFFECT_AQUA:
	case IMAGE_EFFECT_NEGATIVE:
		break;

	default:
		cam_warn("invalid value, %d\n", val);
		val = IMAGE_EFFECT_NONE;
		goto retry;
	}

	cam_trace("X\n");
	return 0;
}



static int rptvp5150_get_exif(struct v4l2_subdev *sd)
{
	struct rptvp5150_state *state = to_state(sd);
	/* standard values */
	u16 iso_std_values[] = { 10, 12, 16, 20, 25, 32, 40, 50, 64, 80,
		100, 125, 160, 200, 250, 320, 400, 500, 640, 800,
		1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000};
	/* quantization table */
	u16 iso_qtable[] = { 11, 14, 17, 22, 28, 35, 44, 56, 71, 89,
		112, 141, 178, 224, 282, 356, 449, 565, 712, 890,
		1122, 1414, 1782, 2245, 2828, 3564, 4490, 5657, 7127, 8909};
	int num, den, i, err;


	return 0;
}

static int rptvp5150_start_capture(struct v4l2_subdev *sd, int val)
{
	struct rptvp5150_state *state = to_state(sd);
	int err, int_factor;
	cam_trace("E\n");


	//state->jpeg.main_size = 200;		
	state->jpeg.main_offset = 0;
	state->jpeg.thumb_offset = RPTVP5150_JPEG_MAXSIZE;
	state->jpeg.postview_offset = RPTVP5150_JPEG_MAXSIZE + RPTVP5150_THUMB_MAXSIZE;

	rptvp5150_get_exif(sd);

	cam_trace("X\n");
	return /*err*/0;
}


static int rptvp5150_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct rptvp5150_state *state = to_state(sd);
	int err = 0;

	printk(KERN_INFO "id %d [%d], value %d\n",
		ctrl->id - V4L2_CID_PRIVATE_BASE,ctrl->id, ctrl->value);

	if (unlikely(state->isp.bad_fw && ctrl->id != V4L2_CID_CAM_UPDATE_FW)) {
		cam_err("\"Unknown\" state, please update F/W");
		return -ENOSYS;
	}

	switch (ctrl->id) {
	case V4L2_CID_CAM_UPDATE_FW:
		break;

	case V4L2_CID_CAMERA_SENSOR_MODE:
		break;

	case V4L2_CID_CAMERA_FLASH_MODE:
		break;

	case V4L2_CID_CAMERA_ISO:
		break;

	case V4L2_CID_CAMERA_METERING:
		break;

	case V4L2_CID_CAMERA_BRIGHTNESS:
		err = rptvp5150_set_exposure(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
		err = rptvp5150_set_whitebalance(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_SCENE_MODE:
		break;

	case V4L2_CID_CAMERA_EFFECT:
		err = rptvp5150_set_effect(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_WDR:
		break;

	case V4L2_CID_CAMERA_ANTI_SHAKE:
		break;

	case V4L2_CID_CAMERA_BEAUTY_SHOT:
		break;

	case V4L2_CID_CAMERA_FOCUS_MODE:
		break;

	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_X:
		state->focus.pos_x = ctrl->value;
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
		state->focus.pos_y = ctrl->value;
		break;

	case V4L2_CID_CAMERA_TOUCH_AF_START_STOP:
		break;

	case V4L2_CID_CAMERA_ZOOM:
		break;

	case V4L2_CID_CAM_JPEG_QUALITY:
		break;

	case V4L2_CID_CAMERA_CAPTURE:
		err = rptvp5150_start_capture(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_HDR:
		break;

	case V4L2_CID_CAMERA_VT_MODE:
		state->vt_mode = ctrl->value;
		break;

	case V4L2_CID_CAMERA_CHECK_DATALINE:
		break;

	case V4L2_CID_CAMERA_CHECK_ESD:
		break;

	default:
		cam_err("no such control id %d, value %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);
		/*err = -ENOIOCTLCMD;*/
		err = 0;
		break;
	}

	if (err < 0 && err != -ENOIOCTLCMD)
		cam_err("failed, id %d, value %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);
	return err;
}

static int rptvp5150_g_ext_ctrl(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	struct rptvp5150_state *state = to_state(sd);
	int err = 0;
	printk("********[rptvp5150_g_ext_ctrl][ctrl->id=%d] [ctrl->value=%d]\n",ctrl->id,ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_CAM_SENSOR_FW_VER:
		strcpy(ctrl->string, state->exif.unique_id);
		break;

	default:
		cam_err("no such control id %d\n", ctrl->id - V4L2_CID_CAMERA_CLASS_BASE);
		/*err = -ENOIOCTLCMD*/
		err = 0;
		break;
	}

	if (err < 0 && err != -ENOIOCTLCMD)
		cam_err("failed, id %d\n", ctrl->id - V4L2_CID_CAMERA_CLASS_BASE);

	return err;
}

static int rptvp5150_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i, err = 0;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		err = rptvp5150_g_ext_ctrl(sd, ctrl);
		printk("########[rptvp5150_g_ext_ctrls][count=%d][ctrl->value=%d]\n",ctrls->count,ctrl->value);
		if (err) {
			ctrls->error_idx = i;
			break;
		}
	}
	return err;
}



static int rptvp5150_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->adapter->dev;
	int  err=0;

	return err;
}

/*
 * v4l2_subdev_video_ops
 */
static const struct rptvp5150_frmsizeenum *rptvp5150_get_frmsize
	(const struct rptvp5150_frmsizeenum *frmsizes, int num_entries, int index)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (frmsizes[i].index == index)
			return &frmsizes[i];
	}

	return NULL;
}


static int rptvp5150_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *ffmt)
{
	return 0;
}

static int rptvp5150_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct rptvp5150_state *state = to_state(sd);

	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = state->fps;

	return 0;
}

static int rptvp5150_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct rptvp5150_state *state = to_state(sd);
	int err;

	u32 fps = a->parm.capture.timeperframe.denominator /
					a->parm.capture.timeperframe.numerator;

	if (unlikely(state->isp.bad_fw)) {
		cam_err("\"Unknown\" state, please update F/W");
		return -ENOSYS;
	}

	if (fps != state->fps) {
		if (fps <= 0 || fps > 30) {
			cam_err("invalid frame rate %d\n", fps);
			fps = 30;
		}
		state->fps = fps;
	}

	err = rptvp5150_set_mode(sd, RPTVP5150_PARMSET_MODE);
	CHECK_ERR(err);

	cam_dbg("fixed fps %d\n", state->fps);

	return 0;
}

static int rptvp5150_enum_framesizes(struct v4l2_subdev *sd,
	struct v4l2_frmsizeenum *fsize)
{

	struct rptvp5150_state *state = to_state(sd);
	u32 err, old_mode;
	printk("%s\n",__func__);

	/*
	* The camera interface should read this value, this is the resolution
	* at which the sensor would provide framedata to the camera i/f
	* In case of image capture,
	* this returns the default camera resolution (VGA)
	*/
//	fsize->discrete.width = 720;	
//	fsize->discrete.height = 240;

	return 0;
}

static int rptvp5150_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rptvp5150_state *state = to_state(sd);
	int err;
	printk("=================>rptvp5150_s_stream [enable=%d] [state->format_mode=%d]\n",enable,state->format_mode);
	cam_trace("E\n");

	if (unlikely(state->isp.bad_fw)) {
		cam_err("\"Unknown\" state, please update F/W");
		return -ENOSYS;
	}

	switch (enable) {
	case STREAM_MODE_CAM_ON:
	case STREAM_MODE_CAM_OFF:
		switch (state->format_mode) {
		case V4L2_PIX_FMT_MODE_CAPTURE:
			break;
		case V4L2_PIX_FMT_MODE_HDR:
			break;
		default:
			break;
		}
		break;
 
	case STREAM_MODE_MOVIE_ON:
		break;

	case STREAM_MODE_MOVIE_OFF:
		break;

	default:
		cam_err("invalid stream option, %d\n", enable);
		break;
	}

	cam_trace("X\n");
	return 0;
}



static int rptvp5150_init(struct v4l2_subdev *sd, u32 val)
{

	struct rptvp5150_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 int_factor;
	int err;
	int i;
	int m;

	//takepicture exceptionally foce quit
	struct v4l2_frmsizeenum cam_frmsize;
	cam_frmsize.discrete.width = 640;
	cam_frmsize.discrete.height = 480;	

	printk("*******%s**********\n",__func__);

	
	s3c_gpio_cfgall_range(EXYNOS4_GPD0(2), 2, S3C_GPIO_SFN(3), S3C_GPIO_PULL_UP);
	
	gpio_direction_output(EXYNOS4212_GPM1(1), 1);

	gpio_direction_output(EXYNOS4212_GPM3(0), 1);
	mdelay(100);
	gpio_direction_output(EXYNOS4_GPL0(1), 1); 
	mdelay(100);//50

	
	gpio_direction_output(EXYNOS4212_GPJ1(4), 0);	
	mdelay(20);//30
	gpio_direction_output(EXYNOS4212_GPJ1(4), 1);
	mdelay(50);//100

	
	gclient = v4l2_get_subdevdata(sd);

	/* Default state values */
	state->isp.bad_fw = 0;

	state->preview = NULL;
	state->capture = NULL;

	state->format_mode = V4L2_PIX_FMT_MODE_PREVIEW;
	state->sensor_mode = SENSOR_CAMERA;
	state->flash_mode = FLASH_MODE_OFF;
	state->beauty_mode = 0;

	state->fps = 0;			/* auto */

	memset(&state->focus, 0, sizeof(state->focus));
	
	printk("*************************************************************%s+++++[value=%d]\n",__func__,val);

	for (m = 0; m < 1; m++)
		{
	for (i = 0; i < rptvp5150_INIT_REGS; i++) {
		printk("*");
		err = rptvp5150_i2c_write(sd, rptvp5150_init_reg[i],
				sizeof(rptvp5150_init_reg[i]));
		if (err < 0)
			v4l_info(client, "%s: %d register set failed\n",
					__func__, i);
		msleep(10);
	}
		}

	return 0;
}

static const struct v4l2_subdev_core_ops rptvp5150_core_ops = {
	.init = rptvp5150_init,		/* initializing API */
	.load_fw = rptvp5150_load_fw,
	.queryctrl = rptvp5150_queryctrl,
	.g_ctrl = rptvp5150_g_ctrl,
	.s_ctrl = rptvp5150_s_ctrl,
	.g_ext_ctrls = rptvp5150_g_ext_ctrls,
};

static const struct v4l2_subdev_video_ops rptvp5150_video_ops = {
	.s_mbus_fmt = rptvp5150_s_fmt,
	.g_parm = rptvp5150_g_parm,
	.s_parm = rptvp5150_s_parm,
	.enum_framesizes = rptvp5150_enum_framesizes,
	.s_stream =rptvp5150_s_stream,
};

static const struct v4l2_subdev_ops rptvp5150_ops = {
	 .core = &rptvp5150_core_ops,
	.video = &rptvp5150_video_ops,
};


static int __devinit rptvp5150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct rptvp5150_state *state;
	struct v4l2_subdev *sd;
	int err = 0;
	unsigned int * base;

	printk("*******%s**********\n",__func__);

	state = kzalloc(sizeof(struct rptvp5150_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, RPTVP5150_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &rptvp5150_ops);

#ifdef CAM_DEBUG
	state->dbg_level = CAM_DEBUG; /*| CAM_TRACE | CAM_I2C;*/
#endif


	return 0;
}

static int __devexit rptvp5150_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct rptvp5150_state *state = to_state(sd);

	if (state->isp.irq > 0)
		free_irq(state->isp.irq, sd);

	v4l2_device_unregister_subdev(sd);

	kfree(state->fw_version);
	kfree(state);

	return 0;
}

static const struct i2c_device_id rptvp5150_id[] = {
	{ RPTVP5150_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rptvp5150_id);

static struct i2c_driver rptvp5150_i2c_driver = {
	.driver = {
		.name	= RPTVP5150_DRIVER_NAME,
	},
	.probe		= rptvp5150_probe,
	.remove		= __devexit_p(rptvp5150_remove),
	.id_table	= rptvp5150_id,
};

static int __init rptvp5150_mod_init(void)
{

	printk("*******%s**********\n",__func__);
	return i2c_add_driver(&rptvp5150_i2c_driver);
}

static void __exit rptvp5150_mod_exit(void)
{
	i2c_del_driver(&rptvp5150_i2c_driver);
}
module_init(rptvp5150_mod_init);
module_exit(rptvp5150_mod_exit);


MODULE_AUTHOR("Goeun Lee <ge.lee@samsung.com>");
MODULE_DESCRIPTION("driver for Fusitju TVP5150 LS 8MP camera");
MODULE_LICENSE("GPL");
