// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Intel Corporation.

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <uapi/linux/v4l2-controls.h>
#include <uapi/linux/videodev2.h>

#include "ov5693.h"

static const uint32_t ov5693_embedded_effective_size = 28;

/* i2c read/write stuff */
static int ov5693_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != OV5693_8BIT && data_length != OV5693_16BIT
					&& data_length != OV5693_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == OV5693_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV5693_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int ov5693_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int ov5693_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV5693_8BIT && data_length != OV5693_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV5693_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV5693_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov5693_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov5693_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_v, reg_v2;
	int ret;

	/* get exposure */
	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_EXPOSURE_L,
					&reg_v);
	if (ret)
		goto err;

	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_EXPOSURE_M,
					&reg_v2);
	if (ret)
		goto err;

	reg_v += reg_v2 << 8;
	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_EXPOSURE_H,
					&reg_v2);
	if (ret)
		goto err;

	*value = reg_v + (((u32)reg_v2 << 16));
err:
	return ret;
}

static int ov5693_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s: FOCUS_POS: 0x%x\n", __func__, value);
	value = clamp(value, 0, OV5693_VCM_MAX_FOCUS_POS);
	if (dev->vcm == VCM_DW9714) {
		/*if (dev->vcm_update) {*/
			/*ret = vcm_dw_i2c_write(client, VCM_PROTECTION_OFF);*/
			/*if (ret)*/
				/*return ret;*/
			/*ret = vcm_dw_i2c_write(client, DIRECT_VCM);*/
			/*if (ret)*/
				/*return ret;*/
			/*ret = vcm_dw_i2c_write(client, VCM_PROTECTION_ON);*/
			/*if (ret)*/
				/*return ret;*/
			/*dev->vcm_update = false;*/
		/*}*/
		/*ret = vcm_dw_i2c_write(client,*/
					   /*vcm_val(value, VCM_DEFAULT_S));*/
	} else if (dev->vcm == VCM_AD5823) {
		/*ad5823_t_focus_abs(sd, value);*/
	}
	if (ret == 0) {
		dev->number_of_steps = value - dev->focus;
		dev->focus = value;
		// TODO
		/*getnstimeofday(&(dev->timestamp_t_focus_abs));*/
	} else
		dev_err(&client->dev,
			"%s: i2c failed. ret %d\n", __func__, ret);

	return ret;
}

static int ov5693_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	return ov5693_t_focus_abs(sd, dev->focus + value);
}

static int ov5693_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5693_device *dev =
	    container_of(ctrl->handler, struct ov5693_device, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		dev_dbg(&client->dev, "%s: CID_FOCUS_ABSOLUTE:%d.\n",
			__func__, ctrl->val);
		ret = ov5693_t_focus_abs(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_FOCUS_RELATIVE:
		dev_dbg(&client->dev, "%s: CID_FOCUS_RELATIVE:%d.\n",
			__func__, ctrl->val);
		ret = ov5693_t_focus_rel(&dev->sd, ctrl->val);
		break;
	/*case V4L2_CID_VCM_SLEW:*/
		/*ret = ov5693_t_vcm_slew(&dev->sd, ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_VCM_TIMEING:*/
		/*ret = ov5693_t_vcm_timing(&dev->sd, ctrl->val);*/
		/*break;*/
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ov5693_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5693_device *dev =
	    container_of(ctrl->handler, struct ov5693_device, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		ret = ov5693_q_exposure(&dev->sd, &ctrl->val);
		break;
	/*case V4L2_CID_FOCAL_ABSOLUTE:*/
		/*ret = ov5693_g_focal(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_FNUMBER_ABSOLUTE:*/
		/*ret = ov5693_g_fnumber(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_FNUMBER_RANGE:*/
		/*ret = ov5693_g_fnumber_range(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_FOCUS_ABSOLUTE:*/
		/*ret = ov5693_q_focus_abs(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_FOCUS_STATUS:*/
		/*ret = ov5693_q_focus_status(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_BIN_FACTOR_HORZ:*/
		/*ret = ov5693_g_bin_factor_x(&dev->sd, &ctrl->val);*/
		/*break;*/
	/*case V4L2_CID_BIN_FACTOR_VERT:*/
		/*ret = ov5693_g_bin_factor_y(&dev->sd, &ctrl->val);*/
		/*break;*/
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = ov5693_s_ctrl,
	.g_volatile_ctrl = ov5693_g_volatile_ctrl
};

struct v4l2_ctrl_config ov5693_controls[] = {
	{
	 .ops = &ctrl_ops,
	 .id = V4L2_CID_EXPOSURE_ABSOLUTE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "exposure",
	 .min = 0x0,
	 .max = 0xffff,
	 .step = 0x01,
	 .def = 0x00,
	 .flags = 0,
	 },
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_FOCAL_ABSOLUTE,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "focal length",*/
	 /*.min = OV5693_FOCAL_LENGTH_DEFAULT,*/
	 /*.max = OV5693_FOCAL_LENGTH_DEFAULT,*/
	 /*.step = 0x01,*/
	 /*.def = OV5693_FOCAL_LENGTH_DEFAULT,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_FNUMBER_ABSOLUTE,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "f-number",*/
	 /*.min = OV5693_F_NUMBER_DEFAULT,*/
	 /*.max = OV5693_F_NUMBER_DEFAULT,*/
	 /*.step = 0x01,*/
	 /*.def = OV5693_F_NUMBER_DEFAULT,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_FNUMBER_RANGE,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "f-number range",*/
	 /*.min = OV5693_F_NUMBER_RANGE,*/
	 /*.max = OV5693_F_NUMBER_RANGE,*/
	 /*.step = 0x01,*/
	 /*.def = OV5693_F_NUMBER_RANGE,*/
	 /*.flags = 0,*/
	 /*},*/
	{
	 .ops = &ctrl_ops,
	 .id = V4L2_CID_FOCUS_ABSOLUTE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "focus move absolute",
	 .min = 0,
	 .max = OV5693_VCM_MAX_FOCUS_POS,
	 .step = 1,
	 .def = 0,
	 .flags = 0,
	 },
	{
	 .ops = &ctrl_ops,
	 .id = V4L2_CID_FOCUS_RELATIVE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "focus move relative",
	 .min = OV5693_VCM_MAX_FOCUS_NEG,
	 .max = OV5693_VCM_MAX_FOCUS_POS,
	 .step = 1,
	 .def = 0,
	 .flags = 0,
	 },
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_FOCUS_STATUS,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "focus status",*/
	 /*.min = 0,*/
	 /*.max = 100,		[> allow enum to grow in the future <]*/
	 /*.step = 1,*/
	 /*.def = 0,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_VCM_SLEW,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "vcm slew",*/
	 /*.min = 0,*/
	 /*.max = OV5693_VCM_SLEW_STEP_MAX,*/
	 /*.step = 1,*/
	 /*.def = 0,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_VCM_TIMEING,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "vcm step time",*/
	 /*.min = 0,*/
	 /*.max = OV5693_VCM_SLEW_TIME_MAX,*/
	 /*.step = 1,*/
	 /*.def = 0,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_BIN_FACTOR_HORZ,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "horizontal binning factor",*/
	 /*.min = 0,*/
	 /*.max = OV5693_BIN_FACTOR_MAX,*/
	 /*.step = 1,*/
	 /*.def = 0,*/
	 /*.flags = 0,*/
	 /*},*/
	/*{*/
	 /*.ops = &ctrl_ops,*/
	 /*.id = V4L2_CID_BIN_FACTOR_VERT,*/
	 /*.type = V4L2_CTRL_TYPE_INTEGER,*/
	 /*.name = "vertical binning factor",*/
	 /*.min = 0,*/
	 /*.max = OV5693_BIN_FACTOR_MAX,*/
	 /*.step = 1,*/
	 /*.def = 0,*/
	 /*.flags = 0,*/
	 /*},*/
};

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between res_w/res_h and w/h.
 * distance = (res_w/res_h - w/h) / (w/h) * 8192
 * res->width/height smaller than w/h wouldn't be considered.
 * The gap of ratio larger than 1/8 wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 1024
static int distance(struct ov5693_resolution *res, u32 w, u32 h)
{
	int ratio;
	int distance;

	if (w == 0 || h == 0 ||
	    res->width < w || res->height < h)
		return -1;

	ratio = res->width << 13;
	ratio /= w;
	ratio *= h;
	ratio /= res->height;

	distance = abs(ratio - 8192);

	if (distance > LARGEST_ALLOWED_RATIO_MISMATCH)
		return -1;

	return distance;
}

/* Return the nearest higher resolution index
 * Firstly try to find the approximate aspect ratio resolution
 * If we find multiple same AR resolutions, choose the
 * minimal size.
 */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	int min_res_w = INT_MAX;
	struct ov5693_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov5693_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
			min_res_w = ov5693_res[i].width;
			continue;
		}
		if (dist == min_dist && ov5693_res[i].width < min_res_w)
			idx = i;
	}

	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov5693_res[i].width)
			continue;
		if (h != ov5693_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int ov5693_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *ov5693_info = NULL;
	int ret = 0;
	int idx;

	if (format->pad)
		return -EINVAL;

	if (!fmt)
		return -EINVAL;

	ov5693_info = v4l2_get_subdev_hostdata(sd);
	if (ov5693_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	idx = nearest_resolution_index(fmt->width, fmt->height);
	if (idx == -1) {
		/* return the largest resolution */
		fmt->width = ov5693_res[N_RES - 1].width;
		fmt->height = ov5693_res[N_RES - 1].height;
	} else {
		fmt->width = ov5693_res[idx].width;
		fmt->height = ov5693_res[idx].height;
	}

	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *fmt;
		mutex_unlock(&dev->input_lock);
		return 0;
	}

	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);
	if (dev->fmt_idx == -1) {
		dev_err(&client->dev, "get resolution fail\n");
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	/*ret = startup(sd);*/
	/*if (ret) {*/
		/*int i = 0;*/
		/*dev_err(&client->dev, "ov5693 startup err, retry to power up\n");*/
		/*for (i = 0; i < OV5693_POWER_UP_RETRY_NUM; i++) {*/
			/*dev_err(&client->dev,*/
				/*"ov5693 retry to power up %d/%d times, result: ",*/
				/*i+1, OV5693_POWER_UP_RETRY_NUM);*/
			/*power_down(sd);*/
			/*ret = power_up(sd);*/
			/*if (!ret) {*/
				/*mutex_unlock(&dev->input_lock);*/
				/*ov5693_init(sd);*/
				/*mutex_lock(&dev->input_lock);*/
			/*} else {*/
				/*dev_err(&client->dev, "power up failed, continue\n");*/
				/*continue;*/
			/*}*/
			/*ret = startup(sd);*/
			/*if (ret) {*/
				/*dev_err(&client->dev, " startup FAILED!\n");*/
			/*} else {*/
				/*dev_err(&client->dev, " startup SUCCESS!\n");*/
				/*break;*/
			/*}*/
		/*}*/
	/*}*/

	/*
	 * After sensor settings are set to HW, sometimes stream is started.
	 * This would cause ISP timeout because ISP is not ready to receive
	 * data yet. So add stop streaming here.
	 */
	ret = ov5693_write_reg(client, OV5693_8BIT, OV5693_SW_STREAM,
				OV5693_STOP_STREAMING);
	if (ret)
		dev_warn(&client->dev, "ov5693 stream off err\n");

	/*ret = ov5693_get_intg_factor(client, ov5693_info,*/
					/*&ov5693_res[dev->fmt_idx]);*/
	/*if (ret) {*/
		/*dev_err(&client->dev, "failed to get integration_factor\n");*/
		/*goto err;*/
	/*}*/

	/*ov5693_info->metadata_width = fmt->width * 10 / 8;*/
	/*ov5693_info->metadata_height = 1;*/
	/*ov5693_info->metadata_effective_width = &ov5693_embedded_effective_size;*/

/*err:*/
	mutex_unlock(&dev->input_lock);
	return ret;
}
static int ov5693_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	if (format->pad)
		return -EINVAL;

	if (!fmt)
		return -EINVAL;

	fmt->width = ov5693_res[dev->fmt_idx].width;
	fmt->height = ov5693_res[dev->fmt_idx].height;
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov5693_check_sensor_id(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;
	u8 revision;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_SC_CMMN_CHIP_ID_H, &high);
	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_SC_CMMN_CHIP_ID_L, &low);
	id = ((((u16) high) << 8) | (u16) low);

	if (id != OV5693_ID) {
		dev_err(&client->dev, "sensor ID error 0x%x\n", id);
		return -ENODEV;
	}

	ret = ov5693_read_reg(client, OV5693_8BIT,
					OV5693_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	dev_dbg(&client->dev, "sensor_revision = 0x%x\n", revision);
	dev_dbg(&client->dev, "detect ov5693 success\n");
	return 0;
}

static int ov5693_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	mutex_lock(&dev->input_lock);

	ret = ov5693_write_reg(client, OV5693_8BIT, OV5693_SW_STREAM,
				enable ? OV5693_START_STREAMING :
				OV5693_STOP_STREAMING);

	mutex_unlock(&dev->input_lock);

	return ret;
}


static int ov5693_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);

	interval->interval.numerator = 1;
	interval->interval.denominator = ov5693_res[dev->fmt_idx].fps;

	return 0;
}

static int ov5693_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ov5693_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov5693_res[index].width;
	fse->min_height = ov5693_res[index].height;
	fse->max_width = ov5693_res[index].width;
	fse->max_height = ov5693_res[index].height;

	return 0;

}

static const struct v4l2_subdev_video_ops ov5693_video_ops = {
	.s_stream = ov5693_s_stream,
	/*.g_parm = ov5693_g_parm,*/
	/*.s_parm = ov5693_s_parm,*/
	.g_frame_interval = ov5693_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov5693_core_ops = {
	/*.s_power = ov5693_s_power,*/
	/*.ioctl = ov5693_ioctl,*/
};

static const struct v4l2_subdev_pad_ops ov5693_pad_ops = {
	.enum_mbus_code = ov5693_enum_mbus_code,
	.enum_frame_size = ov5693_enum_frame_size,
	.get_fmt = ov5693_get_fmt,
	.set_fmt = ov5693_set_fmt,
};

static const struct v4l2_subdev_ops ov5693_ops = {
	.core = &ov5693_core_ops,
	.video = &ov5693_video_ops,
	.pad = &ov5693_pad_ops,
};

static int match_depend(struct device *dev, const void *data)
{
	return (dev && dev->fwnode == data) ? 1 : 0;
}

static int __ov5693_power_off(struct ov5693_device *ov5693)
{
	gpiod_set_value_cansleep(ov5693->xshutdn, 0);
	gpiod_set_value_cansleep(ov5693->pwdnb, 0);
	gpiod_set_value_cansleep(ov5693->led_gpio, 0);

	// TODO: I think these could be device managed
	// TODO: Only call put if they were initialized in the first place?
	gpiod_put(ov5693->xshutdn);
	gpiod_put(ov5693->pwdnb);
	gpiod_put(ov5693->led_gpio);

	return 0;
}

static int __ov5693_power_on(struct ov5693_device *ov5693)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5693->sd);
	struct acpi_handle *dev_handle = ACPI_HANDLE(&client->dev);
	struct acpi_handle_list dep_devices;
	acpi_status status;
	struct device *dev;
	int i;

	// TODO: Refactor this into own function
	// Get dependant INT3472 device
	if (!acpi_has_method(dev_handle, "_DEP")) {
		printk("No dependant devices\n");
		return -100;
	}

	status = acpi_evaluate_reference(dev_handle, "_DEP", NULL,
					 &dep_devices);
	if (ACPI_FAILURE(status)) {
		printk("Failed to evaluate _DEP.\n");
		return -ENODEV;
	}

	for (i = 0; i < dep_devices.count; i++) {
		struct acpi_device *device;
		struct acpi_device_info *info;

		status = acpi_get_object_info(dep_devices.handles[i], &info);
		if (ACPI_FAILURE(status)) {
			printk("Error reading _DEP device info\n");
			return -ENODEV;
		}

		if (info->valid & ACPI_VALID_HID &&
				!strcmp(info->hardware_id.string, "INT3472")) {
			if (acpi_bus_get_device(dep_devices.handles[i], &device))
				return -ENODEV;

			dev = bus_find_device(&platform_bus_type, NULL,
					&device->fwnode, match_depend);
			if (dev) {
				dev_info(&client->dev, "Dependent platform device found %s\n",
					dev_name(dev));
				break;
			}
		}
	}

	ov5693->xshutdn = gpiod_get_index(dev, NULL, 0, GPIOD_ASIS);
	if (IS_ERR(ov5693->xshutdn)) {
		printk("Couldn't get GPIO XSHUTDN\n");
		return -EINVAL;
	}

	ov5693->pwdnb = gpiod_get_index(dev, NULL, 1, GPIOD_ASIS);
	if (IS_ERR(ov5693->pwdnb)) {
		printk("Couldn't get GPIO PWDNB\n");
		return -EINVAL;
	}

	ov5693->led_gpio = gpiod_get_index(dev, NULL, 2, GPIOD_ASIS);
	if (IS_ERR(ov5693->led_gpio)) {
		printk("Couldn't get GPIO 2\n");
		return -EINVAL;
	}

	// TODO: According to DS, only one of these is actually controlled by GPIO.
	gpiod_set_value_cansleep(ov5693->xshutdn, 1);
	gpiod_set_value_cansleep(ov5693->pwdnb, 1);

	// TODO: This doesn't really need to be here.
	// Setting this to 1 is mostly just fun cause you can make a light turn on :)
	/*gpiod_set_value_cansleep(ov5693->led_gpio, 1);*/

	/* according to DS, at least 5ms is needed between DOVDD and PWDN */
	/* add this delay time to 10~11ms*/
	usleep_range(10000, 11000);
	
	return 0;
}

static int ov5693_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	dev_dbg(&client->dev, "ov5693_remove...\n");

	pm_runtime_disable(&client->dev);

	__ov5693_power_off(dev);

	v4l2_device_unregister_subdev(sd);

	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	kfree(dev);

	return 0;
}

static const struct media_entity_operations ov5693_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ov5693_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov5693_device *dev;
	int ret = 0;
	unsigned int i;

	// TODO: devm_kzalloc?
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		ret = -ENOMEM;
		goto out;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&dev->sd, client, &ov5693_ops);

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	dev->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	dev->sd.entity.ops = ov5693_entity_ops;
	ret =
		v4l2_ctrl_handler_init(&dev->ctrl_handler,
				   ARRAY_SIZE(ov5693_controls));
	if (ret)
		goto out;

	for (i = 0; i < ARRAY_SIZE(ov5693_controls); i++)
		v4l2_ctrl_new_custom(&dev->ctrl_handler, &ov5693_controls[i],
					 NULL);

	if (dev->ctrl_handler.error) {
		ret = dev->ctrl_handler.error;
		goto out;
	}

	// Power on
	ret = __ov5693_power_on(dev);
	if (ret)
	{
		dev_err(&client->dev, "could not power on ov5693\n");
		goto out;
	}

	ret = ov5693_check_sensor_id(client);
	if (ret)
		goto out;

	/* Use same lock for controls as for everything else. */
	dev->ctrl_handler.lock = &dev->input_lock;
	dev->sd.ctrl_handler = &dev->ctrl_handler;

	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&dev->sd.entity, 1, &dev->pad);
	if (ret)
		goto out;

	ret = v4l2_async_register_subdev_sensor_common(&dev->sd);
	if (ret) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto out;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

out:
	if (ret)
		ov5693_remove(client);

	return ret;
}

static int __maybe_unused ov5693_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused ov5693_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id ov5693_acpi_ids[] = {
	{"INT33BE"},
	{}
};

MODULE_DEVICE_TABLE(acpi, ov5693_acpi_ids);
#endif

static const struct dev_pm_ops ov5693_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ov5693_suspend, ov5693_resume)
};

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "ov5693",
		.pm = &ov5693_pm_ops,
		.acpi_match_table = ACPI_PTR(ov5693_acpi_ids),
	},
	.probe = ov5693_probe,
	.remove = ov5693_remove,
};

module_i2c_driver(ov5693_i2c_driver);

MODULE_AUTHOR("Jordan Hand <jorhand@linux.microsoft.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision 5693 sensors");
MODULE_LICENSE("GPL");
