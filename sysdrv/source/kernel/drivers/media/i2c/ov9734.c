// SPDX-License-Identifier: GPL-2.0
/*
 * ov9734 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 add function g_mbus_config
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x6)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV9734_LINK_FREQ_180MHZ		180000000ULL
#define OV9734_SCLK			36000000LL
#define OV9734_MCLK			19200000
/* ov9734 only support 1-lane mipi output */
#define OV9734_DATA_LANES		1
#define OV9734_BITS_PER_SAMPLE		10
#define OV9734_RGB_DEPTH		OV9734_BITS_PER_SAMPLE

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define OV9734_PIXEL_RATE		(OV9734_LINK_FREQ_180MHZ * 2 * OV9734_DATA_LANES / OV9734_BITS_PER_SAMPLE)
#define OV9734_XVCLK_FREQ		24000000

#define CHIP_ID				0x9734
#define OV9734_REG_CHIP_ID		0x300a

#define OV9734_REG_CTRL_MODE		0x0100
#define OV9734_MODE_SW_STANDBY		0x0
#define OV9734_MODE_STREAMING		BIT(0)

/* vertical-timings from sensor */
#define OV9734_REG_VTS			0x380e
#define OV9734_VTS_30FPS		0x0322
#define OV9734_VTS_30FPS_MIN		0x0322
#define OV9734_VTS_MAX			0x7fff

/* horizontal-timings from sensor */
#define OV9734_REG_HTS			0x380c

/* Exposure controls from sensor */
#define OV9734_REG_EXPOSURE		0x3500
#define	OV9734_EXPOSURE_MIN		4
#define	OV9734_EXPOSURE_STEP		1
#define OV9734_EXPOSURE_MAX_MARGIN	4

/* Analog gain controls from sensor */
#define OV9734_REG_ANALOG_GAIN		0x350a
#define OV9734_ANAL_GAIN_MIN		16
#define OV9734_ANAL_GAIN_MAX		248
#define OV9734_ANAL_GAIN_STEP		1

#define OV9734_REG_GAIN_H		OV9734_REG_ANALOG_GAIN
#define OV9734_REG_GAIN_L		(OV9734_REG_ANALOG_GAIN+1)
#define OV9734_GAIN_H_MASK		0x03
#define OV9734_GAIN_L_MASK		0xff
#define OV9734_GAIN_MIN			OV9734_ANAL_GAIN_MIN
#define OV9734_GAIN_MAX			OV9734_ANAL_GAIN_MAX
#define OV9734_GAIN_STEP		OV9734_ANAL_GAIN_STEP
#define OV9734_GAIN_DEFAULT		0x0040

/* Digital gain controls from sensor */
#define OV9734_REG_MWB_R_GAIN		0x5180
#define OV9734_REG_MWB_G_GAIN		0x5182
#define OV9734_REG_MWB_B_GAIN		0x5184
#define OV9734_DGTL_GAIN_MIN		256
#define OV9734_DGTL_GAIN_MAX		1023
#define OV9734_DGTL_GAIN_STEP		1
#define OV9734_DGTL_GAIN_DEFAULT	256

/* Test Pattern Control */
#define OV9734_REG_TEST_PATTERN		0x5080
#define OV9734_TEST_PATTERN_ENABLE	BIT(7)
#define OV9734_TEST_PATTERN_DISABLE	0x0
#define OV9734_TEST_PATTERN_BAR_SHIFT	2

/* Group Access */
#define OV9734_REG_GROUP_ACCESS		0x3208
#define OV9734_GROUP_HOLD_START		0x0
#define OV9734_GROUP_HOLD_END		0x10
#define OV9734_GROUP_HOLD_LAUNCH	0xa0

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define OV9734_REG_VALUE_08BIT		1
#define OV9734_REG_VALUE_16BIT		2
#define OV9734_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OV9734_NAME			"ov9734"

enum {
	OV9734_LINK_FREQ_180MHZ_INDEX,
};

static const char * const ov9734_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV9734_NUM_SUPPLIES ARRAY_SIZE(ov9734_supply_names)


struct regval {
	u16 addr;
	u8 val;
};

struct ov9734_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	const struct regval *reg_list;
};

struct ov9734 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV9734_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov9734_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ov9734(sd) container_of(sd, struct ov9734, subdev)

static const s64 link_freq_menu_items[] = {
	OV9734_LINK_FREQ_180MHZ
};

#if 1//KentYu
static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OV9734_DATA_LANES;

	do_div(pixel_rate, OV9734_RGB_DEPTH);

	return pixel_rate;
}

static u64 to_pixels_per_line(u32 hts, u32 f_index)
{
	u64 ppl = hts * to_pixel_rate(f_index);

	do_div(ppl, OV9734_SCLK);

	return ppl;
}
#endif

/*
 * Xclk 24Mhz
 */
static const struct regval ov9734_global_regs[] = {
	{0x0103, 0x01},
	{REG_DELAY, 0x10},
	{0x0100, 0x00},
	{REG_DELAY, 0x10},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3007, 0x00},
	{0x3010, 0x00},
	{0x3011, 0x08},
	{0x3014, 0x22},
	{0x301e, 0x15},
	{0x3030, 0x19},
	{0x3080, 0x02},
	{0x3081, 0x3c},
	{0x3082, 0x04},
	{0x3083, 0x00},
	{0x3084, 0x02},
	{0x3085, 0x01},
	{0x3086, 0x01},
	{0x3089, 0x01},
	{0x308a, 0x00},
	{0x3103, 0x01},
	{0x3600, 0x55},
	{0x3601, 0x02},
	{0x3605, 0x22},
	{0x3611, 0xe7},
	{0x3654, 0x10},
	{0x3655, 0x77},
	{0x3656, 0x77},
	{0x3657, 0x07},
	{0x3658, 0x22},
	{0x3659, 0x22},
	{0x365a, 0x02},
	{0x3784, 0x05},
	{0x3785, 0x55},
	{0x37c0, 0x07},
	{0x3800, 0x00},
	{0x3801, 0x04},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x05},
	{0x3805, 0x0b},
	{0x3806, 0x02},
	{0x3807, 0xdb},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x05},
	{0x380d, 0xc6},
	{0x380e, 0x03},
	{0x380f, 0x22},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3816, 0x00},
	{0x3817, 0x00},
	{0x3818, 0x00},
	{0x3819, 0x04},
	{0x3820, 0x18},
	{0x3821, 0x00},
	{0x382c, 0x06},
	{0x3500, 0x00},
	{0x3501, 0x31},
	{0x3502, 0x00},
	{0x3503, 0x03},
	{0x3504, 0x00},
	{0x3505, 0x00},
	{0x3509, 0x10},
	{0x350a, 0x00},
	{0x350b, 0x40},
	{0x3d00, 0x00},
	{0x3d01, 0x00},
	{0x3d02, 0x00},
	{0x3d03, 0x00},
	{0x3d04, 0x00},
	{0x3d05, 0x00},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d08, 0x00},
	{0x3d09, 0x00},
	{0x3d0a, 0x00},
	{0x3d0b, 0x00},
	{0x3d0c, 0x00},
	{0x3d0d, 0x00},
	{0x3d0e, 0x00},
	{0x3d0f, 0x00},
	{0x3d80, 0x00},
	{0x3d81, 0x00},
	{0x3d82, 0x38},
	{0x3d83, 0xa4},
	{0x3d84, 0x00},
	{0x3d85, 0x00},
	{0x3d86, 0x1f},
	{0x3d87, 0x03},
	{0x3d8b, 0x00},
	{0x3d8f, 0x00},
	{0x4001, 0xe0},
	{0x4009, 0x0b},
	{0x4300, 0x03},
	{0x4301, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4309, 0x00},
	{0x4600, 0x00},
	{0x4601, 0x80},
	{0x4800, 0x00},
	{0x4805, 0x00},
	{0x4821, 0x50},
	{0x4823, 0x50},
	{0x4837, 0x2d},
	{0x4a00, 0x00},
	{0x4f00, 0x80},
	{0x4f01, 0x10},
	{0x4f02, 0x00},
	{0x4f03, 0x00},
	{0x4f04, 0x00},
	{0x4f05, 0x00},
	{0x4f06, 0x00},
	{0x4f07, 0x00},
	{0x4f08, 0x00},
	{0x4f09, 0x00},
	{0x5000, 0x2f},
	{0x500c, 0x00},
	{0x500d, 0x00},
	{0x500e, 0x00},
	{0x500f, 0x00},
	{0x5010, 0x00},
	{0x5011, 0x00},
	{0x5012, 0x00},
	{0x5013, 0x00},
	{0x5014, 0x00},
	{0x5015, 0x00},
	{0x5016, 0x00},
	{0x5017, 0x00},
	{0x5080, 0x00},
	{0x5180, 0x01},
	{0x5181, 0x00},
	{0x5182, 0x01},
	{0x5183, 0x00},
	{0x5184, 0x01},
	{0x5185, 0x00},
	{0x5708, 0x06},
	{0x380f, 0x2a},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x5000, 0x3f},
	
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 800Mbps
 */
static const struct regval ov9734_1280x720_regs[] = {
	{REG_NULL, 0x00},
};

static const struct ov9734_mode supported_modes[] = {
	{
		.width = 1280,//1296,//KentYu1280,
		.height = 720,//734,//KentYu 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x131,//0x03D0,
		.hts_def = 0x5c6,//0x0654,//0x32A*2
		.vts_def = OV9734_VTS_30FPS,//0x03DC,
		.reg_list = ov9734_1280x720_regs,
		.link_freq_index = OV9734_LINK_FREQ_180MHZ_INDEX,//KentYu
	},
};

static const char * const ov9734_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ov9734_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ov9734_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = ov9734_write_reg(client, regs[i].addr,
				OV9734_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ov9734_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ov9734_get_reso_dist(const struct ov9734_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov9734_mode *
ov9734_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov9734_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov9734_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	const struct ov9734_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov9734->mutex);

	mode = ov9734_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov9734->mutex);
		return -ENOTTY;
#endif
	} else {
		ov9734->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov9734->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov9734->vblank, vblank_def,
					 OV9734_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ov9734->mutex);

	return 0;
}

static int ov9734_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	const struct ov9734_mode *mode = ov9734->cur_mode;

	mutex_lock(&ov9734->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov9734->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ov9734->mutex);

	return 0;
}

static int ov9734_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov9734_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

#if 1//KentYu
static int ov9734_update_digital_gain(struct ov9734 *ov9734, u32 d_gain)
{
	int ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_GROUP_ACCESS, 1,
			       OV9734_GROUP_HOLD_START);
	if (ret)
		return ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_MWB_R_GAIN, 2, d_gain);
	if (ret)
		return ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_MWB_G_GAIN, 2, d_gain);
	if (ret)
		return ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_MWB_B_GAIN, 2, d_gain);
	if (ret)
		return ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_GROUP_ACCESS, 1,
			       OV9734_GROUP_HOLD_END);
	if (ret)
		return ret;

	ret = ov9734_write_reg(ov9734->client, OV9734_REG_GROUP_ACCESS, 1,
			       OV9734_GROUP_HOLD_LAUNCH);
	return ret;
}
#endif

static int ov9734_enable_test_pattern(struct ov9734 *ov9734, u32 pattern)
{
	u32 val;

	if (pattern)
		val = ((pattern - 1) < 2) | OV9734_TEST_PATTERN_ENABLE;
	else
		val = OV9734_TEST_PATTERN_DISABLE;

	return ov9734_write_reg(ov9734->client, OV9734_REG_TEST_PATTERN,
				OV9734_REG_VALUE_08BIT, val);
}

static int ov9734_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	const struct ov9734_mode *mode = ov9734->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static void ov9734_get_module_inf(struct ov9734 *ov9734,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, OV9734_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov9734->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov9734->len_name, sizeof(inf->base.lens));
}

static long ov9734_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov9734_get_module_inf(ov9734, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ov9734_write_reg(ov9734->client, OV9734_REG_CTRL_MODE,
				OV9734_REG_VALUE_08BIT, OV9734_MODE_STREAMING);
		else
			ret = ov9734_write_reg(ov9734->client, OV9734_REG_CTRL_MODE,
				OV9734_REG_VALUE_08BIT, OV9734_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov9734_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}
		ret = ov9734_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov9734_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov9734_start_stream(struct ov9734 *ov9734)
{
	int ret;

	ret = ov9734_write_array(ov9734->client, ov9734->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&ov9734->mutex);
	ret = v4l2_ctrl_handler_setup(&ov9734->ctrl_handler);
	mutex_lock(&ov9734->mutex);
	if (ret)
		return ret;

	return ov9734_write_reg(ov9734->client, OV9734_REG_CTRL_MODE,
				OV9734_REG_VALUE_08BIT, OV9734_MODE_STREAMING);
}

static int __ov9734_stop_stream(struct ov9734 *ov9734)
{
	return ov9734_write_reg(ov9734->client, OV9734_REG_CTRL_MODE,
				OV9734_REG_VALUE_08BIT, OV9734_MODE_SW_STANDBY);
}

static int ov9734_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	struct i2c_client *client = ov9734->client;
	int ret = 0;

	mutex_lock(&ov9734->mutex);
	on = !!on;
	if (on == ov9734->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov9734_start_stream(ov9734);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov9734_stop_stream(ov9734);
		pm_runtime_put(&client->dev);
	}

	ov9734->streaming = on;

unlock_and_return:
	mutex_unlock(&ov9734->mutex);

	return ret;
}

static int ov9734_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	struct i2c_client *client = ov9734->client;
	int ret = 0;

	mutex_lock(&ov9734->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov9734->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = ov9734_write_array(ov9734->client, ov9734_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ov9734->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov9734->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov9734->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov9734_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV9734_XVCLK_FREQ / 1000 / 1000);
}

static int __ov9734_power_on(struct ov9734 *ov9734)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov9734->client->dev;

	if (!IS_ERR_OR_NULL(ov9734->pins_default)) {
		ret = pinctrl_select_state(ov9734->pinctrl,
					   ov9734->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(ov9734->xvclk, OV9734_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov9734->xvclk) != OV9734_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov9734->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ov9734->reset_gpio))
		gpiod_set_value_cansleep(ov9734->reset_gpio, 0);

	ret = regulator_bulk_enable(OV9734_NUM_SUPPLIES, ov9734->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov9734->reset_gpio))
		gpiod_set_value_cansleep(ov9734->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(ov9734->pwdn_gpio))
		gpiod_set_value_cansleep(ov9734->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov9734_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov9734->xvclk);

	return ret;
}

static void __ov9734_power_off(struct ov9734 *ov9734)
{
	int ret;

	if (!IS_ERR(ov9734->pwdn_gpio))
		gpiod_set_value_cansleep(ov9734->pwdn_gpio, 0);
	clk_disable_unprepare(ov9734->xvclk);
	if (!IS_ERR(ov9734->reset_gpio))
		gpiod_set_value_cansleep(ov9734->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov9734->pins_sleep)) {
		ret = pinctrl_select_state(ov9734->pinctrl,
					   ov9734->pins_sleep);
		if (ret < 0)
			dev_dbg(&ov9734->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(OV9734_NUM_SUPPLIES, ov9734->supplies);
}

static int ov9734_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9734 *ov9734 = to_ov9734(sd);

	return __ov9734_power_on(ov9734);
}

static int ov9734_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9734 *ov9734 = to_ov9734(sd);

	__ov9734_power_off(ov9734);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov9734_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov9734 *ov9734 = to_ov9734(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov9734_mode *def_mode = &supported_modes[0];

	mutex_lock(&ov9734->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov9734->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov9734_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

#if 0//KentYu
static int ov9734_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	u32 val = 0;

	val = 1 << (OV9734_DATA_LANES - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2_1_LANE;//KentYuV4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}
#endif

static const struct dev_pm_ops ov9734_pm_ops = {
	SET_RUNTIME_PM_OPS(ov9734_runtime_suspend,
			   ov9734_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov9734_internal_ops = {
	.open = ov9734_open,
};
#endif

static const struct v4l2_subdev_core_ops ov9734_core_ops = {
	.s_power = ov9734_s_power,
	.ioctl = ov9734_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov9734_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov9734_video_ops = {
	.s_stream = ov9734_s_stream,
	.g_frame_interval = ov9734_g_frame_interval,
	//KentYu.g_mbus_config = ov9734_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov9734_pad_ops = {
	.enum_mbus_code = ov9734_enum_mbus_code,
	.enum_frame_size = ov9734_enum_frame_sizes,
	.enum_frame_interval = ov9734_enum_frame_interval,
	.get_fmt = ov9734_get_fmt,
	.set_fmt = ov9734_set_fmt,
};

static const struct v4l2_subdev_ops ov9734_subdev_ops = {
	.core	= &ov9734_core_ops,
	.video	= &ov9734_video_ops,
	.pad	= &ov9734_pad_ops,
};

static int ov9734_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9734 *ov9734 = container_of(ctrl->handler,
					     struct ov9734, ctrl_handler);
	struct i2c_client *client = ov9734->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov9734->cur_mode->height + ctrl->val - OV9734_EXPOSURE_MAX_MARGIN/*KentYu*/;
		__v4l2_ctrl_modify_range(ov9734->exposure,
					 ov9734->exposure->minimum, max,
					 ov9734->exposure->step,
					 max);//KentYuov9734->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ov9734_write_reg(ov9734->client, OV9734_REG_EXPOSURE,
				       OV9734_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		#if 1//KentYu
		ret = ov9734_write_reg(ov9734->client, OV9734_REG_GAIN_H,
				       2, ctrl->val);
		#else
		ret = ov9734_write_reg(ov9734->client, OV9734_REG_GAIN_H,
				       OV9734_REG_VALUE_08BIT,
				       (ctrl->val >> 8) & OV9734_GAIN_H_MASK);
		ret |= ov9734_write_reg(ov9734->client, OV9734_REG_GAIN_L,
				       OV9734_REG_VALUE_08BIT,
				       ctrl->val & OV9734_GAIN_L_MASK);
		#endif
		break;
  #if 1////KentYu
	case V4L2_CID_DIGITAL_GAIN:
		ret = ov9734_update_digital_gain(ov9734, ctrl->val);
		break;
  #endif
  
	case V4L2_CID_VBLANK:
		ret = ov9734_write_reg(ov9734->client, OV9734_REG_VTS,
				       OV9734_REG_VALUE_16BIT,
				       ctrl->val + ov9734->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov9734_enable_test_pattern(ov9734, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov9734_ctrl_ops = {
	.s_ctrl = ov9734_set_ctrl,
};

static int ov9734_initialize_controls(struct ov9734 *ov9734)
{
	const struct ov9734_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def, pixel_rate/*KentYu*/;
  #if 1//KentYu
	u32 vblank_min, vblank_max;
  #endif
	u32 h_blank;
	int ret, size/*KentYu*/;

	handler = &ov9734->ctrl_handler;
	mode = ov9734->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &ov9734->mutex;

	size = ARRAY_SIZE(link_freq_menu_items);//KentYu
	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      size - 1/*KentYu*/, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	#if 1//KentYu
	pixel_rate = to_pixel_rate(OV9734_LINK_FREQ_180MHZ_INDEX);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, pixel_rate, 1, pixel_rate);
	#else
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, OV9734_PIXEL_RATE, 1, OV9734_PIXEL_RATE);
	#endif

  #if 1//KentYu
	h_blank = to_pixels_per_line(mode->hts_def, mode->link_freq_index);
	h_blank -= mode->width;
	ov9734->hblank = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops,
					   V4L2_CID_HBLANK, h_blank, h_blank, 1,
					   h_blank);
	if (ov9734->hblank)
		ov9734->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_min = OV9734_VTS_30FPS_MIN - mode->height;
	vblank_max = OV9734_VTS_MAX - mode->height;
	vblank_def = mode->vts_def - mode->height;
	ov9734->vblank = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_min,
					   vblank_max, 1, vblank_def);
  #else
	h_blank = mode->hts_def - mode->width;
	ov9734->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov9734->hblank)
		ov9734->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov9734->vblank = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV9734_VTS_MAX - mode->height,
				1, vblank_def);
  #endif

	exposure_max = mode->vts_def - OV9734_EXPOSURE_MAX_MARGIN/*KentYu*/;
	ov9734->exposure = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops,
				V4L2_CID_EXPOSURE, OV9734_EXPOSURE_MIN,
				exposure_max, OV9734_EXPOSURE_STEP,
  #if 1//KentYu
				exposure_max);
  #else
				mode->exp_def);
  #endif
  
	ov9734->anal_gain = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, OV9734_GAIN_MIN,
				OV9734_GAIN_MAX, OV9734_GAIN_STEP,
  #if 1//KentYu
				OV9734_GAIN_MIN);
  #else
				OV9734_GAIN_DEFAULT);
  #endif

  #if 1//KentYu
	ov9734->digi_gain = v4l2_ctrl_new_std(handler, &ov9734_ctrl_ops, 
			  V4L2_CID_DIGITAL_GAIN, OV9734_DGTL_GAIN_MIN, 
			  OV9734_DGTL_GAIN_MAX, OV9734_DGTL_GAIN_STEP, 
			  OV9734_DGTL_GAIN_DEFAULT);
  #endif
  
	ov9734->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov9734_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov9734_test_pattern_menu) - 1,
				0, 0, ov9734_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov9734->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov9734->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov9734_check_sensor_id(struct ov9734 *ov9734,
				  struct i2c_client *client)
{
	struct device *dev = &ov9734->client->dev;
	u32 id = 0;
	int ret;

	ret = ov9734_read_reg(client, OV9734_REG_CHIP_ID,
			      OV9734_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%04x sensor\n", id);

	return 0;
}

static int ov9734_configure_regulators(struct ov9734 *ov9734)
{
	unsigned int i;

	for (i = 0; i < OV9734_NUM_SUPPLIES; i++)
		ov9734->supplies[i].supply = ov9734_supply_names[i];

	return devm_regulator_bulk_get(&ov9734->client->dev,
				       OV9734_NUM_SUPPLIES,
				       ov9734->supplies);
}

static int ov9734_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov9734 *ov9734;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov9734 = devm_kzalloc(dev, sizeof(*ov9734), GFP_KERNEL);
	if (!ov9734)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov9734->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov9734->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov9734->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov9734->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov9734->client = client;
	ov9734->cur_mode = &supported_modes[0];

	ov9734->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov9734->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov9734->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov9734->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov9734->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ov9734->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ov9734->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov9734->pinctrl)) {
		ov9734->pins_default =
			pinctrl_lookup_state(ov9734->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov9734->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov9734->pins_sleep =
			pinctrl_lookup_state(ov9734->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov9734->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov9734_configure_regulators(ov9734);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov9734->mutex);

	sd = &ov9734->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov9734_subdev_ops);
	ret = ov9734_initialize_controls(ov9734);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov9734_power_on(ov9734);
	if (ret)
		goto err_free_handler;

	ret = ov9734_check_sensor_id(ov9734, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov9734_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov9734->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov9734->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov9734->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov9734->module_index, facing,
		 OV9734_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov9734_power_off(ov9734);
err_free_handler:
	v4l2_ctrl_handler_free(&ov9734->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov9734->mutex);

	return ret;
}

static int ov9734_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9734 *ov9734 = to_ov9734(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov9734->ctrl_handler);
	mutex_destroy(&ov9734->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov9734_power_off(ov9734);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov9734_of_match[] = {
	{ .compatible = "ovti,ov9734" },
	{},
};
MODULE_DEVICE_TABLE(of, ov9734_of_match);
#endif

static const struct i2c_device_id ov9734_match_id[] = {
	{ "ovti,ov9734", 0 },
	{ },
};

static struct i2c_driver ov9734_i2c_driver = {
	.driver = {
		.name = OV9734_NAME,
		.pm = &ov9734_pm_ops,
		.of_match_table = of_match_ptr(ov9734_of_match),
	},
	.probe		= &ov9734_probe,
	.remove		= &ov9734_remove,
	.id_table	= ov9734_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov9734_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov9734_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ov9734 sensor driver");
MODULE_LICENSE("GPL v2");
