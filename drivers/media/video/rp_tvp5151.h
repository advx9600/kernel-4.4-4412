/*
 * Driver for RPTVP5150 (5MP Camera) from NEC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RPTVP5150_H
#define __RPTVP5150_H

#define CONFIG_CAM_DEBUG	1

#define cam_warn(fmt, ...)	\
	do { \
		printk(KERN_WARNING "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_err(fmt, ...)	\
	do { \
		printk(KERN_ERR "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_info(fmt, ...)	\
	do { \
		printk(KERN_INFO "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#ifdef CONFIG_CAM_DEBUG
#define CAM_DEBUG	(1 << 0)
#define CAM_TRACE	(1 << 1)
#define CAM_I2C		(1 << 2)

#define cam_dbg(fmt, ...)	\
	do { \
		if (to_state(sd)->dbg_level & CAM_DEBUG) \
			printk(KERN_DEBUG "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_trace(fmt, ...)	\
	do { \
		if (to_state(sd)->dbg_level & CAM_TRACE) \
			printk(KERN_DEBUG "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define cam_i2c_dbg(fmt, ...)	\
	do { \
		if (to_state(sd)->dbg_level & CAM_I2C) \
			printk(KERN_DEBUG "%s: " fmt, __func__, ##__VA_ARGS__); \
	} while (0)
#else
#define cam_dbg(fmt, ...)
#define cam_trace(fmt, ...)
#define cam_i2c_dbg(fmt, ...)
#endif

enum rptvp5150_prev_frmsize {
	RPTVP5150_PREVIEW_QCIF,
	RPTVP5150_PREVIEW_QCIF2,
	RPTVP5150_PREVIEW_QVGA,
	RPTVP5150_PREVIEW_VGA,
	RPTVP5150_PREVIEW_D1,
	RPTVP5150_PREVIEW_WVGA,
	RPTVP5150_PREVIEW_720P,
	RPTVP5150_PREVIEW_1080P,
	RPTVP5150_PREVIEW_HDR,
};

enum rptvp5150_cap_frmsize {
	RPTVP5150_CAPTURE_VGA,	/* 640 x 480 */
	RPTVP5150_CAPTURE_WVGA,	/* 800 x 480 */
	RPTVP5150_CAPTURE_W1MP,	/* 1600 x 960 */
	RPTVP5150_CAPTURE_2MP,	/* UXGA - 1600 x 1200 */
	RPTVP5150_CAPTURE_W2MP,	/* 2048 x 1232 */
	RPTVP5150_CAPTURE_3MP,	/* QXGA - 2048 x 1536 */
	RPTVP5150_CAPTURE_W4MP,	/* WQXGA - 2560 x 1536 */
	RPTVP5150_CAPTURE_5MP,	/* 2560 x 1920 */
	RPTVP5150_CAPTURE_W6MP,	/* 3072 x 1856 */
	RPTVP5150_CAPTURE_7MP,	/* 3072 x 2304 */
	RPTVP5150_CAPTURE_W7MP,	/* WQXGA - 2560 x 1536 */
	RPTVP5150_CAPTURE_8MP,	/* 3264 x 2448 */
};

struct rptvp5150_control {
	u32 id;
	s32 value;
	s32 minimum;		/* Note signedness */
	s32 maximum;
	s32 step;
	s32 default_value;
};

struct rptvp5150_frmsizeenum {
	unsigned int index;
	unsigned int width;
	unsigned int height;
	u8 reg_val;		/* a value for category parameter */
};

struct rptvp5150_isp {
	wait_queue_head_t wait;
	unsigned int irq;	/* irq issued by ISP */
	unsigned int issued;
	unsigned int int_factor;
	unsigned int bad_fw:1;
};

struct rptvp5150_jpeg {
	int quality;
	unsigned int main_size;	/* Main JPEG file size */
	unsigned int thumb_size;	/* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
};

struct rptvp5150_focus {
	unsigned int mode;
	unsigned int lock;
	unsigned int status;
	unsigned int touch;
	unsigned int pos_x;
	unsigned int pos_y;
};

struct rptvp5150_exif {
	char unique_id[7];
	u32 exptime;		/* us */
	u16 flash;
	u16 iso;
	int tv;			/* shutter speed */
	int bv;			/* brightness */
	int ebv;		/* exposure bias */
};

struct rptvp5150_state {
	struct v4l2_subdev sd;

	struct rptvp5150_isp isp;

	const struct rptvp5150_frmsizeenum *preview;
	const struct rptvp5150_frmsizeenum *capture;

	enum v4l2_pix_format_mode format_mode;
	enum v4l2_sensor_mode sensor_mode;
	enum v4l2_flash_mode flash_mode;
	int vt_mode;
	int beauty_mode;
	int zoom;

	unsigned int fps;
	struct rptvp5150_focus focus;

	struct rptvp5150_jpeg jpeg;
	struct rptvp5150_exif exif;

	int check_dataline;
	char *fw_version;

#ifdef CONFIG_CAM_DEBUG
	u8 dbg_level;
#endif
};


/* UT2055 Sensor Mode */
#define RPTVP5150_SYSINIT_MODE	0x0
#define RPTVP5150_PARMSET_MODE	0x1
#define RPTVP5150_MONITOR_MODE	0x2
#define RPTVP5150_STILLCAP_MODE	0x3



/* ESD Interrupt */
#define RPTVP5150_INT_ESD		(1 << 0)


#define CHIP_DELAY 0xFF

static unsigned char rptvp5150_init_reg[][2] ={ 
#if 0
//  2010-12-16 kim 
	{0x05, 0x01},	// Camera Soft reset. Self cleared after reset.
//	{CHIP_DELAY, 10},
	{0x02,0x00},
	{0x00, 0x00},		// CVBS-CH2
	{0x03,0x6d},		// GPCL HIGH FOR ANALOG SW to CVBS, YUV output enable
	{0x05,0x02},
	{0x0d,0x47},
	{0x11,0x00},
	{0x12,0x04},  // bright
	{0x18,0x00},  // color
	{0x19,0x00}, //Hue
  
//  {0x0e, 0x03},
//  {0x1b, 0x13},

	{0x09,0x80},	 //contrast  
#endif

	{0x05, 0x01},	// Camera Soft reset. Self cleared after reset.
	{0x00, 0x00},		// CVBS-CH2
	{0x03,0x6d},		// GPCL HIGH FOR ANALOG SW to CVBS, YUV output enable
	{0x18,0x7f},  // color
	{0x19,0x7f}, //Hue

	{0x15,0x31},
	

};

#define rptvp5150_INIT_REGS	(sizeof(rptvp5150_init_reg) / sizeof(rptvp5150_init_reg[0]))  



#endif /* __RPTVP5150_H */
