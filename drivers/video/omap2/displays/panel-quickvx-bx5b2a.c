/*
 * Copyright (C) Quicklogic 2012
 *
 * Quicklogic BX5B1D mipi2-LVDS display driver for OMAP
 * Author : Sunny
 *
 * * Based on panel-tc358765.c
 * * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
/*
 * Toshiba TC358765 DSI-to-LVDS chip driver
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * Based on original version from Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define QLVX_DRIVER_VER "panel-quickvx-BX5B1D ver 1.4"
/* Quicklogic BX5B1D MIPI2-LVDS
	20121129 - rev 1.0 Modified from panel-tc358765.c. 
	20130201 - rev 1.1 updated mipi r/w codes.
	20130301 - rev 1.2 remove old code.
    20130305 - rev 1.3 use 1280x800 timing
	20130326 - rev 1.4 disable channel1.
*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <video/omapdss.h>
#include <linux/clk.h>
#include <linux/tca6416a.h>

#define EYEPIECE_I2C_ADDR 	0x48
#define I2C_BUS_NUMBER		3
#define I2C_RETRY_TIMES 	5

#ifndef CONFIG_REGULATOR_GPIO_SWITCH
#define GPIO_EN1V2 			272
#define GPIO_EN1V8  			273
#endif
#define GPIO_EN3V3			275
#define GPIO_EN10V			276
#define GPIO_DISP_OE_RST  	280
#define GPIO_BRG_RSTN  		281

#if defined(CONFIG_REGULATOR_GPIO_SWITCH) /* convert gpio to regulator */
static struct regulator *vdd_1v2 = NULL;
static struct regulator *vdd_1v8 = NULL;
#endif

struct omap_dss_device *saved_dssdev;

struct quickvx_data {
#ifdef QL_BACKLIGHT	
	struct backlight_device *quickvx_bldev;
#endif
	struct omap_dss_device *dssdev;
	struct clk *clk;
	bool clk_on;
	bool vdd_1v2_on;
	bool vdd_1v8_on;
	int channel0;
#ifdef QL_USE_CH1
	int channel1;
#endif
};

#define QL_DBG(f, x...) \
	printk("[QLVX] %s: " f, __func__,## x)
#define QL_DBGL(lvl, f, x...) do {if (lvl) printk("[QLVX] %s: " f, __func__,## x); }while(0)
#define QL_ERR(f, x...) \
	printk("[QLVX] ERROR %s: " f, __func__,## x)
#define QL_DBG_FUNC_ENTER QL_DBG("+++\n");
#define QL_DBG_FUNC_EXIT QL_DBG("---\n");

/* If use i2c for sysfs to access VX registers */
#define QL_VX_SYSFS_I2C_ACCESS_READ
#define QL_VX_SYSFS_I2C_ACCESS_WRITE

//Porting : enable setup BX chip from Diolan board.
//#define QL_CHIP_INIT_EXTERNAL

/* QL Backlight Control */
//#define QL_BACKLIGHT
#define CONFIG_QUICKVX_SYSFS_I2C_ACCESS
/* Function prototypes */
void ql_chip_init(struct omap_dss_device *dssdev);
int ql_i2c_read(uint32_t addr, uint32_t *val, uint32_t data_size);
int ql_i2c_write(long addr, long val, int data_size);
int ql_i2c_release(void);
#ifdef CONFIG_QUICKVX_SYSFS_I2C_ACCESS
static int quickvx_mipi_sysfs_register(struct device *dev);
static int quickvx_mipi_sysfs_unregister(struct device *dev);
#endif
int ql_mipi_write(struct omap_dss_device *dssdev, uint32_t address, uint32_t value, uint32_t data_size);
int ql_mipi_read(struct omap_dss_device *dssdev, uint32_t address, uint32_t *regval, uint32_t data_size);

#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
void lcos_vdd_1v2_pwr_on(struct omap_dss_device *dssdev)
{
	int ret;
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	if(vdd_1v2 != NULL && qd->vdd_1v2_on == false){
		ret = regulator_enable(vdd_1v2);
		if (ret < 0){
			printk(KERN_ERR "Failed to turn on  vdd 1.2v  regulator..\n");
			return;
		}
		qd->vdd_1v2_on = true;
	}
}
EXPORT_SYMBOL(lcos_vdd_1v2_pwr_on);
void lcos_vdd_1v2_pwr_off(struct omap_dss_device *dssdev)
{
	int ret;
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	if(vdd_1v2 != NULL && qd->vdd_1v2_on == true){
		ret = regulator_disable(vdd_1v2);
		if (ret < 0){
			printk(KERN_ERR "Failed to turn off  vdd 1.2v  regulator..\n");
			return;
		}
		qd->vdd_1v2_on = false;
	}
}
EXPORT_SYMBOL(lcos_vdd_1v2_pwr_off);
void lcos_vdd_1v8_pwr_on(struct omap_dss_device *dssdev)
{
	int ret;
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	if(vdd_1v8 != NULL && qd->vdd_1v8_on == false){
		ret = regulator_enable(vdd_1v8);
		if (ret < 0){
			printk(KERN_ERR "Failed to turn on  vdd 1.8v  regulator..\n");
			return;
		}
		qd->vdd_1v8_on = true;
	}
}
EXPORT_SYMBOL(lcos_vdd_1v8_pwr_on);
void lcos_vdd_1v8_pwr_off(struct omap_dss_device *dssdev)
{
	int ret;
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	if(vdd_1v8 != NULL && qd->vdd_1v8_on == true){
		ret = regulator_disable(vdd_1v8);
		if (ret < 0){
			printk(KERN_ERR "Failed to turn off  vdd 1.8v  regulator..\n");
			return;
		}
		qd->vdd_1v8_on = false;
	}
}
EXPORT_SYMBOL(lcos_vdd_1v8_pwr_off);
#endif

//Porting : customize for your display panel's timing.
/* Video Timings */
static struct omap_video_timings quickvx_timings = {
	.x_res = 800, //480,			/* X Resolution in Pixels */
	.y_res = 480, //640,			/* Y Resolution in Pixels */

	.pixel_clock	= 55757, //19200,	/* Pixel Clock in kHz */

	.hsw		= 30,
	.hfp		= 20,
	.hbp		= 30,

	.vsw		= 20,
	.vfp		= 18,
	.vbp		= 10,
};

static int panel_eyepiece_i2c_read(struct i2c_adapter *adapter, char reg, char *rxData)
{
	int ret = -ENODEV;
	unsigned char buf[2];
	buf[0] = reg;	
	struct i2c_msg msgs[2] = {
		{
		 .addr = EYEPIECE_I2C_ADDR,
		 .flags = 0,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = EYEPIECE_I2C_ADDR,
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = rxData,
		 },
	};

	ret = i2c_transfer(adapter, msgs, 2);
	if (ret != 2){
		printk(KERN_WARNING  "i2c read reg failed, slave addr:0x%x\n", EYEPIECE_I2C_ADDR);
		return ret;
	}
	return 0;
}
static int panel_eyepiece_i2c_write(struct i2c_adapter *adapter, char *txData, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = EYEPIECE_I2C_ADDR,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	if (i2c_transfer(adapter, msg, 1) < 0) {
		printk(KERN_WARNING "panel-eyepiece: i2c write reg failed\n");
		return -EIO;
	} else{
		return 0;
	}
}

static char init_data[][2] = 
{
	{0x00, 0x01},
	{0x02, 0x04},
	{0x03, 0x01}, //VCOMPN Setting
	{0x04, 0x0A}, //VCOM Setting
	{0x05, 0x01}, //VCOMPN Setting
	{0x06, 0x0A}, //VCOM Setting
	{0x07, 0xF9}, //VRINGP Setting
	{0x08, 0x05}, //VRINGN Setting
	{0x09, 0xFF}, //RGMAP1 Setting
	{0x0A, 0x84}, //RGMAP2 Setting
	{0x0B, 0x68}, //RGMAP3 Setting
	{0x0C, 0x54}, //RGMAP4 Setting
	{0x0D, 0x34}, //RGMAP5 Setting
	{0x0E, 0x0A}, //RGMAP6 Setting
	{0x0F, 0x00}, //RGMAN1 Setting
	{0x10, 0x7A}, //RGMAN2 Setting
	{0x11, 0x96}, //RGMAN3 Setting
	{0x12, 0xAA}, //RGMAN4 Setting
	{0x13, 0xCA}, //RGMAN5 Setting
	{0x14, 0xFF}, //RGMAN6 Setting
	{0x15, 0xFF}, //GGMAP1 Setting
	{0x16, 0x84}, //GGMAP2 Setting
	{0x17, 0x68}, //GGMAP3 Setting
	{0x18, 0x54}, //GGMAP4 Setting
	{0x19, 0x34}, //GGMAP5 Setting
	{0x1A, 0x0A}, //GGMAP6 Setting
	{0x1B, 0x00}, //GGMAN1 Setting
	{0x1C, 0x7A}, //GGMAN2 Setting
	{0x1D, 0x96}, //GGMAN3 Setting
	{0x1E, 0xAA}, //GGMAN4 Setting
	{0x1F, 0xCA}, //GGMAN5 Setting
	{0x20, 0xFF}, //GGMAN6 Setting
	{0x21, 0xFF}, //BGMAP1 Setting
	{0x22, 0x84}, //BGMAP2 Setting
	{0x23, 0x68}, //BGMAP3 Setting
	{0x24, 0x54}, //BGMAP4 Setting
	{0x25, 0x34}, //BGMAP5 Setting
	{0x26, 0x0A}, //BGMAP6 Setting
	{0x27, 0x00}, //BGMAN1 Setting
	{0x28, 0x7A}, //BGMAN2 Setting
	{0x29, 0x96}, //BGMAN3 Setting
	{0x2A, 0xAA}, //BGMAN4 Setting
	{0x2B, 0XCA}, //BGMAN5 Setting
	{0x2C, 0xFF}, //BGMAN6 Setting

};

static void panel_eyepiece_init()
{
	struct i2c_adapter *adapter;
	u8 * init_datap = &(init_data[0][0]);
	int i, ret;
	int retry_times = I2C_RETRY_TIMES;
	u8 reg_val;

	adapter = i2c_get_adapter(I2C_BUS_NUMBER);
	if (!adapter) {
		printk("panel-eyepiece: Failed to get I2C adapter, bus %d\n", I2C_BUS_NUMBER);
		return;
	}
	#if 0//test
	panel_eyepiece_i2c_read(adapter, 2, &reg_val);
	printk("ljh:panel_eyepiece_init: reg: %d, val:0x%x\n", 2, reg_val);
	#endif
	/*init panel reg*/
	for (i = 0; i < ARRAY_SIZE(init_data); i++)
	{
retry:
		ret = panel_eyepiece_i2c_write(adapter,init_datap, 2);
		if(ret && retry_times > 0){
			retry_times --;
			msleep(100);
			goto retry;
		}
		init_datap += 2;
	}
	#if 0//test
	panel_eyepiece_i2c_read(adapter, 2, &reg_val);
	printk("ljh:panel_eyepiece_init: reg: %d, val:0x%x after writing\n", 2, reg_val);
	#endif
	if(!ret)
		printk("panel-eyepiece: panel init done\n");
}

static void panel_eyepiece_deinit()
{
	struct i2c_adapter *adapter;
	u8 write_data[2] = {0x02, 0x00};
	int ret;
	int retry_times = I2C_RETRY_TIMES;

	adapter = i2c_get_adapter(I2C_BUS_NUMBER);
	if (!adapter) {
		printk("panel-eyepiece: Failed to get I2C adapter, bus %d\n", I2C_BUS_NUMBER);
		return;
	}

deinit_retry:
	ret = panel_eyepiece_i2c_write(adapter,write_data, 2);
	if(ret && retry_times > 0){
		retry_times --;
		msleep(100);
		goto deinit_retry;
	}

	if(!ret)
		printk("panel-eyepiece: panel deinit done\n");
}

static int quickvx_bldev_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	QL_DBG_FUNC_ENTER
	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}


static int quickvx_bldev_get_brightness(struct backlight_device *bl)
{
	QL_DBG_FUNC_ENTER
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}


static const struct backlight_ops quickvx_bldev_ops = {
	.get_brightness = quickvx_bldev_get_brightness,
	.update_status  = quickvx_bldev_update_status,
};


static void quickvx_mipi_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	QL_DBG_FUNC_ENTER
	*timings = dssdev->panel.timings;
}


static void quickvx_mipi_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	QL_DBG_FUNC_ENTER
	dev_info(&dssdev->dev, "set_timings() not implemented\n");
}


static int quickvx_mipi_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	QL_DBG_FUNC_ENTER
	if (quickvx_timings.x_res != timings->x_res ||
			quickvx_timings.y_res != timings->y_res ||
			quickvx_timings.pixel_clock != timings->pixel_clock ||
			quickvx_timings.hsw != timings->hsw ||
			quickvx_timings.hfp != timings->hfp ||
			quickvx_timings.hbp != timings->hbp ||
			quickvx_timings.vsw != timings->vsw ||
			quickvx_timings.vfp != timings->vfp ||
			quickvx_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}


static void quickvx_mipi_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	//QL_DBG_FUNC_ENTER
	*xres = quickvx_timings.x_res;
	*yres = quickvx_timings.y_res;
}


static int quickvx_mipi_hw_reset(struct omap_dss_device *dssdev)
{
	QL_DBG_FUNC_ENTER
	if (dssdev == NULL || dssdev->reset_gpio == -1)
		return 0;

	gpio_set_value(dssdev->reset_gpio, 1);
	udelay(100);
	/* Reset the panel */
	gpio_set_value(dssdev->reset_gpio, 0);
	/* Assert reset */
	udelay(100);
	gpio_set_value(dssdev->reset_gpio, 1);

	/* Wait after releasing reset */
	msleep(100);

	return 0;
}

static void quickvx_hw_power_init(void)
{
#ifndef CONFIG_REGULATOR_GPIO_SWITCH
	gpio_request(GPIO_EN1V2, "GPIO_EN1V2");
	gpio_request(GPIO_EN1V8, "GPIO_EN1V8");
#endif
	gpio_request(GPIO_DISP_OE_RST, "GPIO_DISP_OE_RST");
	gpio_request(GPIO_BRG_RSTN, "GPIO_BRG_RSTN");
	gpio_request(GPIO_EN3V3, "GPIO_EN3V3");
	gpio_request(GPIO_EN10V, "GPIO_EN10V");

#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	int ret;
	vdd_1v2 = regulator_get(NULL, "lcos_v1v2");
	if (IS_ERR_OR_NULL(vdd_1v2)) {
		vdd_1v2 = NULL;
		printk(KERN_ERR "Failed to initialize vdd 1.2v regulator..\n");
		return;
	}
	ret = regulator_enable(vdd_1v2);
	if (ret < 0) {
		printk(KERN_ERR "Failed to control  vdd 1.2v  regulator..\n");
		return;
	}
#else
	gpio_direction_output(GPIO_EN1V2 ,1);
#endif
	msleep(20);
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	vdd_1v8 = regulator_get(NULL, "lcos_v1v8");
	if (IS_ERR_OR_NULL(vdd_1v8)) {
		vdd_1v8 = NULL;
		printk(KERN_ERR "Failed to initialize vdd 1.8v regulator..\n");
		return;
	}
	ret = regulator_enable(vdd_1v8);
	if (ret < 0) {
		printk(KERN_ERR "Failed to control  vdd 1.8v  regulator..\n");
		return;
	}
#else
	gpio_direction_output(GPIO_EN1V8, 0);
#endif
	msleep(20);

#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
	gpio_direction_output(GPIO_EN3V3, 1);
	msleep(20);

	gpio_direction_output(GPIO_EN10V, 1);
	msleep(20);

	gpio_direction_output(GPIO_BRG_RSTN, 1);
	msleep(20);

	gpio_direction_output(GPIO_DISP_OE_RST, 1);
	msleep(20);	
#endif
}

static int quickvx_mipi_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct quickvx_data *qd;
	int r = 0;

	QL_DBG_FUNC_ENTER
	dev_dbg(&dssdev->dev, "quickvx_mipi_probe\n");
	quickvx_hw_power_init();
	
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.dsi_pix_fmt = OMAP_DSS_DSI_FMT_RGB888;
	dssdev->panel.timings = quickvx_timings;
	dssdev->panel.acbi = 0;		/* TODO: Need to verify. AC-bias pin transitions per interrupt */
	dssdev->panel.acb = 0;		/* TODO: Need to verify. AC-bias pin frequency in terms of line clocks */

	qd = kzalloc(sizeof(*qd), GFP_KERNEL);
	if (!qd) {
		r = -ENOMEM;
		goto err;
	}
	qd->vdd_1v2_on = true;
	qd->vdd_1v8_on = true;
	qd->dssdev = dssdev;
	saved_dssdev = dssdev;
	dev_set_drvdata(&dssdev->dev, qd);
	//just for debug 
	qd->clk = clk_get(NULL, "auxclk2_ck");
	if (IS_ERR(qd->clk)) {
		  pr_err("Cannot request auxclk2\n");
		  goto err;
    	} 

	r = clk_set_rate(qd->clk, 19200000);
	if (r) {
		pr_err("unable to set clk to 19200000\n");
		goto err;
	}	

	r = clk_enable(qd->clk);
	if (r) {
		pr_err("Failed to enable sys_clk\n");
		goto err;
	}
	qd->clk_on = true;
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
#ifndef QL_CHIP_INIT_EXTERNAL
	uint32_t val = 0x8899;
#endif

	ql_i2c_read(0x700, &val, 4);
	QL_DBG("chengqm: 0x%x\n", val);

#ifndef QL_CHIP_INIT_EXTERNAL
	ql_i2c_read(0x4fe, &val, 2); 
	QL_DBG("VEE ID 0x%x\n", val);
	ql_i2c_read(0x448, &val, 4);
	ql_i2c_write(0x448, 0x12345678, 4); 
	ql_i2c_read(0x448, &val, 4);
	QL_DBG("R/W Test 0x%x\n", val);
	ql_i2c_read(0x700, &val, 4);
#if 0
	/* Init client */
	ql_i2c_read(0x218, &val, 4); 
	QL_DBG("Turn around timeout counter register 0x%x\n", val);
	ql_i2c_read(0x204, &val, 4); 
	QL_DBG("Interrupt status register 0x%x\n", val);
	ql_i2c_write(0x204, 0xFFFFFFFF, 4); 
	ql_i2c_write(0x218, 0x17, 4); 
	ql_i2c_write(0x238, 0x0060, 4); 
	ql_i2c_write(0x234, 0xCA033A10 , 4); 
	ql_i2c_write(0x240, 0x28614088, 4); 
#endif
	ql_chip_init(NULL);	/* First init don't have struct omap_dss_device *dssdev yet */
	ql_i2c_read(0x448, &val, 4);
	ql_i2c_read(0x700, &val, 4);
	ql_i2c_release();
#endif
#endif

#ifdef QL_BACKLIGHT
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register("quickvx_bl", &dssdev->dev, dssdev,
			&quickvx_bldev_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		goto err;
	}
	qd->quickvx_bldev = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = quickvx_bldev_update_status(bl);
	if (r) {
		dev_err(&dssdev->dev, "Failed to set lcd brightness\n");
	}
#endif
	r = omap_dsi_request_vc(dssdev, &qd->channel0);
	if (r) {
		dev_err(&dssdev->dev, "Failed to get virtual channel0\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, qd->channel0, 0);
	if (r) {
		dev_err(&dssdev->dev, "Failed to set VC_ID0\n");
		goto err;
	}
#ifdef QL_USE_CH1
	r = omap_dsi_request_vc(dssdev, &qd->channel1);
	if (r) {
		dev_err(&dssdev->dev, "Failed to get virtual channel1\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, qd->channel1, 0);
	if (r) {
		dev_err(&dssdev->dev, "Failed to set VC_ID1\n");
		goto err;
	}
#endif
#ifdef CONFIG_QUICKVX_SYSFS_I2C_ACCESS
	r = quickvx_mipi_sysfs_register(&dssdev->dev);
	if (r) {
	        dev_err(&dssdev->dev, "Failed to register mipi sysfs\n");
		goto err;
	}
#endif
	QL_DBG_FUNC_EXIT
	dev_dbg(&dssdev->dev, "quickvx_mipi_probe done\n");
	return 0;
err:
	QL_DBG("err!\n");

	kfree(qd);
	return r;
}

static void quickvx_mipi_remove(struct omap_dss_device *dssdev)
{
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
#ifdef QL_BACKLIGHT
	struct backlight_device *bl = qd->quickvx_bldev;
#endif

#ifdef CONFIG_QUICKVX_SYSFS_I2C_ACCESS
	quickvx_mipi_sysfs_unregister(&dssdev->dev);
#endif

	omap_dsi_release_vc(dssdev, qd->channel0);
#ifdef QL_USE_CH1
	omap_dsi_release_vc(dssdev, qd->channel1);
#endif
#ifdef QL_BACKLIGHT
	bl->props.power = FB_BLANK_POWERDOWN;
	quickvx_bldev_update_status(bl);
	backlight_device_unregister(bl);
#endif
	kfree(qd);
}


static int quickvx_mipi_power_on(struct omap_dss_device *dssdev)
{
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	int r;
	uint32_t val = 0;
	static bool is_first_power_on = true;

	QL_DBG_FUNC_ENTER
	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;
	dev_dbg(&dssdev->dev, "power_on\n");
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (is_first_power_on) {
		/*turn off display backlight while initiliazing dsi to avoid screen flickering*/
		gpio_direction_output(GPIO_EN3V3, 0);
		omapdss_dsi_display_deinit(dssdev);
	}
#endif
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (!dssdev->skip_init) {
#endif
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);
	if(qd->clk_on == false){
		r = clk_enable(qd->clk);
		if (r) {
			pr_err("Failed to enable quickvx working clock\n");
			goto err_disp_enable;
		}
		qd->clk_on = true;
	}
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	}
#endif
	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}
	//omapdss_dsi_vc_enable_hs(dssdev, qd->channel0, true);

	/* Reset quickvx bridge */
	//TODO..laster quickvx_mipi_hw_reset(dssdev);
	/* Do extra job to match kozio registers (???) */
//FIXME.	dsi_videomode_panel_preinit(dssdev);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (!dssdev->skip_init) {
#endif
	panel_eyepiece_init();
	ql_i2c_read(0x4fe, &val, 2); 
	ql_i2c_read(0x700, &val, 4);
	ql_chip_init(dssdev);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	}
#endif
#ifdef QL_USE_CH1
 	 omapdss_dsi_vc_enable_hs(dssdev, qd->channel1, true);
     //try set max return packet to 4 bytes.
     dsi_vc_set_max_rx_packet_size(dssdev, qd->channel1, 0x4);
#else
     //try set max return packet to 4 bytes.
      omapdss_dsi_vc_enable_hs(dssdev, qd->channel0, true);
      dsi_vc_set_max_rx_packet_size(dssdev, qd->channel0, 0x4);
#endif

	r = dsi_enable_video_output(dssdev, qd->channel0);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (!dssdev->skip_init) {
#endif
	ql_i2c_write(0x204, 0xffffffff, 4); 
	ql_i2c_write(0x148, 0xffffffff, 4); 
	msleep(2);
	ql_i2c_read(0x204, &val, 4);
	ql_i2c_read(0x148, &val, 4);
#if 0
	//check DPI Fifo Overrun.
	ql_i2c_write(0x204, 0xffffffff, 4); 
	ql_i2c_read(0x70c, &val, 4); 
#if 1
	ql_i2c_write(0x70c, 0x5644, 0x4);
	ql_i2c_write(0x154, 0x0, 0x4);
	ql_i2c_write(0x154, 0x80000000, 0x4);
#endif
	ql_i2c_read(0x204, &val, 4); 
	ql_i2c_write(0x204, 0xffffffff, 4); 
	msleep(2);
	ql_i2c_read(0x204, &val, 4); 
#endif
	ql_i2c_release();
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	}
#endif
	/*turn on display backlight*/
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if(!is_first_power_on)
#endif
	gpio_direction_output(GPIO_EN3V3, 1);
	is_first_power_on = false;
	dev_dbg(&dssdev->dev, "power_on done\n");
	QL_DBG_FUNC_EXIT
	return r;

err_disp_enable:
	QL_DBG("quickvx_mipi_power_on   error\n");
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void quickvx_mipi_power_off(struct omap_dss_device *dssdev)
{
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	QL_DBG_FUNC_ENTER
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (!dssdev->skip_init) {
#endif
	/*turn off display backlight*/
	gpio_direction_output(GPIO_EN3V3, 0);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	}
#endif
	dsi_disable_video_output(dssdev, qd->channel0);
	omapdss_dsi_display_disable(dssdev, false, false);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (!dssdev->skip_init) {
#endif
	panel_eyepiece_deinit();
	if(qd->clk_on == true){
		clk_disable(qd->clk);
		qd->clk_on = false;
	}
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	}
#endif

}


static void quickvx_mipi_disable(struct omap_dss_device *dssdev)
{
	QL_DBG_FUNC_ENTER
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		// MSI: porting backlight power control
		//gpio_direction_output(41, 0);

		dsi_bus_lock(dssdev);
		quickvx_mipi_power_off(dssdev);
		dsi_bus_unlock(dssdev);
	}
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}


static int quickvx_mipi_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	QL_DBG_FUNC_ENTER
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;
	dsi_bus_lock(dssdev);
	r = quickvx_mipi_power_on(dssdev);
	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		// MSI: porting backlight power control
		//gpio_direction_output(41, 1);
	}

	return r;
}


static int quickvx_mipi_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	QL_DBG_FUNC_ENTER
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (dssdev->skip_init) {
		dssdev->skip_init = false;
	}
#endif
	quickvx_mipi_disable(dssdev);
	return 0;
}


static int quickvx_mipi_resume(struct omap_dss_device *dssdev)
{

	/* Enable the panel and return 0 */
	QL_DBG_FUNC_ENTER
	quickvx_mipi_enable(dssdev);
	return 0;
}


/* QuickLogic MIPI DSS driver */
static struct omap_dss_driver quickvx_mipi_dss_driver = {
	.probe		= quickvx_mipi_probe,
	.remove		= quickvx_mipi_remove,

	.enable		= quickvx_mipi_enable,
	.disable	= quickvx_mipi_disable,
	.suspend	= quickvx_mipi_suspend,
	.resume		= quickvx_mipi_resume,

	.get_resolution	= quickvx_mipi_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= quickvx_mipi_get_timings,
	.set_timings	= quickvx_mipi_set_timings,
	.check_timings	= quickvx_mipi_check_timings,

	.driver         = {
		.name   = "quickvx_mipi",
		.owner  = THIS_MODULE,
	},
};


#define GEN_QL_CSR_OFFSET_LENGTH_PAYLOAD  {\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x41,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* Length LS */\
        0x00,  /* Length MS */\
    }

int ql_mipi_read(struct omap_dss_device *dssdev, uint32_t address, uint32_t *regval, uint32_t data_size)
{
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	int r;
	char buf[] = GEN_QL_CSR_OFFSET_LENGTH_PAYLOAD;
	char rx[10];
	uint32_t data;
	//u16 cmd;

	buf[3] = (uint8_t)(address & 0xff);	/* Address LS */
	buf[4] = (uint8_t)(address >> 8);	/* Address MS */
	buf[5] = (uint8_t) data_size;		/* Value LSB */
	buf[6] = 0;

	/* Send the generic write csr address and offset first
	 * return number of tx command */
	dsi_bus_lock(dssdev);
#ifdef QL_USE_CH1
	r = dsi_vc_generic_write(dssdev, qd->channel1, buf, 7);
#else
	r = dsi_vc_generic_write(dssdev, qd->channel0, buf, 7);
#endif
	dsi_bus_unlock(dssdev);

	if (r) {
		QL_ERR("reg read reg(%x) GEN_QL_CSR_OFFSET_LENGTH_PAYLOAD failed: %d\n",
			       address, r);
		return r;
	}

	// gen read.
	//cmd = 0x0105;//ljh
	dsi_bus_lock(dssdev);
#ifdef QL_USE_CH1
	//r = dsi_vc_gen_read_2(dssdev, qd->channel1, cmd, rx, data_size);
	r = dsi_vc_generic_read_0(dssdev, qd->channel1, rx, data_size);
#else
	//r = dsi_vc_gen_read_2(dssdev, qd->channel0, cmd, rx, data_size);
	r = dsi_vc_generic_read_0(dssdev, qd->channel0, rx, data_size);
#endif
	dsi_bus_unlock(dssdev);
	if (r < 0) {
		QL_ERR("reg read reg(%x) GEN READ failed with status %d\n",
								address, r);
		return r;
	}

	data = rx[0];
	if (data_size > 1)
		data |= ( rx[1] << 8) ;
	if (data_size > 2)
		data |= (rx[2] << 16) | (rx[3] << 24);

	*regval = data;

	QL_DBG("R[0x%x]=0x%x\n", address, *regval);

	return 0;
}

#define QL_CSR_WRITE_PAYLOAD  {\
	0x05,  /* Vendor Id 1 */\
	0x01,  /* Vendor Id 2 */\
	0x40,  /* Vendor Unique Command */\
	0x00,  /* Address LS */\
	0x00,  /* Address MS */\
	0x00,  /* data LS */\
	0x00, \
	0x00, \
	0x00,  /* data MS */\
}

int ql_mipi_write(struct omap_dss_device *dssdev, uint32_t address, uint32_t value, uint32_t data_size)
{
	struct quickvx_data *qd = dev_get_drvdata(&dssdev->dev);
	int r, write_size;
	char buf[] = QL_CSR_WRITE_PAYLOAD;

	QL_DBG("W[0x%x]=0x%x\n", address, value);

	buf[3] = (uint8_t)(address & 0xff);  /* Address LS */
	buf[4] = (uint8_t)(address >> 8);  /* Address MS */
	buf[5] = (uint8_t) value;  /* value LSB */
	if (data_size > 1)
		buf[6] = (uint8_t)(value >> 8); 
	if (data_size > 2) {
		buf[7] = (uint8_t)(value >> 16); 
		buf[8] = (uint8_t)(value >> 24); 
	}

	write_size = 5 + data_size;
	
	dsi_bus_lock(dssdev);
#ifdef QL_USE_CH1
	r = dsi_vc_generic_write(dssdev, qd->channel1, buf, write_size);
#else
	r = dsi_vc_generic_write(dssdev, qd->channel0, buf, write_size);
#endif
	dsi_bus_unlock(dssdev);
	if (r)
		QL_ERR("reg write reg(%x) val(%x) failed: %d\n",
			       address, value, r);
	return r;

}

/* i2c access */
struct i2c_client *quickvx_i2c_client = NULL;

#if 0
/* quickvx i2c address is 0x64 (depending on gpio pin)
 * The platform has to define i2c_board_info and call i2c_register_board_info()
 */
static struct i2c_board_info quickvx_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("quickvx_i2c", 0x64),
	},
};
#endif

#define CONTROL_BYTE_GEN       (0x09u)
#define CONTROL_BYTE_DCS       (0x08u)
#define CONTROL_BYTE_I2C_RELEASE (0x0u)

#define QL_I2C_RELEASE  {\
		CONTROL_BYTE_I2C_RELEASE, \
}


int ql_i2c_release(void)
{
	int write_size;
	int ret = -1;
	char buf[] = QL_I2C_RELEASE;
	if(quickvx_i2c_client == NULL){
		printk("ql_i2c_release: Failed to get I2C adapter, bus 3\n");
		return -1;
	}

	write_size = 1;
	if ((ret = i2c_master_send(quickvx_i2c_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	return 0;
}

#define GEN_QL_CSR_OFFSET_LENGTH  {\
	CONTROL_BYTE_GEN, \
	0x29,  /* Data ID */\
	0x05,  /* Vendor Id 1 */\
	0x01,  /* Vendor Id 2 */\
	0x41,  /* Vendor Unique Command */\
	0x00,  /* Address LS */\
	0x00,  /* Address MS */\
	0x00,  /* Length LS */\
	0x00,  /* Length MS */\
}

#define GEN_QL_CSR_WRITE  {\
	CONTROL_BYTE_GEN, \
	0x29,  /* Data ID */\
	0x05,  /* Vendor Id 1 */\
	0x01,  /* Vendor Id 2 */\
	0x40,  /* Vendor Unique Command */\
	0x00,  /* Address LS */\
	0x00,  /* Address MS */\
	0x00,  /* data LS */\
	0x00, \
	0x00, \
	0x00,  /* data MS */\
}


int ql_i2c_read(uint32_t addr, uint32_t *val, uint32_t data_size) 
{
	uint32_t data;
	char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
	char rx[10];
	int ret = -1;
	int write_size;
	int retry_times = I2C_RETRY_TIMES;
	if(quickvx_i2c_client == NULL){
		printk("ql_i2c_read: Failed to get I2C adapter, bus 3\n");
		return -1;
	}
	buf[5] = addr & 0xff;
	buf[6] = (addr >> 8) & 0xff;
	buf[7] = data_size & 0xff;
	buf[8] = (data_size >> 8) & 0xff;

	write_size = 9;
retry1:
	if ((ret = i2c_master_send( quickvx_i2c_client,
		(char*)(&buf[0]),
		write_size )) != write_size) {
		printk(KERN_ERR "%s: i2c_master_send GEN failed (%d)!\n", __func__, ret);
		if(retry_times -- > 0)
			goto retry1;
		return -1;
	}
	retry_times = I2C_RETRY_TIMES;
	/* Generic read request 0x24 to send generic read command */
	write_size = 4;

	buf[0] = CONTROL_BYTE_GEN;
	buf[1] =    0x24;  /* Data ID */
	buf[2] =    0x05;  /* Vendor Id 1 */
	buf[3] =    0x01;  /* Vendor Id 2 */
retry2:
	if ((ret = i2c_master_send( quickvx_i2c_client,
		(char*)(&buf[0]),
		write_size )) != write_size) {
		printk(KERN_ERR "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		if(retry_times -- > 0)
			goto retry2;
		return -1;
	}
	retry_times = I2C_RETRY_TIMES;
retry3:
	/* Return number of bytes or error */
	if ((ret = i2c_master_recv( quickvx_i2c_client,
		(char*)(&rx[0]),
		data_size )) != data_size) {
		printk(KERN_ERR "%s: i2c_master_recv failed (%d)!\n", __func__, ret);
		if(retry_times -- > 0)
			goto retry3;
		return -1;
	}


	data = rx[0];
	if (data_size > 1) 
	data |= (rx[1] << 8);
	if (data_size > 2)
	data |= (rx[2] << 16) | (rx[3] << 24);

	*val = data;

	QL_DBG("r0x%x=0x%x\n",addr,data);

	return 0;
}


int ql_i2c_write(long addr, long val, int data_size)
{
	int write_size;
	int ret = -1;
	int retry_times = I2C_RETRY_TIMES;
	char buf[] = GEN_QL_CSR_WRITE;
	if(quickvx_i2c_client == NULL){
		printk("ql_i2c_write: Failed to get I2C adapter, bus 3\n");
		return -1;
	}
	//QL_DBG("w0x%lx=0x%lx\n",addr,val);

	buf[5] = (uint8_t)addr;  /* Address LS */
	buf[6] = (uint8_t)(addr >> 8);  /* Address MS */

	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 2) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 7;
	if ((ret = i2c_master_send( quickvx_i2c_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send GEN failed (%d)!\n", __func__, ret);
		return -1;
	}
	return 0;
}

static int __devinit quickvx_i2c_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	int status, r;
	/* Store i2c_client pointer */
	quickvx_i2c_client = client;
	QL_DBG("probing adapter: %s, address: 0x%x\n", adapter->name, client->addr);

	return 0;
}


/* QuickLogic i2c client ID table */
static const struct i2c_device_id quickvx_i2c_idtable[] = {
	{"quickvx_i2c", 0},
	{},
};

/* QuickLogic i2c driver */
static struct i2c_driver quickvx_i2c_driver = {
	.probe     = quickvx_i2c_probe,
	.id_table  = quickvx_i2c_idtable,
	.driver    = {
		.name  = "quickvx_i2c",
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_QUICKVX_SYSFS_I2C_ACCESS
/* VX register sysfs interface */
u32 mipi_read_data = 0;

static ssize_t show_quickvx_mipi_driver_info(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	QL_DBG("+++\n");

	return snprintf(buf, PAGE_SIZE, "%s\n", QLVX_DRIVER_VER);
}

static ssize_t show_quickvx_mipi_read(struct device *dev,
            struct device_attribute *attr,
            char *buf)
{

	QL_DBG("data = 0x%x\n", mipi_read_data);

	return sprintf(buf, "0x%x\n", mipi_read_data);
}

static ssize_t store_quickvx_mipi_read(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
	int ret = 0;
	u32 reg;
	u32 data_size;

	if (sscanf(buf, "%x %x", (u32 *)&reg, (u32 *)&data_size) != 2) {
		dev_err(dev, "%s \n",__func__);
		ret = -EINVAL;
	} else {


		QL_DBG("reg:0x%x size:0x%x\n", reg, data_size);
		mipi_read_data = 0; // clear the data first.

#ifdef QL_VX_SYSFS_I2C_ACCESS_READ
		//i2c access
		ret = ql_i2c_read(reg, &mipi_read_data, data_size);
		ql_i2c_release();

#else
		if (!saved_dssdev)
			return -ENODEV;
		ret = ql_mipi_read(saved_dssdev, reg, &mipi_read_data, data_size);
#endif

		if (ret){
			dev_dbg(dev, "%s: failed to read data\n", __func__);
			return -ENODEV;
		}

		QL_DBG("data = 0x%x\n", mipi_read_data);
	}
	return count; //ret;
}


static ssize_t store_quickvx_mipi_write(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
	int ret = -1;
	u32 reg;
	u32 data = 0;
	u32 data_size;

	if (sscanf(buf, "%x %x %x", (u32 *)&reg, (u32 *)&data, (u32 *)&data_size) != 3) {
		dev_err(dev, "%s\n",__func__);
		return -EINVAL;
	} else {
		QL_DBG("reg:0x%x value:0x%x size:0x%x\n", reg, data, data_size);
#ifdef	QL_VX_SYSFS_I2C_ACCESS_WRITE
		//i2c access
		ret = ql_i2c_write(reg, data, data_size);
		ql_i2c_release();
#else
		if (!saved_dssdev)
		return -ENODEV;
		ret = ql_mipi_write(saved_dssdev, reg, data, data_size);
#endif
		if (ret) {
			dev_err(dev, "%s write failed, ret = %d\n", __func__, ret);
			return -ENODEV;
		}
	}

	return count;
}

static ssize_t store_quickvx_mipi_dsi_read(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
	int ret = 0;
	u32 reg;
	u32 data_size;

	if (sscanf(buf, "%x %x", (u32 *)&reg, (u32 *)&data_size) != 2) {
		dev_err(dev, "%s \n",__func__);
		ret = -EINVAL;
	} else {

		QL_DBG("reg:0x%x size:0x%x\n", reg, data_size);

		if (!saved_dssdev)
			return -ENODEV;
		mipi_read_data = 0; // clear the data first.
		ret = ql_mipi_read(saved_dssdev, reg, &mipi_read_data, data_size);

		if (ret){
			dev_dbg(dev, "%s: failed to read data\n", __func__);
			return -ENODEV;
		}

		QL_DBG("data = 0x%x\n", mipi_read_data);
	}
	return count; //ret;
}


static ssize_t store_quickvx_mipi_dsi_write(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
	int ret = -1;
	u32 reg;
	u32 data = 0;
	u32 data_size;

	if (sscanf(buf, "%x %x %x", (u32 *)&reg, (u32 *)&data, (u32 *)&data_size) != 3) {
		dev_err(dev, "%s\n",__func__);
		return -EINVAL;
	} else {
		QL_DBG("reg:0x%x value:0x%x size:0x%x\n", reg, data, data_size);

		if (!saved_dssdev)
			return -ENODEV;
		ql_i2c_release();
		ret = ql_mipi_write(saved_dssdev, reg, data, data_size);

		if (ret) {
			dev_err(dev, "%s write failed, ret = %d\n", __func__, ret);
			return -ENODEV;
		}
	}

	return count;
}

static struct device_attribute quickvx_mipi_attributes[] = {
    __ATTR(ql_driver_info, 0444, show_quickvx_mipi_driver_info, NULL),
    __ATTR(ql_read_reg, 0644, show_quickvx_mipi_read, store_quickvx_mipi_read),
    __ATTR(ql_write_reg, 0644, NULL, store_quickvx_mipi_write),
    __ATTR(ql_dsi_read_reg, 0644, show_quickvx_mipi_read, store_quickvx_mipi_dsi_read),
    __ATTR(ql_dsi_write_reg, 0644, NULL, store_quickvx_mipi_dsi_write),
};

static int quickvx_mipi_sysfs_register(struct device *dev)
{
	int i;

	QL_DBG("%s ++\n",__func__);

	for (i = 0; i < ARRAY_SIZE(quickvx_mipi_attributes); i++) {
	if (device_create_file(dev, quickvx_mipi_attributes + i))
	    goto error;
	}

	QL_DBG("%s success!\n",__func__);
	return 0;

error:
	for (; i >= 0 ; i--)
		device_remove_file(dev, quickvx_mipi_attributes + i);

	QL_DBG("%s: Unable to create interface\n", __func__);

    return -ENODEV;
}


static int quickvx_mipi_sysfs_unregister(struct device *dev)
{
	int i;

	QL_DBG("%s ++\n",__func__);

	for (i = 0; i < ARRAY_SIZE(quickvx_mipi_attributes); i++) {
		device_remove_file(dev, quickvx_mipi_attributes + i);
	}

	QL_DBG("%s success!\n",__func__);
	return 0;
}

#endif //CONFIG_QUICKVX_SYSFS_I2C_ACCESS

/* QuickVX Register Settings */
struct chip_init_data {
	int reg_addr; 
	int data;
};

//Porting: chip init seq. here.
struct chip_init_data vx3_init_data[] = {
0x700 , 0x40900040 ,
0x704 , 0x1F01E3 ,
0x70C , 0x4604 ,
0x710 , 0x5CD0007 ,
0x714 , 0x0 ,
0x718 , 0x00000102 ,
0x71C , 0xA800B ,
0x720 , 0x0 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x700 , 0x40900040 ,
0x70C , 0x5E26 ,
0x718 , 0x00000002 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x120 , 0x5 ,
0x124 , 0x74F0320 ,
0x128 , 0x10480A ,
0x12C , 0x1D ,
0x130 , 0x3C10 ,
0x134 , 0x5 ,
0x138 , 0xFF8000 ,
0x13C , 0x0 ,
0x140 , 0x10000 ,
0x20C , 0x22 ,
0x21C , 0x780 ,
0x224 , 0x0 ,
0x228 , 0x50000 ,
0x22C , 0xFF06 ,
0x230 , 0x1 ,
0x234 , 0xCA033E10 ,
0x238 , 0x00000060 ,
0x23C , 0x82E86030 ,
0x244 , 0x001E0285 ,
0x258 , 0x14001D ,
0x158 , 0x0 ,
0x158 , 0x1 ,
0x37C , 0x00001063 ,
0x380 , 0x82A86030 ,
0x384 , 0x2861408B ,
0x388 , 0x00130285 ,
0x38C , 0x10630009 ,
0x394 , 0x400B82A8 ,
//0x600 , 0x16CC78C ,
//0x604 , 0x3FFFFFE0 ,
0x608 , 0x50F ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
// internel debug signal
#if 0
0x710, 0x4d000f,
0x71c, 0xa8002b,
0x70c, 0x5666,
0x130, 0x3c78,
0x134, 0x5,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
#endif

#if 0
//don't remove this. Need to restart the host after the init.......
	{0x300,0x00000000},
	{0x300,0x00000001},
	{0x304,0xffffffff},
	{0x204,0xffffffff},
	{0x148,0xffffffff},
#endif
};


void ql_chip_init(struct omap_dss_device *dssdev)
{
	int i;
	int reg_addr;
	int data;
	printk(KERN_ERR "ql_chip_init, dssdev:%p\n", dssdev);
	for (i=0; i<(sizeof(vx3_init_data)/sizeof(struct chip_init_data)); i++) {
		reg_addr = vx3_init_data[i].reg_addr;
		data = vx3_init_data[i].data;
		if (dssdev == NULL) {
			ql_i2c_write(reg_addr, data, 4); 
		} else {
			ql_i2c_write(reg_addr, data, 4); 
			//ql_mipi_write(dssdev, reg_addr, data, 4);// ljh, check this
	}
#if 0
			/* if you are using 32kHz sys_clk, and seeing i2c error after the first 0x154 write,
				you may try enable this, and adjust the delay below, now it's 0.5 seconds */
			if ((addr == 0x154) && (data == 0x80000000))
				MDELAY(500);
#endif
	}

	ql_i2c_release();
	return;
}

static int __init quickvx_mipi_dss_init(void)
{
	int ret;
	//QL_DBG("%s-%s\n", __DATE__, __TIME__);
	ret = i2c_add_driver(&quickvx_i2c_driver);
	if (ret < 0) {
		printk(KERN_ERR "%s: quickvx i2c driver registration failed!\n", __func__);
		return ret;
	}
	ret = omap_dss_register_driver(&quickvx_mipi_dss_driver);
	return 0;
}

static void __exit quickvx_mipi_dss_exit(void)
{
	omap_dss_unregister_driver(&quickvx_mipi_dss_driver);
	i2c_del_driver(&quickvx_i2c_driver);
}

module_init(quickvx_mipi_dss_init);
module_exit(quickvx_mipi_dss_exit);

MODULE_AUTHOR("QuickLogic Corporation");
MODULE_DESCRIPTION("QuickLogic BX DSI to LVDS Driver");
MODULE_LICENSE("GPL");
