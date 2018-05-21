/*
 * arch/arm/mach-omap2/board-44xx-tablet-panel.c
 *
 * Copyright (C) 2011,2012 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/leds-omap4430sdp-display.h>

#include <plat/board.h>
#include <plat/vram.h>
#include <plat/android-display.h>

#include <video/omapdss.h>
#include <linux/tca6416a.h>
#include <linux/delay.h>

#include "mux.h"
#include "control.h"
#include "board-44xx-tablet.h"

#define TABLET_FB_RAM_SIZE		SZ_16M /* 1920×1080*4 * 2 */

#define GPIO_EN1V2 			272
#define GPIO_EN1V8  			273
#define GPIO_EN10V			276
#define GPIO_PWM_HIGH		278
#define GPIO_PWM_MID		279
#define GPIO_DISP_OE_RST  	280
#define GPIO_BRG_RSTN  		281
#define GPIO_PWM_LOW		284

/* HDMI GPIOs */
#define HDMI_GPIO_CT_CP_HPD			60 /* HPD mode enable/disable */
#define HDMI_GPIO_HPD				63 /* Hot plug pin for HDMI */
#define HDMI_GPIO_LS_OE				41 /* Level shifter for HDMI */

static void __init glass_led_hw_init(void)
{
	gpio_request(GPIO_PWM_HIGH, "bl_pwm_high");
	gpio_request(GPIO_PWM_MID, "bl_pwm_mid");
	gpio_request(GPIO_PWM_LOW, "bl_pwm_low");
}

static void omap4_tablet_init_display_led(void)
{
	if(!board_is_c1_evt1()) glass_led_hw_init();
	gpio_direction_output(GPIO_PWM_LOW, 0);
	gpio_direction_output(GPIO_PWM_MID, 1);
	gpio_direction_output(GPIO_PWM_HIGH, 0);
}

static void omap4_tablet_set_primary_brightness(u8 brightness)
{
	/*full brightness is 255*/
	if (0 < brightness &&  brightness <= 36){
		gpio_direction_output(GPIO_PWM_LOW, 1);
		gpio_direction_output(GPIO_PWM_MID, 0);
		gpio_direction_output(GPIO_PWM_HIGH, 0);
	}
	else if (36 < brightness &&  brightness <= 73){
		gpio_direction_output(GPIO_PWM_LOW, 0);
		gpio_direction_output(GPIO_PWM_MID, 1);
		gpio_direction_output(GPIO_PWM_HIGH, 0);
	}
	else if (73 < brightness &&  brightness <= 109){
		gpio_direction_output(GPIO_PWM_LOW, 1);
		gpio_direction_output(GPIO_PWM_MID, 1);
		gpio_direction_output(GPIO_PWM_HIGH, 0);
	}
	else if (109 < brightness &&  brightness <= 146){
		gpio_direction_output(GPIO_PWM_LOW, 0);
		gpio_direction_output(GPIO_PWM_MID, 0);
		gpio_direction_output(GPIO_PWM_HIGH, 1);
	}
	else if (146 < brightness &&  brightness <= 182){
		gpio_direction_output(GPIO_PWM_LOW, 1);
		gpio_direction_output(GPIO_PWM_MID, 0);
		gpio_direction_output(GPIO_PWM_HIGH, 1);
	}
	else if (182 < brightness &&  brightness <= 218){
		gpio_direction_output(GPIO_PWM_LOW, 0);
		gpio_direction_output(GPIO_PWM_MID, 1);
		gpio_direction_output(GPIO_PWM_HIGH, 1);
	}
	else if (218 < brightness &&  brightness <= 255){
		gpio_direction_output(GPIO_PWM_LOW, 1);
		gpio_direction_output(GPIO_PWM_MID, 1);
		gpio_direction_output(GPIO_PWM_HIGH, 1);
	}
}


static struct omap4430_sdp_disp_led_platform_data tablet_disp_led_data = {
	.display_led_init = omap4_tablet_init_display_led,
	.primary_display_set = omap4_tablet_set_primary_brightness,
};

static struct platform_device tablet_disp_led = {
		.name	=	"display_led",
		.id	=	-1,
		.dev	= {
		.platform_data = &tablet_disp_led_data,
		},
};
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
extern void lcos_vdd_1v2_pwr_on(struct omap_dss_device *dssdev);
extern void lcos_vdd_1v2_pwr_off(struct omap_dss_device *dssdev);
extern void lcos_vdd_1v8_pwr_on(struct omap_dss_device *dssdev);
extern void lcos_vdd_1v8_pwr_off(struct omap_dss_device *dssdev);
#endif
static int omap4_display_enable_lcos(struct omap_dss_device *dssdev)
{
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	lcos_vdd_1v2_pwr_on(dssdev);
#else
	gpio_direction_output(GPIO_EN1V2 ,1);
#endif
	msleep(20);
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	lcos_vdd_1v8_pwr_on(dssdev);
#else
	gpio_direction_output(GPIO_EN1V8, 0);
#endif
	msleep(20);
	gpio_direction_output(GPIO_EN10V, 1);
	msleep(20);
	gpio_direction_output(GPIO_BRG_RSTN, 1);
	msleep(20);
	gpio_direction_output(GPIO_DISP_OE_RST, 1);
	msleep(20);	

	return 0;
}

static void omap4_display_disable_lcos(struct omap_dss_device *dssdev)
{
	gpio_direction_output(GPIO_DISP_OE_RST, 0);
	gpio_direction_output(GPIO_BRG_RSTN, 0);
	gpio_direction_output(GPIO_EN10V, 0);
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	lcos_vdd_1v8_pwr_off(dssdev);
#else
	gpio_direction_output(GPIO_EN1V8, 1);
#endif
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	lcos_vdd_1v2_pwr_off(dssdev);
#else
	gpio_direction_output(GPIO_EN1V2 ,0);
#endif
}

struct omap_video_timings tablet_dispc_timings = {
	.x_res = 800,
	.y_res = 480,
	.hfp = 72,
	.hsw = 7,
	.hbp = 1,
	.vfp = 18,
	.vsw = 20,
	.vbp = 10,
};

static struct omap_dss_device tablet_lcd_device = {
	.name                   = "quickvx_mipi",
	.driver_name            = "quickvx_mipi",
	.type                   = OMAP_DISPLAY_TYPE_DSI,
	.phy.dsi                = {
		.clk_lane       = 1,
		.clk_pol        = 0,
		.data1_lane     = 2,
		.data1_pol      = 0,
		.data2_lane     = 3,
		.data2_pol      = 0,
		/*
		.data3_lane     = 4,
		.data3_pol      = 0,
		.data4_lane     = 5,
		.data4_pol      = 0,
		*/
		.module		= 0,
	},

	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 3,
				.lcd_clk_src    =
					OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},

		.dsi = {
			.regn           = 16,
			.regm           = 279,
			.regm_dispc     = 8,
			.regm_dsi       = 8,
			.lp_clk_div     = 9,
			.dsi_fclk_src   = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},

	.panel = {
		.timings = {
			.x_res		= 800,
			.y_res		= 480,
			.pixel_clock	= 55757,
			.hfp		= 20,
			.hsw		= 30,
			.hbp		= 30,
			.vfp		= 18,
			.vsw		= 20,
			.vbp		= 10,
		},

		.dsi_mode = OMAP_DSS_DSI_VIDEO_MODE,
		.dsi_vm_data = {
				.hsa			= 30,
				.hfp			= 20,
				.hbp			= 30,
				.vsa			= 20,
				.vfp			= 18,
				.vbp			= 10,

				.vp_de_pol		= true,
				.vp_vsync_pol		= true,
				.vp_hsync_pol		= true,
				.vp_hsync_end		= false,
				.vp_vsync_end		= false,

				.blanking_mode		= 0,
				.hsa_blanking_mode	= 1,
				.hfp_blanking_mode	= 1,
				.hbp_blanking_mode	= 1,

				.ddr_clk_always_on	= true,

				.window_sync		= 4,
		}
	},

	.ctrl = {
		.pixel_size = 24,
	},

	//.reset_gpio     = 102,
	.channel = OMAP_DSS_CHANNEL_LCD,
	.platform_enable = omap4_display_enable_lcos,
	.platform_disable = omap4_display_disable_lcos,
	.dispc_timings = &tablet_dispc_timings,
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
       .skip_init = true,
#endif
};

static int tablet_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	return 0;
}

static void tablet_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_hdmi_data tablet_hdmi_data = {
	.hpd_gpio = HDMI_GPIO_HPD,
	.ls_oe_gpio = HDMI_GPIO_LS_OE,
	.ct_cp_hpd_gpio = HDMI_GPIO_CT_CP_HPD,
};

static struct omap_dss_device tablet_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.platform_enable = tablet_panel_enable_hdmi,
	.platform_disable = tablet_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.data = &tablet_hdmi_data,
};

static struct omap_dss_device *tablet_dss_devices[] = {
	&tablet_lcd_device,
	&tablet_hdmi_device,
};

static struct omap_dss_device *dss_devices_hdmi_default_display[] = {
	&tablet_hdmi_device,
};

/* Allocate ( 18 + 9 ) MB for TILER1D slot size for WUXGA panel, total of
 * 54 MB of TILER1D
 */
static struct dsscomp_platform_data dsscomp_config_tablet = {
	.tiler1d_slotsz = SZ_16M,
};

static struct dsscomp_platform_data dsscomp_config_hdmi_display = {
	.tiler1d_slotsz = (SZ_16M + SZ_2M + SZ_8M + SZ_1M),
};

/* HACK: use 2 devices, as expected by DDK */
static struct sgx_omaplfb_config omaplfb_config_tablet[] = {
	{
		.tiler2d_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
};

static struct sgx_omaplfb_config omaplfb_config_hdmi_default_display[] = {
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	}
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_tablet = {
	.num_configs = ARRAY_SIZE(omaplfb_config_tablet),
	.configs = omaplfb_config_tablet,
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_hdmi_default_display = {
	.num_configs = ARRAY_SIZE(omaplfb_config_hdmi_default_display),
	.configs = omaplfb_config_hdmi_default_display,
};

static struct omap_dss_board_info tablet_dss_data = {
	.num_devices	= ARRAY_SIZE(tablet_dss_devices),
	.devices	= tablet_dss_devices,
	.default_device	= &tablet_lcd_device,
};

static struct omap_dss_board_info tablet_dss_data_hdmi_default_display = {
	.num_devices    = ARRAY_SIZE(dss_devices_hdmi_default_display),
	.devices        = dss_devices_hdmi_default_display,
	.default_device = &tablet_hdmi_device,
};

static struct omapfb_platform_data tablet_fb_pdata = {
	.mem_desc = {
		.region_cnt = ARRAY_SIZE(omaplfb_config_tablet),
	},
};

static void __init tablet_hdmi_init(void)
{
	/*
	 * OMAP4460SDP/Blaze and OMAP4430 ES2.3 SDP/Blaze boards and
	 * later have external pull up on the HDMI I2C lines
	 */
	if (cpu_is_omap446x() || omap_rev() > OMAP4430_REV_ES2_2)
		omap_hdmi_init(OMAP_HDMI_SDA_SCL_EXTERNAL_PULLUP);
	else
		omap_hdmi_init(0);

	omap_mux_init_gpio(HDMI_GPIO_LS_OE, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(HDMI_GPIO_CT_CP_HPD, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT_PULLDOWN);
}

static void __init tablet_lcd_init(void)
{
	u32 reg;
	int status;

	/* Enable 3 lanes in DSI1 module, disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1f << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1f << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);

		
#if 0
	status = gpio_request_one(tablet_lcd_device.reset_gpio,
				GPIOF_OUT_INIT_LOW, "lcd_reset_gpio");
	gpio_set_value(tablet_lcd_device.reset_gpio, 1);
	if (status)
		pr_err("%s: Could not get lcd_reset_gpio\n", __func__);
#endif

}

static struct i2c_board_info __initdata omap4xx_i2c_bus2_d2l_info[] = {
	{
		I2C_BOARD_INFO("tc358765_i2c_driver", 0x0f),
	},
};

void __init tablet_android_display_setup(void)
{
	if (omap_android_display_is_default(&tablet_hdmi_device))
		omap_android_display_setup(
			&tablet_dss_data_hdmi_default_display,
			&dsscomp_config_hdmi_display,
			&omaplfb_plat_data_hdmi_default_display,
			&tablet_fb_pdata);
	else
		omap_android_display_setup(&tablet_dss_data,
			&dsscomp_config_tablet,
			&omaplfb_plat_data_tablet,
			&tablet_fb_pdata);
}

int __init tablet_display_init(void)
{
	omap_mux_init_signal("fref_clk4_out.fref_clk4_out",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	if(!board_is_c1_evt1())
		platform_device_register(&tablet_disp_led);
	tablet_lcd_init();

	omapfb_set_platform_data(&tablet_fb_pdata);
	omap_vram_set_sdram_vram(TABLET_FB_RAM_SIZE, 0);
	if (omap_android_display_is_default(&tablet_hdmi_device))
		omap_display_init(&tablet_dss_data_hdmi_default_display);
	else
		omap_display_init(&tablet_dss_data);

	tablet_hdmi_init();
#if 0
	i2c_register_board_info(2, omap4xx_i2c_bus2_d2l_info,
		ARRAY_SIZE(omap4xx_i2c_bus2_d2l_info));
#endif
	return 0;
}
