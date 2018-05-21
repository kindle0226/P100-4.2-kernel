/*
 * Board support file for OMAP44XX Tablet.
 *
 * Copyright (C) 2009 - 2012 Texas Instruments
 *
 * Authors:
 *	 Dan Murphy <dmurphy@ti.com>
 *	 Volodymyr Riazantsev <v.riazantsev@ti.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *   by Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/bq2415x.h>
#include <linux/i2c/tmp102.h>
#include <linux/mfd/twl6040.h>
#include <linux/cdc_tcxo.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6130x.h>
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
#include <linux/regulator/gpio-switch-regulator.h>
#endif // CONFIG_REGULATOR_GPIO_SWITCH
#include <linux/platform_data/omap-abe-twl6040.h>
#include <linux/platform_data/thermistor_sensor.h>
#include <linux/platform_data/lm75_platform_data.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/omap_die_governor.h>
#include <linux/tca6416a.h>
#include <linux/i2c/pca953x.h>
#include <linux/input/it7258.h>  
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/omap-secure.h>

#include <plat/drm.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/omap_apps_brd_id.h>

#include "mux.h"
#include "hsmmc.h"
#include "common.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"

#include "board-44xx-tablet.h"
#include "omap4_ion.h"
#include "omap_ram_console.h"

#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138

#define FIXED_REG_VBAT_ID		0
#define FIXED_REG_VWLAN_ID		1
#define TPS62361_GPIO			7

#define OMAP4_MDM_PWR_EN_GPIO		157
#define OMAP4_GPIO_WK30			30

static c1_version C1_VERSION = UNVERSIONED;

static struct twl6040 *twl6040_driver_data;

static struct spi_board_info tablet_spi_board_info[] __initdata = {
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		/*
		 * .irq is set to gpio_to_irq(ETH_KS8851_IRQ)
		 * in omap_tablet_init
		 */
	},
};

static struct gpio tablet_eth_gpios[] __initdata = {
	{ ETH_KS8851_POWER_ON,	GPIOF_OUT_INIT_HIGH,	"eth_power"	},
	{ ETH_KS8851_QUART,	GPIOF_OUT_INIT_HIGH,	"quart"		},
	{ ETH_KS8851_IRQ,	GPIOF_IN,		"eth_irq"	},
};

static int __init omap_ethernet_init(void)
{
	int status;

	/* Request of GPIO lines */
	status = gpio_request_array(tablet_eth_gpios,
				    ARRAY_SIZE(tablet_eth_gpios));
	if (status)
		pr_err("Cannot request ETH GPIOs\n");

	return status;
}

static struct regulator_consumer_supply tablet_vbat_supply[] = {
	REGULATOR_SUPPLY("vddvibl", "twl6040-vibra"),
	REGULATOR_SUPPLY("vddvibr", "twl6040-vibra"),
};

static struct regulator_init_data tablet_vbat_data = {
	.constraints = {
		.always_on	= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(tablet_vbat_supply),
	.consumer_supplies	= tablet_vbat_supply,
};

static struct fixed_voltage_config tablet_vbat_pdata = {
	.supply_name	= "VBAT",
	.microvolts	= 3750000,
	.init_data	= &tablet_vbat_data,
	.gpio		= 151,
	.enabled_at_boot = 1,
	.enable_high	= 1,
};

static struct platform_device tablet_vbat = {
	.name		= "reg-fixed-voltage",
	.id		= FIXED_REG_VBAT_ID,
	.dev = {
		.platform_data = &tablet_vbat_pdata,
	},
};

static struct platform_device tablet_dmic_codec = {
	.name	= "dmic-codec",
	.id	= -1,
};

static struct platform_device tablet_spdif_dit_codec = {
	.name           = "spdif-dit",
	.id             = -1,
};

static struct platform_device tablet_hdmi_audio_codec = {
	.name	= "hdmi-audio-codec",
	.id	= -1,
};

static struct omap_abe_twl6040_data tablet_abe_audio_data = {
	.card_name = "tablet",
	.has_hs		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_hf		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_ep		= 1,
	.has_aux	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_vibra	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,

	.has_abe	= 1,
	.has_dmic	= 1,
	.has_hsmic	= 1,
	.has_mainmic	= 1,
	.has_submic	= 1,
	.has_afm	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,

	.jack_detection = 1,
	/* MCLK input is 38.4MHz */
	.mclk_freq	= 38400000,
};

static struct platform_device tablet_abe_audio = {
	.name		= "omap-abe-twl6040",
	.id		= -1,
	.dev = {
		.platform_data = &tablet_abe_audio_data,
	},
};
#if 0
static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= KEY_CAMERA,
		.gpio			= 256,
		.desc			= "cam_shot",
		.wakeup			= 1,
		.active_low 		= 1,
	},
};

static struct gpio_keys_platform_data gpio_keys_pdata = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device tablet_gpio_keys = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_keys_pdata,
	},
};
#endif
/************************ GPIO based switch regulator ****************/
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
/* tca6416 power switch */
#define TCA6416_GPIO_EN1V2  			272
#define TCA6416_GPIO_EN1V8  			273
#define TCA6416_GPIO_EN2V8  			274

static struct regulator_consumer_supply gpio_switch_tca6416_v1v2_supply[] = {
        REGULATOR_SUPPLY("lcos_v1v2", NULL),
        REGULATOR_SUPPLY("cam_v1v2", NULL),
};
static int gpio_switch_tca6416_v1v2_voltages[] = {1200};

static struct regulator_consumer_supply gpio_switch_tca6416_v1v8_supply[] = {
        REGULATOR_SUPPLY("lcos_v1v8", NULL),
        REGULATOR_SUPPLY("bmx_v1v8", NULL),
        REGULATOR_SUPPLY("proximity_sensor_v1v8", NULL),
};
static int gpio_switch_tca6416_v1v8_voltages[] = {1800};

static struct regulator_consumer_supply gpio_switch_tca6416_v2v8_supply[] = {
        REGULATOR_SUPPLY("tp_v2v8", NULL),
        REGULATOR_SUPPLY("bmx_v2v8", NULL),
        REGULATOR_SUPPLY("proximity_sensor_v2v8", NULL),
};
static int gpio_switch_tca6416_v2v8_voltages[] = {2800};

/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _name, _input_supply, _gpio_nr, _active_low, \
			_init_state, _pg, _enable, _disable)		\
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_name =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

GREG_INIT(0, tca6416_v1v2, NULL, TCA6416_GPIO_EN1V2, false, 1, 0, 0, 0);
GREG_INIT(0, tca6416_v1v8, NULL, TCA6416_GPIO_EN1V8, true, 1, 0, 0, 0);
GREG_INIT(0, tca6416_v2v8, NULL, TCA6416_GPIO_EN2V8, false, 1, 0, 0, 0);

#define ADD_GPIO_REG(_name)	(&gpio_pdata_##_name)
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs[] = {
        ADD_GPIO_REG(tca6416_v1v2),
        ADD_GPIO_REG(tca6416_v1v8),
        ADD_GPIO_REG(tca6416_v2v8),
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata = {
	.num_subdevs	= ARRAY_SIZE(gswitch_subdevs),
	.subdevs	= gswitch_subdevs,
};

static struct platform_device gswitch_regulator_pdata = {
	.name	= "gpio-switch-regulator",
	.id	= -1,
	.dev	= {
		.platform_data = &gswitch_pdata,
	},
};
#endif // CONFIG_REGULATOR_GPIO_SWITCH

static struct platform_device *tablet_devices[] __initdata = {
	&tablet_vbat,
	&tablet_dmic_codec,
	&tablet_spdif_dit_codec,
	&tablet_abe_audio,
	&tablet_hdmi_audio_codec,
//	&tablet_gpio_keys,
#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	&gswitch_regulator_pdata,
#endif // CONFIG_REGULATOR_GPIO_SWITCH
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 200,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.built_in	= 1,
		.no_off_init	= true,
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.pm_caps	= MMC_PM_KEEP_POWER,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.built_in	= 1,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= DUTY_GOVERNOR_DEFAULT_TEMP,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
};

static void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
				   ARRAY_SIZE
				   (omap4_duty_governor_pcb_sections));
}
#else
static void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

/* Initial set of thresholds for different thermal zones */
static struct omap_thermal_zone thermal_zones[] = {
	OMAP_THERMAL_ZONE("safe", 0, 25000, 65000, 250, 1000, 400),
	OMAP_THERMAL_ZONE("monitor", 0, 60000, 80000, 250, 250,	250),
	//OMAP_THERMAL_ZONE("alert", 0, 75000, 90000, 250, 250, 150),
	// Raise cooling level when CPU's temperature hits "alert" zone
	OMAP_THERMAL_ZONE("alert", 1, 75000, 90000, 250, 250, 150),
	OMAP_THERMAL_ZONE("critical", 1, 85000,	115000,	250, 250, 50),
};

static struct omap_die_governor_pdata omap_gov_pdata = {
	.zones = thermal_zones,
	.zones_num = ARRAY_SIZE(thermal_zones),
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int irq = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		irq = twl6030_mmc_card_detect_config();
		if (irq < 0) {
			pr_err("Failed configuring MMC1 card detect\n");
			return irq;
		}
		pdata->slots[0].card_detect_irq = irq;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return 0;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}

static int tps6130x_enable(int on)
{
	u8 mask;
	int ret;

	if (!twl6040_driver_data) {
		pr_err("%s: invalid twl6040 driver data\n", __func__);
		return -EINVAL;
	}

	/*
	 * tps6130x NRESET driven by:
	 * - GPO2 in TWL6040
	 * - GPO in TWL6041 (only one GPO supported)
	 */
	if (twl6040_driver_data->rev >= TWL6041_REV_2_0)
		mask = TWL6040_GPO1;
	else
		mask = TWL6040_GPO2;

	if (on)
		ret = twl6040_set_bits(twl6040_driver_data,
				       TWL6040_REG_GPOCTL, mask);
	else
		ret = twl6040_clear_bits(twl6040_driver_data,
					 TWL6040_REG_GPOCTL, mask);

	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);
	return ret;
}

static struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable	= tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf_data = {
	.constraints = {
		.min_uV			= 4075000,
		.max_uV			= 4950000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies	= twl6040_vddhf_supply,
	.driver_data		= &tps6130x_pdata,
};

static struct twl6040_codec_data twl6040_codec = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
	.vddhf_uV	= 4075000,
};

static struct twl6040_vibra_data twl6040_vibra = {
	.vibldrv_res = 8,
	.vibrdrv_res = 3,
	.viblmotor_res = 10,
	.vibrmotor_res = 10,
	.vddvibl_uV = 0,	/* fixed volt supply - VBAT */
	.vddvibr_uV = 0,	/* fixed volt supply - VBAT */

	.max_timeout = 15000,
	.initial_vibrate = 0,
	.voltage_raise_speed = 0x26,
};

static struct i2c_board_info tablet_i2c_1_tps6130x_boardinfo = {
	I2C_BOARD_INFO("tps6130x", 0x33),
	.platform_data = &twl6040_vddhf_data,
};

static int twl6040_platform_init(struct twl6040 *twl6040)
{
	if (cpu_is_omap447x())
		return 0;

	if (!twl6040) {
		pr_err("%s: invalid twl6040 driver data\n", __func__);
		return -EINVAL;
	}

	twl6040_driver_data = twl6040;

	tps6130x_pdata.adapter = i2c_get_adapter(1);
	if (!tps6130x_pdata.adapter) {
		pr_err("%s: can't get i2c adapter\n", __func__);
		return -ENODEV;
	}

	tps6130x_pdata.client = i2c_new_device(tps6130x_pdata.adapter,
		&tablet_i2c_1_tps6130x_boardinfo);
	if (!tps6130x_pdata.client) {
		pr_err("%s: can't add i2c device\n", __func__);
		i2c_put_adapter(tps6130x_pdata.adapter);
		return -ENODEV;
	}

	return 0;
}

static int twl6040_platform_exit(struct twl6040 *twl6040)
{
	if (cpu_is_omap447x())
		return 0;

	if (tps6130x_pdata.client) {
		i2c_unregister_device(tps6130x_pdata.client);
		tps6130x_pdata.client = NULL;
		i2c_put_adapter(tps6130x_pdata.adapter);
	}

	return 0;
}

static struct twl6040_platform_data twl6040_data = {
	.codec		= &twl6040_codec,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.platform_init	= twl6040_platform_init,
	.platform_exit	= twl6040_platform_exit,
};

/*
 * Setup CFG_TRANS mode as follows:
 * 0x00 (OFF) when in OFF state(bit offset 4)
 * - these bits a read only, so don't touch them
 * 0x00 (OFF) when in sleep (bit offset 2)
 * 0x01 (PWM/PFM Auto) when in ACTive state (bit offset 0)
 */
#define TWL6030_REG_VCOREx_CFG_TRANS_MODE		(0x00 << 4 | \
		TWL6030_RES_OFF_CMD << TWL6030_REG_CFG_TRANS_SLEEP_CMD_OFFSET |\
		TWL6030_RES_AUTO_CMD << TWL6030_REG_CFG_TRANS_ACT_CMD_OFFSET)

#define TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC "OFF=OFF SLEEP=OFF ACT=AUTO"

/* OMAP4430 - All vcores: 1, 2 and 3 should go down with PREQ */
static struct twl_reg_setup_array omap4430_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE1 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE3 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{ .desc = NULL} /* TERMINATOR */
};

/* OMAP4460 - VCORE3 is unused, 1 and 2 should go down with PREQ */
static struct twl_reg_setup_array omap4460_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE1 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_CFG_SMPS_PD,
		.val = 0x77,
		.desc = "VCORE1 disable PD on shutdown",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_GRP,
		.val = 0x00,
		.desc = "VCORE3 - remove binding to groups",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VMEM_CFG_GRP,
		.val = 0x00,
		.desc = "VMEM - remove binding to groups",
	},
	{ .desc = NULL} /* TERMINATOR */
};

// NOTE: The new 1340 mAh battery is used in DVT, PVT & MP.
// And the old 630 mAh one is only used in EVT1, EVT2 & DVT.
// Since these two batteries can't be telled on the HW side,
// please use the following define to switch between them.
//#define DEV_USE_1340MAH_BAT

// 20150126: Update the parameters below for 630 mAh battery
#include "twl-common.h"
#include <linux/power/ti-fg.h>

#define MIN_BATTERY_VOLTAGE 3000 // Unused
#define MAX_BATTERY_VOLTAGE 4200
#define MAX_CHARGER_CURRENT 1000
#define MAX_CHARGER_VOLTAGE MAX_BATTERY_VOLTAGE

// 20150721: Update the parameters below for 3100 mAh battery
/* Fuel Gauge EDV Configuration */
static struct edv_config edv_cfg = {
	.averaging = true,
	.seq_edv = 5,
	.filter_light = 155, // Unused
	.filter_heavy = 199, // Unused
	.overload_current = 1000, // Max load current is about 900 mA
//NOTE: The following table is calculated with 800mA discharging current at 50°C
	.edv = {
		{3342, 0},
		{3375, 4},
		{3428, 10},
	},
};

/* Fuel Gauge OCV Configuration */
static struct ocv_config ocv_cfg = {
	.voltage_diff = 75,
	.current_diff = 30,

	.sleep_enter_current = 30, // should be less than cc_current
	.sleep_enter_samples = 3,

	.sleep_exit_current = 70, // TBD
	.sleep_exit_samples = 3,

	.long_sleep_current = 500,
	.ocv_period = 300,
	.relax_period = 600,

	.flat_zone_low = 30, // TBD
	.flat_zone_high = 50, // TBD

	.max_ocv_discharge = 1082, // 0.325 * design_capacity

//NOTE: The following table is calculated with 200mA discharging current at 50°C
	.table = {
		3453, 3494, 3539, 3584, 3630,
		3670, 3693, 3703, 3709, 3717,
		3730, 3751, 3780, 3815, 3848,
		3882, 3918, 3958, 4003, 4044,
		4102
	},
};

/* Tablet Battery Cell Configuration */
static struct cell_config tablet_cell_cfg =  {
	.cc_voltage = 4175,
	.cc_current = 128, // Defined by battery spec
#ifdef CONFIG_BQ24192_CHARGER
	.cc_capacity = 100, // set to 100% to avoid under-charging
#else
	.cc_capacity = 30, // TBD
#endif
	.seq_cc = 5,

	.cc_polarity = true,
	.cc_out = true,
	.ocv_below_edv1 = false,

	.design_capacity = 3000, // 0.9 * design_capacity
	.design_qmax = 3330, // Unused
	.r_sense = 20,

	.qmax_adjust = 1, // Unused
	.fcc_adjust = 2,

	.max_overcharge = 310, // design_qmax - design_capacity (mAh)
	.electronics_load = 200, // TBD /* *10 uAh */

	.max_increment = 140, // TBD
	.max_decrement = 140, // TBD
	.low_temp = 119,
	.deep_dsg_voltage = 30, // TBD
	.max_dsg_estimate = 60, // TBD
	.light_load = 40, // Min load current is about 50 mA
	.near_full = 62, // 0.02 * design_capacity
	.cycle_threshold = 2914, // 0.875 * design_capacity
	.recharge = 100, // 0.03 * design_capacity

	.mode_switch_capacity = 5, // mAh

	.call_period = 2,

	.ocv = &ocv_cfg,
	.edv = &edv_cfg,
};

// 20150204: Update the parameters below for 1340 mAh battery
/* Fuel Gauge EDV Configuration */
static struct edv_config new_edv_cfg = {
	.averaging = true,
	.seq_edv = 5,
	.filter_light = 155, // Unused
	.filter_heavy = 199, // Unused
	.overload_current = 1000, // Max load current is about 900 mA
//NOTE: The following table is calculated with 536mA discharging current at 25°C
	.edv = {
		{3387, 0},
		{3425, 4},
		{3483, 10},
	},
};

/* Fuel Gauge OCV Configuration */
static struct ocv_config new_ocv_cfg = {
	.voltage_diff = 75,
	.current_diff = 30,

	.sleep_enter_current = 30, // should be less than cc_current
	.sleep_enter_samples = 3,

	.sleep_exit_current = 70, // TBD
	.sleep_exit_samples = 3,

	.long_sleep_current = 500,
	.ocv_period = 300,
	.relax_period = 600,

	.flat_zone_low = 50, // TBD
	.flat_zone_high = 60, // TBD

	.max_ocv_discharge = 436, // 0.325 * design_capacity

//NOTE: The following table is calculated with 201mA discharging current at 25°C
	.table = {
		3455, 3500, 3553, 3605, 3643,
		3678, 3696, 3709, 3718, 3728,
		3740, 3758, 3779, 3810, 3839,
		3876, 3914, 3952, 4000, 4049,
		4119
	},
};

/* Tablet Battery Cell Configuration */
static struct cell_config new_tablet_cell_cfg =  {
	.cc_voltage = 4175,
	.cc_current = 67, // Defined by battery spec
	.cc_capacity = 30, // TBD
	.seq_cc = 5,

	.cc_polarity = true,
	.cc_out = true,
	.ocv_below_edv1 = false,

	.design_capacity = 1206, // 0.9 * design_capacity
	.design_qmax = 1340, // Unused
	.r_sense = 20,

	.qmax_adjust = 1, // Unused
	.fcc_adjust = 2,

	.max_overcharge = 130, // design_qmax - design_capacity (mAh)
	.electronics_load = 200, // TBD /* *10 uAh */

	.max_increment = 60, // TBD
	.max_decrement = 60, // TBD
	.low_temp = 119,
	.deep_dsg_voltage = 30, // TBD
	.max_dsg_estimate = 60, // TBD
	.light_load = 40, // Min load current is about 50 mA
	.near_full = 24, // 0.02 * design_capacity
	.cycle_threshold = 1055, // 0.875 * design_capacity
	.recharge = 36, // 0.03 * design_capacity

	.mode_switch_capacity = 5, // mAh

	.call_period = 2,

	.ocv = &new_ocv_cfg,
	.edv = &new_edv_cfg,
};

static int tablet_batt_table[] = {
	/* adc code for temperature in degree C */
	// The following table is calculated with a 1.8 k-ohm resistance,
	// a 51 k-ohm resistance and the 250 ohm internal resistance
	924, 921, /* -2, -1 */
	919, 916, 913, 910, 907, 904, 901, 898, 894, 891, /* 00 - 09 */
	887, 884, 880, 876, 872, 868, 864, 860, 855, 851, /* 10 - 19 */
	846, 842, 837, 832, 827, 822, 817, 812, 806, 801, /* 20 - 29 */
	796, 790, 784, 778, 773, 767, 760, 754, 748, 742, /* 30 - 39 */
	735, 729, 722, 716, 709, 703, 696, 689, 682, 675, /* 40 - 49 */
	668, 661, 654, 647, 640, 632, 625, 618, 611, 603, /* 50 - 59 */
	596, 589, 581, 574, 567, 559, 552, 545, 537, 530, /* 60 - 69 */
	523, /* 70 */
};

static struct twl4030_bci_platform_data tablet_bci_pdata = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= MAX_CHARGER_CURRENT,
	.max_charger_voltagemV		= MAX_CHARGER_VOLTAGE,
	.max_bat_voltagemV		= MAX_BATTERY_VOLTAGE,
	.low_bat_voltagemV		= MIN_BATTERY_VOLTAGE,
	.battery_tmp_tbl		= tablet_batt_table,
	.tblsize			= ARRAY_SIZE(tablet_batt_table),
	.cell_cfg			= &tablet_cell_cfg,
};

static struct twl4030_platform_data tablet_twldata;

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data tablet_cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,
		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,
		CDC_TCXO_LDOEN1,
		0,
	},
};

static struct bq2415x_platform_data tablet_bqdata = {
		.max_charger_voltagemV = MAX_CHARGER_VOLTAGE,
		.max_charger_currentmA = MAX_CHARGER_CURRENT,
};

static struct i2c_board_info __initdata tablet_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &tablet_cdc_data,
	},
	{
		I2C_BOARD_INFO("bq24156", 0x6a),
		.platform_data = &tablet_bqdata,
	},
};

/* TMP102 PCB Temperature sensor close to OMAP
 * values for .slope and .offset are taken from OMAP5 structure,
 * as TMP102 sensor was not used by domains other then CPU
 */
static struct tmp102_platform_data tmp102_omap_info = {
	.slope = 470,
	.slope_cpu = 1063,
	.offset = -1272,
	.offset_cpu = -477,
	.domain = "pcb", /* for hotspot extrapolation */
};

static struct i2c_board_info __initdata tablet_i2c_4_tmp102_boardinfo[] = {
	{
			I2C_BOARD_INFO("tmp102_temp_sensor", 0x48),
			.platform_data = &tmp102_omap_info,
	},
};

static struct pca953x_platform_data tca64161_0_platform_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= OMAP_IRQ_END,
	.names		= "tca6416-irq",
};

static struct pca953x_platform_data tca64161_1_platform_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES + 16,
	.irq_base	= -1,
	.names		= "tca6416-gpio",
};

static struct it7258_i2c_platform_data it7258_platform_data = {
	.reset_io = 285,
	.enable_io = 286,
	.gpio_irq = 140,
	.max_x = 480,
	.max_y = 800,
	.reverse = true,
	.swap = true,
};

static struct i2c_board_info __initdata tablet_i2c_3_boardinfo[] = {
	{
			I2C_BOARD_INFO("tca6416", 0x20),  //7 bit address
 			.platform_data = &tca64161_0_platform_data,
 			.irq = 39,
	},
	{
			I2C_BOARD_INFO("tca6416", 0x21),  //7 bit address
 			.platform_data = &tca64161_1_platform_data,
 			.irq = -1,
	},
	{
			I2C_BOARD_INFO("quickvx_i2c", 0x64),
	},
	{
			I2C_BOARD_INFO(IT7258_TS_NAME, IT7258_I2C_ADD),
			.platform_data = &it7258_platform_data,
	},
};
static struct it7258_i2c_platform_data it7258_platform_data_v2 = {
	.reset_io = 285,
	.enable_io = 286,
	.gpio_irq = 1,
	.max_x = 480,
	.max_y = 800,
	.reverse = false,
	.swap = true,
};
static struct i2c_board_info __initdata tablet_i2c_3_boardinfo_v2[] = {
	{
			I2C_BOARD_INFO("tca6416", 0x20),  //7 bit address
 			.platform_data = &tca64161_0_platform_data,
 			.irq = 0,
	},
	{
			I2C_BOARD_INFO("tca6416", 0x21),  //7 bit address
 			.platform_data = &tca64161_1_platform_data,
 			.irq = -1,
	},
	{
			I2C_BOARD_INFO("quickvx_i2c", 0x64),
	},
	{
			I2C_BOARD_INFO(IT7258_TS_NAME, IT7258_I2C_ADD),
			.platform_data = &it7258_platform_data_v2,
	},
};

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id,
							USE_MUTEX_LOCK);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata omap4_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &omap4_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &omap4_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &omap4_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &omap4_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &omap4_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &omap4_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &omap4_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &omap4_i2c_4_bus_pdata);

	omap4_pmic_get_config(&tablet_twldata, TWL_COMMON_PDATA_USB |
			TWL_COMMON_PDATA_MADC | \
			/*TWL_COMMON_PDATA_BCI |*/
			TWL_COMMON_PDATA_THERMAL,
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX1 |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VUSIM |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG |
			TWL_COMMON_REGULATOR_V1V8 |
			TWL_COMMON_REGULATOR_V2V1 |
			TWL_COMMON_REGULATOR_SYSEN |
			TWL_COMMON_REGULATOR_CLK32KAUDIO |
			TWL_COMMON_REGULATOR_REGEN1);

	/* Add one-time registers configuration */
	if (cpu_is_omap443x())
		tablet_twldata.reg_setup_script = omap4430_twl6030_setup;
	else if (!cpu_is_apm4460() && cpu_is_omap446x())
		tablet_twldata.reg_setup_script = omap4460_twl6030_setup;

	/* Use custom BCI configuration */
	tablet_twldata.bci = &tablet_bci_pdata;

	/*
	 * tps6130x regulator provides VDDHF supply for hands-free module
	 * (part of twl6040) only on OMAP4430 and OMAP4460 boards.
	 */
	if (cpu_is_omap447x() && twl6040_data.codec)
		twl6040_data.codec->vddhf_uV = 0;

	omap4_pmic_init("twl6030", &tablet_twldata,
			&twl6040_data, OMAP44XX_IRQ_SYS_2N);
	i2c_register_board_info(1, tablet_i2c_boardinfo,
				ARRAY_SIZE(tablet_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	if(board_is_c1_evt1())
		omap_register_i2c_bus(3,400,tablet_i2c_3_boardinfo,
				ARRAY_SIZE(tablet_i2c_3_boardinfo));
	else
		omap_register_i2c_bus(3,400,tablet_i2c_3_boardinfo_v2,
				ARRAY_SIZE(tablet_i2c_3_boardinfo_v2));
	
	omap_register_i2c_bus(4, 400, NULL, 0);

	if (cpu_is_apm4460() || cpu_is_omap447x())
		i2c_register_board_info(4, tablet_i2c_4_tmp102_boardinfo,
					ARRAY_SIZE
					(tablet_i2c_4_tmp102_boardinfo));

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
					| OMAP_PULL_ENA),
	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE),

	/* IO optimization pdpu and offmode settings to reduce leakage */
	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
	OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
					| OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	/*gpio 39, tca6416 irq*/
	OMAP4_MUX(GPMC_AD15, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	/*gpio 140, tp irq*/
	OMAP4_MUX(MCSPI1_CS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
/* GPIO_127, AUDPWRON */
	OMAP4_MUX(HDQ_SIO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),	
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#endif
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux_v2[] __initdata = {
	OMAP4_MUX(MCSPI4_SIMO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
					| OMAP_PULL_ENA),//gpio152 used as R_LED
	OMAP4_MUX(UART4_RX, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
					| OMAP_PULL_ENA),//gpio155 used as B_LED
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
					| OMAP_PULL_ENA),//gpio157 used as G_LED
	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE),

	/* IO optimization pdpu and offmode settings to reduce leakage */
	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
	OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
					| OMAP_OFF_PULL_EN),
					
		/* GPIO_50, VOL_UP */
	OMAP4_MUX(GPMC_NCS0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	/* GPIO_52, VOL_DN */
	OMAP4_MUX(GPMC_NCS2, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
	
	OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
// These two gpios have been init in u-boot
#if 0
	/*gpio wk0, tca6416 irq*/
	OMAP4_MUX(SIM_IO, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
	/*gpio wk1, tp irq*/
	OMAP4_MUX(SIM_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
#endif
/* GPIO_127, AUDPWRON */
	OMAP4_MUX(HDQ_SIO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	OMAP4_MUX(MCSPI4_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA
					 | OMAP_PULL_UP),
	
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux_v2	NULL
#endif
#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_ohci_init(void)
{
	omap_mux_init_signal("fref_clk3_req.gpio_wk30", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE | OMAP_PULL_ENA);

	/* Enable 5V,1A USB power on external HS-USB ports */
	if (gpio_is_valid(OMAP4_GPIO_WK30)) {
		gpio_request(OMAP4_GPIO_WK30, "USB POWER GPIO");
		gpio_direction_output(OMAP4_GPIO_WK30, 1);
		gpio_set_value(OMAP4_GPIO_WK30, 0);
	}

	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);

	/* Power on the ULPI PHY */
/*	if (gpio_is_valid(OMAP4_MDM_PWR_EN_GPIO)) {
		gpio_request(OMAP4_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(OMAP4_MDM_PWR_EN_GPIO, 1);
	}*/

	usbhs_init(&usbhs_bdata);

	return;
}
#else
static void __init omap4_ehci_ohci_init(void){}
#endif

static void __init tablet_camera_mux_init(void)
{
	u32 r = 0;

	/* Enable CSI22 pads for 4460 and 4470 */
	if (cpu_is_omap446x() || cpu_is_omap447x()) {
		r = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX);
		r |= (0x7 << OMAP4_CAMERARX_CSI22_LANEENABLE_SHIFT);
		omap4_ctrl_pad_writel(r,
		OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_CAMERA_RX);

		omap_mux_init_signal("csi22_dx2.csi22_dx2",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
		omap_mux_init_signal("csi22_dy2.csi22_dy2",
				OMAP_PIN_INPUT | OMAP_MUX_MODE0);
	}
}

static struct thermistor_pdata thermistor_device_data = {
	.name = "thermistor_sensor",
	.slope = 1142,
	.offset = -393,
	.domain = "pcb",
};

static struct platform_device thermistor = {
		.name	=	"thermistor",
		.id	=	-1,
		.dev	= {
		.platform_data = &thermistor_device_data,
		},
};

#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
static struct __devinitdata emif_custom_configs custom_configs = {
	/* enable EMIF_CUSTOM_CONFIG_EXTENDED_TEMP_PART to operate above 85 degree C.
	 * NOTE: Turning this flag on might cause potential issues. Use with caution! */
	.mask	= EMIF_CUSTOM_CONFIG_LPMODE | EMIF_CUSTOM_CONFIG_EXTENDED_TEMP_PART,
	.lpmode	= EMIF_LP_MODE_SELF_REFRESH,
	.lpmode_timeout_performance = 512,
	.lpmode_timeout_power = 512,
	/* only at OPP100 should we use performance value */
	.lpmode_freq_threshold = 400000000,
};
#endif

static void set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_setup_oscillator(4000, 1);
}
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_hw_ver_mux[] __initdata = {
// The internal pullup resistors will cause problems on DVT devices
// Do NOT turn on PTU unless for EVT1 devices
#if 0
	/*gpio 48*/
	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/*gpio 49*/
	OMAP4_MUX(GPMC_A25, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
#else
	/*gpio 48*/
	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/*gpio 49*/
	OMAP4_MUX(GPMC_A25, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
#endif
};
#else
#define board_hw_ver_mux	NULL
#endif
bool board_is_c1_evt1(void)
{
	return (C1_VERSION == C1_EVT1);
}
EXPORT_SYMBOL(board_is_c1_evt1);
bool board_is_c1_evt2(void)
{
	return (C1_VERSION == C1_EVT2);
}
EXPORT_SYMBOL(board_is_c1_evt2);
bool board_is_c1_dvt1_atcn(void)
{
	return (C1_VERSION == C1_DVT1_ATCN);
}
EXPORT_SYMBOL(board_is_c1_dvt1_atce);
bool board_is_c1_dvt1_atce(void)
{
	return (C1_VERSION == C1_DVT1_ATCE);
}
EXPORT_SYMBOL(board_is_c1_dvt1_atce);

/* Read board version from GPIO.  Result in C1_VERSION. */
static void __init c1_version_init(void)
{
	int r;
	int package = OMAP_PACKAGE_CBS;

	// mux board version gpio's
	omap4_mux_init(board_hw_ver_mux, NULL, package);

	r = gpio_request_one(GPIO_BOARD_ID0, GPIOF_IN, "board_ver_id0");
	if (r) {
		pr_err("Failed to get gpio %d for id0 pin of board version\n", GPIO_BOARD_ID0);
	}
	r = gpio_request_one(GPIO_BOARD_ID1, GPIOF_IN, "board_ver_id1");
	if (r) {
		pr_err("Failed to get gpio %d for id1 pin of board version\n", GPIO_BOARD_ID1);
	}
	C1_VERSION = gpio_get_value(GPIO_BOARD_ID0) | (gpio_get_value(GPIO_BOARD_ID1) << 1);

	gpio_free(GPIO_BOARD_ID0);
	gpio_free(GPIO_BOARD_ID1);

// debug message for HW revision check
#if 1
	switch (C1_VERSION) {
		case C1_EVT1:
			printk("*** Detected as EVT1 ***\n");
			break;
		case C1_EVT2:
			printk("*** Detected as EVT2 ***\n");
			break;
		case C1_DVT1_ATCN:
			printk("*** Detected as DVT1(ATCN) ***\n");
			break;
		case C1_DVT1_ATCE:
			printk("*** Detected as DVT1(ATCE) ***\n");
			break;
		default:
			printk("*** Unknown HW revision! ***\n");
			break;
	}
#endif
}

static void __init gps_lna_init() {
	int lna_gpio = 45;

	// request the LNA_EN here and set it to output high
	if (gpio_request(lna_gpio, "gps_lna_en"))
		pr_err(" gpio %ld request failed ", lna_gpio);
	else if (gpio_direction_output(lna_gpio, 1))
		pr_err(" unable to configure gpio %ld", lna_gpio);

	// The ON/OFF control will be done in gps_drv.c
}



static void __init omap_tablet_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	c1_version_init();
#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
	if (cpu_is_omap447x() || board_is_c1_dvt1_atce()) {
		omap_emif_set_device_details(1, &lpddr2_elpida_4G_S4_info,
				lpddr2_elpida_4G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
		omap_emif_set_device_details(2, &lpddr2_elpida_4G_S4_info,
				lpddr2_elpida_4G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
	} else {
		omap_emif_set_device_details(1, &lpddr2_elpida_2G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
		omap_emif_set_device_details(2, &lpddr2_elpida_2G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
	}
#endif
	if(board_is_c1_evt1())
		omap4_mux_init(board_mux, NULL, package);
	else
		omap4_mux_init(board_mux_v2, NULL, package);
	omap_init_board_version(0);
	omap_create_board_props();

	gps_lna_init();
#if 0
	if (board_is_c1_dvt1_atcn() || board_is_c1_dvt1_atce()) {
		tablet_bci_pdata.battery_tmp_tbl = dvt2_batt_table;
		tablet_bci_pdata.tblsize = ARRAY_SIZE(dvt2_batt_table);
	}
#endif
	set_osc_timings();
	omap4_i2c_init();
	platform_add_devices(tablet_devices, ARRAY_SIZE(tablet_devices));
	omap4_board_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);

	omap4_ehci_ohci_init();
	usb_musb_init(&musb_board_data);
	init_duty_governor();

	omap_init_dmm_tiler();
	omap4_register_ion();
	omap_die_governor_register_pdata(&omap_gov_pdata);
	tablet_display_init();
	//if (!cpu_is_in_rsa_mode()) tablet_touch_init();
	tablet_camera_mux_init();
	if (!cpu_is_in_rsa_mode()) tablet_sensor_init();
	if (!cpu_is_in_rsa_mode()) tablet_button_init();
	omap4plus_connectivity_init();
	/*if (!cpu_is_in_rsa_mode()) {
		status = omap_ethernet_init();
		if (status) {
			pr_err("Ethernet initialization failed: %d\n", status);
		} else {
			tablet_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
			spi_register_board_info(tablet_spi_board_info,
					ARRAY_SIZE(tablet_spi_board_info));
		}
	}*/

	if (!cpu_is_apm4460() && cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

	omap_enable_smartreflex_on_init();
	if (!cpu_is_apm4460() && cpu_is_omap446x())
		platform_device_register(&thermistor);

}

static void __init omap_tablet_reserve(void)
{
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);
	tablet_android_display_setup();
	omap4_ion_init();
	omap4_secure_workspace_addr_default();
	omap_reserve();
}

static void __init omap_tablet_init_early(void)
{
	omap4430_init_early();
	if (!cpu_is_apm4460() && cpu_is_omap446x())
		omap_tps6236x_gpio_no_reset_wa(TPS62361_GPIO, -1, 32);
}

MACHINE_START(OMAP_BLAZE, "OMAP44XX Tablet board")
	.atag_offset	= 0x100,
	.reserve	= omap_tablet_reserve,
	.map_io		= omap4_map_io,
	.init_early	= omap_tablet_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= omap_tablet_init,
	.timer		= &omap4_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
