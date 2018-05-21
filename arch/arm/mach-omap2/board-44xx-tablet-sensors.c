/*
 * arch/arm/mach-omap2/board-44xx-tablet-sensors.c
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_data/lsm6ds3.h>
#include <linux/input/lis3mdl.h>

#if 0
#include <linux/i2c/bma180.h>
#include <linux/i2c/tsl2771.h>
#include <linux/i2c/mpu3050.h>
#include <linux/platform_data/lm75_platform_data.h>
#endif

#include <linux/input/ltr.h>

#include <plat/i2c.h>

#include "board-44xx-tablet.h"
#include "mux.h"

#if 0
#define OMAP4_BMA180ACCL_GPIO		178
#define OMAP4_TSL2771_INT_GPIO		184
#define OMAP4_TSL2771_PWR_GPIO		188
#define OMAP4_MPU3050GYRO_GPIO		2

/* BMA180 Accelerometer Begin */
static struct bma180accel_platform_data bma180accel_platform_data = {
	.ctrl_reg0	= 0x11,
	.g_range	= BMA_GRANGE_2G,
	.bandwidth	= BMA_BW_10HZ,
	.mode		= BMA_MODE_LOW_NOISE,
	.bit_mode	= BMA_BITMODE_14BITS,
	.smp_skip	= 1,
	.def_poll_rate	= 200,
	.fuzz_x		= 25,
	.fuzz_y		= 25,
	.fuzz_z		= 25,
};

static void blaze_tablet_bma180accl_init(void)
{
	if (gpio_request(OMAP4_BMA180ACCL_GPIO, "Accelerometer") < 0) {
		pr_err("Accelerometer GPIO request failed\n");
		return;
	}
	gpio_direction_input(OMAP4_BMA180ACCL_GPIO);
}
/* BMA180 Accelerometer End */

/* TSL2771 ALS/Prox Begin */
static void omap_tsl2771_power(int state)
{
	gpio_set_value(OMAP4_TSL2771_PWR_GPIO, state);
}

static void blaze_tablet_tsl2771_init(void)
{
	/* TO DO: Not sure what the use case of the proximity is on a tablet
	 * but the interrupt may need to be wakeable if and only if proximity
	 * is enabled but for now leave it alone */
	gpio_request(OMAP4_TSL2771_PWR_GPIO, "tsl2771_power");
	gpio_direction_output(OMAP4_TSL2771_PWR_GPIO, 0);
}

/* TO DO: Need to create a interrupt threshold table here */

struct tsl2771_platform_data tsl2771_data = {
	.irq_flags	= (IRQF_TRIGGER_LOW | IRQF_ONESHOT),
	.flags		= (TSL2771_USE_ALS | TSL2771_USE_PROX),
	.def_enable			= 0x0,
	.als_adc_time			= 0xdb,
	.prox_adc_time			= 0xff,
	.wait_time			= 0x00,
	.als_low_thresh_low_byte	= 0x0,
	.als_low_thresh_high_byte	= 0x0,
	.als_high_thresh_low_byte	= 0x0,
	.als_high_thresh_high_byte	= 0x0,
	.prox_low_thresh_low_byte	= 0x0,
	.prox_low_thresh_high_byte	= 0x0,
	.prox_high_thresh_low_byte	= 0x0,
	.prox_high_thresh_high_byte	= 0x0,
	.interrupt_persistence		= 0xf6,
	.config				= 0x00,
	.prox_pulse_count		= 0x03,
	.gain_control			= 0xE0,
	.glass_attn			= 0x01,
	.device_factor			= 0x34,
	.tsl2771_pwr_control		= omap_tsl2771_power,
};
/* TSL2771 ALS/Prox End */

/* MPU3050 Gyro Begin */
static void blaze_tablet_mpu3050_init(void)
{
	if (gpio_request(OMAP4_MPU3050GYRO_GPIO, "mpu3050") < 0) {
		pr_err("%s: MPU3050 GPIO request failed\n", __func__);
		return;
	}
	gpio_direction_input(OMAP4_MPU3050GYRO_GPIO);
}

static struct mpu3050gyro_platform_data mpu3050_platform_data = {
	.irq_flags = (IRQF_TRIGGER_HIGH | IRQF_ONESHOT),
	.default_poll_rate = 200,
	.slave_i2c_addr = 0x40,
	.sample_rate_div = 0x00,
	.dlpf_fs_sync = 0x10,
	.interrupt_cfg = (MPU3050_INT_CFG_OPEN | MPU3050_INT_CFG_LATCH_INT_EN |
		MPU3050_INT_CFG_MPU_RDY_EN | MPU3050_INT_CFG_RAW_RDY_EN),
};
/* MPU3050 Gyro End */

static struct lm75_platform_data lm75_pdata = {
	.domain = "case",
	.stats_enable = 1,
	.average_period = 2000,
	.average_number = 20,
};

static struct i2c_board_info __initdata blaze_tablet_i2c_bus3_sensor_info[] = {
	{
		I2C_BOARD_INFO("tmp105", 0x48),
		.platform_data = &lm75_pdata,
	},
};

static struct i2c_board_info __initdata blaze_tablet_i2c_bus4_sensor_info[] = {
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	},
	{
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
	{
		I2C_BOARD_INFO("bma180_accel", 0x40),
		.platform_data = &bma180accel_platform_data,
	},
	{
		I2C_BOARD_INFO("mpu3050_gyro", 0x68),
		.platform_data = &mpu3050_platform_data,
	},
	{
		I2C_BOARD_INFO(TSL2771_NAME, 0x39),
		.platform_data = &tsl2771_data,
		.irq = OMAP4_TSL2771_INT_GPIO,
	},
};
#endif

#if defined(CONFIG_SENSORS_BMA2X2) || defined(CONFIG_SENSORS_BMG) || defined(CONFIG_SENSORS_BMM050)
struct bosch_sensor_specific {
	char *name;
	/* 0 to 8; 8 is for C1 only */
	int place;
	int irq;
	int (*irq_gpio_cfg)(void);
};
#endif

#ifdef CONFIG_INPUT_LSM6DS3
#define OMAP4_LSM6DS3_INT1_GPIO      264
#define OMAP4_LSM6DS3_INT2_GPIO      265
static struct lsm6ds3_platform_data lsm6ds3_platform_data = {
        .drdy_int_pin = 1,
};
#define OMAP4_LIS3MDL_INT_GPIO  35
#define OMAP4_LIS3MDL_DRAY_GPIO 33
static struct lis3mdl_mag_platform_data lis3mdl_mag_platform_data = {
                .dray = OMAP4_LIS3MDL_DRAY_GPIO,
};
#endif

#ifdef CONFIG_SENSORS_BMA2X2

#define OMAP4_BMA2X2_INT1_GPIO      264
#define OMAP4_BMA2X2_INT2_GPIO      265

static struct bosch_sensor_specific bma2x2_platform_data = {
	.name  = "bma2x2",
	.place = 8,
};
#endif

#ifdef CONFIG_SENSORS_BMG
static struct bosch_sensor_specific bmg160_platform_data = {
	.name  = "bmg160",
	.place = 8,
};
#endif

#ifdef CONFIG_SENSORS_BMM050
static struct bosch_sensor_specific bmm050_platform_data = {
	.name  = "bmm050",
	.place = 8,
};
#endif

#ifdef CONFIG_INPUT_LTR303
static struct ltr303_platform_data ltr303_sensor_platform_data = {
	.pfd_gpio_int_no = 258,
};
#endif

#ifdef CONFIG_INPUT_LTR659
static struct ltr659_platform_data ltr659_sensor_platform_data = {
	.pfd_ps_lowthresh = 200,
	.pfd_ps_highthresh = 1000,
	.pfd_gpio_int_no = 259,
};
#endif

static struct i2c_board_info __initdata blaze_tablet_i2c_bus4_sensor_info[] = {
#ifdef CONFIG_SENSORS_BMA2X2
	{
		I2C_BOARD_INFO("bma2x2", 0x18),
		.platform_data = &bma2x2_platform_data,
#ifdef CONFIG_SENSORS_BMA2X2_ENABLE_INT1
		.irq = OMAP4_BMA2X2_INT1_GPIO,
#endif
#ifdef CONFIG_SENSORS_BMA2X2_ENABLE_INT2
		.irq = OMAP4_BMA2X2_INT2_GPIO,
#endif
	},
#endif
#ifdef CONFIG_SENSORS_BMG
	{
		I2C_BOARD_INFO("bmg160", 0x68),
		.platform_data = &bmg160_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LSM6DS3
        {
                I2C_BOARD_INFO("lsm6ds3", 0x6a),
                .platform_data = &lsm6ds3_platform_data,
                .irq = OMAP4_LSM6DS3_INT1_GPIO,
        },
        {
                I2C_BOARD_INFO("lis3mdl", 0x38>>1),
                .platform_data = &lis3mdl_mag_platform_data,
                .irq = OMAP4_LIS3MDL_INT_GPIO,
        },
#endif
#ifdef CONFIG_SENSORS_BMM050
	{
		I2C_BOARD_INFO("bmm050", 0x10),
		.platform_data = &bmm050_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LTR303
	{
		I2C_BOARD_INFO("LTR303ALS", 0x29),
		.platform_data = &ltr303_sensor_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_LTR659
	{
		I2C_BOARD_INFO("LTR659ALSPS", 0x23),
		.platform_data = &ltr659_sensor_platform_data,
	},
#endif
};

int __init tablet_sensor_init(void)
{
#if 0
	blaze_tablet_tsl2771_init();
	blaze_tablet_mpu3050_init();
	blaze_tablet_bma180accl_init();

	i2c_register_board_info(3, blaze_tablet_i2c_bus3_sensor_info,
		ARRAY_SIZE(blaze_tablet_i2c_bus3_sensor_info));
#endif
	i2c_register_board_info(4, blaze_tablet_i2c_bus4_sensor_info,
		ARRAY_SIZE(blaze_tablet_i2c_bus4_sensor_info));

	return 0;
}

