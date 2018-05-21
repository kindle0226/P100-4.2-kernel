/* Lite-On LTR-659ALS Android / Linux Driver
 *
 * Copyright (C) 2013 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/version.h>
#include <linux/input/ltr.h>


/* LTR-659 Registers */
#define LTR659_PS_CONTR		0x81
#define LTR659_PS_LED		0x82
#define LTR659_PS_N_PULSES	0x83
#define LTR659_PS_MEAS_RATE	0x84
#define LTR659_PART_ID		0x86
#define LTR659_MANUFACTURER_ID	0x87
#define LTR659_ALS_PS_STATUS	0x8C
#define LTR659_PS_DATA_0	0x8D
#define LTR659_PS_DATA_1	0x8E
#define LTR659_INTERRUPT	0x8F
#define LTR659_PS_THRES_UP_0	0x90
#define LTR659_PS_THRES_UP_1	0x91
#define LTR659_PS_THRES_LOW_0	0x92
#define LTR659_PS_THRES_LOW_1	0x93
#define LTR659_PS_OFFSET_1	0x94
#define LTR659_PS_OFFSET_0	0x95
#define LTR659_INTERRUPT_PRST	0x9E
/* LTR-659 Registers */


#define SET_BIT 1
#define CLR_BIT 0

#define ALS 0
#define PS 1
#define ALSPS 2
//#define PS_W_SATURATION_BIT	3

/* address 0x81 */
#define PS_MODE_ACTIVE		(3 << 0)
#define PS_MODE_STDBY		(0 << 0)
#define PS_GAIN_16x			(0 << 2)
#define PS_GAIN_32x			(2 << 2)
#define PS_GAIN_64x			(3 << 2)
#define PS_SATUR_INDIC_EN	(1 << 5)
#define PS_SATU_INDIC_DIS	(0 << 5)
#define PS_MODE_RDBCK		0
#define PS_GAIN_RDBCK		1
#define PS_SATUR_RDBCK		2
#define PS_CONTR_RDBCK		3

/* address 0x82 */
#define LED_CURR_5MA		(0 << 0)
#define LED_CURR_10MA		(1 << 0)
#define LED_CURR_20MA		(2 << 0)
#define LED_CURR_50MA		(3 << 0)
#define LED_CURR_100MA		(4 << 0)
#define LED_CURR_DUTY_25PC		(0 << 3)
#define LED_CURR_DUTY_50PC		(1 << 3)
#define LED_CURR_DUTY_75PC		(2 << 3)
#define LED_CURR_DUTY_100PC	(3 << 3)
#define LED_PUL_FREQ_30KHZ		(0 << 5)
#define LED_PUL_FREQ_40KHZ		(1 << 5)
#define LED_PUL_FREQ_50KHZ		(2 << 5)
#define LED_PUL_FREQ_60KHZ		(3 << 5)
#define LED_PUL_FREQ_70KHZ		(4 << 5)
#define LED_PUL_FREQ_80KHZ		(5 << 5)
#define LED_PUL_FREQ_90KHZ		(6 << 5)
#define LED_PUL_FREQ_100KHZ	(7 << 5)
#define LED_CURR_RDBCK			0
#define LED_CURR_DUTY_RDBCK	1
#define LED_PUL_FREQ_RDBCK		2
#define PS_LED_RDBCK			3

/* address 0x84 */
#define PS_MEAS_RPT_RATE_50MS		(0 << 0)
#define PS_MEAS_RPT_RATE_70MS		(1 << 0)
#define PS_MEAS_RPT_RATE_100MS	(2 << 0)
#define PS_MEAS_RPT_RATE_200MS	(3 << 0)
#define PS_MEAS_RPT_RATE_500MS	(4 << 0)
#define PS_MEAS_RPT_RATE_1000MS	(5 << 0)
#define PS_MEAS_RPT_RATE_2000MS	(6 << 0)
#define PS_MEAS_RPT_RATE_10MS		(8 << 0)

/* address 0x86 */
#define PART_NUM_ID_RDBCK		0
#define REVISION_ID_RDBCK		1
#define PART_ID_REG_RDBCK		2

/* address 0x8C */
#define PS_DATA_STATUS_RDBCK		0
#define PS_INTERR_STATUS_RDBCK	1
#define ALS_PS_STATUS_RDBCK		2

/* address 0x8F */
#define INT_MODE_00					(0 << 0)
#define INT_MODE_PS_TRIG			(1 << 0)
#define INT_MODE_ALS_TRIG			(2 << 0)
#define INT_MODE_ALSPS_TRIG		(3 << 0)
#define INT_POLAR_ACT_LO			(0 << 2)
#define INT_POLAR_ACT_HI			(1 << 2)
#define INT_MODE_RDBCK				0
#define INT_POLAR_RDBCK			1
#define INT_INTERRUPT_RDBCK		2

/* address 0x9E */
#define PS_PERSIST_SHIFT	4
#define PS_PRST_RDBCK		1

#define PON_DELAY		600

#define PS_MIN_MEASURE_VAL	0
#define PS_MAX_MEASURE_VAL	2047
#define PS_VALID_MEASURE_MASK  PS_MAX_MEASURE_VAL
#define LO_LIMIT			0
#define HI_LIMIT			1
#define LO_N_HI_LIMIT	2
#define PS_OFFSET_MIN_VAL		0
#define PS_OFFSET_MAX_VAL		1023

#define DRIVER_VERSION "1.12"
#define PARTID 0x92
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "LTR659ALSPS"

#define ACT_INTERRUPT 1

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR659_IOCTL_MAGIC      'c'

/* IOCTLs for ltr659 device */
#define LTR659_IOCTL_PS_ENABLE		_IOR(LTR659_IOCTL_MAGIC, 1, int *)
#define LTR659_IOCTL_PS_GET_ENABLED	_IOW(LTR659_IOCTL_MAGIC, 2, int *)

struct ltr659_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
	struct early_suspend early_suspend;
	struct wake_lock ps_wake_lock;
	struct mutex bus_lock;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_suspend_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;

	/* PS */
	uint8_t ps_enable_flag;
	uint8_t ps_suspend_enable_flag;
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;
};

struct ltr659_data *psensor_info;

uint16_t winfac1 = 100;
uint16_t winfac2 = 80;
uint16_t winfac3 = 44;
uint8_t eqn_prev = 0;
uint8_t ratio_old = 0;
uint16_t ps_init_kept_data[8];
uint16_t ps_ct_avg;
uint8_t ps_grabData_stage = 0;
uint32_t ftn_init;
uint32_t ftn_final;
uint32_t ntf_final;
uint16_t lux_val_prev = 0;
uint8_t ps_kept_data_counter = 0;

/* I2C Read */
// take note --------------------------------------- 
// for i2c read, need to send the register address follwed by buffer over to register.
// There should not be a stop in between register address and buffer.  
// There should not be release of lock in between register address and buffer. 
// take note ---------------------------------------
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = psensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = psensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(psensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = psensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(psensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* Set register bit */
static int8_t _ltr659_set_bit(struct i2c_client *client, uint8_t set, uint8_t cmd, uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}


static uint16_t read_ps_adc_value(struct ltr659_data *ltr659)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint8_t buffer[4];

	//mutex_lock(&ltr659->bus_lock);
	
	buffer[0] = LTR659_PS_DATA_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 2);

	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		//mutex_unlock(&ltr659->bus_lock);
		
		return ret;
	}

	ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr659->i2c_client->dev, 
			"%s | ps value = 0x%04X\n", __func__, 
			ps_val);

	if (ps_val > PS_MAX_MEASURE_VAL) {
		dev_err(&ltr659->i2c_client->dev,
		        "%s: PS Value Error: 0x%X\n", __func__,
		        ps_val);
	}
	ps_val &= PS_VALID_MEASURE_MASK;				
					
	value = ps_val;

	//mutex_unlock(&ltr659->bus_lock);

	return value;
}


static int8_t ps_mode_setup (uint8_t psMode_set_reset, struct ltr659_data *ltr659)
{
	int8_t ret = 0;

	ret = _ltr659_set_bit(ltr659->i2c_client, psMode_set_reset, LTR659_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_gain_setup (uint8_t psgain_range, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF3;

	if (psgain_range == 16) {
		value |= PS_GAIN_16x;
	} else if (psgain_range == 32) {
		value |= PS_GAIN_32x;
	} else if (psgain_range == 64) {
		value |= PS_GAIN_64x;
	}

	buffer[0] = LTR659_PS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_satu_indica_setup(uint8_t pssatuindica_enable, struct ltr659_data *ltr659)
{
	int8_t ret = 0;

	ret = _ltr659_set_bit(ltr659->i2c_client, pssatuindica_enable, LTR659_PS_CONTR, PS_SATUR_INDIC_EN);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS saturation indicator setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_contr_setup(uint8_t ps_contr_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR659_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | PS_CONTR (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_contr_readback (uint8_t rdbck_type, uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PS_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == PS_GAIN_RDBCK) {
		*retVal = (value & 0x0C) >> 2;
	} else if (rdbck_type == PS_SATUR_RDBCK) {
		*retVal = (value & 0x20) >> 5;
	} else if (rdbck_type == PS_CONTR_RDBCK) {
		*retVal = value & 0x2F;
	}

	return ret;
}


static int8_t ps_ledCurrent_setup (uint8_t psledcurr_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (psledcurr_val == 5) {
		value |= LED_CURR_5MA;
	} else if (psledcurr_val == 10) {
		value |= LED_CURR_10MA;
	} else if (psledcurr_val == 20) {
		value |= LED_CURR_20MA;
	} else if (psledcurr_val == 50) {
		value |= LED_CURR_50MA;
	} else if (psledcurr_val == 100) {
		value |= LED_CURR_100MA;
	}

	buffer[0] = LTR659_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS LED current setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledCurrDuty_setup (uint8_t psleddutycycle_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE7;

	if (psleddutycycle_val == 25) {
		value |= LED_CURR_DUTY_25PC;
	} else if (psleddutycycle_val == 50) {
		value |= LED_CURR_DUTY_50PC;
	} else if (psleddutycycle_val == 75) {
		value |= LED_CURR_DUTY_75PC;
	} else if (psleddutycycle_val == 100) {
		value |= LED_CURR_DUTY_100PC;
	} 

	buffer[0] = LTR659_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS LED current duty setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledPulseFreq_setup (uint8_t pspulreq_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x1F;

	if (pspulreq_val == 30) {
		value |= LED_PUL_FREQ_30KHZ;
	} else if (pspulreq_val == 40) {
		value |= LED_PUL_FREQ_40KHZ;
	} else if (pspulreq_val == 50) {
		value |= LED_PUL_FREQ_50KHZ;
	} else if (pspulreq_val == 60) {
		value |= LED_PUL_FREQ_60KHZ;
	} else if (pspulreq_val == 70) {
		value |= LED_PUL_FREQ_70KHZ;
	} else if (pspulreq_val == 80) {
		value |= LED_PUL_FREQ_80KHZ;
	} else if (pspulreq_val == 90) {
		value |= LED_PUL_FREQ_90KHZ;
	} else if (pspulreq_val == 100) {
		value |= LED_PUL_FREQ_100KHZ;
	}

	buffer[0] = LTR659_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS LED pulse frequency setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR659_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | PS_LED (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_led_readback (uint8_t rdbck_type, uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == LED_CURR_RDBCK) {
		*retVal = (value & 0x07);
	} else if (rdbck_type == LED_CURR_DUTY_RDBCK) {
		*retVal = (value & 0x18) >> 3;
	} else if (rdbck_type == LED_PUL_FREQ_RDBCK) {
		*retVal = (value & 0xE0) >> 5;
	} else if (rdbck_type == PS_LED_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR659_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val > 15) {
		pspulsecount_val = 15;
	}
	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | PS_LED_COUNT (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t ps_ledPulseCount_readback (uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR659_PS_N_PULSES;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t ps_meas_rate_setup(uint16_t meas_rate_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	//mutex_lock(&ltr659->bus_lock);

	buffer[0] = LTR659_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		//mutex_unlock(&ltr659->bus_lock);
		
		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50) {
		value |= PS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 70) {
		value |= PS_MEAS_RPT_RATE_70MS;
	} else if (meas_rate_val == 100) {
		value |= PS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= PS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= PS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= PS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= PS_MEAS_RPT_RATE_2000MS;
	} else if (meas_rate_val == 10) {
		value |= PS_MEAS_RPT_RATE_10MS;		
	}

	buffer[0] = LTR659_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS measurement rate setup fail...\n", __func__);

		//mutex_unlock(&ltr659->bus_lock);
		
		return ret;
	}

	//mutex_unlock(&ltr659->bus_lock);

	return ret;
}


static int8_t ps_meas_rate_readback (uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR659_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = (value & 0x0F);

	return ret;
}



static int8_t part_ID_reg_readback (uint8_t rdbck_type, uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR659_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PART_NUM_ID_RDBCK) {
		*retVal = (value & 0xF0) >> 4;
	} else if (rdbck_type == REVISION_ID_RDBCK) {
		*retVal = value & 0x0F;
	} else if (rdbck_type == PART_ID_REG_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t manu_ID_reg_readback (uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR659_MANUFACTURER_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t als_ps_status_reg (uint8_t data_status_type, uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (data_status_type == PS_DATA_STATUS_RDBCK) {
		*retVal = (value & 0x01);
	} else if (data_status_type == PS_INTERR_STATUS_RDBCK) {
		*retVal = (value & 0x02) >> 1;
	} else if (data_status_type == ALS_PS_STATUS_RDBCK) {
		*retVal = value;
	}

	return ret;
}


static int8_t interrupt_mode_setup (uint8_t interr_mode_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFC;

	if (interr_mode_val == 0) {
		value |= INT_MODE_00;
	} else if (interr_mode_val == 1) {
		value |= INT_MODE_PS_TRIG;
	} else if (interr_mode_val == 2) {
		value |= INT_MODE_ALS_TRIG;
	} else if (interr_mode_val == 3) {
		value |= INT_MODE_ALSPS_TRIG;
	} 

	buffer[0] = LTR659_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s Interrupt mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_polarity_setup (uint8_t interr_polar_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFB;

	if (interr_polar_val == 0) {
		value |= INT_POLAR_ACT_LO;
	} else if (interr_polar_val == 1) {
		value |= INT_POLAR_ACT_HI;
	}

	buffer[0] = LTR659_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s Interrupt polarity setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_setup(uint8_t interrupt_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR659_INTERRUPT;

	/* Default settings used for now. */
	buffer[1] = interrupt_val;
	buffer[1] &= 0x07;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s |Interrupt (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


static int8_t interrupt_readback (uint8_t rdbck_type, uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == INT_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == INT_POLAR_RDBCK) {
		*retVal = (value & 0x04) >> 2;
	} else if (rdbck_type == INT_INTERRUPT_RDBCK) {
		*retVal = (value & 0x07);
	}

	return ret;
}


static int8_t ps_offset_setup (uint16_t ps_offset_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[3];
	
	buffer[0] = LTR659_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s PS offset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_offset_readback (uint16_t *offsetval, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2];
	uint16_t value;

	buffer[0] = LTR659_PS_OFFSET_1;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value <<= 8;
	value += buffer[1];

	*offsetval = value;

	return ret;
}


static int8_t interrupt_persist_setup (uint8_t interr_persist_val, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	value = interr_persist_val;

	buffer[0] = LTR659_INTERRUPT_PRST;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s Interrupt persist setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_prst_readback (uint8_t *retVal, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR659_INTERRUPT_PRST;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	*retVal = value;

	return ret;
}


/* Set PS range */
static int8_t set_ps_range(uint16_t lt, uint16_t ht, uint8_t lo_hi, struct ltr659_data *ltr659)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	//mutex_lock(&ltr659->bus_lock);

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR659_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR659_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR659_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
	}	

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

		//mutex_unlock(&ltr659->bus_lock);
		
		return ret;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s Set ps range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);

	//mutex_unlock(&ltr659->bus_lock);

	return ret;
}


static int8_t ps_range_readback (uint16_t *lt, uint16_t *ht, struct ltr659_data *ltr659)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo, value_hi;

	//mutex_lock(&ltr659->bus_lock);

	buffer[0] = LTR659_PS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		//mutex_unlock(&ltr659->bus_lock);
		
		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	//mutex_unlock(&ltr659->bus_lock);

	return ret;
}


static uint16_t findCT_Avg (uint16_t *ps_val)
{
	uint8_t i_ctr, min_Index, max_Index;
	uint16_t temp = 0, max_val, min_val;
	//struct ltr659_data *ltr659 = psensor_info;

	//mutex_lock(&ltr659->bus_lock);

	//findMinMaxVal_Index(ps_val, &min_val, &min_Index, &max_val, &max_Index);
	max_val = ps_val[3];
	max_Index = 3;
	min_val = ps_val[3];
	min_Index = 3;

	for (i_ctr = 3; i_ctr < 8; i_ctr++) {
		if (ps_val[i_ctr] > max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = 3; i_ctr < 8; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val) {
		// all values are the same
		for (i_ctr = 3; i_ctr < 6; i_ctr++) {
			temp += ps_val[i_ctr];
		}
	} else {
		for (i_ctr = 3; i_ctr < 8; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index)) {
				temp += ps_val[i_ctr];
			}
		}
	}

	temp = (temp / 3);

	//mutex_unlock(&ltr659->bus_lock);

	return temp;
}


// take note ------------------------------------------
// This function should be called in the function which is called when the CALL button is pressed.
// take note ------------------------------------------
static void setThrDuringCall (void)
{
	int8_t ret;
	struct ltr659_data *ltr659 = psensor_info;

	// set ps measurement rate to 10ms
	ret = ps_meas_rate_setup(10, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;

	ret = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
	}
}


//(Linux RTOS)>
/* Report PS input event */
static void report_ps_input_event(struct ltr659_data *ltr659)
{
#define 	FAR_VAL		1
#define 	NEAR_VAL		0
	int8_t ret;
	uint16_t adc_value;
	//int thresh_hi, thresh_lo, thresh_delta;
	uint16_t thresh_hi, thresh_lo;
	//static uint8_t ps_kept_data_counter;

	//ltr659->mode = PS;
	//adc_value = read_adc_value (ltr659);
	adc_value = read_ps_adc_value (ltr659);
	
	ps_init_kept_data[ps_kept_data_counter] = adc_value;

	if (ps_grabData_stage == 0) {
		if (ps_kept_data_counter < 7) {
			ps_kept_data_counter++;		
		} else if (ps_kept_data_counter >= 7) {
			ps_ct_avg = findCT_Avg(ps_init_kept_data);
			ps_grabData_stage = 1;
			ftn_init = ps_ct_avg * 17;
		}
	}

	if (ps_grabData_stage == 1) {
		if ((ftn_init - (ps_ct_avg * 10)) < 1400) {
			ftn_final = (ps_ct_avg * 10) + 1400;
		} else {
			if ((ftn_init - (ps_ct_avg * 10)) > 1800) {
				ftn_final = (ps_ct_avg * 10) + 1800;
			} else {
				ftn_final = ftn_init;
			}
		}
		ntf_final = (ftn_final - (ps_ct_avg * 10));
		ntf_final *= 4;
		ntf_final /= 100;
		ntf_final += ps_ct_avg;
		ftn_final /= 10;
		if (ntf_final >= 2047) {
			ntf_final = 2047;
		}
		if (ftn_final >= 2047) {
			ftn_final = 2047;
		}
		//ret = set_ps_range((uint16_t)0, (uint16_t)ftn_final, LO_N_HI_LIMIT);
		ret = set_ps_range((uint16_t)ntf_final, (uint16_t)ftn_final, LO_N_HI_LIMIT, ltr659);
		if (ret < 0) {
			dev_err(&ltr659->i2c_client->dev, "%s : PS thresholds setting Fail...\n", __func__);
		}
		// dummy read just to clear the PS interrupt bit
		//ltr659->mode = PS;
		//adc_value = read_adc_value (ltr659);
		adc_value = read_ps_adc_value (ltr659);
		// dummy read just to clear the PS interrupt bit

		ret = ps_meas_rate_setup(50, ltr659);
		if (ret < 0) {
			dev_err(&ltr659->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		}
		
		ps_grabData_stage = 2;
	}

	if (ps_grabData_stage == 2) {
		ret = ps_range_readback(&thresh_lo, &thresh_hi, ltr659);
		if (ret < 0) {
			dev_err(&ltr659->i2c_client->dev, "%s: PS threshold range readback Fail...\n", __func__);
		}
		printk("%s: report distance 0x%x \n", __func__, adc_value);
		input_report_abs(ltr659->ps_input_dev, ABS_DISTANCE, adc_value);
		input_sync(ltr659->ps_input_dev);

		/* Adjust measurement range using a crude filter to prevent interrupt jitter */
		if ((adc_value > ftn_final) || (adc_value < ntf_final)) {
			// FTN
			if (adc_value > ftn_final) {
				if ((thresh_lo == ntf_final) && (thresh_hi == ftn_final)) {
					thresh_lo = ntf_final;
					thresh_hi = 2047;
				}
				//input_report_abs(ltr659->ps_input_dev, ABS_DISTANCE, NEAR_VAL);
				//input_sync(ltr659->ps_input_dev);
			}
			// FTN

			// NTF
			if (adc_value < ntf_final) {
				if ((thresh_lo == ntf_final) && (thresh_hi == 2047)) {
					//thresh_lo = 0;
					thresh_lo = ntf_final;
					thresh_hi = ftn_final;				
				}
				//input_report_abs(ltr659->ps_input_dev, ABS_DISTANCE, FAR_VAL);
				//input_sync(ltr659->ps_input_dev);
			}
			// NTF

			ret = set_ps_range((uint16_t)thresh_lo, (uint16_t)thresh_hi, LO_N_HI_LIMIT, ltr659);
			if (ret < 0) {
				dev_err(&ltr659->i2c_client->dev, "%s : PS thresholds setting Fail...\n", __func__);
			}
			// dummy read just to clear the PS interrupt bit
			//ltr659->mode = PS;
			//adc_value = read_adc_value (ltr659);
			adc_value = read_ps_adc_value (ltr659);
			// dummy read just to clear the PS interrupt bit
		}
	}
}


/* Work when interrupt */
static void ltr659_schedwork(struct work_struct *work)
{
	int8_t ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr659_data *ltr659 = psensor_info;
	uint8_t buffer[2];

	buffer[0] = LTR659_ALS_PS_STATUS;	
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0A;
	newdata = status & 0x05;

	// PS interrupt and PS with new data
	if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
		ltr659->ps_irq_flag = 1;
		report_ps_input_event(ltr659);
		ltr659->ps_irq_flag = 0;
	}

	enable_irq(ltr659->irq);
}

static DECLARE_WORK(irq_workqueue, ltr659_schedwork);


/* IRQ Handler */
static irqreturn_t ltr659_irq_handler(int irq, void *data)
{
	struct ltr659_data *ltr659 = data;

	/* disable an irq without waiting */
	disable_irq_nosync(ltr659->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}


#if 1
static int ltr659_gpio_irq(struct ltr659_data *ltr659)
{
	int rc = 0;

	rc = gpio_request(ltr659->gpio_int_no, DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev,"%s: GPIO %d Request Fail (%d)\n", __func__, ltr659->gpio_int_no, rc);
		return rc;
	}

	rc = gpio_direction_input(ltr659->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Set GPIO %d as Input Fail (%d)\n", __func__, ltr659->gpio_int_no, rc);
		goto out1;
	}

	ltr659->irq = gpio_to_irq(ltr659->gpio_int_no);

	/* Configure an active low trigger interrupt for the device */
	rc = request_threaded_irq(ltr659->irq, NULL, ltr659_irq_handler, IRQF_TRIGGER_FALLING, DEVICE_NAME, ltr659);
	//rc = request_irq(ltr659->irq, ltr659_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Request IRQ (%d) for GPIO %d Fail (%d)\n", __func__, ltr659->irq,
		        ltr659->gpio_int_no, rc);
		goto out1;
	}

	return rc;

out1:
	gpio_free(ltr659->gpio_int_no);

	return rc;
}
#endif
//(Linux RTOS)<


/* PS Enable */
static int8_t ps_enable_init(struct ltr659_data *ltr659)
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read

	setThrDuringCall();

	if (ltr659->ps_enable_flag) {
		dev_info(&ltr659->i2c_client->dev, "%s: already enabled\n", __func__);
		return 0;
	}

	/* Set thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	//rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
	//rc = set_ps_range(PS_MIN_MEASURE_VAL, 400, LO_N_HI_LIMIT);
	rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT, ltr659);
#else
	rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, LO_N_HI_LIMIT, ltr659);
#endif
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	//(Linux RTOS)>
#if 0
	/* Allows this interrupt to wake the system */
	//rc = irq_set_irq_wake(ltr659->irq, 1);
	rc = set_irq_wake(ltr659->irq, 1);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: IRQ-%d WakeUp Enable Fail...\n", __func__, ltr659->irq);
		return rc;
	}
#endif
	//(Linux RTOS)<

	rc = ps_led_setup(0x7F, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_ledPulseCount_setup(0x08, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
	}

	rc = ps_meas_rate_setup(10, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_contr_setup(0x03, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	// dummy read
	buffer[0] = LTR659_PS_CONTR;
	I2C_Read(buffer, 1);
	// dumy read
	
	ltr659->ps_enable_flag = 1;	

	return rc;
}


/* PS Disable */
static int8_t ps_disable(struct ltr659_data *ltr659)
{
	int8_t rc = 0;

	if (ltr659->ps_enable_flag == 0) {
		dev_info(&ltr659->i2c_client->dev, "%s: already disabled\n", __func__);
		return 0;
	}

	//(Linux RTOS)>
#if 0
	/* Don't allow this interrupt to wake the system anymore */
	//rc = irq_set_irq_wake(ltr659->irq, 0);
	rc = set_irq_wake(ltr659->irq, 0);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: IRQ-%d WakeUp Disable Fail...\n", __func__, ltr659->irq);
		return rc;
	}
#endif
	//(Linux RTOS)<

	//rc = _ltr659_set_bit(ltr659->i2c_client, CLR_BIT, LTR659_PS_CONTR, PS_MODE);
	rc = ps_mode_setup(CLR_BIT, ltr659);
	if (rc < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	ltr659->ps_enable_flag = 0;

	return rc;
}


/* PS open fops */
ssize_t ps_open(struct inode *inode, struct file *file)
{
	struct ltr659_data *ltr659 = psensor_info;

	if (ltr659->ps_opened)
		return -EBUSY;

	ltr659->ps_opened = 1;

	return 0;
}


/* PS release fops */
ssize_t ps_release(struct inode *inode, struct file *file)
{
	struct ltr659_data *ltr659 = psensor_info;

	ltr659->ps_opened = 0;

	return ps_disable(ltr659);	
}


/* PS IOCTL */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int ps_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct ltr659_data *ltr659 = psensor_info;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case LTR659_IOCTL_PS_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			rc = val ? ps_enable_init(ltr659) : ps_disable(ltr659);

			break;
		case LTR659_IOCTL_PS_GET_ENABLED:
			rc = put_user(ltr659->ps_enable_flag, (unsigned long __user *)arg);

			break;
		default:
			pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = ps_ioctl
	#else
	.unlocked_ioctl = ps_ioctl
	#endif
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr659_ps",
	.fops = &ps_fops
};

static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;

	ret = sprintf(buf, "%d\n", psensor_info->ps_enable_flag);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int ret;
	struct ltr659_data *ltr659 = psensor_info;

	sscanf(buf, "%d", &param);
	printk("%s: store value = %d\n", __func__, param);

	if(param == 1) {
		ret = ps_enable_init(ltr659);
		if (ret < 0) {
			dev_err(&ltr659->i2c_client->dev, "%s Enable PALS Fail...\n", __func__);
		 	return (-1);
		}
	}else {
		ret = ps_disable(ltr659);
		if (ret < 0) {
			dev_err(&ltr659->i2c_client->dev, "%s Disable PALS Fail...\n", __func__);
			return (-1);
		} 
	}
	return count;	
}

static DEVICE_ATTR(ps_enable, 0666, ps_enable_show, ps_enable_store);

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr659_data *ltr659 = psensor_info;

	//ltr659->mode = PS;
	//value = read_adc_value(ltr659);
	value = read_ps_adc_value(ltr659);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d", value);
	
	return ret;
}

static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, NULL);


static ssize_t psadcsaturationBit_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t value;
	uint8_t saturation_bit;
	int ret;
	uint8_t buffer[3];
	struct ltr659_data *ltr659 = psensor_info;

	buffer[0] = LTR659_PS_DATA_0;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	//ltr659->mode = PS_W_SATURATION_BIT;	
	//value = read_adc_value(ltr659);
	saturation_bit = (value >> 15);
	value &= PS_VALID_MEASURE_MASK;
	ret = sprintf(buf, "%d %d\n", value, saturation_bit);
	
	return ret;
}

static DEVICE_ATTR(psadcsaturationBit, 0666, psadcsaturationBit_show, NULL);


static ssize_t ltr659help_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("To show PS value : cat ps_adc\n");
	printk("To show PS value with saturation bit : cat psadcsaturationBit\n\n");
	
	// address 0x81
	printk("Address 0x81 (PS_CONTR)\n");
	printk("PS active mode : echo 1 > psmodesetup\n");
	printk("PS standby mode : echo 0 > psmodesetup\n");
	printk("To read PS mode : cat psmodesetup\n\n");

	printk("PS gain x16 : echo 16 > psgainsetup\n");
	printk("PS gain x32 : echo 32 > psgainsetup\n");
	printk("PS gain x64 : echo 64 > psgainsetup\n");
	printk("To read PS gain : cat psgainsetup\n\n");

	printk("PS saturation indicator enable : echo 1 > pssatuindicasetup\n");
	printk("PS saturation indicator disable : echo 0 > pssatuindicasetup\n");
	printk("To read back PS saturation indicator : cat pssatuindicasetup\n\n");

	printk("Write value to PS_CONTR register (0x81) : echo [hexcode value] > pscontrsetup\n");
	printk("Example...to write 0x0B : echo B > pscontrsetup or echo b > pscontrsetup\n");
	printk("Example...to write 0x13 : echo 13 > pscontrsetup\n");
	printk("To read register PS_CONTR (0x81) : cat pscontrsetup\n\n");
	// address 0x81

	// address 0x82
	printk("Address 0x82 (PS_LED)\n");
	printk("LED current 5mA : echo 5 > psledcurrsetup\n");
	printk("LED current 10mA : echo 10 > psledcurrsetup\n");
	printk("LED current 20mA : echo 20 > psledcurrsetup\n");
	printk("LED current 50mA : echo 50 > psledcurrsetup\n");
	printk("LED current 100mA : echo 100 > psledcurrsetup\n");
	printk("To read LED current : cat psledcurrsetup\n\n");

	printk("LED current duty 25%% : echo 25 > psledcurrduty\n");
	printk("LED current duty 50%% : echo 50 > psledcurrduty\n");
	printk("LED current duty 75%% : echo 75 > psledcurrduty\n");
	printk("LED current duty 100%% : echo 100 > psledcurrduty\n");
	printk("To read LED current duty : cat psledcurrduty\n\n");

	printk("LED pulse freq 30kHz : echo 30 > psledpulsefreqsetup\n");
	printk("LED pulse freq 40kHz : echo 40 > psledpulsefreqsetup\n");
	printk("LED pulse freq 50kHz : echo 50 > psledpulsefreqsetup\n");
	printk("LED pulse freq 60kHz : echo 60 > psledpulsefreqsetup\n");
	printk("LED pulse freq 70kHz : echo 70 > psledpulsefreqsetup\n");
	printk("LED pulse freq 80kHz : echo 80 > psledpulsefreqsetup\n");
	printk("LED pulse freq 90kHz : echo 90 > psledpulsefreqsetup\n");
	printk("LED pulse freq 100kHz : echo 100 > psledpulsefreqsetup\n");
	printk("To read LED pulse freq : cat psledpulsefreqsetup\n\n");

	printk("Write value to PS_LED register (0x82) : echo [hexcode value] > psledsetup\n");
	printk("Example...to write 0x0B : echo B > psledsetup or echo b > psledsetup\n");
	printk("Example...to write 0x13 : echo 13 > psledsetup\n");
	printk("To read register PS_LED (0x82) : cat psledsetup\n\n");
	// address 0x82

	// address 0x83
	printk("Address 0x83 (PS_N_PULSES)\n");
	printk("To set PS LED pulse count (0x83) : echo [pulse count num] > psledpulsecountsetup\n");
	printk("[pulse count num] must be 0 to 15, inclusive\n");
	printk("Example...to set 0 count : echo 0 > psledpulsecountsetup\n");
	printk("Example...to set 13 counts : echo 13 > psledpulsecountsetup\n");
	printk("To read register PS_N_PULSES (0x83) : cat psledpulsecountsetup\n\n");
	// address 0x83

	// address 0x84
	printk("Address 0x84 (PS_MEAS_RATE)\n");
	printk("PS meas repeat rate 50ms : echo 50 > psmeasratesetup\n");
	printk("PS meas repeat rate 70ms : echo 70 > psmeasratesetup\n");
	printk("PS meas repeat rate 100ms : echo 100 > psmeasratesetup\n");
	printk("PS meas repeat rate 200ms : echo 200 > psmeasratesetup\n");
	printk("PS meas repeat rate 500ms : echo 500 > psmeasratesetup\n");
	printk("PS meas repeat rate 1000ms : echo 1000 > psmeasratesetup\n");
	printk("PS meas repeat rate 2000ms : echo 2000 > psmeasratesetup\n");
	printk("PS meas repeat rate 10ms : echo 10 > psmeasratesetup\n");
	printk("To read register PS_MEAS_RATE (0x84) : cat psmeasratesetup\n\n");
	// address 0x84

	// address 0x86
	printk("To read part ID : cat partid\n");
	printk("To read revision ID : cat revid\n");
	printk("To read PART_ID register (0x86) : cat partidreg\n\n");
	// address 0x86

	// address 0x87
	printk("To read manufacturing ID : cat manuid\n\n");
	// address 0x87

	// address 0x8C
	printk("Address 0x8C (ALS_PS_STATUS)\n");
	printk("To read PS data status : cat psdatastatus\n");
	printk("To read PS interrupt status : cat psinterruptstatus\n");
	printk("To read register ALS_PS_STATUS (0x8C) : cat alspsstatusreg\n\n");
	// address 0x8C

	// address 0x94, 0x95
	printk("To set PS offset (0 ~ 1023) : echo [decimal value] > setpsoffset\n");
	printk("Example...to write 55 : echo 55 > setpsoffset\n");
	printk("To read back the offset value : cat setpsoffset\n\n");
	// address 0x94, 0x95

	// address 0x8F
	printk("Address 0x8F (INTERRUPT)\n");
	printk("INT output pin inactive : echo 0 > interruptmodesetup\n");
	printk("Only PS triggers interrupt : echo 1 > interruptmodesetup\n");
	printk("Only ALS triggers interrupt : echo 2 > interruptmodesetup\n");
	printk("Both ALS PS trigger interrupt : echo 3 > interruptmodesetup\n");
	printk("To read interrupt mode : cat interruptmodesetup\n\n");

	printk("INT output pin active low : echo 0 > interruptpolarsetup\n");
	printk("INT output pin active high : echo 1 > interruptpolarsetup\n");
	printk("To read interrupt pin polarity : cat interruptpolarsetup\n\n");

	printk("Write value to INTERRUPT register (0x8F) : echo [hexcode value] > interruptsetup\n");
	printk("Example...to write 0x0B : echo B > interruptsetup or echo b > interruptsetup\n");
	printk("Example...to write 0x13 : echo 13 > interruptsetup\n");
	printk("To read register INTERRUPT (0x8F) : cat interruptsetup\n\n");
	// address 0x8F

	// address 0x9E
	printk("Address 0x9E (INTERRUPT PERSIST)\n");
	printk("Write value to INTERRUPT register (0x9E) : echo [hexcode value] > interruptpersistsetup\n");
	printk("Example...to write 0x0B : echo B > interruptpersistsetup or echo b > interruptpersistsetup\n");
	printk("Example...to write 0x13 : echo 13 > interruptpersistsetup\n");
	printk("To read register INTERRUPT PERSIST (0x9E) : cat interruptpersistsetup\n\n");
	// address 0x9E

	// PS threshold setting
	printk("PS threshold setting 0x90, 0x91, 0x92, 0x93\n");
	printk("To set PS lo threshold : echo [lo limit in decimal] > setpslothrerange\n");
	printk("Example...To set 20 to lo threshold : echo 20 > setpslothrerange\n");
	printk("To set PS hi threshold : echo [hi limit in decimal] > setpshithrerange\n");
	printk("Example...To set 999 to hi threshold : echo 999 > setpshithrerange\n");
	printk("To read the threshold values : cat disppsthrerange\n\n");
	// PS threshold setting
	
	return 0;
}

static DEVICE_ATTR(ltr659help, 0666, ltr659help_show, NULL);


static ssize_t psmodesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_contr_readback(PS_MODE_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psmodesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr659_data *ltr659 = psensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_mode_setup((uint8_t)param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;
	
}

static DEVICE_ATTR(psmodesetup, 0666, psmodesetup_show, psmodesetup_store);


static ssize_t psgainsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_contr_readback(PS_GAIN_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_GAIN_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psgainsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param;
	int8_t ret;
	int param_temp[2];

	struct ltr659_data *ltr659 = psensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 10) + param_temp[1]);
	
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_gain_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS gain setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psgainsetup, 0666, psgainsetup_show, psgainsetup_store);


static ssize_t pssatuindicasetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_contr_readback(PS_SATUR_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_SATUR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pssatuindicasetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr659_data *ltr659 = psensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_satu_indica_setup((uint8_t)param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS saturation indicator setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(pssatuindicasetup, 0666, pssatuindicasetup_show, pssatuindicasetup_store);


static ssize_t pscontrsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_contr_readback(PS_CONTR_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_CONTR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pscontrsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_contr_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS contr setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(pscontrsetup, 0666, pscontrsetup_show, pscontrsetup_store);


static ssize_t psledcurrsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_led_readback(LED_CURR_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: LED_CURR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];
	
	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <=1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 2) {		
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;

		param_temp[2] = param_temp[0];
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrent_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED current setup Fail...\n", __func__);
		return (-1);
	}

	return count;
	
}

static DEVICE_ATTR(psledcurrsetup, 0666, psledcurrsetup_show, psledcurrsetup_store);


static ssize_t psledcurrduty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_led_readback(LED_CURR_DUTY_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: LED_CURR_DUTY_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t psledcurrduty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];

	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrDuty_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED curent duty setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledcurrduty, 0666, psledcurrduty_show, psledcurrduty_store);


static ssize_t psledpulsefreqsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_led_readback(LED_PUL_FREQ_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: LED_PUL_FREQ_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}


static ssize_t psledpulsefreqsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[3];

	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseFreq_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED pulse frequency setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledpulsefreqsetup, 0666, psledpulsefreqsetup_show, psledpulsefreqsetup_store);


static ssize_t psledsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_led_readback(PS_LED_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_LED_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_led_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED setup Fail...\n", __func__);
		return (-1);
	}

	return count;	
}

static DEVICE_ATTR(psledsetup, 0666, psledsetup_show, psledsetup_store);


static ssize_t psledpulsecountsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_ledPulseCount_readback(&rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED pulse count readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledpulsecountsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];
	
	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if ((count <= 1) || (count > 3)){
		param_temp[0] = 0;
		param_temp[1] = 0;		
	} else if (count == 2) {		
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 10) + param_temp[1]);
	if (param > 15) {
		param = 15;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseCount_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psledpulsecountsetup, 0666, psledpulsecountsetup_show, psledpulsecountsetup_store);


static ssize_t psmeasratesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = ps_meas_rate_readback(&rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS meas rate readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psmeasratesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	//int *param_temp = buf;
	int param_temp[4];
	
	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {		
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_meas_rate_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS measurement rate setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(psmeasratesetup, 0666, psmeasratesetup_show, psmeasratesetup_store);

static ssize_t partid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;

	ret = part_ID_reg_readback(PART_NUM_ID_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PART_NUM_ID_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partid, 0666, partid_show, NULL);


static ssize_t revid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;

	ret = part_ID_reg_readback(REVISION_ID_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: REVISION_ID_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(revid, 0666, revid_show, NULL);


static ssize_t partidreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;

	ret = part_ID_reg_readback(PART_ID_REG_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PART_ID_REG_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(partidreg, 0666, partidreg_show, NULL);


static ssize_t manuid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;

	ret = manu_ID_reg_readback(&rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Manufacturing ID readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(manuid, 0666, manuid_show, NULL);


static ssize_t psdatastatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = als_ps_status_reg(PS_DATA_STATUS_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}

static DEVICE_ATTR(psdatastatus, 0666, psdatastatus_show, NULL);


static ssize_t psinterruptstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = als_ps_status_reg(PS_INTERR_STATUS_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static DEVICE_ATTR(psinterruptstatus, 0666, psinterruptstatus_show, NULL);

static ssize_t alspsstatusreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = als_ps_status_reg(ALS_PS_STATUS_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: ALS_PS_STATUS_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
	
}

static DEVICE_ATTR(alspsstatusreg, 0666, alspsstatusreg_show, NULL);



static ssize_t setpsoffset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val;
	struct ltr659_data *ltr659 = psensor_info;

	ret = ps_offset_readback(&rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS offset readback Fail...\n", __func__);
		return (-1);
	}
	
	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t setpsoffset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t ps_offset = 0;
	uint8_t param_temp[4];
	struct ltr659_data *ltr659 = psensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	ps_offset = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (ps_offset > 1023) {
		ps_offset = 1023;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, ps_offset);

	ret = ps_offset_setup(ps_offset, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: set ps offset Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpsoffset, 0666, setpsoffset_show, setpsoffset_store);


static ssize_t interruptmodesetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = interrupt_readback(INT_MODE_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: INT_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptmodesetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr659_data *ltr659 = psensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_mode_setup((uint8_t)param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: interrupt mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptmodesetup, 0666, interruptmodesetup_show, interruptmodesetup_store);


static ssize_t interruptpolarsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = interrupt_readback(INT_POLAR_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: INT_POLAR_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}


static ssize_t interruptpolarsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr659_data *ltr659 = psensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_polarity_setup((uint8_t)param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: interrupt polarity setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptpolarsetup, 0666, interruptpolarsetup_show, interruptpolarsetup_store);


static ssize_t interruptsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = interrupt_readback(INT_INTERRUPT_RDBCK, &rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: INT_INTERRUPT_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;	
}


static ssize_t interruptsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	//int *param_temp = buf;
	int param_temp[2];

	struct ltr659_data *ltr659 = psensor_info;

	//sscanf(buf, "%d", param_temp);
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = interrupt_setup(param, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: interrupt setup Fail...\n", __func__);
		return (-1);
	}

	return count;
}

static DEVICE_ATTR(interruptsetup, 0666, interruptsetup_show, interruptsetup_store);


static ssize_t interruptpersistsetup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr659_data *ltr659 = psensor_info;	

	ret = interrupt_prst_readback(&rdback_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Interrupt persist readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpersistsetup_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret = 0;
	//uint8_t als_or_ps, prst_val;
	uint8_t prst_val;
	//int *param_temp = buf;
	int param_temp[2];

	struct ltr659_data *ltr659 = psensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70) {
		param_temp[0] -= 55;
	} else if (param_temp[0] >= 97 && param_temp[0] <= 102) {
		param_temp[0] -= 87;
	} else if (param_temp[0] >= 48 && param_temp[0] <= 57) {
		param_temp[0] -= 48;
	} else {
		param_temp[0] = 0;
	}

	if (param_temp[1] >= 65 && param_temp[1] <= 70) {
		param_temp[1] -= 55;
	} else if (param_temp[1] >= 97 && param_temp[1] <= 102) {
		param_temp[1] -= 87;
	} else if (param_temp[1] >= 48 && param_temp[1] <= 57) {
		param_temp[1] -= 48;
	} else {
		param_temp[1] = 0;
	}

	prst_val = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, prst_val);

	//ret = interrupt_persist_setup(als_or_ps, prst_val, ltr659);
	ret = interrupt_persist_setup(prst_val, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Interrupt persist setup Fail...\n", __func__);
		return (-1);
	}

	return count;
		
}

static DEVICE_ATTR(interruptpersistsetup, 0666, interruptpersistsetup_show, interruptpersistsetup_store);



static ssize_t setpslothrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t lo_thr = 0;
	uint8_t param_temp[4];
	struct ltr659_data *ltr659 = psensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	lo_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (lo_thr > 2047) {
		lo_thr = 2047;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, lo_thr);

	ret = set_ps_range(lo_thr, 0, LO_LIMIT, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: set PS lo threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpslothrerange, 0666, NULL, setpslothrerange_store);


static ssize_t setpshithrerange_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t hi_thr = 0;
	uint8_t param_temp[4];
	struct ltr659_data *ltr659 = psensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { // 1 digit
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { // 2 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { // 3 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { // 4 digits
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;		
	}

	//for (i_ctr = 0; i_ctr < sizeof(param_temp); i_ctr++) {
	//	if ((param_temp[i_ctr] < 0) ||(param_temp[i_ctr] > 9)) {
	//		param_temp[i_ctr] = 0;
	//	}
	//}

	hi_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) + (param_temp[2] * 10) + param_temp[3]);
	if (hi_thr > 2047) {
		hi_thr = 2047;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s: store value = %d\n", __func__, hi_thr);

	ret = set_ps_range(0, hi_thr, HI_LIMIT, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: set PS hi threshold Fail...\n", __func__);
		return (-1);
	}
	
	return count;
}

static DEVICE_ATTR(setpshithrerange, 0666, NULL, setpshithrerange_store);


static ssize_t disppsthrerange_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_lo, rdback_hi;
	struct ltr659_data *ltr659 = psensor_info;

	ret = ps_range_readback(&rdback_lo, &rdback_hi, ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS threshold range readback Fail...\n", __func__);
		return (-1);
	}
	
	ret = sprintf(buf, "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

static DEVICE_ATTR(disppsthrerange, 0666, disppsthrerange_show, NULL);


static void sysfs_register_device(struct i2c_client *client) 
{
	int rc = 0;

	rc += device_create_file(&client->dev, &dev_attr_ps_enable);
	rc += device_create_file(&client->dev, &dev_attr_ps_adc);
	rc += device_create_file(&client->dev, &dev_attr_psadcsaturationBit);
	rc += device_create_file(&client->dev, &dev_attr_ltr659help);
	rc += device_create_file(&client->dev, &dev_attr_psmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_psgainsetup);
	rc += device_create_file(&client->dev, &dev_attr_pssatuindicasetup);	
	rc += device_create_file(&client->dev, &dev_attr_pscontrsetup);	
	rc += device_create_file(&client->dev, &dev_attr_psledcurrsetup);	
	rc += device_create_file(&client->dev, &dev_attr_psledcurrduty);	
	rc += device_create_file(&client->dev, &dev_attr_psledpulsefreqsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledsetup);
	rc += device_create_file(&client->dev, &dev_attr_psledpulsecountsetup);
	rc += device_create_file(&client->dev, &dev_attr_psmeasratesetup);
	rc += device_create_file(&client->dev, &dev_attr_partid);
	rc += device_create_file(&client->dev, &dev_attr_revid);
	rc += device_create_file(&client->dev, &dev_attr_partidreg);
	rc += device_create_file(&client->dev, &dev_attr_manuid);	
	rc += device_create_file(&client->dev, &dev_attr_psdatastatus);
	rc += device_create_file(&client->dev, &dev_attr_psinterruptstatus);
	rc += device_create_file(&client->dev, &dev_attr_setpsoffset);	
	rc += device_create_file(&client->dev, &dev_attr_interruptmodesetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpolarsetup);	
	rc += device_create_file(&client->dev, &dev_attr_interruptsetup);
	rc += device_create_file(&client->dev, &dev_attr_interruptpersistsetup);	
	rc += device_create_file(&client->dev, &dev_attr_setpslothrerange);
	rc += device_create_file(&client->dev, &dev_attr_setpshithrerange);
	rc += device_create_file(&client->dev, &dev_attr_disppsthrerange);
	
	if (rc) {
		dev_err(&client->dev, "%s Unable to create sysfs files\n", __func__);
	} else {
		dev_dbg(&client->dev, "%s Created sysfs files\n", __func__);
	}
}


static int ps_setup(struct ltr659_data *ltr659)
{
	int ret;

	ltr659->ps_input_dev = input_allocate_device();
	if (!ltr659->ps_input_dev) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr659->ps_input_dev->name = "ltr659_ps";
	set_bit(EV_ABS, ltr659->ps_input_dev->evbit);
	input_set_abs_params(ltr659->ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr659->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(ltr659->ps_input_dev);
err_ps_register_input_device:
	input_free_device(ltr659->ps_input_dev);

	return ret;
}


static uint8_t _check_part_id(struct ltr659_data *ltr659)
{
	uint8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTR659_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Read failure :0x%02X",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&ltr659->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}

	return 0;
}


static int ltr659_setup(struct ltr659_data *ltr659)
{
	int ret = 0;

	/* Reset the devices */
	ret = _ltr659_set_bit(ltr659->i2c_client, CLR_BIT, LTR659_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltr659->i2c_client->dev, "%s: Reset ltr659 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr659) < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	//(Linux RTOS)>
#if 1
	ret = ltr659_gpio_irq(ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s Requested interrupt\n", __func__);
#endif
	//(Linux RTOS)<

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr659_set_bit(ltr659->i2c_client, SET_BIT, LTR659_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _ltr659_set_bit(ltr659->i2c_client, SET_BIT, LTR659_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev,"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s: Set ltr659 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	ret = _ltr659_set_bit(ltr659->i2c_client, SET_BIT, LTR659_INTERRUPT, INT_MODE_PS_TRIG);
#else
	ret = _ltr659_set_bit(ltr659->i2c_client, SET_BIT, LTR659_INTERRUPT, INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr659->i2c_client->dev, "%s Enabled interrupt to device\n", __func__);

#if 0  
	/* Turn on PS */

	ret = ps_enable_init(ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s Unable to enable PS", __func__);
		goto err_out2;
	}
	dev_info(&ltr659->i2c_client->dev, "%s Turned on proximity sensor\n", __func__);
#else
	ltr659->ps_enable_flag = 0;
#endif 

	return ret;

err_out2:
	free_irq(ltr659->irq, ltr659);
	gpio_free(ltr659->gpio_int_no);

err_out1:
	dev_err(&ltr659->i2c_client->dev, "%s Unable to setup device\n", __func__);

	return ret;
}


static int ltr659_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr659_data *ltr659;
//(Linux RTOS)>
#if 1
	struct ltr659_platform_data *platdata;
#endif
//(Linux RTOS)<

	ltr659 = kzalloc(sizeof(struct ltr659_data), GFP_KERNEL);
	if (!ltr659)
	{
		dev_err(&ltr659->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	psensor_info = ltr659;

	/* Set initial defaults */
	ltr659->ps_enable_flag = 0;

	ltr659->i2c_client = client;
	ltr659->irq = client->irq;

	i2c_set_clientdata(client, ltr659);

	/* Parse the platform data */
	//(Linux RTOS)>
#if 1
	platdata = client->dev.platform_data;
	if (!platdata) {
		dev_err(&ltr659->i2c_client->dev, "%s: Platform Data assign Fail...\n", __func__);
		ret = -EBUSY;
		goto err_out;
	}

	ltr659->gpio_int_no = platdata->pfd_gpio_int_no;
	ltr659->default_ps_lowthresh = platdata->pfd_ps_lowthresh;
	ltr659->default_ps_highthresh = platdata->pfd_ps_highthresh;

#endif
	//(Linux RTOS)<

	if (_check_part_id(ltr659) < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}


	/* Setup the input subsystem for the PS */
	ret = ps_setup(ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Create the workqueue for the interrup handler */
	//(Linux RTOS)>
#if 1
	ltr659->workqueue = create_singlethread_workqueue("ltr659_wq");
	if (!ltr659->workqueue) {
		dev_err(&ltr659->i2c_client->dev, "%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(ltr659->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#endif
	//(Linux RTOS)<

	/* Setup and configure both the ALS and PS on the ltr659 device */
	ret = ltr659_setup(ltr659);
	if (ret < 0) {
		dev_err(&ltr659->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_ltr659_setup;
	}

	/* Setup the suspend and resume functionality */
	//(Linux RTOS)>
#if 0
	ltr659->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ltr659->early_suspend.suspend = ltr659_early_suspend;
	ltr659->early_suspend.resume = ltr659_late_resume;
	register_early_suspend(&ltr659->early_suspend);
#endif
	//(Linux RTOS)<

	/* Register the sysfs files */
	sysfs_register_device(client);
	//sysfs_register_als_device(client, &ltr659->als_input_dev->dev);
	//sysfs_register_ps_device(client, &ltr659->ps_input_dev->dev);

	dev_dbg(&ltr659->i2c_client->dev, "%s: probe complete\n", __func__);

	return ret;

err_ltr659_setup:
	destroy_workqueue(ltr659->workqueue);
err_out:
	kfree(ltr659);

	return ret;
}


static const struct i2c_device_id ltr659_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

static struct i2c_driver ltr659_driver = {
	.probe = ltr659_probe,
	.id_table = ltr659_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};


static int __init ltr659_init(void)
{
	return i2c_add_driver(&ltr659_driver);
}

static void __exit ltr659_exit(void)
{
	i2c_del_driver(&ltr659_driver);
}


module_init(ltr659_init)
module_exit(ltr659_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-659ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);


