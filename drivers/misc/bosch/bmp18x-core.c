/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * Copyright (c) 2011  Unixphere
 * Copyright (c) 2010  Christoph Mair <christoph.mair@gmail.com>
 *
 * @filename bmp18x-core.c
 * @date     "Tue Jan 7 16:02:57 2014 +0800"
 * @id       "fcff9b1"
 * @version  1.3.1
 *
 * @brief
 * The core code of BMP18X device driver
 *
 * @detail
 * This file implements the core code of BMP18X device driver,
 * which includes hardware related functions, input device register,
 * device attribute files, etc.
 * Based on: BMP085 driver, bmp085.c
 * This driver supports the bmp18x digital barometric pressure
 * and temperature sensors from Bosch Sensortec.
*/

#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "bmp18x.h"

#define BMP18X_CHIP_ID                  0x55
#define CHECK_CHIP_ID_TIME_MAX          5

#define BMP18X_CALIBRATION_DATA_START   0xAA
#define BMP18X_CALIBRATION_DATA_LENGTH  11/* 16 bit values */
#define BMP18X_CHIP_ID_REG              0xD0
#define BMP18X_CTRL_REG                 0xF4
#define BMP18X_TEMP_MEASUREMENT         0x2E
#define BMP18X_PRESSURE_MEASUREMENT     0x34
#define BMP18X_CONVERSION_REGISTER_MSB  0xF6
#define BMP18X_CONVERSION_REGISTER_LSB  0xF7
#define BMP18X_CONVERSION_REGISTER_XLSB 0xF8
#define BMP18X_CRC_REG_START            0x80
#define BMP18X_TEMP_CONVERSION_TIME     5

#define ABS_MIN_PRESSURE                30000
#define ABS_MAX_PRESSURE                120000
#define BMP_DELAY_DEFAULT               200

struct bmp18x_calibration_data {
	s16 AC1, AC2, AC3;
	u16 AC4, AC5, AC6;
	s16 B1, B2;
	s16 MB, MC, MD;
};

/* Each client has this additional data */
struct bmp18x_data {
	struct  bmp18x_data_bus data_bus;
	struct  device *dev;
	struct  mutex lock;
	struct  bmp18x_calibration_data calibration;
	u8  oversampling_setting;
	u8  sw_oversampling_setting;
	u32 raw_temperature;
	u32 raw_pressure;
	u32 temp_measurement_period;
	u32 last_temp_measurement;
	s32 b6; /* calculated temperature correction coefficient */
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct input_dev    *input;
	struct delayed_work work;
	u32                 delay;
	u32                 enable;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmp18x_early_suspend(struct early_suspend *h);
static void bmp18x_late_resume(struct early_suspend *h);
#endif

static s32 bmp18x_read_calibration_data(struct bmp18x_data *data)
{
	u16 tmp[BMP18X_CALIBRATION_DATA_LENGTH];
	struct bmp18x_calibration_data *cali = &(data->calibration);
	s32 status = data->data_bus.bops->read_block(data->data_bus.client,
				BMP18X_CALIBRATION_DATA_START,
				BMP18X_CALIBRATION_DATA_LENGTH*sizeof(u16),
				(u8 *)tmp);
	if (status < 0)
		return status;

	if (status != BMP18X_CALIBRATION_DATA_LENGTH*sizeof(u16))
		return -EIO;

	cali->AC1 =  be16_to_cpu(tmp[0]);
	cali->AC2 =  be16_to_cpu(tmp[1]);
	cali->AC3 =  be16_to_cpu(tmp[2]);
	cali->AC4 =  be16_to_cpu(tmp[3]);
	cali->AC5 =  be16_to_cpu(tmp[4]);
	cali->AC6 = be16_to_cpu(tmp[5]);
	cali->B1 = be16_to_cpu(tmp[6]);
	cali->B2 = be16_to_cpu(tmp[7]);
	cali->MB = be16_to_cpu(tmp[8]);
	cali->MC = be16_to_cpu(tmp[9]);
	cali->MD = be16_to_cpu(tmp[10]);
	return 0;
}


static s32 bmp18x_update_raw_temperature(struct bmp18x_data *data)
{
	u16 tmp;
	s32 status;

	mutex_lock(&data->lock);
	status = data->data_bus.bops->write_byte(data->data_bus.client,
				BMP18X_CTRL_REG, BMP18X_TEMP_MEASUREMENT);
	if (status != 0) {
		dev_err(data->dev,
			"Error while requesting temperature measurement.\n");
		goto exit;
	}
	msleep(BMP18X_TEMP_CONVERSION_TIME);

	status = data->data_bus.bops->read_block(data->data_bus.client,
		BMP18X_CONVERSION_REGISTER_MSB, sizeof(tmp), (u8 *)&tmp);
	if (status < 0)
		goto exit;
	if (status != sizeof(tmp)) {
		dev_err(data->dev,
			"Error while reading temperature measurement result\n");
		status = -EIO;
		goto exit;
	}
	data->raw_temperature = be16_to_cpu(tmp);
	data->last_temp_measurement = jiffies;
	status = 0;	/* everything ok, return 0 */

exit:
	mutex_unlock(&data->lock);
	return status;
}

static s32 bmp18x_update_raw_pressure(struct bmp18x_data *data)
{
	u32 tmp = 0;
	s32 status;

	mutex_lock(&data->lock);
	status = data->data_bus.bops->write_byte(data->data_bus.client,
		BMP18X_CTRL_REG, BMP18X_PRESSURE_MEASUREMENT +
		(data->oversampling_setting<<6));
	if (status != 0) {
		dev_err(data->dev,
			"Error while requesting pressure measurement.\n");
		goto exit;
	}

	/* wait for the end of conversion */
	msleep(2+(3 << data->oversampling_setting));

	/* copy data into a u32 (4 bytes), but skip the first byte. */
	status = data->data_bus.bops->read_block(data->data_bus.client,
			BMP18X_CONVERSION_REGISTER_MSB, 3, ((u8 *)&tmp)+1);
	if (status < 0)
		goto exit;
	if (status != 3) {
		dev_err(data->dev,
			"Error while reading pressure measurement results\n");
		status = -EIO;
		goto exit;
	}
	data->raw_pressure = be32_to_cpu((tmp));
	data->raw_pressure >>= (8-data->oversampling_setting);
	status = 0;	/* everything ok, return 0 */

exit:
	mutex_unlock(&data->lock);
	return status;
}


/*
 * This function starts the temperature measurement and returns the value
 * in tenth of a degree celsius.
 */
static s32 bmp18x_get_temperature(struct bmp18x_data *data, int *temperature)
{
	struct bmp18x_calibration_data *cali = &data->calibration;
	long x1, x2;
	int status;

	status = bmp18x_update_raw_temperature(data);
	if (status != 0)
		goto exit;

	x1 = ((data->raw_temperature - cali->AC6) * cali->AC5) >> 15;
	x2 = (cali->MC << 11) / (x1 + cali->MD);
	data->b6 = x1 + x2 - 4000;
	/* if NULL just update b6. Used for pressure only measurements */
	if (temperature != NULL)
		*temperature = (x1+x2+8) >> 4;

exit:
	return status;
}

/*
 * This function starts the pressure measurement and returns the value
 * in millibar. Since the pressure depends on the ambient temperature,
 * a temperature measurement is executed according to the given temperature
 * measurememt period (default is 1 sec boundary). This period could vary
 * and needs to be adjusted accoring to the sensor environment, i.e. if big
 * temperature variations then the temperature needs to be read out often.
 */
static s32 bmp18x_get_pressure(struct bmp18x_data *data, int *pressure)
{
	struct bmp18x_calibration_data *cali = &data->calibration;
	s32 x1, x2, x3, b3;
	u32 b4, b7;
	s32 p;
	int status;
	int i_loop, i;
	u32 p_tmp;

	/* update the ambient temperature according to the given meas. period */
	if (data->last_temp_measurement +
			data->temp_measurement_period < jiffies) {
		status = bmp18x_get_temperature(data, NULL);
		if (status != 0)
			goto exit;
	}

	if ((data->oversampling_setting == 3)
		&& (data->sw_oversampling_setting == 1)) {
		i_loop = 3;
	} else {
		i_loop = 1;
	}

	p_tmp = 0;
	for (i = 0; i < i_loop; i++) {
		status = bmp18x_update_raw_pressure(data);
		if (status != 0)
			goto exit;
		p_tmp += data->raw_pressure;
	}

	data->raw_pressure = (p_tmp + (i_loop >> 1)) / i_loop;

	x1 = (data->b6 * data->b6) >> 12;
	x1 *= cali->B2;
	x1 >>= 11;

	x2 = cali->AC2 * data->b6;
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((s32)cali->AC1) * 4 + x3) << data->oversampling_setting) + 2);
	b3 >>= 2;

	x1 = (cali->AC3 * data->b6) >> 13;
	x2 = (cali->B1 * ((data->b6 * data->b6) >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	b4 = (cali->AC4 * (u32)(x3 + 32768)) >> 15;

	b7 = ((u32)data->raw_pressure - b3) *
					(50000 >> data->oversampling_setting);
	p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) * 2));

	x1 = p >> 8;
	x1 *= x1;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	*pressure = p;

exit:
	return status;
}

/*
 * This function sets the chip-internal oversampling. Valid values are 0..3.
 * The chip will use 2^oversampling samples for internal averaging.
 * This influences the measurement time and the accuracy; larger values
 * increase both. The datasheet gives on overview on how measurement time,
 * accuracy and noise correlate.
 */
static void bmp18x_set_oversampling(struct bmp18x_data *data,
						unsigned char oversampling)
{
	if (oversampling > 3)
		oversampling = 3;
	data->oversampling_setting = oversampling;
}

/*
 * Returns the currently selected oversampling. Range: 0..3
 */
static unsigned char bmp18x_get_oversampling(struct bmp18x_data *data)
{
	return data->oversampling_setting;
}

static int bmp18x_check_calib_param(struct bmp18x_data *data)
{
	struct bmp18x_calibration_data *cali = &(data->calibration);

	/* check that not all calibration parameters are 0 */
	if (cali->AC1 == 0 && cali->AC2 == 0 && cali->AC3 == 0
		&& cali->AC4 == 0 && cali->AC5 == 0 && cali->AC6 == 0) {
		dev_err(data->dev, "all calibration parameters are zero\n");
		return 1;
	}

	/*
	* check whether all the calibration parameters are
	* within 4 sigma range
	*/
	if (cali->AC1 <= -29250 || cali->AC1 >= 32750)
		return 2;
	else if (cali->AC2 <= -4820 || cali->AC2 >= 3626)
		return 3;
	else if (cali->AC3 <= -16660 || cali->AC3 >= -13151)
		return 4;
	else if (cali->AC4 <= 20780 || cali->AC4 >= 44820)
		return 5;
	else if (cali->AC5 <= 21730 || cali->AC5 >= 34475)
		return 6;
	else if (cali->AC6 <= 500 || cali->AC6 >= 56200)
		return 7;
	else if (cali->B1 <= 4935 || cali->B1 >= 8637)
		return 8;
	else if (cali->B2 < -182 || cali->B2 >= 297)
		return 9;

	dev_info(data->dev, "calibration parameters are OK\n");
	return 0;
}

static int bmp18x_check_pt(struct bmp18x_data *data)
{
	int temperature;
	int pressure;

	/* check ut and t */
	bmp18x_get_temperature(data, &temperature);
	if (data->raw_temperature == 0 || data->raw_temperature < 16000
		|| data->raw_temperature == 65535) {
		dev_err(data->dev, "ut is out of range:%d\n",
			data->raw_temperature);
		return 10;
	}
	if (temperature < 0 || temperature >= 40*10) {
		dev_err(data->dev, "temperature value is out of range:%d*0.01degree\n",
			temperature);
		return 11;
	}

	/* check up and p */
	bmp18x_get_pressure(data, &pressure);
	if (data->raw_pressure == 0 || data->raw_pressure < 15000
		|| data->raw_pressure == 65535) {
		dev_err(data->dev, "up is out of range:%d\n",
			data->raw_pressure);
		return 12;
	}
	if (pressure < 900*100 || pressure > 1100*100) {
		dev_err(data->dev, "pressure value is out of range:%d Pa\n",
			pressure);
		return 13;
	}

	dev_info(data->dev, "bmp18x temperature and pressure values are OK\n");
	return 0;
}

static unsigned char bmp18x_st_calc_crc(unsigned char seed, unsigned char data)
{
	unsigned char poly = 0x1D;
	unsigned char bit, din;

	for (bit = 0; bit < 8; bit++) {
		if (((seed & 0x80) > 0) ^ ((data & 0x80) > 0))
			din = 1;
		else
			din = 0;
		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (poly * din);
	}

	return seed;
}

static int bmp18x_check_crc(struct bmp18x_data *data)
{
	unsigned char i;
	unsigned char current_register;
	unsigned char crc_val;
	unsigned char registers[32];
	s32 status;

	/* read all relevant registers for CRC calculation */
	status = data->data_bus.bops->read_block(data->data_bus.client,
			BMP18X_CRC_REG_START,
			32,
			registers);
	if (status < 0)
		return 14;

	crc_val = 0xFF;

	for (i = 0; i < 32; i++) {
		current_register = registers[i];
		if (i != 4) /* that is ee_crc register */
			crc_val = bmp18x_st_calc_crc(crc_val, current_register);
	}

	crc_val = (crc_val ^ 0xFF);
	if (crc_val == registers[4])/* calculated crc is correct */
		return 0;
	else/* data or crc is wrong */
		return 15;
}

static int bmp18x_do_selftest(struct bmp18x_data *data)
{
	int err = 0;
	/* 0: failed, 1: success */
	u8 selftest;

	err = bmp18x_check_calib_param(data);
	if (err) {
		selftest = 0;
		dev_err(data->dev, "bmp18x_check_calib_param:err=%d\n", err);
		goto exit;
	}

	err = bmp18x_check_pt(data);
	if (err) {
		selftest = 0;
		dev_err(data->dev, "bmp18x_check_pt:err=%d\n", err);
		goto exit;
	}

	err = bmp18x_check_crc(data);
	if (err) {
		selftest = 0;
		dev_err(data->dev, "bmp18x_check_crc:err=%d\n", err);
		goto exit;
	}

	/* selftest is OK */
	selftest = 1;
	dev_info(data->dev, "bmp18x self test is OK\n");
exit:
	return selftest;
}

/* sysfs callbacks */
static ssize_t set_oversampling(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	unsigned long oversampling;
	int success = kstrtoul(buf, 10, &oversampling);
	if (success == 0) {
		mutex_lock(&data->lock);
		bmp18x_set_oversampling(data, oversampling);
		if (oversampling != 3)
			data->sw_oversampling_setting = 0;
		mutex_unlock(&data->lock);
		return count;
	}
	return success;
}

static ssize_t show_oversampling(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", bmp18x_get_oversampling(data));
}
static DEVICE_ATTR(oversampling, S_IWUSR | S_IRUGO,
					show_oversampling, set_oversampling);

static ssize_t set_sw_oversampling(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	unsigned long sw_oversampling;
	int success = kstrtoul(buf, 10, &sw_oversampling);
	if (success == 0) {
		mutex_lock(&data->lock);
		data->sw_oversampling_setting = sw_oversampling ? 1 : 0;
		mutex_unlock(&data->lock);
		return count;
	}
	return success;
}

static ssize_t show_sw_oversampling(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", data->sw_oversampling_setting);
}
static DEVICE_ATTR(sw_oversampling, S_IWUSR | S_IRUGO,
				show_sw_oversampling, set_sw_oversampling);

static ssize_t show_delay(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", data->delay);
}

static ssize_t set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	unsigned long delay;
	int success = kstrtoul(buf, 10, &delay);
	if (success == 0) {
		mutex_lock(&data->lock);
		data->delay = delay;
		mutex_unlock(&data->lock);
		return count;
	}
	return success;
}
static DEVICE_ATTR(delay, S_IWUSR | S_IRUGO,
				show_delay, set_delay);

static ssize_t show_enable(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", data->enable);
}

static ssize_t set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
	unsigned long enable;
	int success = kstrtoul(buf, 10, &enable);
	if (success == 0) {
		mutex_lock(&data->lock);
		data->enable = enable ? 1 : 0;

		if (data->enable) {
			bmp18x_enable(dev);
			schedule_delayed_work(&data->work,
						msecs_to_jiffies(data->delay));
		} else {
			cancel_delayed_work_sync(&data->work);
			bmp18x_disable(dev);
		}
		mutex_unlock(&data->lock);
		return count;
	}
	return success;
}
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
				show_enable, set_enable);

static ssize_t show_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int temperature;
	int status;
	struct bmp18x_data *data = dev_get_drvdata(dev);

	status = bmp18x_get_temperature(data, &temperature);
	if (status != 0)
		return status;
	else
		return sprintf(buf, "%d\n", temperature);
}
static DEVICE_ATTR(temp0_input, S_IRUGO, show_temperature, NULL);


static ssize_t show_pressure(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int pressure;
	int status;
	struct bmp18x_data *data = dev_get_drvdata(dev);

	status = bmp18x_get_pressure(data, &pressure);
	if (status != 0)
		return status;
	else
		return sprintf(buf, "%d\n", pressure);
}
static DEVICE_ATTR(pressure0_input, S_IRUGO, show_pressure, NULL);

static ssize_t show_selftest(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", bmp18x_do_selftest(data));
}

static DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest, NULL);

static struct attribute *bmp18x_attributes[] = {
	&dev_attr_temp0_input.attr,
	&dev_attr_pressure0_input.attr,
	&dev_attr_oversampling.attr,
	&dev_attr_sw_oversampling.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_selftest.attr,
	NULL
};

static const struct attribute_group bmp18x_attr_group = {
	.attrs = bmp18x_attributes,
};

static void bmp18x_work_func(struct work_struct *work)
{
	struct bmp18x_data *client_data =
		container_of((struct delayed_work *)work,
		struct bmp18x_data, work);
	unsigned long delay = msecs_to_jiffies(client_data->delay);
	unsigned long j1 = jiffies;
	int pressure;
	int status;

	status = bmp18x_get_pressure(client_data, &pressure);

	if (status == 0) {
		input_event(client_data->input, EV_MSC, MSC_RAW, pressure);
		input_sync(client_data->input);
	}

	schedule_delayed_work(&client_data->work, delay-(jiffies-j1));
}

static int bmp18x_input_init(struct bmp18x_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = BMP18X_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input = dev;

	return 0;
}

static void bmp18x_input_delete(struct bmp18x_data *data)
{
	struct input_dev *dev = data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmp18x_init_client(struct bmp18x_data *data,
			      struct bmp18x_platform_data *pdata)
{
	int status = bmp18x_read_calibration_data(data);
	if (status != 0)
		goto exit;
	data->last_temp_measurement = 0;
	data->temp_measurement_period =
		pdata ? (pdata->temp_measurement_period/1000)*HZ : 1*HZ;
	data->oversampling_setting = pdata ? pdata->default_oversampling : 3;
	if (data->oversampling_setting == 3)
		data->sw_oversampling_setting
			= pdata ? pdata->default_sw_oversampling : 0;
	mutex_init(&data->lock);
exit:
	return status;
}

__devinit int bmp18x_probe(struct device *dev, struct bmp18x_data_bus *data_bus)
{
	struct bmp18x_data *data;
	struct bmp18x_platform_data *pdata = dev->platform_data;
	u8 chip_id = pdata && pdata->chip_id ? pdata->chip_id : BMP18X_CHIP_ID;
	u8 read_count = 0;
	int err = 0;

	if (pdata && pdata->init_hw) {
		err = pdata->init_hw();
		if (err) {
			printk(KERN_ERR "%s: init_hw failed!\n",
				BMP18X_NAME);
			goto exit;
		}
	}

	/*read chip id*/
	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		if (data_bus->bops->read_byte(data_bus->client,
					BMP18X_CHIP_ID_REG) == chip_id) {
			break;
		}
		msleep(1);
	}
	if (read_count > CHECK_CHIP_ID_TIME_MAX) {
		printk(KERN_ERR "%s: chip_id failed!\n", BMP18X_NAME);
		err = -ENODEV;
		goto exit;
	}

	data = kzalloc(sizeof(struct bmp18x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata(dev, data);
	data->data_bus = *data_bus;
	data->dev = dev;

	/* Initialize the BMP18X chip */
	err = bmp18x_init_client(data, pdata);
	if (err != 0)
		goto exit_free;

	/* Initialize the BMP18X input device */
	err = bmp18x_input_init(data);
	if (err != 0)
		goto exit_free;

	/* Register sysfs hooks */
	err = sysfs_create_group(&data->input->dev.kobj, &bmp18x_attr_group);
	if (err)
		goto error_sysfs;
	/* workqueue init */
	INIT_DELAYED_WORK(&data->work, bmp18x_work_func);
	data->delay  = BMP_DELAY_DEFAULT;
	data->enable = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bmp18x_early_suspend;
	data->early_suspend.resume = bmp18x_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	dev_info(dev, "Succesfully initialized bmp18x!\n");
	return 0;

error_sysfs:
	bmp18x_input_delete(data);
exit_free:
	kfree(data);
exit:
	if (pdata && pdata->deinit_hw)
		pdata->deinit_hw();
	return err;
}
EXPORT_SYMBOL(bmp18x_probe);

int bmp18x_remove(struct device *dev)
{
	struct bmp18x_data *data = dev_get_drvdata(dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &bmp18x_attr_group);
	kfree(data);

	return 0;
}
EXPORT_SYMBOL(bmp18x_remove);

#ifdef CONFIG_PM
int bmp18x_disable(struct device *dev)
{
	struct bmp18x_platform_data *pdata = dev->platform_data;

	if (pdata && pdata->deinit_hw)
		pdata->deinit_hw();

	return 0;
}
EXPORT_SYMBOL(bmp18x_disable);

int bmp18x_enable(struct device *dev)
{
	struct bmp18x_platform_data *pdata = dev->platform_data;

	if (pdata && pdata->init_hw)
		return pdata->init_hw();

	return 0;
}
EXPORT_SYMBOL(bmp18x_enable);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmp18x_early_suspend(struct early_suspend *h)
{
	struct bmp18x_data *data =
		container_of(h, struct bmp18x_data, early_suspend);
	if (data->enable) {
		cancel_delayed_work_sync(&data->work);
		(void) bmp18x_disable(data->dev);
	}
}

static void bmp18x_late_resume(struct early_suspend *h)
{
	struct bmp18x_data *data =
		container_of(h, struct bmp18x_data, early_suspend);

	if (data->enable) {
		(void) bmp18x_enable(data->dev);
		schedule_delayed_work(&data->work,
					msecs_to_jiffies(data->delay));
	}

}
#endif

MODULE_AUTHOR("contact@bosch-sensortec.com");
MODULE_DESCRIPTION("BMP18X PRESSURE SENSOR DRIVER");
MODULE_LICENSE("GPL v2");
