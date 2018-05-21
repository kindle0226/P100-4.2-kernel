/*
 * Driver for keys on TCA6416A I2C IO expander
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author : Sriramakrishnan.A.G. <srk@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/tca6416a.h>




static const struct i2c_device_id tca6416a_id[] = {
	{ "tca6416a", 0, },
 	{ }
};
MODULE_DEVICE_TABLE(i2c, tca6416a_id);


struct tca6416a_chip {
	uint8_t reg_output;
	uint8_t reg_direction;
	uint8_t reg_input;

	struct i2c_client *client;
	struct delayed_work dwork;
};

static struct tca6416a_chip *tc6416a_chip;
static int tca6416a_write_reg(struct tca6416a_chip *chip, uint8_t reg, uint8_t val)
{
	int error;

	error =   i2c_smbus_write_byte_data(chip->client, reg, val);
	if (error < 0) {
		dev_err(&chip->client->dev,
			"%s failed, reg: %d, val: %d, error: %d\n",
			__func__, reg, val, error);
		return error;
	}
	
	return 0;
}

static int tca6416a_read_reg(struct tca6416a_chip *chip, uint8_t reg, uint8_t *val)
{
	int retval;

	retval =  i2c_smbus_read_byte_data(chip->client, reg);
	if (retval < 0) {
		dev_err(&chip->client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, reg, retval);
		return retval;
	}
	printk("chengqm:%s:%d-val=0x%x\n", __FUNCTION__,__LINE__, retval);
	*val = (u8)retval;
	return 0;
}

static int tca6416a_gpio_cmd_read(uint8_t cmd, uint8_t *val)
{
	if(tc6416a_chip != NULL)
		return tca6416a_read_reg(tc6416a_chip, cmd, val);
	else
		return 1;
}

static int tca6416a_gpio_cmd_write(uint8_t cmd, uint8_t val)
{
	if(tc6416a_chip != NULL)
		return tca6416a_write_reg(tc6416a_chip, cmd, val);
	else
		return 1;
}
int tca6416a_gpio_cmd_write_bit(uint8_t cmd,uint8_t bit, uint8_t val)
{
	int retval;
	uint8_t reg;

	if(tc6416a_chip != NULL)
	{
		 tca6416a_read_reg(tc6416a_chip, cmd, reg);
		 if(val)
		 	reg |= ( 1<<bit );
		 else
		 	reg &= ~( 1<<bit);
		return tca6416a_write_reg(tc6416a_chip, cmd, reg);
	}	
	else
		return 1;
}


static ssize_t show_input_port0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_INPUT_PORT0, &val);
	return sprintf(buf, "0x%x\n", val);
}

static ssize_t show_input_port1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_INPUT_PORT1, &val);
	return sprintf(buf, "0x%x\n", val);
}


static ssize_t show_output_port0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_OUTPUT_PORT0, &val);
	return sprintf(buf, "0x%x\n", val);
}

static ssize_t set_output_port0(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_OUTPUT_PORT0, val);     }
    return count; 
}

static ssize_t show_output_port1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_OUTPUT_PORT1, &val);
	return sprintf(buf, "0x%x\n", val);
}



static ssize_t set_output_port1(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_OUTPUT_PORT1, val);     
	}
    return count; 
}


static ssize_t show_invert_port0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_INVERT_PORT0, &val);
	return sprintf(buf, "0x%x\n", val);
}

static ssize_t set_invert_port0(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_INVERT_PORT0, val);     
	}
    return count; 
}

static ssize_t show_invert_port1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_INVERT_PORT1, &val);
	return sprintf(buf, "0x%x\n", val);
}



static ssize_t set_invert_port1(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_INVERT_PORT1, val);     
	}
    return count; 
}

//Port0 cofiguration

static ssize_t show_confg_port0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_CONFG_PORT0, &val);
	return sprintf(buf, "0x%x\n", val);
}


static ssize_t set_confg_port0(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_CONFG_PORT0, val);     
	}
    return count; 
}
//Port1 cofiguration
static ssize_t show_confg_port1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t val;
	tca6416a_read_reg(tc6416a_chip, TCA6416A_CONFG_PORT1, &val);
	return sprintf(buf, "0x%x\n", val);
}


static ssize_t set_confg_port1(struct device *dev,
            struct device_attribute *attr,
            const char *buf,
            size_t count)
{
    int ret = 0;
    uint8_t val;
	u32 data_size;

    if (sscanf(buf, "%x", (u32 *)&val) != 1) {
        dev_err(dev, "%s \n",__func__);
        ret = -EINVAL;
    } else {
 
        printk("val=:0x%x\n", val);
		tca6416a_write_reg(tc6416a_chip, TCA6416A_CONFG_PORT1, val);     
	}
    return count; 
}



static DEVICE_ATTR(input_port0,  S_IRUGO, show_input_port0, NULL);
static DEVICE_ATTR(input_port1, S_IRUGO, show_input_port1, NULL);
static DEVICE_ATTR(output_port0, S_IWUSR | S_IRUGO, show_output_port0, set_output_port0);
static DEVICE_ATTR(output_port1, S_IWUSR | S_IRUGO, show_output_port1, set_output_port0);
static DEVICE_ATTR(invert_port0, S_IWUSR | S_IRUGO, show_invert_port0, set_invert_port0);
static DEVICE_ATTR(invert_port1, S_IWUSR | S_IRUGO, show_invert_port1, set_invert_port0);
static DEVICE_ATTR(confg_port0, S_IWUSR | S_IRUGO, show_confg_port0, set_confg_port0);
static DEVICE_ATTR(confg_port1, S_IWUSR | S_IRUGO, show_confg_port1, set_confg_port0);



static struct attribute *tca6416a_attributes[] = {
	&dev_attr_input_port0.attr,
	&dev_attr_input_port1.attr,
	&dev_attr_output_port0.attr,
	&dev_attr_output_port1.attr,
	&dev_attr_invert_port0.attr,
	&dev_attr_invert_port1.attr,
	&dev_attr_confg_port0.attr,
	&dev_attr_confg_port1.attr,
	NULL
};

static const struct attribute_group tca6416a_group = {
	.attrs = tca6416a_attributes,
};


static int __devinit tca6416a_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tca6416a_platform_data *pdata;
	struct tca6416a_chip *chip;
	struct input_dev *input;
	int error;
	int i;

	/* Check functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct tca6416a_chip),GFP_KERNEL);
	if (!chip) {
		error = -ENOMEM;
		goto fail;
	}

	chip->client = client;

	tc6416a_chip = chip;
	 
	i2c_set_clientdata(client, chip);

	error = sysfs_create_group(&client->dev.kobj, &tca6416a_group);
	if (error)
		goto fail;

	tca6416a_gpio_cmd_write_bit(TCA6416A_OUTPUT_PORT0,0,0); //output EN1V2
	tca6416a_gpio_cmd_write_bit(TCA6416A_OUTPUT_PORT0,1,0); //output EN1V21
	tca6416a_gpio_cmd_write_bit(TCA6416A_OUTPUT_PORT0,3,0); //output EN3V3	
	tca6416a_gpio_cmd_write_bit(TCA6416A_OUTPUT_PORT1,0,0); //output DISP_OE		
	tca6416a_gpio_cmd_write_bit(TCA6416A_OUTPUT_PORT1,1,0); //output Bridge Reset	
	return 0;
fail:
	kfree(chip);
	return error;
}

static int __devexit tca6416a_remove(struct i2c_client *client)
{
	struct tca6416a_chip *chip = i2c_get_clientdata(client);

	kfree(chip);

	return 0;
}

static struct i2c_driver tca6416a_driver = {
	.driver = {
		.name	= "tca6416a",	
	},
	.probe		= tca6416a_probe,
	.remove		= __devexit_p(tca6416a_remove),
	.id_table	= tca6416a_id,
};

static int __init tca6416a_init(void)
{
	return i2c_add_driver(&tca6416a_driver);
}

subsys_initcall(tca6416a_init);

static void __exit tca6416a_exit(void)
{
	i2c_del_driver(&tca6416a_driver);
}
module_exit(tca6416a_exit);

MODULE_AUTHOR("	Qiming <qiming.cheng@agreeyamobility.net>");
MODULE_DESCRIPTION("Driver for tca6146 IO expander");
MODULE_LICENSE("GPL");
