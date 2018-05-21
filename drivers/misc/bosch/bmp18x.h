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
 * @filename bmp18x.h
 * @date     "Fri Aug 2 17:41:45 2013 +0800"
 * @id       "644147c"
 *
 * @brief
 * The head file of BMP18X device driver core code
*/
#ifndef _BMP18X_H
#define _BMP18X_H

#define BMP18X_NAME "bmp18x"

/**
 * struct bmp18x_platform_data - represents platform data for the bmp18x driver
 * @chip_id: Configurable chip id for non-default chip revisions
 * @default_oversampling: Default oversampling value to be used at startup,
 * value range is 0-3 with rising sensitivity.
 * @default_sw_oversampling: Default software oversampling value to be used
 * at startup,value range is 0(Disabled) or 1(Enabled). Only take effect
 * when default_oversampling is 3.
 * @temp_measurement_period: Temperature measurement period (milliseconds), set
 * to zero if unsure.
 * @init_hw: Callback for hw specific startup
 * @deinit_hw: Callback for hw specific shutdown
 */
struct bmp18x_platform_data {
	u8  chip_id;
	u8  default_oversampling;
	u8  default_sw_oversampling;
	u32 temp_measurement_period;
	int (*init_hw)(void);
	void (*deinit_hw)(void);
};

struct bmp18x_bus_ops {
	int (*read_block)(void *client, u8 reg, int len, char *buf);
	int (*read_byte)(void *client, u8 reg);
	int (*write_byte)(void *client, u8 reg, u8 value);
};

struct bmp18x_data_bus {
	const struct bmp18x_bus_ops *bops;
	void  *client;
};

int bmp18x_probe(struct device *dev, struct bmp18x_data_bus *data_bus);
int bmp18x_remove(struct device *dev);
#ifdef CONFIG_PM
int bmp18x_enable(struct device *dev);
int bmp18x_disable(struct device *dev);
#endif

#endif
