/*
 * tca6416a expander support
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TCA6416A_H
#define _TCA6416A_H

#include <linux/types.h>

#define TCA6416A_INPUT_PORT0          0
#define TCA6416A_INPUT_PORT1          1
#define TCA6416A_OUTPUT_PORT0         2
#define TCA6416A_OUTPUT_PORT1         3
#define TCA6416A_INVERT_PORT0         4
#define TCA6416A_INVERT_PORT1         5
#define TCA6416A_CONFG_PORT0		      6
#define TCA6416A_CONFG_PORT1		      7


struct tca6416a_platform_data {
	int gpio_irq;
};
extern int tca6416a_gpio_cmd_write_bit(uint8_t cmd,uint8_t bit, uint8_t val);
#endif
