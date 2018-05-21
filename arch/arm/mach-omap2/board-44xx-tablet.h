/*
 * arch/arm/mach-omap2/board-44xx-tablet.h
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

#ifndef _MACH_OMAP_BOARD_44XX_TABLET_H
#define _MACH_OMAP_BOARD_44XX_TABLET_H

#include "board-omap4plus-common.h"

#define GPIO_BOARD_ID0                  48
#define GPIO_BOARD_ID1                  49
typedef enum {
	C1_EVT1		= 3,
	C1_EVT2		= 0,
	C1_DVT1_ATCN	= 1,
	C1_DVT1_ATCE	= 2,
	UNVERSIONED	= 4,
} c1_version;

bool board_is_c1_evt1(void);
bool board_is_c1_evt2(void);
bool board_is_c1_dvt1_atce(void);
int __init tablet_display_init(void);
int __init tablet_touch_init(void);
int __init tablet_sensor_init(void);
int __init tablet_button_init(void);
void tablet_android_display_setup(void);
#endif
