/*
* include/linux/ft5306_ts.h
*
* Truly CT0402 ft5306 TouchScreen driver header.
*
* Copyright (c) 2012, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

*/

struct ft5306_platform_data {
	char *name;
	int numtouch;
	int display_minx;
	int display_miny;
	int display_maxx;
	int display_maxy;
	int irq_gpio;
	int (*init_platform_hw)(void);
	int (*exit_platform_hw) (void);
	int (*init_gpio)(void);
};


