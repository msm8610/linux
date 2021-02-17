/*
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __GP2AP002_H_
#define __GP2AP002_H_

struct gp2ap002_platform_data {
	int (*get_adc)(void);
	void (*power_enable)(int);
	int vo_gpio;
	int prox_mode;
	int enable;
};

#endif
