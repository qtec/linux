/*
 * VTI CMR3000_D0x Gyroscpe driver
 *
 * Copyright (C) 2011 Qtechnology
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>
 *
 * Based on:
 *	drivers/input/misc/cma3000_d0x.h by Hemanth V
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _INPUT_CMR3000_H
#define _INPUT_CMR3000_H

#include <linux/types.h>
#include <linux/input.h>

struct device;
struct cmr3000_gyro_data;

struct cmr3000_bus_ops {
	u16 bustype;
	u8 ctrl_mod;
	int (*read) (struct device *, u8, char *);
	int (*write) (struct device *, u8, u8, char *);
};

struct cmr3000_gyro_data *cmr3000_init(struct device *dev, int irq,
				       const struct cmr3000_bus_ops *bops);
void cmr3000_exit(struct cmr3000_gyro_data *);
void cmr3000_suspend(struct cmr3000_gyro_data *);
void cmr3000_resume(struct cmr3000_gyro_data *);

#endif
