/*
 * VTI CMR3000_Dxx Gyroscope driver
 *
 * Copyright (C) 2011 Qtechnology
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>
 *
 * Copyright (C) 2010 Texas Instruments
 * Author: Hemanth V <hemanthv@ti.com>
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

#ifndef _LINUX_CMR3000_H
#define _LINUX_CMR3000_H

#define CMRMODE_DEFAULT    0
#define CMRMODE_STANDBY    1
#define CMRMODE_MEAS20     2
#define CMRMODE_MEAS80     3
#define CMRMODE_POFF       0

#define CMRIRQLEVEL_LOW    1
#define CMRIRQLEVEL_HIGH   0

#define CMRRANGE   3072

/**
 * struct cmr3000_i2c_platform_data - CMR3000 Platform data
 * @fuzz_x: Noise on X Axis
 * @fuzz_y: Noise on Y Axis
 * @fuzz_z: Noise on Z Axis
 * @mode: Operating mode
 * @irq_level: Irq level
 */
struct cmr3000_platform_data {
	int fuzz_x;
	int fuzz_y;
	int fuzz_z;
	uint8_t irq_level;
	uint8_t mode;
	unsigned long irqflags;
};

#endif
