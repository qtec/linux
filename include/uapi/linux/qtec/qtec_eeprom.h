/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */

#ifndef QTEC_EEPROM_H
#define QTEC_EEPROM_H

#include <linux/types.h>

#define EMPTY_HEAD "\xFF\xFF\xFF\xFF"
#define MAGIC_QTEC "QTEC"
#define SERIAL_VERSION 1
#define N_VAR 8

enum PARAMS_HEAD_CCD{CCD_N=0,CCD_BAYER};
enum PARAMS_HEAD_CMOSISV0{VERSION=0,CMOSIS_BAYER};
enum PARAMS_HEAD_CMOSISV1{CMOSIS_OFFSET=CMOSIS_BAYER+1,CMOSIS_ADCGAIN,CMOSIS_VRAMP};
enum PARAMS_HEAD_ROICV0{ROIC_INGAS0=CMOSIS_BAYER+1};
enum PARAMS_HEAD_ROICV1{ROIC_INGAS1=N_VAR-1}; //Roic values starts from bottom to up
enum PARAMS_HEAD_GOGGLE{CMOSIS_BAYER1=VERSION,CMOSIS_OFFSET1=CMOSIS_VRAMP+1,CMOSIS_ADCGAIN1,CMOSIS_VRAMP1};
enum PARAMS_BODY{VOLT_SENSOR=0};

struct qt5023_serial_info{
	char magic[4]; //Q T E C//
	__u8 version;
	char product[16]; //ie Q T 5 0 1 2 C M B V 0//
	__u8 mac[6];// Mac address (and serial)
	char manuf_date[9];// Manuf date in american format 19831130 (30 of nov of 1983)
	__u32 var[N_VAR];
	__u32 crc;
}__attribute__((__packed__));;

#ifdef QTEC_EEPROM_STRING_DESC

struct qtec_eeprom_desc{
	char product[16]; //ie Q T 5 0 1 2 C M B V 0//
	char desc[N_VAR][128];
	__s32 def_value[N_VAR];
};

static const struct qtec_eeprom_desc qtec_eeprom_desc[] ={
	{
		.product = "Q5NEWTECV00",
	},
	{
		.product = "Q5CCDV00",
		.desc = {
			"N_CCD",
			"BAYER",
		},
		.def_value = {
			3,
			0,
		},
	},
	{
		.product = "QCMOSISV00",
		.desc = {
			"VERSION (set to 1)",
			"BAYER mono (0), Bayer (1), 5x5 NIR Mosaic (2), 100 Band LS (3)",
			"OFFSET (440 on 8M, 530 on 12M)",
			"ADC_GAIN (32 on 8M, 0 on 12M)",
			"VRAMP (70 on 8M, 104 on 12M)",
		},
		.def_value = {
			1,
			0,
			-61,
			50,
			109,
		},
	},
	{
		.product = "QROICV00",
		.desc = {
			"VERSION (set to 1)",
			"BAYER mono (0), Bayer (1), 5x5 NIR Mosaic (2), 100 Band LS (3)",
			"OFFSET (440 on 8M)",
			"ADC_GAIN (32 on 8M)",
			"VRAMP (70 on 8M)",
			"",
			"",
			"INGAS",
		},
		.def_value = {
			1,
			0,
			-61,
			50,
			109,
			0,
			0,
			0,
		},
	},
	{
		.product = "QGOGGLEV00",
		.desc = {
			"BAYER_1 Right! mono (0), Bayer (1), 5x5 NIR Mosaic (2), 100 Band LS (3)",
			"BAYER_0 Left! mono (0), Bayer (1), 5x5 NIR Mosaic (2), 100 Band LS (3)",
			"OFFSET_0 Left",
			"ADC_GAIN_0 Left",
			"VRAMP_0 Left",
			"OFFSET_1 Right",
			"ADC_GAIN_1 Right",
			"VRAMP_1 Right",
		},
		.def_value = {
			0,
			0,
			-61,
			50,
			109,
			-61,
			50,
			109,
		},
	},
	{
		.product = "QT5023RevB",
		.desc = {
			"VOLT_SENSOR",
		},
		.def_value = {
			15000,
		},
	},
};
#endif

#endif
