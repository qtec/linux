/*
 * (C) Copyright 2012-2014
 * Ricado Ribalda-ricardo.ribalda@gmail.com
 * This work has been supported by: QTechnology  http://qtec.com/
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>                  // Needed for printf()
#include <stdlib.h>                 // Needed for rand()
#include <stdint.h>

enum mode {SPI=0,BPI,KBPI};
#define N_FF 9
#define N_FF_KBPI 10

/*
 * Reversed engineered from promgen tool from xilinx
 */

static void use(char *name){
	fprintf(stderr,"use: %s BPI/SPI/KBPI file.bit filen.bin\n",name);
}

static int find_ff(FILE *file ,int n_ff){
	uint8_t in;
	int loc=0;
	int ret;
	int n_read=0;

	while(loc!=n_ff){
		ret=fread(&in,1,1,file);
		if (ret!=1)
			return -1;

		n_read++;
		if (in==0xff){
			loc++;
			continue;
		}
		else
			loc=0;
	}
	return n_read;
}

static uint8_t invert_byte(uint8_t byte){
	uint8_t out=0;
	int i;

	for (i=0;i<8;i++){
		out <<= 1;
		out |= byte&1;
		byte >>= 1;
	}

	return out;
}

static int write_ff(FILE *fout, int n_ff){
	int i;
	uint8_t out;

	for (i=0;i<n_ff;i++){
		out=0xff;
		if (fwrite(&out,1,1,fout)<1)
			return -1;
	}

	return 0;
}

static int convert_bit(FILE *fin, FILE *fout, int mode){
	uint8_t in;
	uint8_t in2;
	int ret;

	while ((ret=fread(&in,1,1,fin))>0){
		if (mode==BPI)
			in=invert_byte(in);
		if (mode == KBPI){
			in=invert_byte(in);
			ret=fread(&in2,1,1,fin);
			if (ret<1)
				break;
			in2=invert_byte(in2);
			ret=fwrite(&in2,1,1,fout);
			if (ret<1)
				break;
		}
		ret=fwrite(&in,1,1,fout);
		if (ret<1)
			break;
	}

	return ret;
}

//===== Main program ==========================================================
int main(int argc, char *argv[])
{
	FILE *fin,*fout;
	int mode=SPI;
	int err;

	if (argc!=4){
		use(argv[0]);
		return -1;
	}

	if ((argv[1][0]=='B')||(argv[1][0]=='b'))
		mode=BPI;
	if ((argv[1][0]=='K')||(argv[1][0]=='k'))
		mode=KBPI;

	fin=fopen(argv[2],"r");
	if (!fin){
		perror("fopen_read");
		return -1;
	}

	fout=fopen(argv[3],"w");
	if (!fout){
		fclose(fin);
		perror("fopen_write");
		return -1;
	}

	err=find_ff(fin,(mode==KBPI)?N_FF_KBPI:N_FF);
	if (err<0){
		fprintf(stderr,"Sync not found\n");
		fclose(fin);
		fclose(fout);
		return -1;
	}
	//fprintf(stdout,"Sync found at 0x%x\n",err-N_FF);

	err=write_ff(fout,(mode==KBPI)?N_FF_KBPI:N_FF);
	if (err<0){
		fprintf(stderr,"Unable to write sync\n");
		fclose(fin);
		fclose(fout);
	}

	err=convert_bit(fin,fout,mode);
	if (err<0){
		fprintf(stderr,"Unable to convert bit\n");
		fclose(fin);
		fclose(fout);
		return -1;
	}

	//fprintf(stdout,"Bitfile converted\n");

	fclose(fin);
	fclose(fout);

	return 0;
}
