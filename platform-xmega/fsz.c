/*
 * Copyright (c) 2015, Manuel Vetterli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <avr/io.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "eeprom.h"
#include "shiftreg.h"
#include "fsz.h"
#include "gbm8.h"

PROCESS(fsz_process, "FSZ Handler");

fsz_reg_t fsz_register[2];
uint8_t fsz_sr_reg[4];
uint8_t fsz_sv_temp;

#define MAP_BITS(SRC_REG, DEST_REG, SRC_BIT, DEST_BIT) if (SRC_REG&(1<<(SRC_BIT))) DEST_REG |= (1<<(DEST_BIT)); else DEST_REG &= ~(1<<(DEST_BIT))


PROCESS_THREAD(fsz_process, ev, data)
{
	uint8_t index, update_sr;
		
	PROCESS_BEGIN();
	
	// Initialization
	shiftreg_init();
	
	while (1)
	{
		PROCESS_PAUSE();
		update_sr = 0;
		for (index=0;index<2;index++)
		{
			if (fsz_register[index].set!=0)
			{
				fsz_register[index].value |= fsz_register[index].set;
				fsz_register[index].set = 0;
			}
			
			if (fsz_register[index].clear!=0)
			{
				fsz_register[index].value &= ~fsz_register[index].clear;
				fsz_register[index].clear = 0;
			}
			
			if (fsz_register[index].value!=fsz_register[index].shadow)
			{
				fsz_register[index].shadow = fsz_register[index].value;
				update_sr = 1;
				if (index==0)
				{
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 0, 6);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 1, 7);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 2, 4);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 3, 3);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 4, 2);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 5, 1);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 6, 0);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[0], 7, 5);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 8, 2);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 9, 3);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 10, 4);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 11, 5);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 12, 1);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 13, 0);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 14, 7);
					MAP_BITS(fsz_register[0].value, fsz_sr_reg[1], 15, 6);
				}
				else
				{
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 0, 6);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 1, 7);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 2, 4);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 3, 3);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 4, 2);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 5, 1);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 6, 0);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[2], 7, 5);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 8, 2);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 9, 3);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 10, 4);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 11, 5);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 12, 1);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 13, 0);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 14, 7);
					MAP_BITS(fsz_register[1].value, fsz_sr_reg[3], 15, 6);
				}
			}
		}
		
		if (update_sr!=0)
		{
			shiftreg_out(fsz_sr_reg, 4, 1);
			update_sr = 0;
		}
	}
		
	PROCESS_END();
}

void fsz_sv_helper_set(void)
{
	if (fsz_sv_temp>200)
	{
		fsz_sv_temp -= 201;
		if (fsz_sv_temp<16)
		{
			fsz_register[1].set = (1<<fsz_sv_temp);
		}
	}
	else if (fsz_sv_temp>100)
	{
		fsz_sv_temp -= 101;
		if (fsz_sv_temp<16)
		{
			fsz_register[0].set = (1<<fsz_sv_temp);
		}
	}
	else
	{
		fsz_sv_temp--;
		if (fsz_sv_temp<16)
		{
			fsz_register[0].set = (1<<fsz_sv_temp);
		}
	}
	
	fsz_sv_temp = 0;
}

void fsz_sv_helper_clear(void)
{
	if (fsz_sv_temp>200)
	{
		fsz_sv_temp -= 201;
		if (fsz_sv_temp<16)
		{
			fsz_register[1].clear = (1<<fsz_sv_temp);
		}
	}
	else if (fsz_sv_temp>100)
	{
		fsz_sv_temp -= 101;
		if (fsz_sv_temp<16)
		{
			fsz_register[0].clear = (1<<fsz_sv_temp);
		}
	}
	else
	{
		fsz_sv_temp--;
		if (fsz_sv_temp<16)
		{
			fsz_register[0].clear = (1<<fsz_sv_temp);
		}
	}
	
	fsz_sv_temp = 0;
}
