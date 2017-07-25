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
#include "sys/pt.h"
#include "eeprom.h"
#include "shiftreg.h"
#include "sbk.h"
#include "gbm8.h"

static struct etimer sbk_in_timer, sbk_out_timer;


#if defined(__AVR__)
#define DBG(...)
#else
#define DBG(...)        printf(__VA_ARGS__)
#endif


PROCESS(sbk_in_process, "SBK In");
PROCESS(sbk_out_process, "SBK Out");

volatile uint16_t shiftreg;
volatile uint8_t sbk_in_track;
volatile uint8_t sbk_out_track;

volatile uint8_t sbk_mode = SBK_MODE_NORMAL;

void sbk_init(void)
{	
	shiftreg_init();
	
	shiftreg_out16(0,1);
	shiftreg_enable();
}

void sbk_update_track(void)
{
	if (gbm_mode!=GBM_MODE_SBK)
		return;
	
	if (sbk_out_track==0)
		sbk_out_abort();
	
	if (sbk_in_track==0)
		sbk_in_abort();
		
	process_poll(&sbk_out_process);
	process_poll(&sbk_in_process);
}

void sbk_out_abort(void)
{
	shiftreg &= ~((1<<RELAIS_G1_OUT)
				|(1<<RELAIS_G2_OUT)
				|(1<<RELAIS_G3_OUT)
				|(1<<RELAIS_G4_OUT)
				|(1<<RELAIS_G5_OUT)
				|(1<<RELAIS_G6_OUT));
	shiftreg_out16(shiftreg,1);
	
	PT_INIT(&sbk_out_process.pt);
	sbk_out_track = 0;
}

void sbk_in_abort(void)
{
	shiftreg &= ~((1<<RELAIS_G1_IN)
				|(1<<RELAIS_G2_IN)
				|(1<<RELAIS_G3_IN)
				|(1<<RELAIS_G4_IN)
				|(1<<RELAIS_G5_IN)
				|(1<<RELAIS_G6_IN)
				|(1<<RELAIS_FSS));

	shiftreg_out16(shiftreg,1);
	
	PT_INIT(&sbk_in_process.pt);	
	sbk_in_track = 0;
}

PROCESS_THREAD(sbk_out_process, ev, data)
{
	PROCESS_BEGIN();
	
	while (1)
	{
		PROCESS_WAIT_UNTIL(sbk_out_track!=0);
		
		if (sbk_in_track==sbk_out_track)
			DBG("Durchfahrt auf Gleis ");
		else
			DBG("Ausfahrt von Gleis ");
		DBG('0' + sbk_out_track);
		DBG("\r\n");
		
		PROCESS_WAIT_UNTIL(sbk_out_track!=sbk_in_track);
		
		switch (sbk_out_track)
		{
			case 1:
				shiftreg |= (1<<RELAIS_G1_OUT);
				break;
			case 2:
				shiftreg |= (1<<RELAIS_G2_OUT);
				break;
			case 3:
				shiftreg |= (1<<RELAIS_G3_OUT);
				break;
			case 4:
				shiftreg |= (1<<RELAIS_G4_OUT);
				break;
			case 5:
				shiftreg |= (1<<RELAIS_G5_OUT);
				break;
			case 6:
				shiftreg |= (1<<RELAIS_G6_OUT);
				break;
			default:
				break;
		}
		
		shiftreg_out16(shiftreg,1);

		// First: wait until track is occupied (allows setting route even when track is empty)
		while ((gbm_register_filt_filt&(1<<sbk_out_track))==0)
		{
			PROCESS_PAUSE();
		}
		
		// Second: wait until track is free again.
		while ((gbm_register_filt_filt&(1<<sbk_out_track))!=0)
		{
			PROCESS_PAUSE();
		}
		
/*
		while (1)
		{			
			if (!gbm_register_filt&(1<<TRACK2GBM(sbk_out_track)))
			{
				break;
			}

			PT_YIELD(pt);
		}
*/		

		etimer_set(&sbk_out_timer, 20*CLOCK_SECOND);
		//sbk_out_timer = TickGet();
		
		// Wait for timeout before resetting sbk exit (this can be overridden by the upper control system)
		//PT_WAIT_UNTIL(pt, (TickGet()-sbk_out_timer) >= 20*TICKS_PER_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sbk_out_timer));
		
		shiftreg &= ~((1<<RELAIS_G1_OUT)
					|(1<<RELAIS_G2_OUT)
					|(1<<RELAIS_G3_OUT)
					|(1<<RELAIS_G4_OUT)
					|(1<<RELAIS_G5_OUT)
					|(1<<RELAIS_G6_OUT));
		
		shiftreg_out16(shiftreg,1);
		
		sbk_out_track = 0;
		process_poll(&sbk_in_process);
	}
	
	PROCESS_END();
}

PROCESS_THREAD(sbk_in_process, ev, data)
{

	PROCESS_BEGIN();
	
	while (1)
	{
		PROCESS_WAIT_UNTIL(sbk_in_track!=0);
		
		DBG("Einfahrt auf Gleis ");
		DBG('0' + sbk_in_track);
		DBG("\r\n");
		
		switch (sbk_in_track)
		{
			case 1:
				shiftreg |= (1<<RELAIS_G1_IN);
				break;
			case 2:
				shiftreg |= (1<<RELAIS_G2_IN);
				break;
			case 3:
				shiftreg |= (1<<RELAIS_G3_IN);
				break;
			case 4:
				shiftreg |= (1<<RELAIS_G4_IN);
				break;
			case 5:
				shiftreg |= (1<<RELAIS_G5_IN);
				break;
			case 6:
				shiftreg |= (1<<RELAIS_G6_IN);
				break;
			default:
				break;
		}
		
		shiftreg_out16(shiftreg,1);
		
#if 0
		// First: wait until track is free (allows proper functioning of the cleaning mode)
		PROCESS_WAIT_UNTIL((gbm_register_filt_filt&(1<<sbk_in_track))==0);
		
		// Second: wait until track is occupied again
		PROCESS_WAIT_UNTIL((gbm_register_filt_filt&(1<<sbk_in_track))!=0);
#else
		while ((gbm_register_filt_filt&(1<<sbk_in_track))!=0)
		{
			PROCESS_PAUSE();
		}
		
		while ((gbm_register_filt_filt&(1<<sbk_in_track))==0)
		{
			PROCESS_PAUSE();
		}

#endif		
		if ((sbk_in_track!=sbk_out_track) && (sbk_mode==SBK_MODE_FSS))
		{
			// --> FSS aktivieren via LN --> 120612: nicht nötig, da ständig aktiv
			shiftreg |= (1<<RELAIS_FSS);
			shiftreg_out16(shiftreg,1);
			etimer_set(&sbk_in_timer, 3*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sbk_in_timer));
		}
		
		//PT_WAIT_UNTIL(pt, (TickGet()-sbk_in_timer) >= 3*TICKS_PER_SECOND);
		
		shiftreg &= ~((1<<RELAIS_G1_IN)
					|(1<<RELAIS_G2_IN)
					|(1<<RELAIS_G3_IN)
					|(1<<RELAIS_G4_IN)
					|(1<<RELAIS_G5_IN)
					|(1<<RELAIS_G6_IN)
					|(1<<RELAIS_FSS));
		
		// FSS deaktivieren (oder automatisch nach Ablauf von Timeout)
		shiftreg_out16(shiftreg,1);
		
		sbk_in_track = 0;
		process_poll(&sbk_out_process);
	}
	
	PROCESS_END();
}
