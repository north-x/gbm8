/*
 * Copyright (c) 2014, Manuel Vetterli
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
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "sys/utils.h"
#include "eeprom.h"
#include "usb_support.h"
#include "ln_support.h"
#include "usb/usb.h"
#include "gbm8.h"
#include "fsz.h"
#include "sbk.h"
#include "gbm.h"

void init(void);
PROCESS_NAME(fsz_process);
PROCESS_NAME(sbk_in_process);
PROCESS_NAME(sbk_out_process);
PROCESS_NAME(gbm_io_process);
PROCESS_NAME(gbm_process);

uint16_t deviceID;
uint8_t gbm_mode = GBM_MODE_INIT;
volatile uint8_t gbm_version;

void init(void)
{
	USB_ConfigureClock();
	
	eeprom_load_status();
	eeprom_load_storage();

	deviceID = getID16();
	srand(deviceID);
	
	process_init();
	clock_init();
	
	process_start(&etimer_process, NULL);
	
#if defined GBM8_20121212	
	gbm8_hw_detect();
	
	//usb_init();
#endif
	loconet_init();
	
	// Reconfigure comparator to match hardware
	if (gbm_version==3)
	{
		ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
		ACA.AC1MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
		eeprom.gbm_mode = GBM_MODE_FSZ;
	}
	
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	update_gbm_mode();
}

int main(void)
{	
	init();
	
    while(1)
    {
        process_run();
    }
}

void update_gbm_mode(void)
{
	if (gbm_mode==eeprom.gbm_mode)
		return;

	switch (gbm_mode)
	{
		case GBM_MODE_NORMAL:
			process_exit(&gbm_io_process);
			process_exit(&gbm_process);
			PORTC.DIR = 0;
			PORTC.OUT = 0;
			break;
		case GBM_MODE_SBK:
			process_exit(&gbm_process);
			process_exit(&sbk_out_process);
			process_exit(&sbk_in_process);
			break;
		case GBM_MODE_FSZ:
			process_exit(&fsz_process);
			break;
	}
	
	switch (eeprom.gbm_mode)
	{
		case GBM_MODE_NORMAL:
			gbm_init();
			process_start(&gbm_process, NULL);
			process_start(&gbm_io_process, NULL);
			break;
		case GBM_MODE_SBK:
			gbm_init();
			sbk_init();
			process_start(&gbm_process, NULL);
			process_start(&sbk_out_process, NULL);
			process_start(&sbk_in_process, NULL);
			break;
		case GBM_MODE_FSZ:
			process_start(&fsz_process, NULL);
			break;
	}
	
	gbm_mode = eeprom.gbm_mode;
}

/**
 * \brief      Detect board version
 * \param c    Briefly describe all parameters.
 * \return     Briefly describe the return value.
 * \retval 0   Functions that return a few specified values
 * \retval 1   can use the \retval keyword instead of \return.
 *
 *             This function automatically detects the available LN communication
 *             hardware and initializes the needed peripherals.
 *
 *			   Idea: Enable pullup on LN_RX lines, if threshold is not reached
 *			   the pin is a GBM input
 */
void gbm8_hw_detect(void)
{
	ACA.CTRLB = 14; // 750mV Threshold
	
	PORTA.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
	
	// connect comparators to pins
	ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_SCALER_gc;
	ACA.AC1MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_SCALER_gc;
	
	// Enable comparator
	ACA.AC0CTRL = AC_ENABLE_bm | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;
	ACA.AC1CTRL = AC_ENABLE_bm | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;
	_delay_ms(1);
	/*
	switch ((ACA.STATUS>>AC_AC0STATE_bp)&3)
	{
		case 0:
			// Board is a GBM8 20120104 (not supported)
		case 1:
			// not valid
		case 2:
			// Board is a GBM8 20121212
		case 3:
			// Board is an FSZ
	}
	*/
	gbm_version = (ACA.STATUS>>AC_AC0STATE_bp)&3;
	
	ACA.AC0CTRL = 0;
	ACA.AC1CTRL = 0;
	
	PORTA.PIN3CTRL = PORT_OPC_TOTEM_gc;
	PORTA.PIN5CTRL = PORT_OPC_TOTEM_gc;
}