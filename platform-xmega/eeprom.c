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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>

#include "gbm8.h"
#include "eeprom.h"
#include "port.h"
#include "platform.h"
#include "ubasic.h"

struct t_eeprom_storage eeprom;
struct t_eeprom_storage eeprom_shadow;
struct t_eeprom_storage eeprom_eemem EEMEM;
struct t_eeprom_status eeprom_status;
struct t_eeprom_status eeprom_status_shadow;
struct t_eeprom_status eeprom_status_eemem EEMEM;

struct t_eeprom_storage eeprom_default = {
			.salt = 0xAA+SOFTWARE_VERSION,
			.sv_serial_number = 0xFFFF,
			.sv_destination_id = 0xFFFF,
			.ubasic_autostart = (1<<0)|(1<<1)|(0<<2)|(0<<3),
			.configA = 0,
			.configB = 0,
			.ln_threshold = 50,
			.gbm_mode = GBM_MODE_NORMAL,
			.gbm_threshold_on = 15,
			.gbm_threshold_off = 10,
			.gbm_delay_on = 5,
			.gbm_delay_off = 40,
/*			.ln_gpio_opcode =
			{{ 0xB2, 0x7D, 0x50}, //251 1 (Addr Status)
			 { 0xB2, 0x7D, 0x40},
			 { 0xB2, 0x7D, 0x70}, //252 1
			 { 0xB2, 0x7D, 0x60},
			 { 0xB2, 0x7E, 0x50}, //253 1
			 { 0xB2, 0x7E, 0x40},
			 { 0xB2, 0x7E, 0x70}, //254 1
			 { 0xB2, 0x7E, 0x60},
			 { 0xB2, 0x79, 0x50}, //243 1
			 { 0xB2, 0x79, 0x40},
			 { 0xB2, 0x78, 0x70}, //242 1
			 { 0xB2, 0x78, 0x60},
			 { 0xB2, 0x78, 0x50}, //241 1
			 { 0xB2, 0x78, 0x40},
			 { 0xB2, 0x77, 0x70}, //240 1
			 { 0xB2, 0x77, 0x60}},*/
        };

struct t_eeprom_status eeprom_status_default = {
	.flags = 0,
	.ln_gpio_status = 0,
	};

void eeprom_load_status(void)
{
	eeprom_read_block(&eeprom_status, &eeprom_status_eemem, sizeof(t_eeprom_status));
	
	memcpy(&eeprom_status_shadow, &eeprom_status, sizeof(t_eeprom_status));
}

void eeprom_load_storage(void)
{
    eeprom_read_block(&eeprom, &eeprom_eemem, sizeof(t_eeprom_storage));
	
	memcpy(&eeprom_shadow, &eeprom, sizeof(t_eeprom_storage));
	
	if (eeprom.salt!=eeprom_default.salt)
		eeprom_load_defaults();
}

void eeprom_load_defaults(void)
{
	memcpy(&eeprom, &eeprom_default, sizeof(t_eeprom_storage));
	memcpy(&eeprom_status, &eeprom_status_default, sizeof(t_eeprom_status));
}

void eeprom_sync_storage(void)
{
    if (memcmp(&eeprom, &eeprom_shadow, sizeof(t_eeprom_storage)))
    {
        eeprom_update_block(&eeprom, &eeprom_eemem, sizeof(t_eeprom_storage));        
        memcpy(&eeprom_shadow, &eeprom, sizeof(t_eeprom_storage));
    }
}

void eeprom_sync_status(void)
{  
    if (memcmp(&eeprom_status, &eeprom_status_shadow, sizeof(t_eeprom_status)))
    {
        eeprom_update_block(&eeprom_status, &eeprom_status_eemem, sizeof(t_eeprom_status));
	    memcpy(&eeprom_status_shadow, &eeprom_status, sizeof(t_eeprom_status));
    }
}
