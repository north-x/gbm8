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

#include "wa2.h"
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
			.configA = (0<<WA2_CONF_PWM_OUTPUTS_ENABLE)|(0<<WA2_CONF_PWM_CHANNEL7_ENABLE)|(1<<WA2_CONF_INPUTS_PULLUP_ENABLE)|(1<<WA2_CONF_SERVO_ENABLE_PWM_A),
			.configB = 0,
			.ln_threshold = 25,
			.servo_startup_delay = 80,
			.servo_timeout = 0,
			.servo_start_method = 1,
#if 0
			.servo_min = {15000, 15000},
			.servo_max = {45000, 45000},
#else
			.servo_min = {32767, 32767},
			.servo_max = {32768, 32768},
#endif
			.servo_time_ratio = {16, 16},
			.ln_gpio_opcode =
			{{ 0xB0, 0x01, 0x20},
			 { 0xB0, 0x01, 0x00},
			 { 0xB0, 0x02, 0x20},
			 { 0xB0, 0x02, 0x00},
			 { 0xB0, 0x03, 0x20},
			 { 0xB0, 0x03, 0x00},
			 { 0, 0, 0},
			 { 0, 0, 0},
			 { 0xB0, 0x04, 0x20},
			 { 0xB0, 0x04, 0x00},
			 { 0xB0, 0x05, 0x20},
			 { 0xB0, 0x05, 0x00},
			 { 0xB0, 0x06, 0x20},
			 { 0xB0, 0x06, 0x00},
			 { 0, 0, 0},
			 { 0, 0, 0}},
        };

struct t_eeprom_status eeprom_status_default = {
	.flags = 0,
	.ln_gpio_status = 0,
	.relay_request = 0,
	.servo_position = { 127, 127 }
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
