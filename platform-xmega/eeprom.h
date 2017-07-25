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

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif


   
typedef struct t_eeprom_storage {
	uint8_t salt;
	uint16_t sv_serial_number;
	uint16_t sv_destination_id;
	uint8_t ubasic_autostart;
	uint8_t configA;
	uint8_t configB;
	uint8_t ln_threshold;
	uint8_t gbm_mode;
	uint8_t gbm_threshold_on[8];
	uint8_t gbm_threshold_off[8];
	uint8_t gbm_delay_on[8];
	uint8_t gbm_delay_off[8];
	uint8_t ln_gpio_opcode[16][3];
} t_eeprom_storage;

typedef struct t_eeprom_status {
	uint8_t flags;
	uint8_t ln_gpio_status;
} t_eeprom_status;

void eeprom_init(void);
void eeprom_read(unsigned char addr, unsigned char len, unsigned char *buf);
unsigned char eeprom_write(unsigned char addr, unsigned char len, unsigned char *buf);

void eeprom_load_storage(void);
void eeprom_sync_storage(void);
void eeprom_load_status(void);
void eeprom_sync_status(void);
void eeprom_load_defaults(void);

extern struct t_eeprom_storage eeprom;
extern struct t_eeprom_storage eeprom_shadow;
extern struct t_eeprom_status eeprom_status;
extern struct t_eeprom_status eeprom_status_shadow;
extern struct t_eeprom_storage eeprom_eemem;
extern struct t_eeprom_status eeprom_status_eemem;

extern unsigned char eeprom_temp;
void eeprom_test_read(void);
void eeprom_test_write(void);

#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

