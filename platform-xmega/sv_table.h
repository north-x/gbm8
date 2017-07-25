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

/*
 * sv_table.h
 *
 */ 

#ifndef PARAMETER_TABLE_H_
#define PARAMETER_TABLE_H_

#include "gbm8.h"
#include "eeprom.h"
#include "platform.h"
#include "ubasic.h"
#include "servo.h"
#include "port.h"
#include "fsz.h"
#include "gbm.h"
#include "sbk.h"
#include "usb/usb.h"
#include <avr/wdt.h>
#include <string.h>

void cmd_exec(void);
void tse_update_page_read(void);
void tse_update_page_write(void);
void tse_update_time_ratio(void);
void dimm_target_parameter_update(void);
void dimm_delta_parameter_update(void);
void ln_update_threshold(void);

extern uint8_t ln_gpio_status;
extern uint8_t ln_gpio_status;
extern uint8_t ln_gpio_status_pre;
extern uint8_t ln_gpio_status_tx;
extern uint8_t ln_gpio_opcode_tx;
extern uint8_t ln_gpio_opcode_tx2;
extern uint8_t ln_wdt_counter;
extern rwSlotDataMsg rSlot;

uint8_t dimm_target_temp;
uint8_t dimm_delta_temp;
uint8_t dimm_parameter_select;
uint8_t cmd_register = 0;

extern uint8_t ubasic_script_status;

// Block mapping of TSE scripts starting at SV 112/170/240/304 (0x70/0xB0/0xF0/0x130)
SV_BLOCK_TABLE_BEGIN()
SV_BLOCK_TABLE_END();

SV_TABLE_BEGIN()
SV_CONST(1, "EEPROM Size", 0)
SV_CONST(2, "SW Version", SOFTWARE_VERSION)
SV_LSB(3, "Serial Number L", eeprom.sv_serial_number, 0)
SV_MSB(4, "Serial Number H", eeprom.sv_serial_number, 0)
SV(5, "Command Register", cmd_register, cmd_exec)
SV(6, "Config Register 1", eeprom.configA, 0)
SV(7, "Config Register 2", eeprom.configB, 0)
SV(8, "SBK In Track", sbk_in_track, sbk_update_track)
SV(9, "SBK Out Track", sbk_out_track, sbk_update_track)
SV(10, "LN GPIO Status", ln_gpio_status, 0)
SV(11, "LN GPIO Status Transmit", ln_gpio_status_tx, 0)
SV(12, "LN Threshold Voltage x10", eeprom.ln_threshold, ln_update_threshold)
SV(18, "FSZ Set Num", fsz_sv_temp, fsz_sv_helper_set)
SV(19, "FSZ Clear Num", fsz_sv_temp, fsz_sv_helper_clear)
SV_LSB(20, "FSZ1 Set LSB", fsz_register[0].set, 0)
SV_MSB(21, "FSZ1 Set MSB", fsz_register[0].set, 0)
SV_LSB(22, "FSZ1 Clear LSB", fsz_register[0].clear, 0)
SV_MSB(23, "FSZ1 Clear MSB", fsz_register[0].clear, 0)
SV_LSB(24, "FSZ1 Reg LSB", fsz_register[0].value, 0)
SV_MSB(25, "FSZ1 Reg MSB", fsz_register[0].value, 0)
SV_LSB(26, "FSZ2 Set LSB", fsz_register[1].set, 0)
SV_MSB(27, "FSZ2 Set MSB", fsz_register[1].set, 0)
SV_LSB(28, "FSZ2 Clear LSB", fsz_register[1].clear, 0)
SV_MSB(29, "FSZ2 Clear MSB", fsz_register[1].clear, 0)
SV_LSB(30, "FSZ2 Reg LSB", fsz_register[1].value, 0)
SV_MSB(31, "FSZ2 Reg MSB", fsz_register[1].value, 0)
SV(32, "GBM Value 1", gbm_avg[0], 0)
SV(33, "GBM Value 2", gbm_avg[1], 0)
SV(34, "GBM Value 3", gbm_avg[2], 0)
SV(35, "GBM Value 4", gbm_avg[3], 0)
SV(36, "GBM Value 5", gbm_avg[4], 0)
SV(37, "GBM Value 6", gbm_avg[5], 0)
SV(38, "GBM Value 7", gbm_avg[6], 0)
SV(39, "GBM Value 8", gbm_avg[7], 0)
SV(40, "GBM Track Select L", gbm_track_select_L, 0)
SV(41, "GBM Track Select H", gbm_track_select_H, 0)
SV(42, "GBM Threshold ON Multi", gbm_temp_multi, gbm_helper_multi_threshold_on)
SV(43, "GBM Threshold OFF Multi", gbm_temp_multi, gbm_helper_multi_threshold_off)
SV(44, "GBM Delay ON Multi", gbm_temp_multi, gbm_helper_multi_delay_on)
SV(45, "GBM Delay OFF Multi", gbm_temp_multi, gbm_helper_multi_delay_off)
SV(50, "GBM Threshold ON 1", eeprom.gbm_threshold_on[0], 0)
SV(51, "GBM Threshold ON 2", eeprom.gbm_threshold_on[1], 0)
SV(52, "GBM Threshold ON 3", eeprom.gbm_threshold_on[2], 0)
SV(53, "GBM Threshold ON 4", eeprom.gbm_threshold_on[3], 0)
SV(54, "GBM Threshold ON 5", eeprom.gbm_threshold_on[4], 0)
SV(55, "GBM Threshold ON 6", eeprom.gbm_threshold_on[5], 0)
SV(56, "GBM Threshold ON 7", eeprom.gbm_threshold_on[6], 0)
SV(57, "GBM Threshold ON 8", eeprom.gbm_threshold_on[7], 0)
SV(58, "GBM Threshold OFF 1", eeprom.gbm_threshold_off[0], 0)
SV(59, "GBM Threshold OFF 2", eeprom.gbm_threshold_off[1], 0)
SV(60, "GBM Threshold OFF 3", eeprom.gbm_threshold_off[2], 0)
SV(61, "GBM Threshold OFF 4", eeprom.gbm_threshold_off[3], 0)
SV(62, "GBM Threshold OFF 5", eeprom.gbm_threshold_off[4], 0)
SV(63, "GBM Threshold OFF 6", eeprom.gbm_threshold_off[5], 0)
SV(64, "GBM Threshold OFF 7", eeprom.gbm_threshold_off[6], 0)
SV(65, "GBM Threshold OFF 8", eeprom.gbm_threshold_off[7], 0)
SV(66, "GBM Delay ON 1", eeprom.gbm_delay_on[0], 0)
SV(67, "GBM Delay ON 2", eeprom.gbm_delay_on[1], 0)
SV(68, "GBM Delay ON 3", eeprom.gbm_delay_on[2], 0)
SV(69, "GBM Delay ON 4", eeprom.gbm_delay_on[3], 0)
SV(70, "GBM Delay ON 5", eeprom.gbm_delay_on[4], 0)
SV(71, "GBM Delay ON 6", eeprom.gbm_delay_on[5], 0)
SV(72, "GBM Delay ON 7", eeprom.gbm_delay_on[6], 0)
SV(73, "GBM Delay ON 8", eeprom.gbm_delay_on[7], 0)
SV(74, "GBM Delay OFF 1", eeprom.gbm_delay_off[0], 0)
SV(75, "GBM Delay OFF 2", eeprom.gbm_delay_off[1], 0)
SV(76, "GBM Delay OFF 3", eeprom.gbm_delay_off[2], 0)
SV(77, "GBM Delay OFF 4", eeprom.gbm_delay_off[3], 0)
SV(78, "GBM Delay OFF 5", eeprom.gbm_delay_off[4], 0)
SV(79, "GBM Delay OFF 6", eeprom.gbm_delay_off[5], 0)
SV(80, "GBM Delay OFF 7", eeprom.gbm_delay_off[6], 0)
SV(81, "GBM Delay OFF 8", eeprom.gbm_delay_off[7], 0)
SV(82, "LN GPIO 1 On Opcode 1", eeprom.ln_gpio_opcode[0][0], 0)
SV(83, "LN GPIO 1 On Opcode 2", eeprom.ln_gpio_opcode[0][1], 0)
SV(84, "LN GPIO 1 On Opcode 3", eeprom.ln_gpio_opcode[0][2], 0)
SV(85, "LN GPIO 1 Off Opcode 1", eeprom.ln_gpio_opcode[1][0], 0)
SV(86, "LN GPIO 1 Off Opcode 2", eeprom.ln_gpio_opcode[1][1], 0)
SV(87, "LN GPIO 1 Off Opcode 3", eeprom.ln_gpio_opcode[1][2], 0)
SV(88, "LN GPIO 2 On Opcode 1", eeprom.ln_gpio_opcode[2][0], 0)
SV(89, "LN GPIO 2 On Opcode 2", eeprom.ln_gpio_opcode[2][1], 0)
SV(90, "LN GPIO 2 On Opcode 3", eeprom.ln_gpio_opcode[2][2], 0)
SV(91, "LN GPIO 2 Off Opcode 1", eeprom.ln_gpio_opcode[3][0], 0)
SV(92, "LN GPIO 2 Off Opcode 2", eeprom.ln_gpio_opcode[3][1], 0)
SV(93, "LN GPIO 2 Off Opcode 3", eeprom.ln_gpio_opcode[3][2], 0)
SV(94, "LN GPIO 3 On Opcode 1", eeprom.ln_gpio_opcode[4][0], 0)
SV(95, "LN GPIO 3 On Opcode 2", eeprom.ln_gpio_opcode[4][1], 0)
SV(96, "LN GPIO 3 On Opcode 3", eeprom.ln_gpio_opcode[4][2], 0)
SV(97, "LN GPIO 3 Off Opcode 1", eeprom.ln_gpio_opcode[5][0], 0)
SV(98, "LN GPIO 3 Off Opcode 2", eeprom.ln_gpio_opcode[5][1], 0)
SV(99, "LN GPIO 3 Off Opcode 3", eeprom.ln_gpio_opcode[5][2], 0)
SV(100, "LN GPIO 4 On Opcode 1", eeprom.ln_gpio_opcode[6][0], 0)
SV(101, "LN GPIO 4 On Opcode 2", eeprom.ln_gpio_opcode[6][1], 0)
SV(102, "LN GPIO 4 On Opcode 3", eeprom.ln_gpio_opcode[6][2], 0)
SV(103, "LN GPIO 4 Off Opcode 1", eeprom.ln_gpio_opcode[7][0], 0)
SV(104, "LN GPIO 4 Off Opcode 2", eeprom.ln_gpio_opcode[7][1], 0)
SV(105, "LN GPIO 4 Off Opcode 3", eeprom.ln_gpio_opcode[7][2], 0)
SV(106, "LN GPIO 5 On Opcode 1", eeprom.ln_gpio_opcode[8][0], 0)
SV(107, "LN GPIO 5 On Opcode 2", eeprom.ln_gpio_opcode[8][1], 0)
SV(108, "LN GPIO 5 On Opcode 3", eeprom.ln_gpio_opcode[8][2], 0)
SV(109, "LN GPIO 5 Off Opcode 1", eeprom.ln_gpio_opcode[9][0], 0)
SV(110, "LN GPIO 5 Off Opcode 2", eeprom.ln_gpio_opcode[9][1], 0)
SV(111, "LN GPIO 5 Off Opcode 3", eeprom.ln_gpio_opcode[9][2], 0)
SV(112, "LN GPIO 6 On Opcode 1", eeprom.ln_gpio_opcode[10][0], 0)
SV(113, "LN GPIO 6 On Opcode 2", eeprom.ln_gpio_opcode[10][1], 0)
SV(114, "LN GPIO 6 On Opcode 3", eeprom.ln_gpio_opcode[10][2], 0)
SV(115, "LN GPIO 6 Off Opcode 1", eeprom.ln_gpio_opcode[11][0], 0)
SV(116, "LN GPIO 6 Off Opcode 2", eeprom.ln_gpio_opcode[11][1], 0)
SV(117, "LN GPIO 6 Off Opcode 3", eeprom.ln_gpio_opcode[11][2], 0)
SV(118, "LN GPIO 7 On Opcode 1", eeprom.ln_gpio_opcode[12][0], 0)
SV(119, "LN GPIO 7 On Opcode 2", eeprom.ln_gpio_opcode[12][1], 0)
SV(120, "LN GPIO 7 On Opcode 3", eeprom.ln_gpio_opcode[12][2], 0)
SV(121, "LN GPIO 7 Off Opcode 1", eeprom.ln_gpio_opcode[13][0], 0)
SV(122, "LN GPIO 7 Off Opcode 2", eeprom.ln_gpio_opcode[13][1], 0)
SV(123, "LN GPIO 7 Off Opcode 3", eeprom.ln_gpio_opcode[13][2], 0)
SV(124, "LN GPIO 8 On Opcode 1", eeprom.ln_gpio_opcode[14][0], 0)
SV(125, "LN GPIO 8 On Opcode 2", eeprom.ln_gpio_opcode[14][1], 0)
SV(126, "LN GPIO 8 On Opcode 3", eeprom.ln_gpio_opcode[14][2], 0)
SV(127, "LN GPIO 8 Off Opcode 1", eeprom.ln_gpio_opcode[15][0], 0)
SV(128, "LN GPIO 8 Off Opcode 2", eeprom.ln_gpio_opcode[15][1], 0)
SV(129, "LN GPIO 8 Off Opcode 3", eeprom.ln_gpio_opcode[15][2], 0)
SV(130, "WDT Reset Counter", ln_wdt_counter, 0)
SV_TABLE_END()


void cmd_exec(void)
{
	switch (cmd_register)
	{
		default:
		case 0:
			break;
		case 1:
			eeprom_sync_storage();
			break;
		case 2:
			wdt_enable(WDTO_1S);
			break;
		case 3:
			eeprom_load_defaults();
			break;
		case 4:
			USB_enter_bootloader();
			break;
	}
	
	cmd_register = 0;
}

void ln_update_threshold(void)
{
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
}

#endif /* PARAMETER_TABLE_H_ */