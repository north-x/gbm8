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
#include "sys/process.h"
#include "sys/etimer.h"
#include "eeprom.h"
#include "loconet/utils.h"
#include "ln_interface.h"
#include "sv.h"
#include "IdStorage.h"
#include "ln_support.h"
#include "usb_support.h"
#include "servo.h"

PROCESS(ln_process, "Loconet Handler");
static LnBuf LnBuffer;
uint8_t ln_gpio_status;
uint8_t ln_gpio_status_pre;
uint8_t ln_gpio_status_tx;
uint8_t ln_gpio_opcode_tx;
uint8_t ln_gpio_opcode_tx2;

extern uint16_t deviceID;
rwSlotDataMsg rSlot;

void loconet_init(void)
{
	initLnBuf(&LnBuffer);
	initLocoNet(&LnBuffer);
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	
	if (readSVDestinationId()==0xFFFF)
	{
		writeSVDestinationId(deviceID);
	}
	
	if (eeprom.sv_serial_number==0xFFFF)
	{
		eeprom.sv_serial_number = deviceID;
		eeprom_sync_storage();
	}
	
	process_start(&ln_process, NULL);
}

PROCESS_THREAD(ln_process, ev, data)
{
	lnMsg *LnPacket;
	
	PROCESS_BEGIN();
	
	// Initialization
	ln_gpio_status = eeprom_status.ln_gpio_status;
	ln_gpio_status_pre = eeprom_status.ln_gpio_status;
	rSlot.slot = 0xFF;
	
	while (1)
	{
		PROCESS_PAUSE();
		
		eeprom_status.ln_gpio_status = ln_gpio_status;
		
		doSVDeferredProcessing();
		
		LnPacket = recvLocoNetPacket();
	
		if (LnPacket)
		{   
			sendLocoNetPacketUSB(LnPacket);
			
			ln_gpio_process_rx(LnPacket);
			
			if ((LnPacket->sz.command == OPC_GPON))
			{
				// Force transmission of current state
				ln_gpio_status_tx = 0xFF;
			}
			/*
			else if (BootloaderParseMessage(LnPacket)==1)
			{
				//BootloaderEnter(); // enter bootloader from running application
			}*/
			else
				processSVMessage(LnPacket);
		}
		else
		{
			ln_gpio_process_tx();
		}
		
	}
	
	PROCESS_END();
}

void ln_gpio_process_tx(void)
{
	uint8_t *msg;
	static uint8_t current_bit = 0;
	
	if (current_bit>7)
		current_bit = 0;
	
	if (((ln_gpio_status^ln_gpio_status_pre)|ln_gpio_status_tx) & (1<<current_bit))
	{
		if (ln_gpio_status&(1<<current_bit))
		{
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2]))
			{
				ln_gpio_status_pre |= (1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);				
			}					
		}
		else
		{
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2+1]))
			{
				ln_gpio_status_pre &= ~(1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);				
			}
		}
	}
	
	if (ln_gpio_opcode_tx&(1<<current_bit))
	{
		msg = eeprom.ln_gpio_opcode[current_bit];
		if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
			ln_gpio_opcode_tx &= ~(1<<current_bit);
	}

	if (ln_gpio_opcode_tx2&(1<<current_bit))
	{
		msg = eeprom.ln_gpio_opcode[current_bit+8];
		if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
			ln_gpio_opcode_tx2 &= ~(1<<current_bit);
	}
	
	current_bit++;
}

uint8_t ln_create_message(uint8_t *msg)
{
	if (msg[0]==0)
		return 1;
		
	switch (msg[0])
	{
		// translate switch request to switch report
		case OPC_SW_REQ:
			//msg0 = OPC_SW_REP;
			//msg2 = (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0);
			return (sendLocoNet4BytePacket(OPC_SW_REP, msg[1], (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?0:OPC_SW_REP_HI))==LN_DONE)?1:0;
	}
	
	return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)?1:0;
}

void ln_gpio_process_rx(lnMsg *LnPacket)
{
	uint8_t index;
	
	if (!LnPacket)
	return;
	
	if (getLnMsgSize(LnPacket)>4)
	return;
	
	for (index=0;index<16;index++)
	{		
		if (LnPacket->sr.command!=eeprom.ln_gpio_opcode[index][0])
		continue;
		
		if (LnPacket->srq.sw1!=eeprom.ln_gpio_opcode[index][1])
		continue;
		
		if (LnPacket->srq.sw2!=eeprom.ln_gpio_opcode[index][2])
		continue;
		
		// We have a match, now set or clear the status
		if ((index%2)==0)
		{
			ln_gpio_status &= ~(1<<(index/2));
			ln_gpio_status_pre &= ~(1<<(index/2));
		}
		else
		{
			ln_gpio_status |= (1<<(index/2));
			ln_gpio_status_pre |= (1<<(index/2));
		}
	}
}
