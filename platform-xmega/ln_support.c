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
PROCESS(ln_ack_process, "Loconet Ack Handler");

static struct etimer ln_ack_timer;

static LnBuf LnBuffer;
uint8_t ln_gpio_status;
uint8_t ln_gpio_status_pre;
uint8_t ln_gpio_status_ack;
uint8_t ln_gpio_status_tx;
uint8_t ln_gpio_ack_tx;
uint8_t ln_gpio_opcode_tx;
uint8_t ln_gpio_opcode_tx2;

extern uint16_t deviceID;
rwSlotDataMsg rSlot;

void loconet_init(void)
{
	initLnBuf(&LnBuffer);
	initLocoNet(&LnBuffer);
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	
	if (eeprom.sv_serial_number==0xFFFF)
	{
		eeprom.sv_serial_number = deviceID;
		ln_load_board_config();
		eeprom_sync_storage();
	}
	
	if (readSVDestinationId()==0xFFFF)
	{
		writeSVDestinationId(deviceID);
	}
	
	process_start(&ln_process, NULL);
	process_start(&ln_ack_process, NULL);
}

PROCESS_THREAD(ln_ack_process, ev, data)
{
	static uint8_t current_bit = 0;
	
	PROCESS_BEGIN();
	
	etimer_set(&ln_ack_timer, 250E-3*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_YIELD();
		etimer_reset(&ln_ack_timer);
		
		if (current_bit>7)
			current_bit = 0;
		
		if (ln_gpio_status_ack&(1<<current_bit))
		{
			// Re transmit
			ln_gpio_status_tx |= (1<<current_bit);
		}
		
		current_bit++;
	}
	
	PROCESS_END();
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
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2+1]))
			{
				ln_gpio_status_pre |= (1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);
				ln_gpio_status_ack |= (1<<current_bit);			
			}					
		}
		else
		{
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2]))
			{
				ln_gpio_status_pre &= ~(1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);
				ln_gpio_status_ack |= (1<<current_bit);				
			}
		}
	}
	else if (ln_gpio_ack_tx&(1<<current_bit))
	{
		if (ln_gpio_status&(1<<current_bit))
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_bit*2+1]))
			{
				ln_gpio_ack_tx &= ~(1<<current_bit);
			}
		}
		else
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_bit*2]))
			{
				ln_gpio_ack_tx &= ~(1<<current_bit);
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
			return (sendLocoNet4BytePacket(OPC_SW_REP, msg[1], (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0))==LN_DONE)?1:0;
	}
	
	return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)?1:0;
}

uint8_t ln_create_message_ack(uint8_t *msg)
{
	if (msg[0]==0)
	return 1;
	
	return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2]^(1<<6))==LN_DONE)?1:0;
}

void ln_gpio_process_rx(lnMsg *LnPacket)
{
	uint8_t index;
	uint8_t ack;
	
	if (!LnPacket)
		return;
	
	if (getLnMsgSize(LnPacket)>4)
		return;
	
	for (index=0;index<16;index++)
	{	
		if ((LnPacket->sr.command==eeprom.ln_gpio_opcode[index][0])
		|| ((LnPacket->sr.command==OPC_SW_REP) && (eeprom.ln_gpio_opcode[index][0]==OPC_SW_REQ)))
		{
			
		}
		else
			continue;
		
		if (LnPacket->srq.sw1!=eeprom.ln_gpio_opcode[index][1])
			continue;
			
		ack = 0;
		
		switch (LnPacket->sr.command)
		{
			case OPC_SW_REQ:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
					
				if ((LnPacket->srq.sw2&(1<<6))==0)
				{
					// Command
					
				}
				else
				{
					// Ack
					ack = 1;
				}
				break;
			case OPC_INPUT_REP:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
					
				if ((LnPacket->srq.sw2&(1<<6))!=0)
				{
					// Command
					// Ignore if we are waiting for ack
					if ((ln_gpio_status_ack&(1<<(index/2)))!=0)
						continue;
				}
				else
				{
					// Ack
					ack = 1;
				}
				break;
			case OPC_SW_REP:
				if ((LnPacket->srq.sw2&0xF)!=(eeprom.ln_gpio_opcode[index][2]&0xF))
					continue;
					
				if ((LnPacket->srq.sw2&(1<<6))==0)
				{
					// Ack
					ack = 1;
					ln_gpio_status_ack &= ~(1<<(index/2));
				}
				else
				{
					// Command					
				}
				
				continue;
				break;
			default:
				if (LnPacket->srq.sw2!=eeprom.ln_gpio_opcode[index][2])
					continue;
		}
		
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
		
		if (ack)
		{
			ln_gpio_status_ack &= ~(1<<(index/2));
			//ln_gpio_status_tx &= ~(1<<(index/2));
		}
		else
			ln_gpio_ack_tx |= (1<<(index/2));
	}
}

void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr)
{
	switch (opc)
	{
		case OPC_SW_REQ:
			addr--;
			buf[0] = OPC_SW_REQ;
			buf[3] = OPC_SW_REQ;
			buf[1] = addr&0x7F;
			buf[4] = buf[1];
			buf[5] = (addr>>7)&0x7F;
			buf[2] = buf[5]|OPC_SW_REQ_DIR;
			break;
		case OPC_INPUT_REP:
			addr--;
			buf[0] = OPC_INPUT_REP;
			buf[3] = OPC_INPUT_REP;
			buf[1] = (addr>>1)&0x7F;
			buf[4] = buf[1];
			buf[2] = ((addr>>8)&0x7F)|OPC_INPUT_REP_CB|((addr&1)?0x20:0);
			buf[5] = buf[2]|OPC_INPUT_REP_HI;
			break;
	}
}

void ln_load_board_config(void)
{
	uint8_t index;
	
	switch (deviceID)
	{
		case 0x2CFC:
			eeprom.sv_destination_id = deviceID;
			eeprom.sv_serial_number = deviceID;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 251);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 252);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 253);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 254);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 243);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 242);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 241);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 240);
			break;
		case 0x3924:
			eeprom.sv_destination_id = 492;
			eeprom.sv_serial_number = 492;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 251);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 252);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 253);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 254);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 243);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 242);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 241);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 240);
			break;
		case 0xDCB:
			eeprom.sv_destination_id = 494;
			eeprom.sv_serial_number = 494;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 255);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 244);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 246);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 245);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 247);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 249);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 250);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 248);
			break;
		case 0x7B5A:
			eeprom.sv_destination_id = 498;
			eeprom.sv_serial_number = 498;
			break;
#if 0
		case 0x7C84:
			eeprom.sv_destination_id = 481;
			eeprom.sv_serial_number = 481;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 125);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 124);
			break;
		case 0x87E7:
			eeprom.sv_destination_id = 482;
			eeprom.sv_serial_number = 482;
			eeprom.servo_startup_delay = 200;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 123);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 122);
			break;
		case 0xF23E:
			eeprom.sv_destination_id = 483;
			eeprom.sv_serial_number = 483;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 121);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 120);
			break;
		case 0x7401:
			eeprom.sv_destination_id = 484;
			eeprom.sv_serial_number = 484;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 118);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 117);
			break;
		case 0x85E:
			eeprom.sv_destination_id = 485;
			eeprom.sv_serial_number = 485;
			eeprom.servo_startup_delay = 100;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 116);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 115);
			break;
		case 0x80EA:
			eeprom.sv_destination_id = 486;
			eeprom.sv_serial_number = 486;
			eeprom.servo_startup_delay = 50;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 113);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 112);
			break;
		case 0xC71F:
			eeprom.sv_destination_id = 487;
			eeprom.sv_serial_number = 487;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 111);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 110);
			break;
		case 0x488:
			eeprom.sv_destination_id = 488;
			eeprom.sv_serial_number = 488;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 114);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 109);
			break;
		case 0x489:
			eeprom.sv_destination_id = 489;
			eeprom.sv_serial_number = 489;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 108);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 107);
			break;
		case 0x490:
			eeprom.sv_destination_id = 490;
			eeprom.sv_serial_number = 490;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 106);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 105);
			break;
#endif
		case 0x6CF3:
			eeprom.sv_destination_id = deviceID;
			eeprom.sv_serial_number = deviceID;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 125);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 124);
			for (index=4;index<16;index++)
			{
				eeprom.ln_gpio_opcode[index][0] = 0;
			}
			return;
			break;
		default:
			return;
	}
	/*
	for (index=4;index<16;index++)
	{
		eeprom.ln_gpio_opcode[index][0] = 0;
	}
	
	eeprom.ln_threshold = 50;
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	*/
}