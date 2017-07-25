/*
 * utils.c
 *
 * Created: 31.05.2014 16:35:03
 *  Author: manu
 */ 

#include <stddef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sys/utils.h"

#if 1
#include <util/crc16.h>
uint16_t getID16(void)
{
	uint8_t address, temp8;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		temp8 = pgm_read_byte(address);
		id = _crc_xmodem_update(id, temp8);
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	return id;
}

#else
uint16_t getID16(void)
{
	uint8_t address, temp8;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	CRC.CTRL = CRC_RESET0_bm | CRC_SOURCE_IO_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		temp8 = pgm_read_byte(address);
		CRC.DATAIN = temp8;
		id += temp8;
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	id = (CRC.CHECKSUM1<<8) + CRC.CHECKSUM0;
	return id;
}
#endif
