#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "shiftreg.h"

void shiftreg_shutdown(void)
{

}

volatile uint8_t shiftreg_isr;

void shiftreg_init(void)
{
	uint8_t index;
	SHIFTREG_DDR = SHIFTREG_SCK|SHIFTREG_DIN|SHIFTREG_SS|SHIFTREG_RCK;
	SHIFTREG_PORT = 0;

	// SPI: Enable + Master + Mode 0 + Prescaler 128
	SPIC.CTRL = SPI_MODE_0_gc | SPI_PRESCALER_DIV128_gc | SPI_ENABLE_bm | SPI_MASTER_bm;
	SPIC.INTCTRL = SPI_INTLVL_LO_gc;
	
	for (index=0;index<4;index++)
	{
		shiftreg_isr = 0;
		SPIC.DATA = 0;
		while (shiftreg_isr==0);
	}
    
	shiftreg_rck_set();
	
	// Enable outputs
	shiftreg_enable();
}

void shiftreg_out(uint8_t *data, uint8_t len, uint8_t assertRegisterClock)
{
	shiftreg_rck_clear();
	
	while (len--)
	{
		shiftreg_isr = 0;
		
		SPIC.DATA = data[len];

		while (shiftreg_isr==0);
	}
	
	if (assertRegisterClock)
	{
		shiftreg_rck_set();		
	}
}

void shiftreg_out16(uint16_t data, uint8_t assertRegisterClock)
{
	shiftreg_rck_clear();	
	
	shiftreg_isr = 0;
	SPIC.DATA = (uint8_t) (data>>8)&0xFF;
	while (shiftreg_isr==0);

	shiftreg_isr = 0;
	SPIC.DATA = (uint8_t) data&0xFF;
	while (shiftreg_isr==0);
	
	if (assertRegisterClock)
	{
		shiftreg_rck_set();
	}
}

ISR(SPIC_INT_vect)
{
	shiftreg_isr = 1;
}