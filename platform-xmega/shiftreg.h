#ifndef __SHIFTREG_H__
#define __SHIFTREG_H__

#define SHIFTREG_PORT	PORTC.OUT
#define	SHIFTREG_DDR	PORTC.DIR
#define	SHIFTREG_RCK	(1<<3)
#define	SHIFTREG_SCK	(1<<7)
#define SHIFTREG_DIN	(1<<5)
#define	SHIFTREG_OE		(1<<1)
#define SHIFTREG_SS		(1<<4)

#define nop() asm volatile("nop")
#define shiftreg_enable()	PORTC.DIRSET = SHIFTREG_OE
#define shiftreg_disable()	PORTC.DIRCLR = SHIFTREG_OE

#define shiftreg_rck_set()		PORTC.OUTSET = SHIFTREG_RCK
#define shiftreg_rck_clear()	PORTC.OUTCLR = SHIFTREG_RCK


void shiftreg_init(void);
void shiftreg_out(uint8_t *data, uint8_t len, uint8_t assertRegisterClock);
void shiftreg_out16(uint16_t data, uint8_t assertRegisterClock);
void shiftreg_shutdown(void);

#endif
