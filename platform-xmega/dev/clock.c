/*
 * Copyright (c) 2012, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
 /**
 *  \brief This module contains AVRXMEGA-specific code to implement
 *  the Contiki core clock functions.
 *  
 *  \author Manuel Vetterli <manuel.vetterli@gmail.com> and others.
 *
*/
/** \addtogroup avr
 * @{
 */
 /**
 *  \defgroup avrclock AVRXMEGA clock implementation
 * @{
 */
/**
 *  \file
 *  This file contains AVRXMEGA-specific code to implement the Contiki core clock functions.
 *
 */
/**
 * These routines define the AVR-specific calls declared in /core/sys/clock.h
 * CLOCK_SECOND is the number of ticks per second.
 * It is defined through CONF_CLOCK_SECOND in the contiki-conf.h for each platform.
 * The usual AVR defaults are 128 or 125 ticks per second, counting a prescaled CPU clock
 * using the 8 bit timer0.
 * 
 * clock_time_t is usually declared by the platform as an unsigned 16 bit data type,
 * thus intervals up to 512 or 524 seconds can be measured with ~8 millisecond precision.
 * For longer intervals the 32 bit clock_seconds() is available.
 * 
 * Since a carry to a higer byte can occur during an interrupt, declaring them non-static
 * for direct examination can cause occasional time reversals!
 *
 * clock-avr.h contains the specific setup code for each mcu.
 */
#include "sys/clock.h"
#include "dev/clock-avr.h"
#include "sys/etimer.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "port.h"

/* Two tick counters avoid a software divide when CLOCK_SECOND is not a power of two. */
#if CLOCK_SECOND && (CLOCK_SECOND - 1)
#define TWO_COUNTERS 1
#endif

/* count is usually a 16 bit variable, although the platform can declare it otherwise */
static volatile clock_time_t count;
#if TWO_COUNTERS
/* scount is the 8 bit counter that counts ticks modulo CLOCK_SECONDS */
static volatile clock_time_t scount;
#endif
/* seconds is available globally but non-atomic update during interrupt can cause time reversals */
volatile unsigned long seconds;
/* sleepseconds is the number of seconds sleeping since startup, available globally */
long sleepseconds;

#define MAX_TICKS (~((clock_time_t)0) / 2)

/*---------------------------------------------------------------------------*/
/**
 * Start the clock by enabling the timer comparison interrupts. 
 */
void
clock_init(void)
{
  //cli ();
  OCRSetup();
  //sei ();
}
/*---------------------------------------------------------------------------*/
/**
 * Return the tick counter. When 16 bit it typically wraps every 10 minutes.
 * The comparison avoids the need to disable clock interrupts for an atomic
 * read of the multi-byte variable.
 */
clock_time_t
clock_time(void)
{
  clock_time_t tmp;
  do {
    tmp = count;
  } while(tmp != count);
  return tmp;
}
/*---------------------------------------------------------------------------*/
/**
 * Return seconds, default is time since startup.
 * The comparison avoids the need to disable clock interrupts for an atomic
 * read of the four-byte variable.
 */
unsigned long
clock_seconds(void)
{
  unsigned long tmp;
  do {
    tmp = seconds;
  } while(tmp != seconds);
  return tmp;
}
/*---------------------------------------------------------------------------*/
/**
 * Set seconds, e.g. to a standard epoch for an absolute date/time.
 */
void
clock_set_seconds(unsigned long sec)
{
  seconds = sec;
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a number of clock ticks.
 */
void
clock_wait(clock_time_t t)
{
  clock_time_t endticks = clock_time() + t;
  if (sizeof(clock_time_t) == 1) {
    while ((signed char )(clock_time() - endticks) < 0) {;}
  } else if (sizeof(clock_time_t) == 2) {
    while ((signed short)(clock_time() - endticks) < 0) {;}
  } else {
    while ((signed long )(clock_time() - endticks) < 0) {;}
  }
}


/*---------------------------------------------------------------------------*/
/**
 * Adjust the system current clock time.
 * \param dt   How many ticks to add
 *
 * Typically used to add ticks after an MCU sleep
 * clock_seconds will increment if necessary to reflect the tick addition.
  * Leap ticks or seconds can (rarely) be introduced if the ISR is not blocked.
 */
void
clock_adjust_ticks(clock_time_t howmany)
{
  uint8_t sreg = SREG;cli();
  count  += howmany;
#if TWO_COUNTERS
  howmany+= scount;
#endif
  while(howmany >= CLOCK_SECOND) {
    howmany -= CLOCK_SECOND;
    seconds++;
    sleepseconds++;
  }
#if TWO_COUNTERS
  scount = howmany;
#endif
  SREG=sreg;
}
/*---------------------------------------------------------------------------*/
/* This it the timer comparison match interrupt.
 * It maintains the tick counter, clock_seconds, and etimer updates.
 *
 * If the interrupts derive from an external crystal, the CPU instruction
 * clock can optionally be phase locked to it. This allows accurate rtimer
 * interrupts for strobe detection during radio duty cycling.
 * Phase lock is accomplished by adjusting OSCCAL based on the phase error
 * since the last interrupt.
 */
/*---------------------------------------------------------------------------*/
#if defined(DOXYGEN)
/** \brief ISR for the TIMER0 or TIMER2 interrupt as defined in
 *  clock-avr.h for the particular MCU.
 */
void AVR_OUTPUT_COMPARE_INT(void);
#else
ISR(AVR_OUTPUT_COMPARE_INT)
{
	static uint8_t subcount = 0;
	TCC1.CCB += CLOCK_PERVAL/4;
	
	subcount++;
	
	if ((subcount%4)!=0)
		return;
	
    count++;
#if TWO_COUNTERS
  if(++scount >= CLOCK_SECOND) {
    scount = 0;
#else
  if(count%CLOCK_SECOND==0) {
#endif
    seconds++;
  }
 
/*  gcc will save all registers on the stack if an external routine is called */
  if(etimer_pending() &&
    (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
	etimer_request_poll();
  }
  
#if 0
  if(etimer_pending()) {
    etimer_request_poll();
  }
#endif

}
#endif /* defined(DOXYGEN) */
/** @} */
/** @} */
