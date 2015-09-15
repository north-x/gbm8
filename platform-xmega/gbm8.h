/*
 * Copyright (c) 2015, Manuel Vetterli
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

#ifndef GBM8_H_
#define GBM8_H_

#define MAP_BITS(SRC_REG, DEST_REG, SRC_BIT, DEST_BIT) if (SRC_REG&(1<<(SRC_BIT))) DEST_REG |= (1<<(DEST_BIT)); else DEST_REG &= ~(1<<(DEST_BIT))

/************************************************************************/
/* Config Flags                                                         */
/************************************************************************/
#define GBM_MODE_INIT 0
#define GBM_MODE_NORMAL	1
#define GBM_MODE_SBK 2
#define GBM_MODE_FSZ 3



/************************************************************************/
/* Function Prototypes                                                  */
/************************************************************************/
void update_gbm_mode(void);
void gbm8_hw_detect(void);

extern uint8_t gbm_mode;
extern uint8_t gbm_register_filt;
extern uint8_t gbm_register_filt_filt;

#endif /* GBM8_H_ */