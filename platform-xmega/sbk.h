#ifndef _SBK_H
#define _SBK_H

void sbk_init(void);
void sbk_update_track(void);
void sbk_in_abort(void);
void sbk_out_abort(void);

extern volatile uint8_t sbk_mode;
extern volatile uint8_t sbk_in_track;
extern volatile uint8_t sbk_out_track;

#define SBK_MODE_NORMAL	0
#define SBK_MODE_FSS	1

#define RELAIS_G3_OUT	2
#define RELAIS_G3_IN	3
#define RELAIS_G2_OUT	4
#define RELAIS_G2_IN	5
#define RELAIS_G1_IN	6
#define RELAIS_G1_OUT	7
#define RELAIS_G4_OUT	8
#define RELAIS_G4_IN	9
#define RELAIS_FSS		11
#define RELAIS_G6_OUT	12
#define RELAIS_G6_IN	13
#define RELAIS_G5_IN	14
#define RELAIS_G5_OUT	15

#endif
