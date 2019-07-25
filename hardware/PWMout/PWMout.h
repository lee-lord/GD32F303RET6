#ifndef _PWMOUT_H_
#define _PWMOUT_H_
#include "sysType.h"

void PWMoutInitial(void);
void morotrAWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir);

void morotrBWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir);

void morotrCWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir);


#endif

