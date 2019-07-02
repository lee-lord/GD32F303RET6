#ifndef _SYS_TICK_H_
#define _SYS_TICK_H_
#include "sys.h"


#define SYSTICK_PRE_MS 1000
#define SYSTICK_PER_US 1000000
extern volatile uint32_t systemTickMs;

void SystemTick_Init(u16 PreCLK);
void Update_SystemTick(void);

#endif
