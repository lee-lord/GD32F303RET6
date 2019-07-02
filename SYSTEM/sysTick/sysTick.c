#include "sysTick.h"
#include "STM_GCS.h"
#include "PMU.h"
#define SYSTEMTICK_PERIOD_MS       1               //1msÒ»´ÎsystemTick

volatile uint32_t systemTickMs;

void SystemTick_Init(u16 PreCLK)
{
	RCC_ClocksTypeDef RCC_Clocks;
	
	RCC_GetClocksFreq(&RCC_Clocks);
	//ÅäÖÃ10msÖÐ¶Ï
  SysTick_Config(108000000 / PreCLK);
	NVIC_SetPriority (SysTick_IRQn, 14); //lowest Priority
	systemTickMs = 0;
}


__inline void Update_SystemTick(){
	static u32 lastTime=0;
	systemTickMs += SYSTEMTICK_PERIOD_MS;
	Buzzer_interrupt();
	if(Binding_flag==1)
	{
		if(lastTime==0) lastTime=systemTickMs;
		if(systemTickMs-lastTime > 120000) Binding_flag=0;
	}
		
}
