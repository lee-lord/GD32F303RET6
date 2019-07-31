/*!
    \file  systick.c
    \brief the systick configuration file

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "systick.h"
#include "sysType.h"

volatile static U32 delay;
volatile U32 systemTickMs;
/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
// void SystemTick_Init(U32 PreCLK)
// {
//     RCC_ClocksTypeDef RCC_Clocks;
//     RCC_GetClocksFreq(&RCC_Clocks);
//     //配置10ms中断
// //  SysTick_Config(RCC_Clocks.SYSCLK_Frequency / PreCLK);
// }
/// 1ms 
void systick_config(U32 FrqHz)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / FrqHz)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
    systemTickMs = 0;
    // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    // nvic_irq_enable(SysTick_IRQn, 0, 0);


}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(U32 count)
{
    U64 tmp = (U64)count+(U64)systemTickMs; 
    if(tmp>0xffffffff) 
      {
        delay = tmp-0xffffffff;
        while(systemTickMs != 0xffffffff){} 
        while(systemTickMs != delay){}
      }
    else{
            delay = count+systemTickMs;
            while(systemTickMs != delay){} 
       }
    
}


void delay_us(U32 us)
{
  U32 startTimeus=micros();
  U32 deltaTime=0,tmp=0;
  while(deltaTime>us)
  {
     tmp =micros();
    if((tmp-startTimeus)>0)
      {
            deltaTime = tmp-startTimeus;
      }
      else
      {
           deltaTime = 0xffffffff+tmp-startTimeus;
      }
  }
}

  void Update_SystemTick(){
    systemTickMs += SYSTEMTICK_PERIOD_MS;
}

U32 micros(void)
{
    U32 Fm=0,Lm=0;
    U32 tickValue=0;
    do{
        Fm=systemTickMs;
      tickValue=SysTick->VAL;
        Lm=systemTickMs;
    }while(Fm!=Lm);
    return (1000*(Lm+1)-tickValue/SYSTICK_PRE_US);
}

U32 millis(void)
{
    return systemTickMs;
}
