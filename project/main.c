/*!
    \file  main.c
    \brief led spark with systick, USART print and key example

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
#include <stdio.h>
#include "main.h"
#include "gd32f303e_eval.h"
#include "hardware.h"
#include "TLE5012B.h"
#include "FOCdriver.h"
#include "flash_para.h"
/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED2);
        }else{
            gd_eval_led_off(LED2);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}
void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER7_Channel_IRQn, 1, 2);//pwm in capture
    /////////////////////////////////
    nvic_irq_enable(DMA0_Channel0_IRQn,0,2);//for ADC_dma
    // nvic_irq_enable(DMA0_Channel1_IRQn, 1, 1);
    // nvic_irq_enable(DMA0_Channel3_IRQn,1,0);

    /// timer4 ISR at 10Khz
    nvic_irq_enable(TIMER4_IRQn, 0, 1); 
    ////systemtick
    nvic_irq_enable(SysTick_IRQn, 0, 0);

    

}


void gloable_led_initial(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13|GPIO_PIN_7);
}
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
U32 tmp=0,tmp2;
int main(void)
{  float i=0,a,b;
    U16 dataLen=0,datalen2=0;
    
    systick_config(1000);
    nvic_config();
    gloable_led_initial();
    PORTn_BITx_OUT(GPIOC,13)=0;// gd_eval_led_init(LED2);
    PORTn_BITx_OUT(GPIOC,7)=0;
    gloable_led_initial();//gd_eval_led_init(LED3);
    UartC_DmaInitial(USART2);
    UartA_DmaInitial(USART0);
    adc_dma_config();
	  delay_1ms(10);
    UartA_write("system start now \r\n",19);//UAprintf("system start now \r\n");
    PWM_PPM_inInitial();
    /////////////////////
    FOC_control_initialise();

    tmp = micros();
    PORTn_BITx_OUT(GPIOC,13)=1;// gd_eval_led_init(LED2);
    PORTn_BITx_OUT(GPIOC,7)=1;
    tmp2 = micros();

    while(1){
        /* turn on led2, turn off led5 */
        TOGGLE_GPIOx_PIN(GPIOC,LEDA);
        delay_1ms(10);
        /* turn on led3, turn off led2 */
        TOGGLE_GPIOx_PIN(GPIOC,LEDB);
        TOGGLE_GPIOx_PIN(GPIOC,LEDA);
        delay_1ms(10);
        TOGGLE_GPIOx_PIN(GPIOC,LEDB);
        delay_1ms(10);
        i=readAnglesDegreesB(1);
        b=readAnglesDegrees(1);
        a=i+1;
        
        dataLen = UartC_Available();
        datalen2 = UartA_Available();

///////////////////main function
        //////SPWMF4_FeedBack_test_noBlock();

		 UCprintf("i=%5.2f,b=%5.2f,time=%dus,Adc=%d\r\n",i,b,tmp2-tmp,adc_value);// ,Uart2Add=%x,,&(USART_DATA(USART2))
         UAprintf("i=%5.2f,b=%5.2f,time=%dus,Adc=%d\r\n",i,b,tmp2-tmp,adc_value);
          tmp = micros();
        
       if(datalen2>0)
          {
            for(U16 m=0;m<datalen2;m++)
            {      if(dataLen)
                     tmp2 = UartC_read();
					if(datalen2)
					  tmp2 = UartA_read();							
			}
					 tmp2 = micros();
           // UCprintf("i=%f,a=%f time=%d us\r\n",i,b,tmp2);// ,Uart2Add=%x,,&(USART_DATA(USART2))
            tmp2 = micros();
						UAprintf("PWM1=%d,PWM2=%d time=%d us,Frequs=%d\r\n",PWM1Value,PWM2Value,tmp2-tmp,Frequs);
          }
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    // usart_data_transmit(EVAL_COM2, (uint8_t)ch);
    // while(RESET == usart_flag_get(EVAL_COM2, USART_FLAG_TBE));
     U8 data=(uint8_t)ch;
     UartC_write(&data,1);
    return ch;
}
