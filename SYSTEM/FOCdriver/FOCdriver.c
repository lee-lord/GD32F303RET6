#include "gd32f30x.h"
#include "sysType.h"
#include "PWMout.h"
#include "PWM_PPM_capture.h"
#include "TLE5012B.h"


void foc_initialise(void)
{



}


//////////update isr of timer4 to trige the foc motors
void TIMER4_IRQHandler(void)   //TIM5中断
{
    if (timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
        {
        timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP  );  //清除TIMx的中断待处理位:TIM 中断源 
        /* Pin PD.02 toggling with frequency = 10KHz */
        //gpio_bit_write(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_PIN_13)));
        TOGGLE_GPIO(GPIOC,GPIO_PIN_13);
       // gd_eval_led_toggle(LED2);
        // here we will call the FOC driver  at 10Khz
        }
}

