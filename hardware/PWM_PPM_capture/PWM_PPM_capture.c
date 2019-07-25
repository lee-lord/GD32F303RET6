#include "gd32f30x.h"
#include "sysType.h"
#include "PWM_PPM_capture.h"
volatile uint16_t PWM1Value = 0, PWM2Value = 0;

void PWM_PPM_inInitial(void)
{
   timer_captur_configuration();// pwm in capture PC6 PC8 as PWM input pin
   // PPM in cature
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_captur_configuration(void)
{
    /* TIMER7 configuration: input capture mode -------------------
    the external signal is connected to TIMER7 CH0 pin (PB4)
    the rising edge is used as active edge
    the TIMER7 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    

    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER7);

    /*configure PA6 PB8 (TIMER7 CH0 ch2) as alternate function*/
    gpio_init(GPIOC,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_6|GPIO_PIN_8);

    timer_deinit(TIMER7);

    /* TIMER7 configuration */
    timer_initpara.prescaler         = 119;//1Mhz counter @1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 65535;//full duty 65.535ms
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER7,&timer_initpara);

    /* TIMER7  configuration */
    /* TIMER7 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER7,TIMER_CH_0,&timer_icinitpara);
    timer_input_capture_config(TIMER7,TIMER_CH_2,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER7);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER7,TIMER_INT_FLAG_CH0|TIMER_INT_FLAG_CH2);
    /* channel 0 2 interrupt enable */
    timer_interrupt_enable(TIMER7,TIMER_INT_CH0|TIMER_INT_CH2);


    /* TIMER7 counter enable */
    timer_enable(TIMER7);

    // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    // nvic_irq_enable(TIMER7_Channel_IRQn, 1, 1);

}

void timer_capture_polarity_set(uint32_t timer_periph,U16 channel,U16 polarity)
{
    switch(channel)
     {
       case TIMER_CH_0:
                   /* reset the CH0P and CH0NP bits */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            TIMER_CHCTL2(timer_periph) |= (uint32_t)(polarity);
       break;
       case TIMER_CH_1:
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(polarity) << 4U);
       break;       
       case TIMER_CH_2:
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH2P | TIMER_CHCTL2_CH2NP));
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(polarity) << 8U);
       break;
       case TIMER_CH_3:
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH3P | TIMER_CHCTL2_CH3P));
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(polarity) << 12U);
       break;       
     }
        

}





///////////////pwm input capture.
void TIMER7_Channel_IRQHandler(void)
{
    static U16 lastPwm1=0,lastPwm2=0;
    static U8 capflag1=0,capflag2=0;
    U16 tmpCunt=0;
    if(SET == timer_interrupt_flag_get(TIMER7,TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        tmpCunt = TIMER_CH0CV(TIMER7);
        timer_interrupt_flag_clear(TIMER7,TIMER_INT_FLAG_CH0);        
        if(capflag1==0)
        {
            capflag1=1;
           lastPwm1 = tmpCunt;
        }
        else if(capflag1==1)
        {
            capflag1=0;
            if(tmpCunt>lastPwm1)
                PWM1Value = tmpCunt-lastPwm1;
              else
                PWM1Value = 0xFFFFU- lastPwm1 +tmpCunt;
        }

       TIMER_CHCTL2(TIMER7) ^= (uint32_t)(TIMER_IC_POLARITY_FALLING);//invert the polarity
    }
    
    if(SET == timer_interrupt_flag_get(TIMER7,TIMER_INT_FLAG_CH2)){
        /* clear channel 0 interrupt bit */
        tmpCunt = TIMER_CH2CV(TIMER7);// 
        timer_interrupt_flag_clear(TIMER7,TIMER_INT_FLAG_CH2);
        if(capflag2==0)
        {
            capflag2=1;
           lastPwm2 = tmpCunt;
        }
        else if(capflag2==1)
        {
            capflag2=0;
            if(tmpCunt>lastPwm2)
                PWM2Value = tmpCunt-lastPwm2;
              else
                PWM2Value = 0xFFFFU- lastPwm2 +tmpCunt;
        }

       TIMER_CHCTL2(TIMER7) ^= (U32)((uint32_t)(TIMER_IC_POLARITY_FALLING)<<8);//invert the polarity
    }
}
