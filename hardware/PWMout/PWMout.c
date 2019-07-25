#include "gd32f30x.h"
#include "sysType.h"
#include "PWMout.h"

//     GPIO_InitTypeDef GPIO_InitStructure;
// 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
// 	TIM_OCInitTypeDef  TIM_OCInitStructure;
// //////GPIO config
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM2, ENABLE);
	
//  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  |RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
	/* Configure PA8(LED0) as input floating */	 //用于Tim4 TIM3 TIM2的CH2输出的PWM通过该LED显示
	/*GPIOB Configuration: TIM3 channel3*/	  //设置该引脚为复用输出功能,输出TIM3 CH3的PWM脉冲波形
	// GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_0;//|GPIO_PIN_1; //TIM_CH3
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
	// GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_7; //TIM_CH3
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOA, &GPIO_InitStructure);
// ///time base configure
// 	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 32K
// 	TIM_TimeBaseStructure.TIM_Prescaler =0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
// 	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
// 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//TIM_CounterMode_Up;  //TIM向上计数模式
// 	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
//     TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
// 	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
/*
//TIM2 master configure
  	/// Output Compare Active Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = arr; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
//  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	//select Master Slave mode
    TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);
	//Master Mode Selection
	TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_Enable);//TIM_TRGOSource_Update
	
	////slave configure
	//TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	
	//TIM3 slave mode 
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Gated);
	TIM_SelectInputTrigger(TIM3,TIM_TS_ITR1);
	//TIM4 Slavemode
	TIM_SelectSlaveMode(TIM4,TIM_SlaveMode_Gated);
	TIM_SelectInputTrigger(TIM4,TIM_TS_ITR1);
	
	
		  	    TIM3->CR1 |= TIM_CR1_CEN;	
				TIM4->CR1 |= TIM_CR1_CEN;	
	            TIM2->CR1 |= TIM_CR1_CEN;///Timer1 will triger timer2 Timer3	


*/

////MotorA PA0 PA1 PA2 TIMER1
////motorB PB6 PB7 PB8 TIMER3
////motorC PA6 PA7 PB0 TIMER2
void synchronTim123(U16 arr,U16 psc)
{
////////////////////////////////////////////////////////////////////////////////////////////////
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
            
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_0);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_7);	
//////////////////////////////////////////////
    timer_deinit(TIMER1);
    timer_deinit(TIMER2);
	timer_deinit(TIMER3);
    /* TIMER1,3,4 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = arr;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);
    timer_init(TIMER2,&timer_initpara);
    timer_init(TIMER3,&timer_initpara);
//////////////////////////////////////
      /* CH0,CH1 and CH2 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);

        /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,0);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,0);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,0);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);


///////////////////////////timer2
    timer_channel_output_config(TIMER2,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_2,&timer_ocintpara);

        /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_2,0);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
//////////////////////////////////timer3
    timer_channel_output_config(TIMER3,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER3,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER3,TIMER_CH_2,&timer_ocintpara);

        /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_0,0);
    timer_channel_output_mode_config(TIMER3,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_1,0);
    timer_channel_output_mode_config(TIMER3,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_2,0);
    timer_channel_output_mode_config(TIMER3,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);

/////////////////////////////////////master -slave mode 
        /* select the master slave mode  timer1 master mode */
    timer_master_slave_mode_config(TIMER1,TIMER_MASTER_SLAVE_MODE_ENABLE);
    /* TIMER1 update event is used as trigger output */
    timer_master_output_trigger_source_select(TIMER1,TIMER_TRI_OUT_SRC_ENABLE);

     /*timer2 ,timer3 as slave*/
    timer_slave_mode_select(TIMER2,TIMER_SLAVE_MODE_PAUSE);//
    timer_input_trigger_source_select(TIMER2,TIMER_SMCFG_TRGSEL_ITI1);
  
    timer_slave_mode_select(TIMER3,TIMER_SLAVE_MODE_PAUSE);
    timer_input_trigger_source_select(TIMER3,TIMER_SMCFG_TRGSEL_ITI1);

    /* auto-reload preload enable */
    timer_enable(TIMER1);
    timer_enable(TIMER2);
    timer_enable(TIMER3);


}



void PWMoutInitial(void)
{
    synchronTim123(3000,120);
}


void morotrAWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir)
{
	TIMER_CH0CV(TIMER1)=a;
	if(dir==0)
	{ TIMER_CH1CV(TIMER1)=b;
      TIMER_CH2CV(TIMER1)=c;
    }
    else
    {
      TIMER_CH1CV(TIMER1)=c;
      TIMER_CH2CV(TIMER1)=b;
    }
		 
}

void morotrBWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir)
{
	TIMER_CH0CV(TIMER3)=a;
	if(dir==0){
	   TIMER_CH1CV(TIMER3)=b;
       TIMER_CH2CV(TIMER3)=c;
	}
	else
	{
		TIMER_CH1CV(TIMER3)=c;
        TIMER_CH2CV(TIMER3)=b;
	}
	
}

void morotrCWrite(uint16_t a,uint16_t b,uint16_t c,uint16_t dir)
{
	TIMER_CH0CV(TIMER2)=a;
	if(dir==0){
		TIMER_CH1CV(TIMER2)=b;
        TIMER_CH2CV(TIMER2)=c;
	}
	else
	{
		TIMER_CH1CV(TIMER2)=c;
        TIMER_CH2CV(TIMER2)=b;
	}
	
}
