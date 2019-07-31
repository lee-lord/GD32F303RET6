#include "gd32f30x.h"
#include "gd32f303e_eval.h"
#include "sysType.h"
#include "Timer.h"


void Timerx_Init(U16 freq_hz,U32 Timerx)
{
////////////////////////////////////////////////////////
    timer_parameter_struct timer_initpara;
    U32  cyclesPerMicros=0;
    if(Timerx==TIMER0||Timerx==TIMER7||Timerx==TIMER8||Timerx==TIMER9||Timerx==TIMER10)
       cyclesPerMicros=120;
     else
       cyclesPerMicros=120;
    U32 microseconds = 1000000 / freq_hz; // per period =1000000/500=2000 us
    U32 period_cyc = microseconds * cyclesPerMicros;//CYCLES_PER_MICROSECOND; //=2000*120=240000 system clock cycles per period
    // This picks the smallest prescaler that allows an overflow < 2^16.
    U16 prescaler = (U16)(period_cyc / MAX_OVERFLOW + 1);//240000/65535+1=3.6+1=
    U16 overflow = (U16)(period_cyc / (prescaler));// (prescaler+1)
    U16 _clocks_per_msecon = SystemCoreClock / (prescaler) / 1000;//prescaler+1 //120 000 = 1

    rcu_periph_clock_enable(RCU_TIMER4);
    timer_deinit(TIMER4);
    timer_initpara.prescaler         = prescaler-1;//clk=1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = overflow-1;//microseconds;//10k
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER4,&timer_initpara);
		 timer_auto_reload_shadow_enable(TIMER4);
    timer_update_event_enable(TIMER4);
    timer_interrupt_enable(TIMER4,TIMER_INT_UP);//TIM 中断源
    ///////////////
    // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    // nvic_irq_enable(TIMER4_IRQn, 1, 0); 
    timer_enable(TIMER4);
                             
}

