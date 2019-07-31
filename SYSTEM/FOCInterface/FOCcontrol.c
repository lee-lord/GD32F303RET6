#include "gd32f30x.h"
#include "sysType.h"
#include "UART_DMA.h"
#include "PWMout.h"
#include "MyMath.h"
#include "PWM_PPM_capture.h"
#include "TLE5012B.h"
#include "systick.h"
#include "FOCdriver.h"
#include "flash_para.h"
#include "FOCcontrol.h"

volatile float tangle=0.0f,tangleB=0.0f;
U8 dataBuf[32],dataBufB[32];
volatile U16 SVPWM_flag_on;

//////////update isr of timer4 to trige the foc motors
void TIMER4_IRQHandler(void)   //TIM5中断
{
	static U32 lastTimer=0;
	U32 tmp;
	float tmpAngle=0.0f;
    if (timer_interrupt_flag_get(TIMER4, TIMER_INT_FLAG_UP) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
        {
        timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_UP  );  //清除TIMx的中断待处理位:TIM 中断源 
        
        tmp = micros();
        Frequs = tmp-lastTimer;
        lastTimer = tmp;
        SVPWM_flag_on=1; 

        TOGGLE_GPIOx_PIN(GPIOC,GPIO_PIN_13);
        ///tle5012 read 
        tmpAngle=readAnglesDegrees(HALFscale);
        motors.SVPWM_F4_DQ_speed2PostionA(tangle+MotorA_MachineOffset,tmpAngle);
        ///tle5012 read 
        tmpAngle=readAnglesDegreesB(HALFscale);
        motors.SVPWM_F4_DQ_speed2PostionB(tangleB+MotorB_MachineOffset,tmpAngle);
        // here we will call the FOC driver  at 10Khz
        }
}
#define PWM_CLK_HZ 30000

void FOC_control_initialise(void)
{
    ////initialise focdriver 
    foc_initialise(PWM_CLK_HZ);//30Khz PWM
    //////
    Timerx_Init(PWM_CLK_HZ/3,TIMER4);//10Khz will start the motor.
////////////if calibration had finished ,we can run normal mode .
///////////else we will come in the calibration mode 
   if(Tle5012b_para.caliFlagbits==0x000f)
      timer_enable(TIMER4); // Start the timer counting
    else
      {
        timer_disable(TIMER4);
        motors.output_min();
       }

    motorSixStep_offset_caliA();

    motorSixStep_offset_caliB();
}

void Test_Motor_AB(void)
{float tmpA,tmpB,lastA=0,lastB=0,deltaA=0,deltaB=0,baseLineA=0.0f,baseLineB=0.0f;
  U8 first=0,stopA=0,stopB=0;
  while(1){
    for(float i=0;i<360; i+=1.0f)
      {
        if(stopA!=1)
         motors.WriteMotorDirect(i, 0.45f,M_CCW,MTA);
        if(stopB!=1)
         motors.WriteMotorDirect(i, 0.45f,M_CCW,MTB);
         delay_1ms(4);
         tmpA  = readAnglesDegrees(HALFscale);
         tmpB  = readAnglesDegreesB(HALFscale);
         if(first==0) 
           {
             first=1;
             lastA =tmpA;
             lastB =tmpB;
           }
           else
           {
             deltaA=tmpA-lastA;
             deltaB=tmpB-lastB;
             if(fabs(deltaA)<0.2f)
              {
                baseLineA = tmpA;
              }
              if(fabs(deltaB)<0.2f)
              {
                baseLineB = tmpB;
              }
             lastA =tmpA;
             lastB =tmpB;
            // deltaA=tmpA-baseLineA;
            // deltaB=tmpB-baseLineB;
             //if(fabs(deltaA)>4) stopA=1;
             //if(fabs(deltaB)>4) stopB=1;

           }
        ANO_DT_Send_Status(1,deltaA,tmpA,deltaB,(tmpB)*100,1,1);   
        // UAprintf("angleA[%f]:%f\r\n",i,tmp);
      }
    }
  while(1){
    for(float i=0;i<360; i+=0.2f)
      {
         motors.WriteMotorDirect(i, 0.45f,1,1);
         delay_1ms(10);
         tmpB = readAnglesDegreesB(0);
          ANO_DT_Send_Status(1,tmpA,tmpB,tangle,(tmpA)*100,1,1);  
         //UAprintf("angleB[%f]:%f\r\n",i,tmpB);
      }
  }
}

void SearchBlockPositionAndFindDirection(U8 CW_or_CCW,U8 motorsNum)
{
  U16 j=0,k=0,BlockCunt=0;
  S16 dcunt=0;
  float ix,angle,lastAngle;
    for(j=0;j<16;j++)
     for(ix=0.0f,k=0; ix< 360.0f;ix+=0.9f, k++) //-->0.01 degree
       {
          motors.WriteMotorDirect(ix, 0.55f,CW_or_CCW,motorsNum);
          delay_1ms(3);
          if(motorsNum==MTB)
           angle = readAnglesDegreesB(FULLscale);
           else
            angle = readAnglesDegrees(FULLscale);
          if((j==0)&&(k==0))//first 
            lastAngle = angle;
           else
           {
            if(fabs(angle-lastAngle)<0.04) BlockCunt++;
              else   BlockCunt=0;
  
            if((BlockCunt>10)&&(motorsNum==MTB))   break;
              else if((BlockCunt>30)&&(motorsNum==MTA)) break;
            lastAngle = angle;
           }
          UAprintf("Angle=%f,delta=%f,cunt=%d\r\n",angle,angle-lastAngle,BlockCunt);
       }
//////first run a sector 
     for(j=0;j<2;j++)
     for(ix=0.0f,k=0; ix< 360.0f;ix+=1.0f, k++) //-->0.01 degree
       {
          motors.WriteMotorDirect(ix, 0.55f,!CW_or_CCW,motorsNum);
          delay_1ms(2);
       }
/////then run back and find the direction of wire
    for(j=0;j<2;j++)
     for (ix=0.0f,k=0; ix< 360.0f; k++) //-->0.01 degree
       {
          motors.WriteMotorDirect(ix, 0.55f,CW_or_CCW,motorsNum);
          if(motorsNum==MTB)
             angle = readAnglesDegreesB(FULLscale);
           else
             angle = readAnglesDegrees(FULLscale);
          if(angle<lastAngle)
             ix+=1.0f;
          delay_1ms(1);
          if(j>0)
            {
               if((angle-lastAngle)<-0.1f)    dcunt--;
                else if((angle-lastAngle)>0.1f) dcunt++;  
            }
           lastAngle=angle; 
       }
    if(dcunt>0)
      if(motorsNum==MTB)
          motors.dirB=0;
       else
          motors.dirA=1;
    else  
     {
      if(motorsNum==MTB)
         motors.dirB =1;// set direction of wiring
       else
         motors.dirA =0;
     }
      
/////
    UAprintf("directon=%d,cunt=%d cw=%d,ccw=%d\r\n",motors.dirB,dcunt,CW_or_CCW,!CW_or_CCW);  
}

void motorSixStep_offset_caliA(void)
{
    U16 j=0,k=0,dragCunt=0;
    U16 i=0;
    float angle=0,avg=0,lastAngle=0,eAngle,AngleOffset,deltaAngle=0.0f,ix;
    S16 dcunt=0;
    if((Tle5012b_para.caliFlagbits&MTA_EOFF_MASK)==MTA_EOFF_MASK)
       {
        UAprintf("MTA_eAngOff=%f\r\n",motors.elect_machion_offsetA);
        return;
       }
            //motors.SetUpZeroPosition(1,0,0);// lock position of stator.
    delay_1ms(50);//wait for stablise
    angle = readAnglesDegrees(FULLscale);
    UAprintf("start angle:%f\r\n",angle);
////goto the block position by detect the deltaAngle=angle-lastAngle
    SearchBlockPositionAndFindDirection(M_CW,MTA);//M_CCW
 ////////////fist we need to get the right direction of motor 
    dcunt=0;
    lastAngle=0; 
   for(j=0;j<5;j++)//7 poles
    {
      for (i=1,k=0; i< 7; i++,k++) //-->0.01 degree
       {
        motors.SetUpZeroPosition(i,motors.dirA,MTA);//motors.WriteMotorDirect(i, 0.45f,0);
        delay_1ms(300);
        angle = readAnglesDegrees(FULLscale);
        // if(motors.dirB==0)
        //   {
        //     angle =360.0f-angle;
        //   }
           if(dragCunt>1)
           {  
                deltaAngle = angle-lastAngle;
                eAngle = angle;
                if(deltaAngle<0.0001f) deltaAngle = deltaAngle+360.0f;
               UAprintf("angle[%d]:%f Vs deltaE=%f ",j,angle,deltaAngle);

                while(eAngle>=51.4285714f) eAngle  -=51.4285714f;
                AngleOffset =eAngle*7-60.0f*k;

                if(AngleOffset<0.0) AngleOffset +=360.0f;
                if((deltaAngle>7.8f) && (deltaAngle<10.2f))
                  {
                   avg =avg + AngleOffset;
                   dcunt++;
                  }
               UAprintf(" electriAngleOff[%d]=%f,dcunt=%d\r\n",k,(AngleOffset),dcunt);  
          }
          lastAngle=angle; 
         dragCunt++; 
        }
      TOGGLE_GPIOx_PIN(GPIOC,GPIO_PIN_13);//hal.gpio->toggle(LED1);
      TOGGLE_GPIOx_PIN(GPIOC,GPIO_PIN_7);//hal.gpio->toggle(LED2);
    }
    if(dcunt==0) return;
    UAprintf("AVG_eAngOff=%f\r\n",avg/dcunt); 
    motors.elect_machion_offsetA = (avg/dcunt);
    ////////follow is a resault of testing .
    ///get the offset betwwen electriAngle and machineAgle
    motors.elect_machion_offsetA = (180.0f-motors.elect_machion_offsetA)*DEG_TO_RAD;//the offset of eangle in radin
    
    Tle5012b_para.MotorA.direction = motors.dirA;
    Tle5012b_para.MotorA.Elec_offset=motors.elect_machion_offsetA;
    Tle5012b_para.caliFlagbits|=MTA_EOFF_MASK;
    UAprintf("Tle5012b_para.MotorA.Elec_offset=%f,motors.offsetA=%f\r\n",Tle5012b_para.MotorA.Elec_offset,motors.elect_machion_offsetA);

    parametersSave();
}

void motorSixStep_offset_caliB(void)
{
    U16 j=0,k=0,dragCunt=0;
    U16 i=0;
    float angle=0,avg=0,lastAngle=0,eAngle,AngleOffset,deltaAngle=0.0f,ix;
    S16 dcunt=0;
    if((Tle5012b_para.caliFlagbits&MTB_EOFF_MASK)==MTB_EOFF_MASK)
       {
        UAprintf("MTB_eAngOff=%f\r\n",motors.elect_machion_offsetB);
        return;
       }
        
    //motors.SetUpZeroPosition(1,0,0);// lock position of stator.
    delay_1ms(500);//wait for stablise
    angle = readAnglesDegreesB(FULLscale);
    UAprintf("start angle:%f\r\n",angle);
////goto the block position by detect the deltaAngle=angle-lastAngle
    SearchBlockPositionAndFindDirection(M_CCW,MTB);//M_CCW
 ////////////fist we need to get the right direction of motor 
    dcunt=0;
    lastAngle=0; 
   for(j=0;j<5;j++)//7 poles
    {
      for (i=1,k=0; i< 7; i++,k++) //-->0.01 degree
       {
        motors.SetUpZeroPosition(i,motors.dirB,MTB);//motors.WriteMotorDirect(i, 0.45f,0);
        delay_1ms(300);
        angle = readAnglesDegreesB(FULLscale);
        // if(motors.dirB==0)
        //   {
        //     angle =360.0f-angle;
        //   }
           if(dragCunt>1)
           {  
                deltaAngle = angle-lastAngle;
                eAngle = angle;
                if(deltaAngle<0.0001f) deltaAngle = deltaAngle+360.0f;
               UAprintf("angle[%d]:%f Vs deltaE=%f ",j,angle,deltaAngle);

                while(eAngle>=51.4285714f) eAngle  -=51.4285714f;
                AngleOffset =eAngle*7-60.0f*k;///

                if(AngleOffset<0.0) AngleOffset +=360.0f;
                if((deltaAngle>7.8f) && (deltaAngle<10.2f))
                  {
                   avg =avg + AngleOffset;
                   dcunt++;
                  }


               UAprintf(" electriAngleOff[%d]=%f,dcunt=%d\r\n",k,(AngleOffset),dcunt);  
          }
          lastAngle=angle; 
         dragCunt++; 
        }
     TOGGLE_GPIOx_PIN(GPIOC,GPIO_PIN_13);// hal.gpio->toggle(LED1);
     TOGGLE_GPIOx_PIN(GPIOC,GPIO_PIN_7);// hal.gpio->toggle(LED2);
    }

    if(dcunt==0) return;

    UAprintf("AVG_eAngOff=%f\r\n",avg/dcunt); 
    motors.elect_machion_offsetB = (avg/dcunt);
    ////////follow is a resault of testing .
    ///get the offset betwwen electriAngle and machineAgle
    motors.elect_machion_offsetB = (180.0f-motors.elect_machion_offsetB)*DEG_TO_RAD;//the offset of eangle in radin
    ///////////save parameters
    Tle5012b_para.MotorB.direction = motors.dirB;
    Tle5012b_para.MotorB.Elec_offset=motors.elect_machion_offsetB;
    Tle5012b_para.caliFlagbits|=MTB_EOFF_MASK;
    UAprintf("Tle5012b_para.MotorB.Elec_offset=%f,motors.offsetB=%f\r\n",Tle5012b_para.MotorB.Elec_offset,motors.elect_machion_offsetB);

    parametersSave();
}



//////////put the gimble to a fiture and send a cmd by uart then we will start to get the offset

void getMachine_A_B_offset(void)
{
   float TempA=0.0f,TempB=0.0f,tmp1,tmp2;
   U16 i=0;
   timer_disable(TIMER4);//_failsafe_timer.pause();//pause the timer
   //reset  offset value
    //offsetA=0.0f;//Tle5012b_para.MotorA.Elec_offset;
    //offsetB=0.0f;//Tle5012b_para.MotorB.Elec_offset;
    motors.output_min();// disable the motor driver
   for(i=0;i<360;i++)
   {
       tmp1=readAnglesDegrees(HALFscale);
       tmp2=readAnglesDegreesB(HALFscale);
       TempA  += tmp1;
       TempB  += tmp2;

       delay_1ms(2);
      // ANO_DT_Send_Status(1,tmp1,tmp2,tangle,(tmp1)*100,1,1);  
   }
   TempA = TempA/360.0f;
   TempB = TempB/360.0f;
  // offsetA=TempA;//
   Tle5012b_para.MotorA.MachineOffset=TempA;
  /// offsetB=TempB;//
    Tle5012b_para.MotorB.MachineOffset=TempB;

  MotorA_MachineOffset=Tle5012b_para.MotorA.MachineOffset;
  MotorB_MachineOffset=Tle5012b_para.MotorB.MachineOffset;
  // Tle5012b_para.caliFlagbits++;
  ///save parmeters
  Tle5012b_para.caliFlagbits|=MTB_OFF_MASK;
  Tle5012b_para.caliFlagbits|=MTA_OFF_MASK;

  UAprintf("MtA_MachineOffset=%f,MtB_MachineOffset=%f\r\n",MotorA_MachineOffset,MotorB_MachineOffset);
  parametersSave();//hal.storage->write_block(0, (void *)(&Tle5012b_para), sizeof(Tle5012b_para));
  timer_enable(TIMER4);//_failsafe_timer.resume();// resume the timer ISR
}

U8 uart_processor(void)
{
  static U8 index=0,headflag=0;
   U8 Udata=0;
   S16 aLen=0;
   aLen = UartA_Available();
  // if(aLen>3) UAprintf("aLen=%d",aLen);
   while(aLen)
   {
      aLen--;
      Udata=(U8)UartA_read();//re
      //UAprintf("Udata=%x index=%d",Udata,index);
      if(headflag==0)//search the head flag 0xaa
        {
          if(Udata==0xaa) {headflag=1;index=0;}
        }
       else
       {
         dataBuf[index++]=Udata;
         if(index>31) {headflag=0;}//overflow or an error aer there.
         if(dataBuf[0]==index) {headflag=0; return 1;}//we have got a whole packaget.
       }
   }
  return 0;
}


U8 uartB_processor(void)
{
  static U8 index=0,headflag=0;
   U8 Udata=0;
   S16 aLen=0;
   aLen = UartC_Available();
  // if(aLen>3) hal.uartB->printf("aLen=%d",aLen);
   while(aLen)
   {
      aLen--;
      Udata=(U8)UartC_read();//re
      //hal.uartB->printf("Udata=%x index=%d",Udata,index);
      if(headflag==0)//search the head flag 0xaa
        {
          if(Udata==0xaa) {headflag=1;index=0;}
        }
       else
       {
         dataBufB[index++]=Udata;
         if(index>31) {headflag=0;}//overflow or an error aer there.
         if(dataBufB[0]==index) {headflag=0; return 1;}//we have got a whole packaget.
       }
   }
  return 0;
}

void comunication_withUser(void)
{
   U8 cmd=0;
   S16 datain=0;
   float fdata=0;
   cmd=uart_processor();
   if(cmd){
    cmd=0;
    switch(dataBuf[1])
    {
      case 1:///set angle target
            datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
            fdata =datain/100.0f;
            tangle = fdata;// first motor

            datain=(S16)(((dataBuf[4]<<8)|dataBuf[5]));
            fdata =datain/100.0f;
            tangleB = fdata;///second motor 
      break;
      case 2://set AngleP (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000.0f;
           motors.angle_P=fdata;
           Tle5012b_para.MotorA.angleP=fdata;
      break;
      case 3://set OmegaI (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000000.0f;
           motors.omegaI=fdata;
           Tle5012b_para.MotorA.omegaI=fdata;//;
      break;
      case 4://set OmegaD (floatX1000000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000000.0f;
           motors.omegaD=fdata;
           Tle5012b_para.MotorA.omegaD=fdata;
      break;
      case 5://set omegaP (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000.0f;
           motors.omegaP =fdata;
           Tle5012b_para.MotorA.omegaP =fdata;
      break;
      case 6://set toque P (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000.0f;
           motors.Tq_P=fdata;
           Tle5012b_para.MotorA.Tq_P=fdata;
      break;
      case 7://set toque I (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000.0f;
           motors.Tq_I=fdata;
      break;
      case 8://set toque D (floatX1000,)
           datain=(S16)(((dataBuf[2]<<8)|dataBuf[3]));
           fdata=datain/1000.0f;
           motors.Tq_D=fdata;
      break;
      case 9://get machine offset pointer (floatX1000,)
           getMachine_A_B_offset();
          break;
      case 10://get the offset between electronic Angle and magnet Angle
            timer_disable(TIMER4);//_failsafe_timer.pause();
               Tle5012b_para.caliFlagbits &=0xfffc; 
               motorSixStep_offset_caliA();

               motorSixStep_offset_caliB();  
           timer_enable(TIMER4);// _failsafe_timer.resume();        
           break;
      case 11:
           break;
      case 12: //save data
          parametersSave();// hal.storage->write_block(0, (void *)(&Tle5012b_para), sizeof(Tle5012b_para));
      break;
    }
      //UAprintf("CMD=%d,data:%f\r\n",dataBuf[1],fdata);
  }
}


void comunication_withUserB(void)
{
   U8 cmd=0;
   S16 datain=0;
   float fdata=0;
   cmd=uartB_processor();
   if(cmd){
    cmd=0;
    switch(dataBufB[1])
    {
      case 1:///set angle target
            datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
            fdata =datain/100.0f;
            tangle = fdata;// first motor

            datain=(S16)(((dataBufB[4]<<8)|dataBufB[5]));
            fdata =datain/100.0f;
            tangleB = fdata;///second motor 
      break;
      case 2://set AngleP (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000.0f;
           motors.angle_P=fdata;
           Tle5012b_para.MotorA.angleP=fdata;
      break;
      case 3://set OmegaI (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000000.0f;
           motors.omegaI=fdata;
           Tle5012b_para.MotorA.omegaI=fdata;//;
      break;
      case 4://set OmegaD (floatX1000000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000000.0f;
           motors.omegaD=fdata;
           Tle5012b_para.MotorA.omegaD=fdata;
      break;
      case 5://set omegaP (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000.0f;
           motors.omegaP =fdata;
           Tle5012b_para.MotorA.omegaP =fdata;
      break;
      case 6://set toque P (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000.0f;
           motors.Tq_P=fdata;
           Tle5012b_para.MotorA.Tq_P=fdata;
      break;
      case 7://set toque I (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000.0f;
           motors.Tq_I=fdata;
      break;
      case 8://set toque D (floatX1000,)
           datain=(S16)(((dataBufB[2]<<8)|dataBufB[3]));
           fdata=datain/1000.0f;
           motors.Tq_D=fdata;
      break;
      case 9://get machine offset pointer (floatX1000,)
           getMachine_A_B_offset();
          break;
      case 10://get the offset between electronic Angle and magnet Angle
            timer_disable(TIMER4);//_failsafe_timer.pause();
               Tle5012b_para.caliFlagbits &=0xfffc; 
               motorSixStep_offset_caliA();

               motorSixStep_offset_caliB();  
            timer_enable(TIMER4);//_failsafe_timer.resume();        
           break;
      case 11:
           break;
      case 12: //save data
           parametersSave();//hal.storage->write_block(0, (void *)(&Tle5012b_para), sizeof(Tle5012b_para));
      break;
    }
     // UAprintf("CMD=%d,data:%f\r\n",dataBufB[1],fdata);
  }
}


void SPWMF4_FeedBack_test_noBlock(void)
{
  float AngleOffset=0;
  U32 cunt=0;
  AngleOffset= readAnglesDegrees(1);
  UAprintf("SvPWMF4_lock@angle:%f\r\n",AngleOffset);
  
    //runto90degree();
    for(;;)
      {
        comunication_withUser();// for the IMU data from coptor
        comunication_withUserB();//for set the parameters
        if(SVPWM_flag_on)
          {
            SVPWM_flag_on=0;
            cunt++;
            if(cunt==50)
              {cunt=0;
               //ANO_DT_Send_Status(sAngle,0,tangle,(sAngle)*100,1,1);
              }
          }
      }
}
