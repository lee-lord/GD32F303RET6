#include "sys.h"
#include "usart.h"
#include "Stick.h"
#include "adc.h"
#include "flash_config.h"
#include "delay.h"
#include "lcd.h"
#include "key.h"
#include "systick.h"
#include "config.h"
#include "radio.h"
#include "STM_GCS.h"

#define DeadH 1520
#define DeadL 1480
extern BoardConfig boardConfig;
RC_BATT_INFO RCx_BatInfo;
/////////remap the chx to IO_pin
const unsigned char CHxMap[9]={13,10,0,1,11,12,8};

float StickSense[3]={0.8,1.0,1.2};
typedef struct {
u16 max;
u16 min;
u16 mid;	
} CH_Max_Min;
/*

#define THRO  0
#define AILE  1
#define ELEV  2
#define RUDD  3
#define AUX1  4
#define AUX2  5 
#define AUX3  6
#define AUX4  7

*/
void Stick_Init(void)
{
	 unsigned char i=0;
  ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC	, ENABLE );	  //使能ADC1通道时钟
	/* Configure ADCCLK such as ADCCLK = PCLK2/6 */ 
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //72M/6=12,ADC最大时间不能超过14M
#ifdef watch
	//PA0/1/2/3 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	


	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
	
		/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 2;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
#else
  //PA0/1/2/3 作为模拟通道输入引脚                         
	//PA0/1/2/3 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;//AUX1 AUX2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //CH0--Ch3  BATT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //CH0--Ch3  BATT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
	
		/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 6;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   	
		
#endif
  ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
	
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
	
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
#ifdef watch

//to do 是否已校準過否則不初始化最大最小值
if(boardConfig.board.channel_ok==0)
{   RCx_BatInfo.RC[0].min=0;
		RCx_BatInfo.RC[0].max=4096;
		RCx_BatInfo.RC[0].gainH=0.2441;
}
else
{
    RCx_BatInfo.RC[0].min=boardConfig.board.ch0_min;
		RCx_BatInfo.RC[0].max=boardConfig.board.ch0_max;
		RCx_BatInfo.RC[0].gainH=boardConfig.board.ch0_gain;
}
#else
//to do 是否已校準過否則不初始化最大最小值
if(boardConfig.board.channel_ok==0)
{  for(i=0;i<6;i++)
     {
      RCx_BatInfo.RC[i].min=2048;
			RCx_BatInfo.RC[i].max=2048;
			RCx_BatInfo.RC[i].gainH=0.2441;
     }
}
else
{
    RCx_BatInfo.RC[0].min=boardConfig.board.ch0_min;
		RCx_BatInfo.RC[0].max=boardConfig.board.ch0_max;
		RCx_BatInfo.RC[0].gainH=boardConfig.board.ch0_gainH;
    RCx_BatInfo.RC[0].gainL=boardConfig.board.ch0_gainL;
	  RCx_BatInfo.RC[0].middle=boardConfig.board.ch0_mid;
	
	  RCx_BatInfo.RC[1].min=boardConfig.board.ch1_min;
		RCx_BatInfo.RC[1].max=boardConfig.board.ch1_max;
		RCx_BatInfo.RC[1].gainH=boardConfig.board.ch1_gainH;
    RCx_BatInfo.RC[1].gainL=boardConfig.board.ch1_gainL;
	  RCx_BatInfo.RC[1].middle=boardConfig.board.ch1_mid;
	
	  RCx_BatInfo.RC[2].min=boardConfig.board.ch2_min;
		RCx_BatInfo.RC[2].max=boardConfig.board.ch2_max;
		RCx_BatInfo.RC[2].gainH=boardConfig.board.ch2_gainH;
		RCx_BatInfo.RC[2].gainL=boardConfig.board.ch2_gainL;	
		RCx_BatInfo.RC[2].middle=boardConfig.board.ch2_mid;	
	
	  RCx_BatInfo.RC[3].min=boardConfig.board.ch3_min;
		RCx_BatInfo.RC[3].max=boardConfig.board.ch3_max;
		RCx_BatInfo.RC[3].gainH=boardConfig.board.ch3_gainH;
		RCx_BatInfo.RC[3].gainL=boardConfig.board.ch3_gainL;		
		RCx_BatInfo.RC[3].middle=boardConfig.board.ch3_mid;		
	
	  RCx_BatInfo.RC[4].min=boardConfig.board.ch4_min;
		RCx_BatInfo.RC[4].max=boardConfig.board.ch4_max;
		RCx_BatInfo.RC[4].gainH=boardConfig.board.ch4_gainH;
		RCx_BatInfo.RC[4].gainL=boardConfig.board.ch4_gainL;		
		RCx_BatInfo.RC[4].middle=boardConfig.board.ch4_mid;		
	
	  RCx_BatInfo.RC[5].min=boardConfig.board.ch5_min;
		RCx_BatInfo.RC[5].max=boardConfig.board.ch5_max;
		RCx_BatInfo.RC[5].gainH=boardConfig.board.ch5_gainH;
		RCx_BatInfo.RC[5].gainL=boardConfig.board.ch5_gainL;		
		RCx_BatInfo.RC[5].middle=boardConfig.board.ch5_mid;		

}
#endif


}

void ResetRCValue(void)
{
#ifdef watch

//to do 是否已校準過否則不初始化最大最小值
    RCx_BatInfo.RC[0].min=2048;
		RCx_BatInfo.RC[0].max=2048;
		RCx_BatInfo.RC[0].gainH=1.0;
	  //保存在flash
#else
//to do 是否已校準過否則不初始化最大最小值
	u8 i=0;
     for(i=0;i<6;i++)
     {
      RCx_BatInfo.RC[i].min=4096;
			RCx_BatInfo.RC[i].max=0;
			RCx_BatInfo.RC[i].gainH=1.0;
			RCx_BatInfo.RC[i].gainL=1.0;
			RCx_BatInfo.RC[i].middle=1000;
     }
		 
		 //保存在flash
#endif
		 
#ifdef watch
//to do 是否已校準過否則不初始化最大最小值
    boardConfig.board.ch0_min =RCx_BatInfo.RC[0].min;
		boardConfig.board.ch0_max =RCx_BatInfo.RC[0].max;
		boardConfig.board.ch0_gain=RCx_BatInfo.RC[0].gainH;
	
#else
//to do 是否已校準過否則不初始化最大最小值
    boardConfig.board.ch0_min=RCx_BatInfo.RC[0].min;
		boardConfig.board.ch0_max=RCx_BatInfo.RC[0].max;
		boardConfig.board.ch0_gainH=RCx_BatInfo.RC[0].gainH;
		boardConfig.board.ch0_gainL=RCx_BatInfo.RC[0].gainL;
		boardConfig.board.ch0_mid=RCx_BatInfo.RC[0].middle;
		 
	  boardConfig.board.ch1_min=RCx_BatInfo.RC[1].min;
		boardConfig.board.ch1_max=RCx_BatInfo.RC[1].max;
		boardConfig.board.ch1_gainH=RCx_BatInfo.RC[1].gainH;
		boardConfig.board.ch1_gainL=RCx_BatInfo.RC[1].gainL; 
		boardConfig.board.ch1_mid=RCx_BatInfo.RC[1].middle;
		
	  boardConfig.board.ch2_min=RCx_BatInfo.RC[2].min;
		boardConfig.board.ch2_max=RCx_BatInfo.RC[2].max;
		boardConfig.board.ch2_gainH=RCx_BatInfo.RC[2].gainH;
		boardConfig.board.ch2_gainL=RCx_BatInfo.RC[2].gainL;	
		boardConfig.board.ch2_mid=RCx_BatInfo.RC[2].middle;
		
	  boardConfig.board.ch3_min=RCx_BatInfo.RC[3].min;
		boardConfig.board.ch3_max=RCx_BatInfo.RC[3].max;
		boardConfig.board.ch3_gainH=RCx_BatInfo.RC[3].gainH;
		boardConfig.board.ch3_gainL=RCx_BatInfo.RC[3].gainL;
    boardConfig.board.ch3_mid=RCx_BatInfo.RC[3].middle;		
		
	  boardConfig.board.ch4_min=RCx_BatInfo.RC[4].min;
		boardConfig.board.ch4_max=RCx_BatInfo.RC[4].max;
		boardConfig.board.ch4_gainH=RCx_BatInfo.RC[4].gainH;
		boardConfig.board.ch4_gainL=RCx_BatInfo.RC[4].gainL;	
		boardConfig.board.ch4_mid=RCx_BatInfo.RC[4].middle;
		
	  boardConfig.board.ch5_min=RCx_BatInfo.RC[5].min;
		boardConfig.board.ch5_max=RCx_BatInfo.RC[5].max;
		boardConfig.board.ch5_gainH=RCx_BatInfo.RC[5].gainH;
		boardConfig.board.ch5_gainL=RCx_BatInfo.RC[5].gainL;	
		boardConfig.board.ch5_mid=RCx_BatInfo.RC[5].middle;
		
#endif	 
boardConfig.board.channel_ok=0;
writeBoardConfig(&boardConfig);
}



void SaveRC_ValueinFlash(void)
{
#ifdef watch

//to do 是否已校準過否則不初始化最大最小值
    boardConfig.board.ch0_min =RCx_BatInfo.RC[0].min;
		boardConfig.board.ch0_max =RCx_BatInfo.RC[0].max;
		boardConfig.board.ch0_gain=RCx_BatInfo.RC[0].gainH;
	
#else
//to do 是否已校準過否則不初始化最大最小值
    boardConfig.board.ch0_min=RCx_BatInfo.RC[0].min;
		boardConfig.board.ch0_max=RCx_BatInfo.RC[0].max;
		boardConfig.board.ch0_gainH=RCx_BatInfo.RC[0].gainH;
		boardConfig.board.ch0_gainL=RCx_BatInfo.RC[0].gainL;
		boardConfig.board.ch0_mid=RCx_BatInfo.RC[0].middle;
		 
	  boardConfig.board.ch1_min=RCx_BatInfo.RC[1].min;
		boardConfig.board.ch1_max=RCx_BatInfo.RC[1].max;
		boardConfig.board.ch1_gainH=RCx_BatInfo.RC[1].gainH;
		boardConfig.board.ch1_gainL=RCx_BatInfo.RC[1].gainL; 
		boardConfig.board.ch1_mid=RCx_BatInfo.RC[1].middle;
		
	  boardConfig.board.ch2_min=RCx_BatInfo.RC[2].min;
		boardConfig.board.ch2_max=RCx_BatInfo.RC[2].max;
		boardConfig.board.ch2_gainH=RCx_BatInfo.RC[2].gainH;
		boardConfig.board.ch2_gainL=RCx_BatInfo.RC[2].gainL;	
		boardConfig.board.ch2_mid=RCx_BatInfo.RC[2].middle;
		
	  boardConfig.board.ch3_min=RCx_BatInfo.RC[3].min;
		boardConfig.board.ch3_max=RCx_BatInfo.RC[3].max;
		boardConfig.board.ch3_gainH=RCx_BatInfo.RC[3].gainH;
		boardConfig.board.ch3_gainL=RCx_BatInfo.RC[3].gainL;
    boardConfig.board.ch3_mid=RCx_BatInfo.RC[3].middle;		
		
	  boardConfig.board.ch4_min=RCx_BatInfo.RC[4].min;
		boardConfig.board.ch4_max=RCx_BatInfo.RC[4].max;
		boardConfig.board.ch4_gainH=RCx_BatInfo.RC[4].gainH;
		boardConfig.board.ch4_gainL=RCx_BatInfo.RC[4].gainL;	
		boardConfig.board.ch4_mid=RCx_BatInfo.RC[4].middle;
		
	  boardConfig.board.ch5_min=RCx_BatInfo.RC[5].min;
		boardConfig.board.ch5_max=RCx_BatInfo.RC[5].max;
		boardConfig.board.ch5_gainH=RCx_BatInfo.RC[5].gainH;
		boardConfig.board.ch5_gainL=RCx_BatInfo.RC[5].gainL;	
		boardConfig.board.ch5_mid=RCx_BatInfo.RC[5].middle;	
#endif
boardConfig.board.channel_ok=1;
writeBoardConfig(&boardConfig);

}

u16 Get_Stick_ADC(u8 ch)   
{
  	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, CHxMap[ch], 1, ADC_SampleTime_71Cycles5 );	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期	  			    
	//ADC1->CR2|=1<<22;       //启动规则转换通道 
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	//while(!(ADC1->SR&1<<1));//等待转换结束	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

void findMax_Min(CHANNEL *Prc)
{
	if(Prc->ch>Prc->max) Prc->max=Prc->ch;
	if(Prc->ch<Prc->min) Prc->min=Prc->ch;
}


void getReal_RC_channel()
{
 unsigned char i=0;
#ifdef watch
    RCx_BatInfo.RC[0].ch=Get_Stick_ADC(CH0);
	  findMax_Min(&RCx_BatInfo.RC[0]);
#else
     for(i=0;i<4;i++)
     {
	  
       RCx_BatInfo.RC[i].ch=Get_Stick_ADC(CH0+i);
	   findMax_Min(&RCx_BatInfo.RC[i]);
	   
     }
	   RCx_BatInfo.RC[4].ch=0;
	   findMax_Min(&RCx_BatInfo.RC[4]);
	  RCx_BatInfo.RC[4].ch=4095;
	   findMax_Min(&RCx_BatInfo.RC[4]);
	   RCx_BatInfo.RC[5].ch=0;
	   findMax_Min(&RCx_BatInfo.RC[5]);
	  RCx_BatInfo.RC[5].ch=4095;
	   findMax_Min(&RCx_BatInfo.RC[5]);
	 
#endif

}

void getMiddleValue(void)
{
	u8 i=0,j=0;
	 for(j=0;j<20;j++){
      for(i=0;i<4;i++){
       RCx_BatInfo.RC[i].middle=Get_Stick_ADC(CH0+i);
			 //findMax_Min(&RCx_BatInfo.RC[i]);
				delay_ms(1);
        }
			}
	 RCx_BatInfo.RC[4].middle=2047;
	  RCx_BatInfo.RC[5].middle=2047;		
}

void GetRC_Gain()
{
	unsigned char i=0;
#ifdef watch
	  if((RCx_BatInfo.RC[0].max-RCx_BatInfo.RC[0].min))
       RCx_BatInfo.RC[0].gainH=STROKE/(float)(RCx_BatInfo.RC[0].max-RCx_BatInfo.RC[0].min);
		else
			 RCx_BatInfo.RC[0].gainH=1.0;
#else
     for(i=0;i<6;i++)
     {  ///get H _gain
		if((RCx_BatInfo.RC[i].max-RCx_BatInfo.RC[i].middle))
      RCx_BatInfo.RC[i].gainH=STROKE/(float)(RCx_BatInfo.RC[i].max-RCx_BatInfo.RC[i].middle);
		else
			 RCx_BatInfo.RC[i].gainH=1.0;
        ///get L_gain
		if((RCx_BatInfo.RC[i].middle-RCx_BatInfo.RC[i].min))
      RCx_BatInfo.RC[i].gainL=STROKE/(float)(RCx_BatInfo.RC[i].middle-RCx_BatInfo.RC[i].min);
		else
			 RCx_BatInfo.RC[i].gainL=1.0;
     }
#endif
}
#define MiddleMax   2200
#define MiddleMin   1800

#define DeltaMin   1500
void calibrationStick(u16 keyValue)
{
	unsigned int times=0;
	u16 i=0;
	u16 color=WHITE;
	u16 tempCh=0;
	char txt[30];
CH_Max_Min ChRefer[6];
for(i=0;i<6;i++)
 {
	 ChRefer[i].max=MiddleMax;
	 ChRefer[i].min=MiddleMin;
  }	
	if(keyValue==NEEDCALIBRATION)
		  goto cali;
	if(boardConfig.board.channel_ok==1) return;
cali:
  	ResetRCValue();
	//LCD display calibration status

LCD_ShowString(0,0,128,128,12,"Start Cali    ",WHITE,BLACK);	
LCD_ShowString(0,16,128,128,12,"Release Key    ",WHITE,BLACK);	
sprintf(txt,"%s-%s",__DATE__,__TIME__);
LCD_ShowString(0,32,128,128,12,txt,WHITE,BLACK);
sprintf(txt,"%s",VERSION);
LCD_ShowString(0,48,128,128,12,txt,WHITE,BLACK);	
	for(i=0;i<2000;i++)
	 { 
		 KEY_Scan();
	   delay_ms(1);
	 }
	 delay_ms(500);
   while(times<700)  //about 10 s to calibrate 
    {
	if(times%10==0)
	{
		for(i=0;i<6;i++)
    {
		ChRefer[i].mid=(ChRefer[i].max+ChRefer[i].min)>>1;
		ChRefer[i].max=ChRefer[i].mid+250;
		ChRefer[i].min=ChRefer[i].mid-250;
    }
  }
       getReal_RC_channel();
       times++;
			// delay_ms(1);
if(RCx_BatInfo.RC[0].ch<=ChRefer[0].max&&RCx_BatInfo.RC[0].ch>=ChRefer[0].min)			
	color=GREEN;
else
	color=WHITE;	
sprintf(txt,"C1=%4d,",RCx_BatInfo.RC[0].ch);
LCD_ShowString(0,16,128,128,16,txt,color,BLACK);
////////////////////////////////////////////////////////////////////////////////////			
if(RCx_BatInfo.RC[1].ch<=ChRefer[1].max&&RCx_BatInfo.RC[1].ch>=ChRefer[1].min)			
	color=GREEN;
else
	color=WHITE;			
sprintf(txt,"C2=%4d",RCx_BatInfo.RC[1].ch);
LCD_ShowString(64,16,128,128,16,txt,color,BLACK);			
//////////////////////////////////////////////////////////////////////////////////			
if(RCx_BatInfo.RC[2].ch<=ChRefer[2].max&&RCx_BatInfo.RC[2].ch>=ChRefer[2].min)			
	color=GREEN;
else
	color=WHITE;	
sprintf(txt,"C3=%4d,",RCx_BatInfo.RC[2].ch);
LCD_ShowString(0,32,128,128,16,txt,color,BLACK);
////////////////////////////////////////////////////////////////////////////////////			
if(RCx_BatInfo.RC[3].ch<=ChRefer[3].max&&RCx_BatInfo.RC[3].ch>=ChRefer[3].min)			
	color=GREEN;
else
	color=WHITE;	
sprintf(txt,"C4=%4d",RCx_BatInfo.RC[3].ch);
LCD_ShowString(64,32,128,128,16,txt,color,BLACK);			
///////////////////////////////////////////////////////////////////////
tempCh=Get_Stick_ADC(CH0+4);
if(tempCh<=MiddleMax&&tempCh>=MiddleMin)			
	color=GREEN;
else
	color=GREEN;				
			
sprintf(txt,"C5=%4d,",tempCh);
LCD_ShowString(0,48,128,128,16,txt,color,BLACK);
////////////////////////////////////////////////////////////////////////////////////
tempCh=Get_Stick_ADC(CH0+5);
if(tempCh<=MiddleMax&&tempCh>=MiddleMin)			
	color=GREEN;
else
	color=GREEN;
sprintf(txt,"C6=%4d",tempCh);
LCD_ShowString(64,48,128,128,16,txt,color,BLACK);			
			
/////////////////////////////////////////////////////////////////////////////////////////////			
if(RCx_BatInfo.RC[0].max-RCx_BatInfo.RC[0].min >DeltaMin)			
	color=GREEN;
else
	color=WHITE;
sprintf(txt,"C1=%4d,=%4d",RCx_BatInfo.RC[0].max,RCx_BatInfo.RC[0].min);
LCD_ShowString(0,64,128,128,16,txt,color,BLACK);
if(RCx_BatInfo.RC[1].max-RCx_BatInfo.RC[1].min >DeltaMin)			
	color=GREEN;
else
	color=WHITE;
sprintf(txt,"C2=%4d,=%4d",RCx_BatInfo.RC[1].max,RCx_BatInfo.RC[1].min);
LCD_ShowString(0,80,128,128,16,txt,color,BLACK);
if(RCx_BatInfo.RC[2].max-RCx_BatInfo.RC[2].min >DeltaMin)			
	color=GREEN;
else
	color=WHITE;
sprintf(txt,"C3=%4d,=%4d",RCx_BatInfo.RC[2].max,RCx_BatInfo.RC[2].min);
LCD_ShowString(0,96,128,128,16,txt,color,BLACK);
if(RCx_BatInfo.RC[3].max-RCx_BatInfo.RC[3].min >DeltaMin)			
	color=GREEN;
else
	color=WHITE;
sprintf(txt,"C4=%4d,=%4d",RCx_BatInfo.RC[3].max,RCx_BatInfo.RC[3].min);
LCD_ShowString(0,110,128,128,16,txt,color,BLACK);						
			
//sprintf(txt,"CH1=%4d,CH2=%4d",RCx_BatInfo.RC[0].ch,RCx_BatInfo.RC[1].ch);
//LCD_ShowString(0,16,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"CH3=%4d,CH4=%4d",RCx_BatInfo.RC[2].ch,RCx_BatInfo.RC[3].ch);
//LCD_ShowString(0,32,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"CH5=%4d,CH6=%4d",RCx_BatInfo.RC[4].ch,RCx_BatInfo.RC[5].ch);
//LCD_ShowString(0,48,128,128,12,txt,WHITE,BLACK);
//			
//sprintf(txt,"1max=%4d,min=%4d",RCx_BatInfo.RC[0].max,RCx_BatInfo.RC[0].min);
//LCD_ShowString(0,64,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"2max=%4d,min=%4d",RCx_BatInfo.RC[1].max,RCx_BatInfo.RC[1].min);
//LCD_ShowString(0,80,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"3max=%4d,min=%4d",RCx_BatInfo.RC[2].max,RCx_BatInfo.RC[2].min);
//LCD_ShowString(0,96,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"4max=%4d,min=%4d",RCx_BatInfo.RC[3].max,RCx_BatInfo.RC[3].min);
//LCD_ShowString(0,110,128,128,12,txt,WHITE,BLACK);			
//			LCD_Color_Fill(0,  0,RCx_BatInfo.RC[0].ch*128/4096,16,RED);
//			LCD_Color_Fill(16,16,RCx_BatInfo.RC[1].ch*128/4096,32,RED);
//			LCD_Color_Fill(32,32,RCx_BatInfo.RC[2].ch*128/4096,48,RED);
//			LCD_Color_Fill(48,48,RCx_BatInfo.RC[3].ch*128/4096,64,RED);
//			LCD_Color_Fill(64,64,RCx_BatInfo.RC[4].ch*128/4096,80,RED);
//			LCD_Color_Fill(80,80,RCx_BatInfo.RC[5].ch*128/4096,96,RED);
sprintf(txt,"TIMES=%5d    ",times);
LCD_ShowString(0,0,128,128,12,txt,WHITE,BLACK);		
      if(KEY_Scan()) break;
    }

Donormal:
		DISPLAY_COLOR(RED);	
   // sprintf(txt,"Release all stick!",times);
    LCD_ShowString(0,0,128,128,12,"Release all stick!",WHITE,BLACK);
		getMiddleValue();
    GetRC_Gain();
		SaveRC_ValueinFlash(); 
    LCD_ShowString(0,32,128,128,12,"PRESS key to Bar",WHITE,BLACK);
	//LCD display calibration status
	 for(i=0;i<200;i++)
	 {  KEY_Scan();
	   delay_ms(1);
	 }
	 DISPLAY_COLOR(BLACK);	
//		if(KEY_Scan()) goto DisplayBar;
//		return;
DisplayBar:	
   LCD_ShowString(48,0,128,128,12,"Bar ",WHITE,BLACK);	 
//	 for(i=0;i<1000;i++)
//	 {  KEY_Scan();
//	   delay_ms(1);
//	 }
	 color=GREEN;
		while(1)
		{
			
			rcin_loop();
			///////////////////////////////////////
			if(RCx_BatInfo.RC[3].ch>=1480&&RCx_BatInfo.RC[3].ch<=1520) color=GREEN;else color = RED;
		  DrawBar(10  ,16,(RCx_BatInfo.RC[3].ch-BASEline)*50/STROKE,                                100,2,color);		
			sprintf(txt,"CH3=%4d",RCx_BatInfo.RC[3].ch);
      LCD_ShowString(12,26,128,128,12,txt,color,BLACK);
			///////////////////////////////////////
			if(RCx_BatInfo.RC[0].ch>=1480&&RCx_BatInfo.RC[0].ch<=1520) color=GREEN;else color = RED;
      DrawBar(10  ,42,(RCx_BatInfo.RC[0].ch-BASEline)*50/STROKE,                                100,2,color);
		  sprintf(txt,"CH0=%4d",RCx_BatInfo.RC[0].ch);
      LCD_ShowString(12,54,128,128,12,txt,color,BLACK);
      ///////////////////////////////////////
			if(RCx_BatInfo.RC[4].ch>=1480&&RCx_BatInfo.RC[4].ch<=1520) color=GREEN;else color = RED;
		  DrawBar(10  ,70,(RCx_BatInfo.RC[4].ch-BASEline)*50/STROKE,                                100,2,color);
			sprintf(txt,"CH4=%4d",RCx_BatInfo.RC[4].ch);
      LCD_ShowString(12,82,128,128,12,txt,color,BLACK);
			////////////////////////////////////////////////
			if(RCx_BatInfo.RC[5].ch>=1480&&RCx_BatInfo.RC[5].ch<=1520) color=GREEN;else color = RED;
		  DrawBar(10  ,102,(RCx_BatInfo.RC[5].ch-BASEline)*50/STROKE,100                                ,2,color);
			sprintf(txt,"CH5=%4d",RCx_BatInfo.RC[5].ch);
      LCD_ShowString(12,114,128,128,12,txt,color,BLACK);
			///////////////////////////////////////////////////
			if(RCx_BatInfo.RC[1].ch>=1480&&RCx_BatInfo.RC[1].ch<=1520) color=GREEN;else color = RED;
		  DrawBar(115,16,(RCx_BatInfo.RC[1].ch-BASEline)*50/STROKE,100                                ,1,color);
		 sprintf(txt,"CH1=%4d",RCx_BatInfo.RC[1].ch);
      LCD_ShowString(78,0,128,128,12,txt,color,BLACK);
			///////////////////////////////////////////////////
			if(RCx_BatInfo.RC[2].ch>=1480&&RCx_BatInfo.RC[2].ch<=1520) color=GREEN;else color = RED;
		  DrawBar(0,16,(RCx_BatInfo.RC[2].ch-BASEline)*50/STROKE,100                                ,1,color);
			sprintf(txt,"CH2=%4d",RCx_BatInfo.RC[2].ch);
      LCD_ShowString(0,0,128,128,12,txt,color,BLACK);
		 
			 if(KEY_Scan()) break;
    }
		for(i=0;i<1000;i++)
	 {  
		 KEY_Scan();
	   delay_ms(1);
	 }
	//LCD display cal stick OK
}

void rcin_loop(void)
{
	 unsigned char i=0;
	char txt[30];
	 static u16 filter[6];
	static u32 last_ADC_Time=0;
	static u32 lastBatt=4200;
	float sense=1.0;
	u16 tmpadc=0;
	if(systemTickMs-last_ADC_Time>=20)
	{
		last_ADC_Time=systemTickMs;
#ifdef watch
    RCx_BatInfo.RC[0].ch=(float)abs(Get_Stick_ADC(CH0)-RCx_BatInfo.RC[0].min)*RCx_BatInfo.RC[0].gainH+BASEline;
#else
     for(i=0;i<6;i++)
     {
			 tmpadc=Get_Stick_ADC(CH0+i);
//			   if(i==3)
//				 {  if((tmpadc-RCx_BatInfo.RC[i].middle)>700)
//					   sense=StickSense[1];
//					   else if((tmpadc-RCx_BatInfo.RC[i].middle)>0)
//						  sense=StickSense[0];	 
//						  else if((tmpadc-RCx_BatInfo.RC[i].middle)<0 && (tmpadc-RCx_BatInfo.RC[i].middle)>-700)
//							  sense=StickSense[2];
//							 else if((tmpadc-RCx_BatInfo.RC[i].middle)<-700)
//								  sense=StickSense[1];
//				 }
//				 else
					   sense=1.0;
			 if(tmpadc>=RCx_BatInfo.RC[i].middle)
				  RCx_BatInfo.RC[i].ch=(float)abs(tmpadc-RCx_BatInfo.RC[i].middle)*RCx_BatInfo.RC[i].gainH*sense +MiddleBASEline;
				 else
          RCx_BatInfo.RC[i].ch=(float)abs(tmpadc-RCx_BatInfo.RC[i].min)*RCx_BatInfo.RC[i].gainL*sense +BASEline;
				 
				 if(RCx_BatInfo.RC[i].ch<DeadH&&RCx_BatInfo.RC[i].ch>DeadL&&i<4)
				 {
					 RCx_BatInfo.RC[i].ch=1500;
         }
     }

//    if(filter[0]==0) {
//       for(i=0;i<6;i++){
//     filter[i]= RCx_BatInfo.RC[i].ch;
//     }
//     }
//		 for(i=0;i<6;i++)
//     {
//    //  RCx_BatInfo.RC[i].ch=(filter[i]*2+RCx_BatInfo.RC[i].ch*8)/10;
//     } 
//		 for(i=0;i<6;i++){
//     filter[i]= RCx_BatInfo.RC[i].ch;
//     }
//sprintf(txt,"CH1=%4d,CH2=%4d",RCx_BatInfo.RC[0].ch,RCx_BatInfo.RC[1].ch);
//LCD_ShowString(0,32,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"CH3=%4d,CH4=%4d",RCx_BatInfo.RC[2].ch,RCx_BatInfo.RC[3].ch);
//LCD_ShowString(0,64,128,128,12,txt,WHITE,BLACK);
//sprintf(txt,"CH5=%4d,CH6=%4d",RCx_BatInfo.RC[4].ch,RCx_BatInfo.RC[5].ch);
//LCD_ShowString(0,96,128,128,12,txt,WHITE,BLACK);
#endif
     RCx_BatInfo.Battmv=(float)Get_Stick_ADC(BATT)*6600/4096;
	 lastBatt=(1*RCx_BatInfo.Battmv+9*lastBatt)/10;
	 RCx_BatInfo.Battmv=lastBatt;
//		 sprintf(txt,"ADC=%d,batt=%d",RCx_BatInfo.RC[0].ch,RCx_BatInfo.Battmv);
//    LCD_ShowString(0,16,128,128,12,txt,WHITE,BLACK);
	 }
}

u8 menu_stick(){
	rcin_loop();
	if(boardConfig.board.StickMode==LEFT_THRO_MODE)
	{
		if(RCx_BatInfo.RC[1].ch>1600)	   return UP;
		else if(RCx_BatInfo.RC[1].ch<1400) return DOWN;
		else if(RCx_BatInfo.RC[0].ch<1400) return LEFT;
		else if(RCx_BatInfo.RC[0].ch>1600) return RIGHT;
	}
	if(boardConfig.board.StickMode==RIGHT_THRO_MODE)
	{	
		if(RCx_BatInfo.RC[2].ch<1400)	   return UP;
		else if(RCx_BatInfo.RC[2].ch>1600) return DOWN;
		else if(RCx_BatInfo.RC[3].ch>1600) return LEFT;
		else if(RCx_BatInfo.RC[3].ch<1400) return RIGHT;
	}
	return 0;

}

