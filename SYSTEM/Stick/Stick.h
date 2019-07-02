#ifndef _STICK_H_
#define _STICK_H_

#include "sys.h"

void Stick_Init(void);
void calibrationStick(u16 keyValue);
uint16_t Get_Stick_ADC(uint8_t ch);
void rcin_loop(void);
u8 menu_stick();

//#define watch

typedef __packed struct{
unsigned int ch;
unsigned int max;
unsigned int min;
unsigned int middle;
float  gainH;
float  gainL;
}CHANNEL;
#ifdef watch 
typedef __packed struct{
CHANNEL RC[1];
unsigned int Battmv;
}RC_BATT_INFO;
#else
typedef __packed struct{
CHANNEL RC[6];
float Battmv;
}RC_BATT_INFO;
#endif


#define THRO  0
#define AILE  1
#define ELEV  2
#define RUDD  3
#define AUX1  4
#define AUX2  5 
#define AUX3  6
#define AUX4  7
#define BASEline  1070 // 1100  
#define MiddleBASEline 1500
#define STROKE   430.0

#define UP    1
#define DOWN  2
#define LEFT  3
#define RIGHT 4

typedef enum{
CH0=0,
CH1=1,
CH2=2,
CH3=3,
CH4=4,
CH5=5,
BATT=6
}ADCCHX;
#endif

