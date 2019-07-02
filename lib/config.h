#ifndef _CONFIG_H_
#define _CONFIG_H_
// in lcd.c declare the logog array .
#define SIMTOOLOGO // not define ,use the hover logo 
//#define CHN //for chinese menu  
#define TAKEOFFALT  7.0  
//m
//not define ,use the english language
//////////////////////////////////////////for newer high 
#define LimitHigh 50000
//////////////////////////300000 ///////////////
#define ProLimitHigh 150000
//mm .>LimitHigh  then stop the thro to grow up 
// 
#define Stop_To_Loitor 4000 
// mm
#define LostBeatHeartTime 8000  //ms
#define VERSION "SimToo-1.32"

#define LOWVOL_STATE 1
//#define RTL_STATE 2
//#define LAND_STATE 3
#define DISCON_STATE 2
#define TIMES_STATE 3
//#define TAKEOFF_STATE 6

#define Circle_M        100000 //cm
#define Default_RADIUS     0
#define FAKE_TEST       
#define SIGNAL     125

#define RC_AGENT_OF_COPTOR 0
#define RC_STICK   1
#define RC_WATCH   0

/////////////////if you want to debug then enbale it
#define DEBUG_INLINE 
#endif