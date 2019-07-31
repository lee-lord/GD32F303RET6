#ifndef _FOC_DRIVER_H_
#define _FOC_DRIVER_H_
#define VqMax 0.8f
#define VqMin -0.8f
#define MTA 0
#define MTB 1

#define M_CW  0
#define M_CCW 1

#define FULLscale 1 
#define HALFscale 0
#define MTA_EOFF_MASK 0x0001
#define MTB_EOFF_MASK 0x0002
#define MTA_OFF_MASK 0x0004
#define MTB_OFF_MASK 0x0008
typedef __packed struct{
    float tA;//from sensor tle5012B
    float tB;
    float tC;

    float Ua,Ub,Uc;
    float alpha,beta;
 ////
    U8 dirA;//motor direction
    U8 dirB;//motor direction
/////////

    float  PWM_F; // <- chang for PWM_F
	float T_svm;
	float  MI;
	float clock;
	float ARR_val;
	float spc_angle,spc_mag,theta;
	U16 sector;
	float T1,T2,T0,T1n,T2n,T0n;
	float P_Tsvm_arr;
	/////////////////////PID 
	float omegaP ;

    float omegaP1 ;
    float omegaP2 ;    
    // float omegaP3 ;    
    // float omegaP4 ; 
    float omegaI;   
    float omegaD ;
    float angle_P ;
    float Tq_P;
    float Tq_I;
    float Tq_D;
    U8 Poles;
    float DegreeToRadin;
    float elect_machion_offsetA;
    float elect_machion_offsetB;
////////////////////////////////pointer of function
    void (* SVPWM_F4_DQ_speed2PostionA)(float Sensorangle,float senDgr);
    void (* SVPWM_F4_DQ_speed2PostionB)(float Sensorangle,float senDgr);
    void (* WriteMotorDirect)(float angledegree,float PwrScal,U8 dir,U8 motorX);
    void (* SetUpZeroPosition)(U16 SwitchMode,U8 Tdir,U8 motorX);
    void (* output_min)(void);

}MOTORS_Driver;
extern U32 Frequs;
extern volatile MOTORS_Driver motors;
extern volatile float MotorA_MachineOffset,MotorB_MachineOffset;

void foc_initialise(U32 _speed_hz);
uint8_t sec_idn(float angle);
void tim_cal(void);

void gSVPWM_F4_DQ_speed2PostionA(float Sensorangle,float senDgr);
void gSVPWM_F4_DQ_speed2PostionB(float Sensorangle,float senDgr);
void gWriteMotorDirect(float angledegree,float PwrScal,U8 dir,U8 motorX);
void gSetUpZeroPosition(U16 SwitchMode,U8 Tdir,U8 motorX);
void goutput_min(void);

void parameter_checkANDset(void);


#endif
