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
U32 Frequs=0;
volatile MOTORS_Driver motors;
volatile float MotorA_MachineOffset=0.0f,MotorB_MachineOffset=0.0f;

///////power on then load the parameter .
void setPIDandTLEParameters(void)
{
   motors.angle_P=Tle5012b_para.MotorA.angleP;
   motors.omegaP= Tle5012b_para.MotorA.omegaP;
   motors.omegaI= Tle5012b_para.MotorA.omegaI;
   motors.omegaD= Tle5012b_para.MotorA.omegaD;
   motors.Poles = Tle5012b_para.MotorA.poles;

   motors.Tq_P=Tle5012b_para.MotorA.Tq_P;
   motors.Tq_I=Tle5012b_para.MotorA.Tq_I;
   motors.Tq_D=Tle5012b_para.MotorA.Tq_D;
  ////electronic angle offset
  motors.elect_machion_offsetA = Tle5012b_para.MotorA.Elec_offset;
  motors.elect_machion_offsetB = Tle5012b_para.MotorB.Elec_offset;
  /////machine angle offset 
  MotorA_MachineOffset         =Tle5012b_para.MotorA.MachineOffset;
  MotorB_MachineOffset         =Tle5012b_para.MotorB.MachineOffset;
  
  motors.dirA=Tle5012b_para.MotorA.direction;
  motors.dirB=Tle5012b_para.MotorB.direction;
}


void parameter_checkANDset(void)
{    //first load all parameters from eeprom of flash
    parametersLoad();//
   if(Tle5012b_para.magic!=0xaa55aa55)
     {
        fmc_erase_pages();
        Tle5012b_para.magic=0xaa55aa55;
        Tle5012b_para.MotorA.direction =0;
        Tle5012b_para.MotorA.Elec_offset =0;
        Tle5012b_para.MotorA.angleP=1.0f;

        Tle5012b_para.MotorA.omegaP=0.031f;
        Tle5012b_para.MotorA.omegaI=0.000000f;//0.045f;//0.21;
        Tle5012b_para.MotorA.omegaD=0.000069f;//0.00035
        
        Tle5012b_para.MotorA.poles=7;

        Tle5012b_para.MotorA.Tq_P=0.028f;
        Tle5012b_para.MotorA.Tq_I=0.014f;
        Tle5012b_para.MotorA.Tq_D=0.007f;
        Tle5012b_para.MotorA.MachineOffset =0.0f;
        /////////////////////////////////////////
        Tle5012b_para.MotorB.direction =0;
        Tle5012b_para.MotorB.Elec_offset =0;
        Tle5012b_para.MotorB.angleP=1.0f;

        Tle5012b_para.MotorB.omegaP=0.031f;
        Tle5012b_para.MotorB.omegaI=0.000000f;//0.045f;//0.21;
        Tle5012b_para.MotorB.omegaD=0.000069f;//0.00035
        
        Tle5012b_para.MotorB.poles=7;

        Tle5012b_para.MotorB.Tq_P=0.028f;
        Tle5012b_para.MotorB.Tq_I=0.014f;
        Tle5012b_para.MotorB.Tq_D=0.007f;
        Tle5012b_para.MotorB.MachineOffset =0.0f;

        Tle5012b_para.caliFlagbits=0;// reset to default mode
    ///////save all the parameters
         fmc_program_Bytes((U8 *)(&Tle5012b_para),sizeof(Tle5012b_para),0);
        }
     setPIDandTLEParameters();
}


void foc_initialise(U32 _speed_hz)
{  U16 overflow=0;
	parameter_checkANDset();

    motors.SVPWM_F4_DQ_speed2PostionA= gSVPWM_F4_DQ_speed2PostionA;
    motors.SVPWM_F4_DQ_speed2PostionB= gSVPWM_F4_DQ_speed2PostionB;
    motors.WriteMotorDirect = gWriteMotorDirect;
    motors.SetUpZeroPosition = gSetUpZeroPosition;
    motors.output_min= goutput_min;

	TLE5012B_initialise();
    //////////motor initialise//////////////
    overflow=synchronTim123(_speed_hz);//PWMoutInitial();
     motors.dirA=0;
     motors.dirB=0;
     motors.PWM_F= _speed_hz; // <- chang for PWM_F
     motors.MI=1.0f;
    // clock=120000000;
//float theta1=0,theta2=120,theta3=240,Va,Vb,Vc,Val,Vbe,spc_angle,spc_mag; //<-- angle and signal declaration

    motors.T_svm= 1000000.0f/_speed_hz;//sample time of PWM 0.0002;
    motors.ARR_val =overflow;//(clock/2)/PWM_F;
    motors.P_Tsvm_arr = motors.ARR_val/motors.T_svm;
    motors.DegreeToRadin = DEG_TO_RAD*7.0f;





}


void goutput_min(void)
{
  morotrAWrite(1,1,1,0);
  morotrBWrite(1,1,1,0);
}

void gSetUpZeroPosition(U16 SwitchMode,U8 Tdir,U8 motorX)
{
  /// here we will rotate the frame to a level position and set the position flags.
////first move to left  and check the angle
///3960   120  60
///////  180_\/_ 0
///////      /\
///////   240  300
	U16 curReal_positonA; // max = N_SIN*Polar=7*1024 one circle
    U16 curReal_positonB;
    U16 curReal_positonC;

    switch(SwitchMode)
    {
        case 0:
        case 7:
            curReal_positonA=0;
            curReal_positonB=0;
            curReal_positonC=0;
        break;
        case 1://100   
            curReal_positonA=3300;
            curReal_positonB=0;
            curReal_positonC=0;
        break;
        case 2://110
            curReal_positonA=3300;
            curReal_positonB=3300;//3960;
            curReal_positonC=0;
        break;
        case 3://010
            curReal_positonA=0;
            curReal_positonB=3300;//3960;
            curReal_positonC=0;//3960;
        break;
        case 4://011
            curReal_positonA=0;//3960;
            curReal_positonB=3300;
            curReal_positonC=3300;
        break;
        case 5://001
            curReal_positonA=0;//3960;
            curReal_positonB=0;
            curReal_positonC=3300;//3960;
                
        break;     
        case 6://101
            curReal_positonA=3300;//3960;
            curReal_positonB=0;//3960;
            curReal_positonC=3300;
            break;

    }
    if(motorX==0)
      morotrAWrite(curReal_positonA,curReal_positonB,curReal_positonC,Tdir);
    else if(motorX==1)
      morotrBWrite(curReal_positonA,curReal_positonB,curReal_positonC,Tdir);
}
//the input range() is (-180.0 180.0)
void gWriteMotorDirect(float angledegree,float PwrScal,U8 dir,U8 motorX)
{
    float angleRadian = 0.0f,A,B,C;
    angleRadian = angledegree*PI/180.0f;

    A=(1.0f+f32sin(angleRadian))*2000.0f*PwrScal;
    B=(1.0f+f32sin(angleRadian+2.094395f))*2000.0f*PwrScal;
    C=(1.0f+f32sin(angleRadian-2.094395f))*2000.0f*PwrScal;
    if(motorX==0)
      morotrAWrite(A,B,C,dir);
    else if(motorX==1)
      morotrBWrite(A,B,C,dir);
   // hal.uartD->printf("A=%f,B=%f,C=%f",A,B,C);
    //hal.uartD->printf("a=%f,b=%f,c=%f",angleRadian,angleRadian-2.094395f,f32sin(angleRadian-2.094395f));
}


U8 sec_idn(float angle)
{
    U8 sec_sig;
    sec_sig = (angle/60 )+ 1;
    if(sec_sig == 7)
    {
        sec_sig =6;
    }
    return sec_sig;
}
void tim_cal(void)
{
    //U8 ac_f =0;
    motors.T1 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin((motors.sector*PI/3.0f)-motors.spc_angle);//sqrt(3)
    motors.T2 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin(motors.spc_angle-(motors.sector-1)*PI/3.0f);//sqrt(3)
    motors.T0 = motors.T_svm -motors.T1 -motors.T2;
}



void gSVPWM_F4_DQ_speed2PostionA(float Sensorangle,float senDgr)
{
//#define omegaI 0.7f
#define POSITION_PID1
    float Devation,dt=0,angleSpeed=0,agError=0;//tmpEangle ,
    static  float lastError=0.0f,Integrator=0.0f,lastAngle=0,Vd=0 ,Vq=0,lastDervation=0;

    static U32 lastUs=0,cunt=0;
        dt= (float)(micros()-lastUs)/1000000.0f;
        lastUs = micros();
    /////////////first PID Loop -> positon  of angle
///have 7 pole
#ifdef POSITION_PID1
        agError = (Sensorangle-senDgr);//180.0f-Sensorangle;
        if(agError>=180.0f)
        {
          agError=agError-360.0f;
        }
        else if(agError<=-180.0f)
        {
          agError=agError+360.0f;
        }
         agError = agError*motors.angle_P;
        Integrator += (agError*motors.omegaI*dt);// will run forever
         // if(fabs(agError)<=8.0f)
         //   angleSpeed =Integrator+ omegaP*(agError) + omegaD*(agError-lastError)/dt;//+ lastError;//0.0025f*agError/dt;//10.0f*lastError+  
         // else if(fabs(agError)<15.0f)
        if(Integrator>0.6f) Integrator=0.6f;
        else if(Integrator<-0.6f) Integrator= -0.6f;

        Devation = motors.omegaD*(agError-lastError)/dt;
        lastDervation += (Devation-lastDervation)*0.3f;//lowpass filter tmp*a +(lastDervation)*(1-a);

        angleSpeed =Integrator+ motors.omegaP*(agError)+ lastDervation;
        lastError = agError;

#endif
        if(angleSpeed>VqMax) Vq=VqMin;
        else if(angleSpeed<VqMin) Vq = VqMax;
        else Vq= -angleSpeed;
        //else if(angleSpeed<0.20f) angleSpeed=0.2f;
    //////////////////protype///////////////////////////////////////////////
        //Vα = Vd*cosθ - Vq*sinθ;
        //Vβ = Vq*cosθ + Vd*sinθ;
      ///first set Vd=0 ,just set the vq.
        Vd = agError/180.0f;
        if(Vd>1.0f) Vd =0.95f;
        else if(Vd<-1.0f) Vd=-0.95f;
      //  
        motors.theta =  senDgr*motors.DegreeToRadin + motors.elect_machion_offsetA;

        motors.alpha = Vd*f32cos(motors.theta)-Vq*f32sin(motors.theta);
        motors.beta  = Vq*f32cos(motors.theta)+Vd*f32sin(motors.theta);
        motors.spc_angle = fast_atan2(motors.beta,motors.alpha); 

                if(motors.spc_angle <0)
                {
                    motors.spc_angle = TwoPI + motors.spc_angle;
                }
                /////////////////////////Q_r waest more time here 
                motors.spc_mag= InvSqrt(motors.alpha*motors.alpha+motors.beta*motors.beta);//InvSqrt
                //////sector = sec_idn(motors.spc_angle*RAD_TO_DEG);
                motors.sector = (motors.spc_angle*RAD_TO_DEG/60 )+ 1;
               if(motors.sector == 7) motors.sector =6;
               ///////////////////time cal in sector//////////////////////////
                motors.T1 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin((motors.sector*PAIbyTri)-motors.spc_angle);//sqrt(3)
                motors.T2 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin(motors.spc_angle-(motors.sector-1)*PAIbyTri);//sqrt(3)
                motors.T0 = motors.T_svm -motors.T1 -motors.T2;
                ///tim_cal();//cal the Tua Tub Tuc 
                
                switch(motors.sector)
                {
                       case 1:
                            motors.tA = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T0/2.0f;
                            break;
                        case 2:
                            motors.tA = motors.T_svm - (motors.T2 + motors.T0/2.0f);
                            motors.tB = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T0/2.0f;
                            break;
                        case 3:
                            motors.tA = motors.T0/2.0f;
                            motors.tB = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T2 + motors.T0/2.0f;
                            break;
                        case 4:
                            motors.tA = motors.T0/2.0f;
                            motors.tC = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T_svm-(motors.T2 + motors.T0/2.0f);
                            break;
                        case 5:                             
                            motors.tA = motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T0/2.0f;
                            motors.tC = motors.T1 + motors.T2 + motors.T0/2.0f;
                            break;
                        case 6:
                            motors.tA = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T0/2.0f;
                            motors.tC = motors.T_svm-(motors.T2 + motors.T0/2.0f);
                            break;
                }
                 motors.tA *= motors.P_Tsvm_arr;
                 motors.tB *= motors.P_Tsvm_arr;
                 motors.tC *= motors.P_Tsvm_arr;
    morotrAWrite(motors.tA,motors.tB,motors.tC,motors.dirA);
    cunt++;
   if(cunt==100) //10ms 100Hz
     {
        ANO_DT_Send_Status(1,angleSpeed,senDgr,motors.theta,(senDgr-lastAngle)/dt*100,1,1);//,(micros()-lastUs)
        cunt=0;
     }
     lastAngle = senDgr;
}
void gSVPWM_F4_DQ_speed2PostionB(float Sensorangle,float senDgr)
{ 
//#define omegaI 0.7f
#define POSITION_PID2
    float Devation,dt=0,angleSpeed=0,agError=0;//tmpEangle ,
    static  float lastError=0.0f,Integrator=0.0f,Vd=0 ,Vq=0,lastAngle=0,lastDervation=0;//

    static U32 lastUs=0,cunt=0;
        dt= (float)(micros()-lastUs)/1000000.0f;
        lastUs = micros();
    /////////////first PID Loop -> positon  of angle
///have 7 pole
#ifdef POSITION_PID2
        agError = (Sensorangle-senDgr);//180.0f-Sensorangle;
        if(agError>=180.0f)
        {
          agError=agError-360.0f;
        }
        else if(agError<=-180.0f)
        {
          agError=agError+360.0f;
        }
        agError = agError*motors.angle_P;
        Integrator += (agError*motors.omegaI*dt);// will run forever
         // if(fabs(agError)<=8.0f)
         //   angleSpeed =Integrator+ omegaP*(agError) + omegaD*(agError-lastError)/dt;//+ lastError;//0.0025f*agError/dt;//10.0f*lastError+  
         // else if(fabs(agError)<15.0f)
        if(Integrator>0.6f) Integrator=0.6f;
        else if(Integrator<-0.6f) Integrator= -0.6f;

        Devation = motors.omegaD*(agError-lastError)/dt;
        lastDervation += (Devation-lastDervation)*0.3f;//lowpass filter tmp*a +(lastDervation)*(1-a);

        angleSpeed =Integrator+ motors.omegaP*(agError)+ lastDervation;
        lastError = agError;

#endif
        if(angleSpeed>VqMax) Vq=VqMin;
        else if(angleSpeed<VqMin) Vq = VqMax;
        else Vq= -angleSpeed;
        //else if(angleSpeed<0.20f) angleSpeed=0.2f;
    //////////////////protype///////////////////////////////////////////////
        //Vα = Vd*cosθ - Vq*sinθ;
        //Vβ = Vq*cosθ + Vd*sinθ;
      ///first set Vd=0 ,just set the vq.
        Vd = agError/180.0f;
        if(Vd>1.0f) Vd =0.95f;
        else if(Vd<-1.0f) Vd=-0.95f;
      //  
        motors.theta =  senDgr*motors.DegreeToRadin+motors.elect_machion_offsetB;

        motors.alpha = Vd*f32cos(motors.theta)-Vq*f32sin(motors.theta);
        motors.beta  = Vq*f32cos(motors.theta)+Vd*f32sin(motors.theta);
        motors.spc_angle = fast_atan2(motors.beta,motors.alpha); 

                if(motors.spc_angle <0)
                {
                    motors.spc_angle = TwoPI + motors.spc_angle;
                }
                /////////////////////////Q_r waest more time here 
                motors.spc_mag= InvSqrt(motors.alpha*motors.alpha+motors.beta*motors.beta);//InvSqrt
                //////sector = sec_idn(motors.spc_angle*RAD_TO_DEG);
                motors.sector = (motors.spc_angle*RAD_TO_DEG/60 )+ 1;
               if(motors.sector == 7) motors.sector =6;
               ///////////////////time cal in motors.sector//////////////////////////
                motors.T1 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin((motors.sector*PAIbyTri)-motors.spc_angle);//sqrt(3)
                motors.T2 = 1.73205f*motors.T_svm*motors.spc_mag*f32sin(motors.spc_angle-(motors.sector-1)*PAIbyTri);//sqrt(3)
                motors.T0 = motors.T_svm -motors.T1 -motors.T2;
                ///tim_cal();//cal the Tua Tub Tuc 
                
                switch(motors.sector)
                {
                       case 1:
                            motors.tA = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T0/2.0f;
                            break;
                        case 2:
                            motors.tA = motors.T_svm - (motors.T2 + motors.T0/2.0f);
                            motors.tB = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T0/2.0f;
                            break;
                        case 3:
                            motors.tA = motors.T0/2.0f;
                            motors.tB = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tC = motors.T2 + motors.T0/2.0f;
                            break;
                        case 4:
                            motors.tA = motors.T0/2.0f;
                            motors.tC = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T_svm-(motors.T2 + motors.T0/2.0f);
                            break;
                        case 5:                             
                            motors.tA = motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T0/2.0f;
                            motors.tC = motors.T1 + motors.T2 + motors.T0/2.0f;
                            break;
                        case 6:
                            motors.tA = motors.T1 + motors.T2 + motors.T0/2.0f;
                            motors.tB = motors.T0/2.0f;
                            motors.tC = motors.T_svm-(motors.T2 + motors.T0/2.0f);
                            break;
                }
                 motors.tA *= motors.P_Tsvm_arr;
                 motors.tB *= motors.P_Tsvm_arr;
                 motors.tC *= motors.P_Tsvm_arr;
    morotrBWrite(motors.tA,motors.tB,motors.tC,motors.dirB);
    cunt++;
   if(cunt==100) //5ms 
     {
       // ANO_DT_Send_Status(1,angleSpeed,senDgr,theta,(senDgr-lastAngle)/dt*100,1,1);//,(micros()-lastUs)
        cunt=0;
     }
     lastAngle = senDgr;
}

