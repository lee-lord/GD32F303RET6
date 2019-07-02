#include "Mavlink_GCS.h"
#include "STM_GCS.h"
#include "StateMachine.h"
#include "lcd.h"
#include "adc.h"
#include "key.h"
#include "delay.h"
#include "flash_config.h"
#include "gps.h"
#include "sysTick.h"
#include <math.h>
#include "Stick.h"
#include "24L01.h"
#include "ppm.h"
#include "config.h"
#include "drv_mpu6050.h"
#include "pwm.h"
#include "PIDctrl.h"
Gps_Status gps_status;
#define Deg2Rad   0.01745329
#define LEADDISTANCE 7.0
uint32_t lastMavlinkTick;
RemoteStatus remoteStatus;
FLY_SET_PARA g_flight_Param[16]={
{ 0, 0,"WPNAV_SPEED", 0    },
{ 0, 0,"WPNAV_ACCEL",  0   },
{ 0, 0,"WP_YAW_BEHAVIOR",0 },
{ 0, 0,"CIRCLE_RATE",  0  },
{ 0, 0,"CIRCLE_RADIUS", 0  },
{ 0, 0,"FS_BATT_VOLTAGE", 0  },
{ 0, 0,"SIMPLE", 0  },
{ 0, 0,"SUPER_SIMPLE", 0  },
{ 0, 0,"RC2_TRIM", 0  },
{ 0, 0,"RC3_DZ", 0  },
{ 0, 0,"RC3_TRIM", 0  },
{ 0, 0,"RC4_DZ", 0  },
{ 0, 0,"RC4_TRIM", 0  }
};

extern BoardConfig boardConfig;
//extern RC_BATT_INFO RCx_BatInfo;
extern uint8_t key_flag;
//extern RC_BATT_INFO RCx_BatInfo;
//volatile u16 g_flight_mod=0;
//volatile u16 rFLyMODE=0;
volatile u16 g_thro=1100; 
volatile u16 ch1_Roll=1500;
volatile u16 ch2_Pitch=1500;
volatile u16 ch5_gimble=1500;
volatile u16 ConditionYawHZ=_1HZ;
 typedef struct{
 s32 lat;
 s32 lon;
} GPS_RAW;

volatile GPS_RAW SendGPS;

mavlink_param_value_t Flyparam;
mavlink_gps_raw_int_t Flygps;
mavlink_heartbeat_t HEART;
mavlink_scaled_pressure_t pressure;
mavlink_command_ack_t FLYcommand_ack;
mavlink_radio_t radio;
mavlink_mission_ack_t FLYmission_ack;
mavlink_statustext_t  statustext;
mavlink_sys_status_t sys_status;
mavlink_global_position_int_t global_pos_int;
extern Gps_Status gps_status;
extern mavlink_channel_t downstream_channel;
MSGBOX sys_Info_Box;
u32 send=0;
u16 ProcessorCunt=0;
//u8 FllowMeStart=0;
u8 MAV_StatusStart=1;
u32 beatHart=0;
float Coptor_User_bear=0;

extern u16 ch5;

static u8 state=0;
static u32 lastBuzzerTime=0;
static u32 ContinuTimes=100;//ms 
static u8 times=0;            //times=(times of buzz)*2-1
//u8 yaw_start_stop=0;
/////WPNAV_SPEED
/////WPNAV_ACCEL
/////WP_YAW_BEHAVIOR
/////CIRCLE_RATE
/////CIRCLE_RADIUS

//return 1 = armed   0 = disarmed


static uint16_t m_parameter_i = ONBOARD_PARAM_COUNT;
extern 	volatile u8 fixed_CH_set;


u8 Disarm_Arm_check(void)
{ 
	u8 temp =!((sys_Info_Box.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0);
  return(temp);
}
u8 CheckANDcompMODE(u16 modes)
{
  if(sys_Info_Box.custom_mode==modes)
		return 1;
	else
		return 0;
}

void Mavlink_Init(){
	memset(&remoteStatus,0,sizeof(RemoteStatus));
	Flyparam.param_index=0;
	Flyparam.param_count=0xfff;
	sys_Info_Box.Loitor_yaw=1500;
	sys_Info_Box.FllowMeStart=0;
	sys_Info_Box.LeadMeStart=0;
  sys_Info_Box.yaw_start_stop=0;
	sys_Info_Box.oneKeyFly=0;
	sys_Info_Box.voltage_battery=10800;
	sys_Info_Box.CH8_Mode_Super=1100;
	sys_Info_Box.eph=1000;
	sys_Info_Box.warring=0;
		sys_Info_Box.SYS_SHOW=0;
	sys_Info_Box.ch6_Land_on_off=1;
	sys_Info_Box.ch6_led_on_off=1;
  gps_status.lat=0;
  gps_status.lng=0;
	sys_Info_Box.cells=3;
//	if(CheckANDcompMODE(MODE_LOITER)==0)
//	  _send_SET_MODE(01,MODE_LOITER);
//	delay_ms(100);
//	if(CheckANDcompMODE(MODE_LOITER)==0)
//	_send_SET_MODE(01,MODE_LOITER);
}
////////////local function
void SetProfessionalMode(u8 pmode)
{
  if(pmode==PROFESSIONAL_MODE)
	{
//		if(Disarm_Arm_check()==ARMED)
//		_send_SET_MODE(01,MODE_STABILIZE);//set to stablise mode for professional
		sys_Info_Box.user_mode =PROFESSIONAL_MODE;
  }
	else if(pmode==JUNIOR_MODE) // set to loitor mode for junior operator
	{
		sys_Info_Box.user_mode =JUNIOR_MODE;
//	if(CheckANDcompMODE(MODE_LOITER)==0)
//	  _send_SET_MODE(01,MODE_LOITER);
//	   delay_ms(100);
//	if(CheckANDcompMODE(MODE_LOITER)==0)
//	   _send_SET_MODE(01,MODE_LOITER);
  }

}

void set_OPERATOR_mode(u16 mode)
{
   sys_Info_Box.StickMode=mode;
}



void setParam(char *ptr,float value)
{u8 i=0;
	for(i=0;i<5;i++)
	if(strcmp(ptr,g_flight_Param[i].param_id)==0)
	{
		    g_flight_Param[i].param_value=value;
   _send_PARAM_SET(g_flight_Param[i].param_id,value,g_flight_Param[i].param_type);
  }
}

void reset_mavlink(){
	remoteStatus.heartbeatCount = 0;
}

void STM_update(mavlink_channel_t chan, HANDLER handler) {
    mavlink_message_t msg;
    mavlink_status_t status;
    while(comm_get_available(chan)){
        uint8_t c = (uint8_t)comm_receive_ch(chan);
        bool newmsg = mavlink_parse_char(chan, c, &msg, &status);
        if (newmsg) {
            handler(chan, &msg);
        }
    }
}



__inline void downstream_handler(mavlink_channel_t chan, mavlink_message_t* msg) {
	//s8 text[50];
	static u32 lastWarringTime=0;
	static u16 warring=0;
	static u32 lastBeatTime=0;
	u8 i=0;
    switch (msg->msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT://reuest the GPS ,get the flymode current,
			{
				mavlink_msg_heartbeat_decode(msg,&HEART);
				sys_Info_Box.custom_mode=HEART.custom_mode;
				sys_Info_Box.base_mode = HEART.base_mode;
				sys_Info_Box.system_status= HEART.system_status;
				Remo_Status.base_mode =HEART.base_mode;
				Remo_Status.custom_mode =HEART.system_status;
			  remoteStatus.heartbeatCount++;
				sys_Info_Box.connect_state=1;//connected
				lastBeatTime=systemTickMs;
//				if(HEART.system_status == MAV_STATE_STANDBY&&send==0) 
//				{// only when coptor in ground disarmed ,to read parameter
				if(Disarm_Arm_check()==ARMED&&remoteStatus.heartbeatCount>2)
				{ if(send==0)
					_send_PARAM_REUEST_READ();//para request
       
				///start sysstatus read cmd
				if(MAV_StatusStart==3){	
					_send_DATA_STREAM_REQUEST(MAV_DATA_STREAM_EXTRA3,1);// we need the unix time from coptor per second
					MAV_StatusStart=0;
				   }
					if(MAV_StatusStart==2)
					{  MAV_StatusStart=3;
						_send_DATA_STREAM_REQUEST(MAV_DATA_STREAM_POSITION,1);  //MAV_DATA_STREAM_POSITION
           }
				 if(MAV_StatusStart==1)
					{  MAV_StatusStart=2;
            _send_DATA_STREAM_REQUEST(MAV_DATA_STREAM_EXTENDED_STATUS,1);//MAV_DATA_STREAM_POSITION MSG_ATTITUDE MAV_DATA_STREAM_RAW_SENSORS STREAM_POSITION
          }
				}
//	      sprintf(text,"Fmod=%1d,ARMED=%d",rFLyMODE,(HEART.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0);
//        LCD_ShowString(0,28,128,128,12,text,RED,GREEN);
        	 
        break;
     }
      case MAVLINK_MSG_ID_GPS_RAW_INT:
				{ 
          mavlink_msg_gps_raw_int_decode(msg,&Flygps);
				  sys_Info_Box.satellites_visible=Flygps.satellites_visible;
					sys_Info_Box.lat          =Flygps.lat;
					sys_Info_Box.lon          = Flygps.lon;
				  sys_Info_Box.eph 					= Flygps.eph;
					sys_Info_Box.cog          = Flygps.cog;
					sys_Info_Box.vel          = Flygps.vel;
					Remo_Gps.ground_speed     = sys_Info_Box.vel;
					sys_Info_Box.fix_type     =Flygps.fix_type;
					if(sys_Info_Box.satellites_visible==255) 
						 sys_Info_Box.satellites_visible=0;
					Remo_Status.GPS_Staus     = (sys_Info_Box.satellites_visible<<3)|Flygps.fix_type;
					
					Remo_Gps.lat= sys_Info_Box.lat;
					Remo_Gps.lon = sys_Info_Box.lon;
					//Remo_Status.
		     if(gps_status.lat==0&&gps_status.lng==0)//initiallize the home location only once  when we connect the coptor
				 {// initial Home location
         gps_status.lat=Flygps.lat;
         gps_status.lng=Flygps.lon;}
			   }//sys_Info_Box.fixed=;
			break;
			 case MAVLINK_MSG_ID_PARAM_VALUE: // only once trig by 
			 {int i=0;
				 mavlink_msg_param_value_decode( msg, &Flyparam);
				 for(i=0;i<6;i++)
				    {
							if(strcmp(Flyparam.param_id,g_flight_Param[i].param_id)==0)
							{
								g_flight_Param[i].param_index=Flyparam.param_index;
								g_flight_Param[i].param_type=Flyparam.param_type;
								g_flight_Param[i].param_value=Flyparam.param_value;
								// sprintf(text,"%s",g_flight_Param[i].param_id);
								// LCD_ShowString(0,24,128,128,12,text,RED,GREEN);
								break;
							}
						}
		//		setParam("WP_YAW_BEHAVIOR",2);////////2016-3-22  only one
				send =1;		
			ProcessorCunt++;
			 }
				 break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST://get SYSID COPID
			 break;
			case MAVLINK_MSG_ID_GPS_STATUS:
             break;
			case MAVLINK_MSG_ID_SYS_STATUS:
				 mavlink_msg_sys_status_decode( msg,&sys_status);
			   sys_Info_Box.voltage_battery=  sys_status.voltage_battery;
			u8 Cells=3;
			u8 tempV=0;
	   if(g_flight_Param[5].param_value>14.0) 
	      {
		       Cells=4;
	      }
		  else if(g_flight_Param[5].param_value>10.5) 
		   {
			    Cells=3;
	    	}

		if(sys_Info_Box.voltage_battery<(3600*Cells)) sys_Info_Box.voltage_battery=3600*Cells;
				tempV=(sys_Info_Box.voltage_battery-(3600*Cells))*11/(600*Cells);
		    if(tempV>11)tempV=11;	
			   Remo_Status.voltage_battery =tempV;// set battV
			                              // =sys_status.drop_rate_comm;
			   
//			   sprintf(text,"BA=%d",sys_status.voltage_battery);
//    				LCD_ShowString(0,64,128,128,12,text,RED,GREEN);
				break;
			case MAVLINK_MSG_ID_SCALED_PRESSURE:
			{ 
				mavlink_msg_scaled_pressure_decode(msg,&pressure);
				sys_Info_Box.temperature = pressure.temperature;
				sys_Info_Box.press_abs   = pressure.press_abs;
//				sprintf(text,"Temp=%d,P=%4.4f",pressure.temperature,pressure.press_abs);
//				LCD_ShowString(0,24,128,128,12,text,RED,GREEN);
      }
			break;
			case MAVLINK_MSG_ID_COMMAND_ACK:
				mavlink_msg_command_ack_decode(msg, &FLYcommand_ack);
//				sprintf(text,"ACK=%d",msg->magic);
//        LCD_ShowString(10,40,128,128,12,text,RED,GREEN);
			break;
			case MAVLINK_MSG_ID_MISSION_ACK:
				mavlink_msg_mission_ack_decode(msg,&FLYmission_ack);
			 sys_Info_Box.target_component  =  FLYmission_ack.target_component;
			 sys_Info_Box.target_system     =  FLYmission_ack.target_system;
			break;
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				mavlink_attitude_t atti;
				mavlink_msg_attitude_decode(msg,&atti);
//				sprintf(text,"ROLL=%4f PITCH=%4f YAW=%4f",atti.roll,atti.pitch,atti.yaw);
//				LCD_ShowString(0,60,128,128,12,text,RED,GREEN);
			}
			break;
			case MAVLINK_MSG_ID_SENSOR_OFFSETS://150
			{mavlink_sensor_offsets_t offsef;
				mavlink_msg_sensor_offsets_decode(msg,&offsef);
//				sprintf(text,"GYRO x=%4fY=%4fZ=%4f",offsef.accel_cal_x,offsef.accel_cal_y,offsef.accel_cal_z);
//				LCD_ShowString(0,36,128,128,12,text,RED,GREEN);
			}
				break;
			case MAVLINK_MSG_ID_RAW_IMU://27
			{
				mavlink_raw_imu_t imu;
				mavlink_msg_raw_imu_decode(msg,&imu);
//				sprintf(text,"Acc X=%4dY=%4dZ=%4d",imu.xacc,imu.yacc,imu.zacc);
//				LCD_ShowString(0,48,128,128,12,text,RED,GREEN);
			}
				break;
			case MAVLINK_MSG_ID_RADIO_STATUS:
			case MAVLINK_MSG_ID_RADIO:
				mavlink_msg_radio_decode( msg, &radio);
			sys_Info_Box.remrssi=radio.remrssi;
			sys_Info_Box.rssi= radio.rssi;
//			  sprintf(text,"rssi=%d,rrssi=%3d",radio.rssi,(u8)radio.remrssi);
//				LCD_ShowString(0,40,128,128,12,text,RED,GREEN);
				break;
			//case MAVLINK_MSG_ID_RALLY_POINT:// unsurport in apm low cpu
				//mavlink_msg_rally_point_decode(msg, &rally_point);
//				break;
     case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			  mavlink_msg_global_position_int_decode( msg, &global_pos_int);
		    sys_Info_Box.alt=global_pos_int.relative_alt;
		    sys_Info_Box.hdg=global_pos_int.hdg ;
		    sys_Info_Box.vx =global_pos_int.vx ;
		    sys_Info_Box.vy =global_pos_int.vy ;
		   Remo_Gps.velned_x = sys_Info_Box.vx;
		   Remo_Gps.velned_y =  sys_Info_Box.vy;
		   
		   Remo_Status.high =(u16)(sys_Info_Box.alt/1000);///m
		   Remo_Status.Hdop =sys_Info_Box.hdg;
#if RC_WATCH
		 if(sys_Info_Box.alt>=50000)
		   { 
				{ 
			 if(1600<=g_thro)//climb ups 
			    set_g_thro(1500);//stop climbe up
		    }
			 }
#endif
			 //stop climbe up
//		 	   sprintf(text,"Alt%3d,bear=%4d ",global_pos_int.relative_alt,(u16)(global_pos_int.hdg)/100);
//				LCD_ShowString(0,36,128,128,12,text,RED,GREEN);
			 break;
		 case MAVLINK_MSG_ID_SYSTEM_TIME:
		 {mavlink_system_time_t  system_time;
			 mavlink_msg_system_time_decode(msg, &system_time);
			 sys_Info_Box.UNIX_time    = system_time.time_unix_usec ;
			 Remo_Status.UNIX_time = system_time.time_unix_usec ;
//				   sprintf(text,"%16x", sys_Info_Box.UNIX_time);
//				  LCD_ShowString(0,16,128,128,12,text,RED,GREEN);
//			    sprintf(text,"%8x", (sys_Info_Box.UNIX_time)>>32);
//				  LCD_ShowString(0,16,128,128,12,text,RED,GREEN);
			 MAV_StatusStart=0;
		 }
			 break;
		 case MAVLINK_MSG_ID_STATUSTEXT:
     {
			mavlink_msg_statustext_decode(msg, &statustext);
			if(statustext.severity==SEVERITY_HIGH)
			{
				 for(i=0;i<35;i++)
					 {
					    if(0==strstr(statustext.text,&CoptorWaring[i].PWaring[0]))
							{ 
								Remo_Status.TxtNum= CoptorWaring[i].WarID;
								break;
							}
					  }
				if(0!=strstr(statustext.text,"PreArm: "))
				 {
		       // LCD_ShowString(0,64,128,128,12,statustext.text+8,RED,GREEN);
					 warring=1;
					 lastWarringTime=systemTickMs;
              }
            }
			}
			 break;
      default:
				break;
    }
		if(systemTickMs-lastBeatTime>LostBeatHeartTime)
		{
			  sys_Info_Box.connect_state=0;
			  sys_Info_Box.custom_mode=66;
			 //MAV_StatusStart =1; //try to reconnection
			 //send=0;
		}
		 if((systemTickMs-lastWarringTime>LostBeatHeartTime)&&warring)
	     {
			   ///LCD_ShowString(0,64,128,128,12,"                                     ",RED,BLACK);
		     warring=0;
			   Remo_Status.TxtNum=0;
	  }
}

void send_RF_Parameter(void)
{
  if (m_parameter_i < ONBOARD_PARAM_COUNT)
	{
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
				boardConfig.board.gimble.param_name[m_parameter_i],
				boardConfig.board.gimble.param[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
		(m_parameter_i)++;
	}

}


__inline void upstream_handler(mavlink_channel_t chan, mavlink_message_t* msg) {
	//s8 text[50];
//	static u32 lastWarringTime=0;
//	static u16 warring=0;
	static u32 lastBeatTime=0;
	u8 i=0;
    switch (msg->msgid) 
			{
      case MAVLINK_MSG_ID_HEARTBEAT://reuest the GPS ,get the flymode current,
		//		m_parameter_i = 0;
        break;
		 case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		   {
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system
					== (uint8_t) boardConfig.board.gimble.param[PARAM_SYSTEM_ID]
												   && (uint8_t) set.target_component
												   == (uint8_t) boardConfig.board.gimble.param[PARAM_COMPONENT_ID])
			{
				char* key = (char*) set.param_id;
				if (set.param_id[0] != (char)-1)
				{
					/* Choose parameter based on index */
					if ((set.param_index >= 0) && (set.param_index < ONBOARD_PARAM_COUNT))
					{
						/* Report back value */
				mavlink_msg_param_value_send(chan,
								boardConfig.board.gimble.param_name[set.param_index],
								boardConfig.board.gimble.param[set.param_index], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, set.param_index);
					}
				}
				else
				{
					int i =0,j=0;
					for (  i = 0; i < ONBOARD_PARAM_COUNT; i++)
					{
						bool match = true;
						for (  j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
						{
							/* Compare */
							if (((char) (boardConfig.board.gimble.param_name[i][j]))
									!= (char) (key[j]))
							{
								match = false;
							}

							/* End matching if null termination is reached */
							if (((char) boardConfig.board.gimble.param_name[i][j]) == '\0')
							{
								break;
							}
						}

						/* Check if matched */
						if (match)
						{
							/* Report back value */
							mavlink_msg_param_value_send(chan,
									boardConfig.board.gimble.param_name[i],
									boardConfig.board.gimble.param[i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, i);
						}
					}
				}
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			/* Start sending parameters */
			m_parameter_i = 0;
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system
					== (uint8_t) boardConfig.board.gimble.param[PARAM_SYSTEM_ID]
												   && (uint8_t) set.target_component
												   == (uint8_t) boardConfig.board.gimble.param[PARAM_COMPONENT_ID])
			{
				int i=0,j=0;
				char* key = (char*) set.param_id;

				for (  i = 0; i < ONBOARD_PARAM_COUNT; i++)
				{
					bool match = true;
					for (  j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
					{
						/* Compare */
						if (((char) (boardConfig.board.gimble.param_name[i][j]))
								!= (char) (key[j]))
						{
							match = false;
						}

						/* End matching if null termination is reached */
						if (((char) boardConfig.board.gimble.param_name[i][j]) == '\0')
						{
							break;
						}
					}

					/* Check if matched */
					if (match)
					{
						/* Only write and emit changes if there is actually a difference
						 * AND only write if new value is NOT "not-a-number"
						 * AND is NOT infinity
						 */
						if ((boardConfig.board.gimble.param[i]!= set.param_value)
								&& !isnan(set.param_value)
								&& !isinf(set.param_value)
								&& boardConfig.board.gimble.param_access[i])
						    {
							     boardConfig.board.gimble.param[i] = set.param_value;
						    }

							/* report back new value */
							mavlink_msg_param_value_send(chan,
									boardConfig.board.gimble.param_name[i],
									boardConfig.board.gimble.param[i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, i);
						if(i==(u8)PARAM_ORENTATIONACC)//if(0!=strstr((const char *)&(set.param_id),"ORENTATIONACC"))
						{////set orentation set 
                accAlign=(u8)set.param_value;
						}else if(i==(u8)PARAM_ORENTATIONGYRO)//(0!=strstr((const char *)&(set.param_id),"ORENTATIONGYR"))
						{////set orentation set 
						    gyroAlign=(u8)set.param_value;
							
						}
						else if(i==(u8)PARAM_PITCH_DIR)
						{
							P_Pid.dir=(s8)set.param_value;
						}
							else if(i==(u8)PARAM_ROLL_DIR)
							{
								R_Pid.dir=(s8)set.param_value;
							}
								else if(i==(u8)PARAM_YAW_DIR)
								{
									Y_Pid.dir=(s8)set.param_value;
								}
						else if(i==(u8)DEBUG_VARIABLE)
						{
							if(set.param_value>1){
								  P_Pid.run=0;
		               R_Pid.run=0;
		               Y_Pid.run=0;}
							   else
								 {
                   P_Pid.run=1;
		               R_Pid.run=1;
		               Y_Pid.run=1;
								 }
						}
						else if(boardConfig.board.gimble.param[PARAM_DATA_SAVE]==1)//PARAM_DATA_SAVE
						{
							boardConfig.board.gimble.param[PARAM_DATA_SAVE]=0;
							writeBoardConfig(&boardConfig);//save all data in flash
							NVIC_SystemReset();
						}
						}
						else
						{
							/* send back current value because it is not accepted or not write access*/
							mavlink_msg_param_value_send(chan,
									boardConfig.board.gimble.param_name[i],
									boardConfig.board.gimble.param[i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, i);
						}
						
					}
				}
			}
		break;

		case MAVLINK_MSG_ID_PING:
		{
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			if (ping.target_system == 0 && ping.target_component == 0)
			{
				/* Respond to ping */
				uint64_t r_timestamp = systemTickMs;
				mavlink_msg_ping_send(chan, ping.seq, msg->sysid, msg->compid, r_timestamp);
			}
		}
		break;
//      case MAVLINK_MSG_ID_GPS_RAW_INT:
//			break;
//			 case MAVLINK_MSG_ID_PARAM_VALUE: // only once trig by 
//				 break;
//			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST://get SYSID COPID
//			 break;
//			case MAVLINK_MSG_ID_GPS_STATUS:
//             break;
//			case MAVLINK_MSG_ID_SYS_STATUS:
//				break;
//			case MAVLINK_MSG_ID_SCALED_PRESSURE:
//			break;
//			case MAVLINK_MSG_ID_COMMAND_ACK:
//			break;
//			case MAVLINK_MSG_ID_MISSION_ACK:
//			break;
//			case MAVLINK_MSG_ID_ATTITUDE:
//			break;
//			case MAVLINK_MSG_ID_SENSOR_OFFSETS://150
//				break;
//			case MAVLINK_MSG_ID_RAW_IMU://27

//				break;
//			case MAVLINK_MSG_ID_RADIO_STATUS:
//			case MAVLINK_MSG_ID_RADIO:
//				break;
//     case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
//			 break;
//		 case MAVLINK_MSG_ID_SYSTEM_TIME:

//			 break;
//		 case MAVLINK_MSG_ID_STATUSTEXT:

//			 break;
      default:
				break;
    }

		
}

#if RC_STICK==1
////here we define the funtion for RC stick to get the data from RF24L01 and tranlate it to SYSINFO struct.
void SiMToo_Update_Loop(void)
{
static	u32 lastRFdataTime=0;
static	u8 lsatWarringNum=0;
  if(systemTickMs-lastRFdataTime>_50HZ)
	  { 
	 sys_Info_Box.rssi=RFbuf.UsrRssi;			
	 sys_Info_Box.base_mode=Remo_Status.base_mode;
	 sys_Info_Box.hdg=Remo_Status.Hdop;
	 sys_Info_Box.alt=Remo_Status.high*1000;
	 sys_Info_Box.voltage_battery=(Remo_Status.voltage_battery&0x07ff)*10;//1mv
	  sys_Info_Box.cells=(u8)(Remo_Status.voltage_battery>>13);
	 sys_Info_Box.satellites_visible=Remo_Status.GPS_Staus>>3;
	 sys_Info_Box.fix_type=Remo_Status.GPS_Staus&0x07;
	
	 sys_Info_Box.lat = Remo_Gps.lat;
	 sys_Info_Box.lon = Remo_Gps.lon;
   sys_Info_Box.vx  = Remo_Gps.velned_x;
	 sys_Info_Box.vy  = Remo_Gps.velned_y;
	 sys_Info_Box.vel      = Remo_Gps.ground_speed;
			Flygps.lat=sys_Info_Box.lat;
			Flygps.lon=sys_Info_Box.lon;
			lastRFdataTime=systemTickMs;
			sys_Info_Box.UNIX_time=(uint64_t)(Remo_Status.UNIX_time);
			if(sys_Info_Box.rssi==0)//have no data recived 
			{
			  sys_Info_Box.connect_state=0;
			  sys_Info_Box.custom_mode=66;
				sys_Info_Box.remrssi=0;
			}
			else
			{
			  sys_Info_Box.remrssi=Remo_Status.remrssi;
			  sys_Info_Box.custom_mode=Remo_Status.custom_mode;
				sys_Info_Box.connect_state=1;
			}
			
			if(Remo_Status.TxtNum!=lsatWarringNum)
			  { lsatWarringNum=Remo_Status.TxtNum;
					if(Remo_Status.TxtNum)
					 {
//					    LCD_ShowString(0,64,128,128,12,(u8 *)CoptorWaring[Remo_Status.TxtNum].PWaring,RED,GREEN);
					  }
						else
						{
//						  LCD_ShowString(0,64,128,128,12,(u8 *)CoptorWaring[Remo_Status.TxtNum].PWaring,BLACK,BLACK);
						}
				}
		}
	}

void SiMToo_Update_Loop1(void)
{
	static u32 lastMavlinkTick2=0;

	if( systemTickMs-lastMavlinkTick2 >=_50HZ){
			STM_update(downstream_channel,upstream_handler);
			lastMavlinkTick2 = systemTickMs;
		  send_RF_Parameter();
		}
	
}
#elif RC_WATCH ==1
void SiMToo_RF2401_Trancver(u16 * keyInput)
{
  ///////////////////Send GPS of Watch , but RC_Stick has no GPS
	//update RC_ch8s 
	
	////// send RC_CH8s ,USR key input

}

#elif RC_AGENT_OF_COPTOR ==1
// this is for Agent of Coptor  to communication by  mavlink 
void SiMToo_Update_Loop(void)
{
	if(boardConfig.board.radio_ok && systemTickMs-lastMavlinkTick >=_50HZ){
			STM_update(downstream_channel,downstream_handler);
			lastMavlinkTick = systemTickMs;
		}
}
//get the RC_channle , USER GPS status, USER cmd.
//send the status of Coptor(ARMed ,ALT,BattV,GPS FIXED NUMs HDOP,FlightMode,), GPS status,
void SiMToo_RF2401_Trancver(void)
{
     if(RFbuf.Rxok)
		 {
		   RFbuf.Rxok=0;
			 ///////////////////get GPS of Watch , but RC_Stick has no GPS
		   gps_status.hdop=LocalGPS_info.Hdop;
		   gps_status.ground_speed=LocalGPS_info.ground_speed;
			 gps_status.lat=LocalGPS_info.lat;
			 gps_status.lng=LocalGPS_info.lon;
			 gps_status.velned_x=LocalGPS_info.velned_x;
			 gps_status.velned_y=LocalGPS_info.velned_y;
			 gps_status.fix=LocalGPS_info.GPS_Staus&0x0F;
		   gps_status.num_sats=LocalGPS_info.GPS_Staus>>4;
			 gps_status.heading_2d=LocalGPS_info.Hdg;
			 
			 ////// get RC_CH8s ,USR key input
//			                      =SendRc_CHS_CMD.keyValue;
//		                        =SendRc_CHS_CMD.set_alt;
//			                      =SendRc_CHS_CMD.rc[0];
		 }
     
     
}
#endif
////////we need wait for the param of flycontrolor 
u8 check_param_processor(void)
{
   u8 text[50];
	u8 lastcunt=0;
	u32 lastTime=systemTickMs;
  while(1)
	{
//    STM_update(downstream_channel, downstream_handler);
		
		if(RFbuf.recived-lastcunt>30)
	  { 
			 //DrawBar(0  ,32,(RFbuf.recived)*127/125,128,2,GREEN);
          lastcunt=RFbuf.recived;			
//			sprintf(text,"perce=%2d%%",(Flyparam.param_index+1)*100/Flyparam.param_count);
//      LCD_ShowString(32,64,128,128,12,text,RED,GREEN);
			lastTime=systemTickMs;
	  }
		if(RFbuf.recived>=125) break;
		
		if(KEY_Scan()) return 0;
		if(systemTickMs-lastTime>30000)
		{
//			if(sys_Info_Box.connect_state==0)
//				LCD_ShowString(0,64,128,128,12,"lost connect!",RED,BLACK);
//				else
//			  LCD_ShowString(0,64,128,128,12,"ARMED!Can't read Para",RED,BLACK);
				delay_ms(500);
			return 1;
    }
  }
   return 1;
}

void set_g_thro(u16 thro)
{
	if(thro==1100) //if in the air , the cmd will be deny
	{
      if(Disarm_Arm_check()==DISARMED)
       return; 
  }  
	 sys_Info_Box.Holdalt=sys_Info_Box.alt;// update alt when some trige the rise or down key .when user stop rise or down.
   g_thro=thro;
}

void setLedOnOff(u16 * keyvaule)
{
  static u8 status=0;
	static u32 lastRCTime=0;
  if(systemTickMs-lastRCTime>=_2HZ){
		lastRCTime=systemTickMs;
	if(*keyvaule!=LEDONOFF) return;
		 //*keyvaule=0; //clear keys.
		if(sys_Info_Box.user_mode == PROFESSIONAL_MODE) //ADV_pro mode ,check whether in ground
		   {
				  if(sys_Info_Box.system_status == MAV_STATE_STANDBY) // in air ,we need to switch the changed mode to ALT mode .
					  {
							goto LEDContrl;
                      }
						else
						{//in air we cange to ALT_HOLD
							_send_SET_MODE(1,MODE_ALT_HOLD);//setmode
							return;
						}
			}
LEDContrl:			 
		if(status==0){
		       sys_Info_Box.ch6_led_on_off=1;//1900;//on 
		       status=1;}
		else{ sys_Info_Box.ch6_led_on_off=0;status=0;}//1100off light
	 }
}

void Set_Reset_SuperMode(void)
{
	// if coptor is in air ,we can switch to super simple mode
		 //then send superSimple mode to coptor
if(sys_Info_Box.system_status > MAV_STATE_STANDBY)//if(Disarm_Arm_check()!=ARMED) 
	{
if(sys_Info_Box.CH8_Mode_Super==1100)
	 sys_Info_Box.CH8_Mode_Super=1900;
  else
		sys_Info_Box.CH8_Mode_Super=1100;
}
}


void update_RC_20HZ(void)
{
//  static u32 lastRCTime=0;
//	vs32 altmm=0;
//	u16 CH6_Led_LAND=1600;
//	if(FAILSAFEcnt>FAILSAFE_CUNTS) return;
//  if(systemTickMs-lastRCTime>=_20HZ)
//	{
//		char txt[30];
//		lastRCTime =systemTickMs;
//     ch5_gimble= RCx_BatInfo.RC[0].ch;
//		 sys_Info_Box.localBatt=  RCx_BatInfo.Battmv;
//	//	_send_RC_OVERRIDE_Dummy(g_thro,ch1_Roll,ch2_Pitch,sys_Info_Box.Loitor_yaw,ch5_gimble,sys_Info_Box.ch6_led_on_off);
//		CheckUsrMode();//check ARM request
//		if(sys_Info_Box.ch6_Land_on_off==1&&sys_Info_Box.ch6_led_on_off==0) CH6_Led_LAND=1800;  
//			else if(sys_Info_Box.ch6_Land_on_off==1&&sys_Info_Box.ch6_led_on_off==1) CH6_Led_LAND=1600;  
//				else if(sys_Info_Box.ch6_Land_on_off==0&&sys_Info_Box.ch6_led_on_off==1) CH6_Led_LAND=1400;   
//					else if(sys_Info_Box.ch6_Land_on_off==0&&sys_Info_Box.ch6_led_on_off==0) CH6_Led_LAND=1200;  
//		
//		 if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE){
//							 if(sys_Info_Box.alt>ProLimitHigh) altmm = LimitHigh+100;//not limit the hight
//			          else altmm = 10000;
//		          }
//				else
//							 altmm = sys_Info_Box.alt;
//		if(sys_Info_Box.oneKeyFly==0){ /// manual control 
//		     if(sys_Info_Box.StickMode==LEFT_THRO_MODE)
//				 {
//					      _send_RC_OVERRIDE(CH6_Led_LAND,altmm,sys_Info_Box.CH8_Mode_Super);//LEFT thro mode
//				 }
//		    else if(sys_Info_Box.StickMode==RIGHT_THRO_MODE){
//		            _send_RC_OVERRIDE_RIGHT_MODE(CH6_Led_LAND,altmm,sys_Info_Box.CH8_Mode_Super);
//				}
//	  }
//		else{ // auto control
//                _send_RC_OVERRIDE_Dummy(g_thro,1500,1500,1500,RCx_BatInfo.RC[4].ch,CH6_Led_LAND);//RCx_BatInfo.RC[5].ch
//    } 
////		sprintf(txt,"ADC=%d,batt=%d",RCx_BatInfo.RC[0].ch,RCx_BatInfo.Battmv);
////    LCD_ShowString(0,16,128,128,12,txt,WHITE,BLACK);
//  }
}

void update_heartBeat(void)
{
 static u32 lastRCTime=0;
  if(systemTickMs-lastRCTime>=_1HZ)
	{
		lastRCTime =systemTickMs;
    _send_beat_heart(1);
   }
}

void SetYaw_Slow(u16 ch4yaw)
{static u32 lastPressTime=0;
	static s16 lastdata=0;
	if(systemTickMs-lastPressTime>20)
	{  
		lastPressTime=systemTickMs;
   }
	 else
		 return;
	if(Disarm_Arm_check()==ARMED) 
	 { 
		sys_Info_Box.Loitor_yaw=1500;
	 }
		else
    {
			if(ch4yaw==1500)
			{
				sys_Info_Box.Loitor_yaw=ch4yaw;
				lastdata=0;
			}
			else if(ch4yaw==1300)
			{
				if(lastdata<200)
					lastdata+=5;
				else
					lastdata=200;
					sys_Info_Box.Loitor_yaw=1500-lastdata;
      }
			else if(ch4yaw==1700)
			{
         if(lastdata<200)
					lastdata+=5;
				else
					lastdata=200;
					sys_Info_Box.Loitor_yaw=1500+lastdata;
      }
		}
}

void SetYaw(u16 ch4yaw)
{ static u32 lastPressTime=0;
	static s16 lastdata=0;
	if(systemTickMs-lastPressTime>20)
	{  
		lastPressTime=systemTickMs;
   }
	 else
		 return;
	if(Disarm_Arm_check()==ARMED) 
	 { 
		sys_Info_Box.Loitor_yaw=1500;
	 }
		else
    {
			sys_Info_Box.Loitor_yaw=ch4yaw;
		}
}


u8 checkCMD_ACK(u16 cmd)
{
	u8 res=0;
   if(FLYcommand_ack.command==cmd )
	 {//u8 text[50];
	  // sprintf(text,"CMDA=%d",FLYcommand_ack.result);
      //  LCD_ShowString(86,76,128,128,12,text,RED,GREEN);
		 res=FLYcommand_ack.result;
      return res;
	 }
	 else
		  return 0xFF;
}
u8 checkMISSION_ACK(void)
{
   if(FLYmission_ack.type==0 )
	 {
		    u8 text[50];
	      sprintf(text,"MISA=%d",FLYmission_ack.type);
//        LCD_ShowString(88,76,128,128,12,text,RED,GREEN);
      return FLYmission_ack.type;
	 }
	 else
		  return 0xFF;
}

u8 check_Remote_GPS(void)
{
 if(Flygps.satellites_visible>GPS_SATS&&Flygps.eph<GPS_HDOP)
    return 1;
  else return 0xff;
}
//1 << Button_KEY7|(1<<LONGPRESS)
__inline void sendDisArm(u16 *keyvaule)
{
  static u32 lastDisTime=0;
	static u32 lastKeyPress=0;
	static u8 state=0;
  if(systemTickMs-lastDisTime>=_50HZ)
	{
		lastDisTime =systemTickMs;
  }
		else
		{
		return;
	}
//	if(Disarm_Arm_check()!=ARMED) return;
	if(*keyvaule!=(1 << Button_KEY7|(1<<LONGPRESS))&&state==0)return;
	else if((*keyvaule==(1 << Button_KEY7|(1<<LONGPRESS)))&&state==0)
	{
		*keyvaule=0;
		 if(systemTickMs-lastKeyPress>1200)
		 {lastKeyPress = systemTickMs;
			 if(Disarm_Arm_check()==ARMED)
			 {_send_SET_MODE(1,MODE_STABILIZE);
	       _send_DISARM_ARM(ARMED);
			 }
		 }
  }
}
///no need by later 
void _oneChangeAlt(u16 keyvaule)
{
	static u32 state=0;
	static u32 lastRCTime=0;
	static u32 lastKeyPress=0;
	float alt=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	{
		lastRCTime =systemTickMs;
  }
	else
		return;
		if(Disarm_Arm_check()==ARMED) {	 return;}
	if(keyvaule!=(1 << Button_KEY8)&&keyvaule!=(1 << Button_KEY10)&&state==0)return;
	else if((keyvaule==(1 << Button_KEY8)||keyvaule==(1 << Button_KEY10))&&state==0)
	{
		  state=1;
		if(keyvaule==(1 << Button_KEY8)&&keyvaule==(1 << Button_KEY10)) state=0;
		if(keyvaule==(1 << Button_KEY8)) alt =1.0;
		if(keyvaule==(1 << Button_KEY10)) alt =-1.0;
		if(CheckANDcompMODE(MODE_LOITER)!=1) {state=0;return;}// not in GUID mode  can not do it
  }
	switch(state)
	{
    case 1://
			_send_ALT_CHANGED(alt);
		  state++;
			break;
		case 10:
			if(checkMISSION_ACK()!=0) 
			{	
				_send_ALT_CHANGED(alt);
				state=2;
			}
			 else
					 { state=0; return;}
			break;
		default:
			state++;
			break;
  }
}

void _oneKeyFly(u16 *keyvaule)
{
	static u32 state=0;
	static u32 lastRCTime=0;
	static u32 lastKeyPress=0;
	static u32 takeOffTime=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	  {
		    lastRCTime =systemTickMs;
    }
	else
		   return;
	if(*keyvaule!=FLY_KEY&&state==0)
		return;
	else if(*keyvaule==FLY_KEY&&state==0)
	{
		*keyvaule=0;
			if(Disarm_Arm_check()==ARMED){sys_Info_Box.warring=1; 	  return;}
		  state=1;
  }
	switch(state)
   {
		 case 1://preflight check the status of GPS
			 if(check_Remote_GPS()==1)
			 { 
					if(HEART.system_status == MAV_STATE_STANDBY)
					{	 
						_send_SET_MODE(01,MODE_GUIDED);
						state++;
						sys_Info_Box.oneKeyFly=2;
						g_thro=1100;
					}
					else
					{ 
						state =0;
						return;
           }
				}
				else
				  {state =0; return;}
		 break;
		 case 12:
     g_thro=1500;//_send_DISARM_ARM(ARMED);
		 state++;
		 break;
		 case 20:
			 /////check ACK of  MODE_GUIDED;
		 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
		 { _send_SET_MODE(01,MODE_GUIDED);
			 state=4;
     }
		 else
			    state++;
		 break;
	   case 30:
	   _send_TAKEOFF(TAKEOFFALT);
		 takeOffTime=systemTickMs;
		 state++;
		 sys_Info_Box.Holdalt=600;//initial high of fllowme when take off  
		 break;
		 case 80:
			 /////check ACK of  MODE_GUIDED;
		 if(0!=checkCMD_ACK(MAV_CMD_NAV_TAKEOFF))
		 {_send_TAKEOFF(TAKEOFFALT);
		   //state=31;
     }
		 else {
			 state++;
			 sys_Info_Box.oneKeyFly=0; // give up control 
		 }
			    //{ state=0; return;}
		 break;
		 case 100://wait for alt == 5.0m then to switch to loitor mode
			 if(sys_Info_Box.alt>=Stop_To_Loitor)
			 { 
				 _send_SET_MODE(01,MODE_LOITER);
				 state=0; return;
			 }
			 if(systemTickMs-takeOffTime>8000) {state=0;_send_SET_MODE(01,MODE_LOITER); return;}//timeout !!!!
		 break;
	    default:
		 state++;
		 break;
	 }
}

//////need return user local
//////如果飞机电压低，就近降落吗
void _oneKeyLand(u16 *keyvaule)
{
	static u32 state=0;
	static u32 lastRCTime=0;
	static u32 lastKeyPress=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	{
		lastRCTime =systemTickMs;
  }
	else
		return;
	if(*keyvaule!=LAND_KEY&&state==0)
		return;
	else if(*keyvaule==LAND_KEY&&state==0)
	{
		*keyvaule=0;
		  state=1;
  }
	switch(state)
   { case 1:
		     _send_SET_MODE(01,MODE_LAND);
		     state++;
		 break;
     case 40:
			if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			{ 
				_send_SET_MODE(01,MODE_LAND);
				state=2;
			}
		   else
			  state++;
		 break;
		 case 50:  //MAV_STATE_STANDBY
			 if(HEART.system_status == MAV_STATE_STANDBY)
			 { state++;}
			 else
			 { state=41;
       }
		 break;
		 case 60:
     g_thro=1100;
		 state++;
		 break;
     case 70:
		 state++;
		 //recive ACK  to do it
		 _send_DISARM_ARM(DISARMED);
	   break;
     case 100:
			 //if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
	       _send_SET_MODE(01,MODE_STABILIZE);     
			 //else
			//	  _send_SET_MODE(01,MODE_LOITER); 
		 state++;
		  break;
		 case 140:
		   if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE)){
			  // if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
	          _send_SET_MODE(01,MODE_STABILIZE);
			   // else
				  // _send_SET_MODE(01,MODE_LOITER);  
				 state=101;
			 }
		    else
			      state++;
				
		  break;
	   case 148:
		 _send_MISSION_CLEAR();
		 sys_Info_Box.oneKeyFly=0; // give up control 
		 { state=0; return;}
		 break;
	   default:
		 state++;
		 break;
	 }
}

void one_key_Circle(u16 *keyvaule)
{
    static u32 state=0;
		static u32 lastRCTime=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	{
		lastRCTime =systemTickMs;
  }
		else
		return;
	//////////////////////////////////////////////////////////?????????????????????????????need check the mode of circle ////
	/////the center is the user's location
	if(*keyvaule!=CIRCLE_KEY)
		    return;
	else if(*keyvaule==CIRCLE_KEY&&state==0)
	{
		*keyvaule=0;
		if(Disarm_Arm_check()==ARMED) {
     sys_Info_Box.warring=1;  
     return;
     }
			state=1;
//		if(HEART.system_status == MAV_STATE_ACTIVE)
//		{	 state=0; return;}
  }
	switch(state)
   { case 1:
		 setParam("CIRCLE_RADIUS",500);
		  state++;
		 break;
		 case 20:
			 ////need check the param set ok
		   if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==500) 
						 state=40;
			     else
					 {
						 setParam("CIRCLE_RADIUS",500);
						 state++;
           }
				}else
				  state++;
		 break;
		 case 40:
			 if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==500) 
						 state=50;
			     else
					 {
						 setParam("CIRCLE_RADIUS",500);
						 state++;
           }
				}
				else
					state++;
			break; 
		 case 50:
		 //_send_CIRCLE(0,5.0,0.0,Flygps.lat,Flygps.lon,5.0);
		 _send_SET_MODE(1,MODE_CIRCLE);
		 state++;
		 break;
		 case 80:
			 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			     _send_SET_MODE(01,MODE_CIRCLE);
		  { state=0; return;}
		 break;
	   default:
		 state++;
		 break;
	 }
}

void one_key_rotate_self1(u16 * keyvaule)
{
    static u32 state=0;
		static u32 lastRCTime=0;
	  static u32 lastKeyPress=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	{
		lastRCTime =systemTickMs;
  }
		else
		{
		return;
	}
		if(Disarm_Arm_check()==ARMED) {	  return;}
	//////////////////////////////////////////////////////////?????????????????????????????need check the mode of circle ////
	/////the center is the user's location
	if(*keyvaule!=(1 << Button_KEY9|(1<<LONGPRESS))&&state==0)
		    return;
	else if(*keyvaule==((1 << Button_KEY9)|(1<<LONGPRESS))&&state==0)
	{
		*keyvaule=0;
		  state=1;
  }

	switch(state)
   { case 1:
		      _send_SET_MODE(01,MODE_LOITER);
		  state++;
		 break;
		 case 20:
			if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
		     _send_SET_MODE(01,MODE_LOITER);
			  state++;
		 break;
		 case 80:
			if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
		     _send_SET_MODE(01,MODE_LOITER);
		  { state=0; return;}
		 break;
	   default:
		 state++;
		 break;
	 }
}


void one_key_rotate_self(u16 * keyvaule)
{
    static u32 state=0;
		static u32 lastRCTime=0;
	  static u32 lastKeyPress=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	{
		lastRCTime =systemTickMs;
  }
	else
		return;
		if(Disarm_Arm_check()==ARMED) {	  return;} 
	//////////////////////////////////////////////////////////?????????????????????????????need check the mode of circle ////
	/////the center is the user's location
	if(*keyvaule!=(1 << Button_KEY9|(1<<LONGPRESS))&&state==0)
		    return;
	else if(*keyvaule==((1 << Button_KEY9)|(1<<LONGPRESS))&&state==0)
	{
		  lastKeyPress = systemTickMs;
		*keyvaule=0;
  }

	switch(state)
   { case 1:
		 setParam("CIRCLE_RADIUS",0);
		  state++;
		 break;
		 case 20:
			 ////need check the param set ok
		   if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==0) 
						 state=40;
			     else
					 {
						 setParam("CIRCLE_RADIUS",0);
						 state++;
           }
				}else
				  state++;
		 break;
		 case 40:
			 if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==0) 
						 state=50;
			     else
					 {
						 setParam("CIRCLE_RADIUS",0);
						 state++;
           }
				}
				else
					state++;
			break; 
		 case 50:
		 //_send_CIRCLE(0,5.0,0.0,Flygps.lat,Flygps.lon,5.0);
		 _send_SET_MODE(1,MODE_CIRCLE);
		 state++;
		 break;
		 case 80:
			 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			     _send_SET_MODE(01,MODE_CIRCLE);
		  { state=0; return;}
		 break;
	   default:
		 state++;
		 break;
	 }
}


void resetSuperMode(void)
{
	sys_Info_Box.CH8_Mode_Super=1100;
}

void StopFollowme(void)
{
	sys_Info_Box.FllowMeStart=0;
}
void clearFlyModeFlag(void)
{
	sys_Info_Box.FllowMeStart=0; //StopFollowme();
	sys_Info_Box.yaw_start_stop=0;	// setConditionYaw(0);
	sys_Info_Box.LeadMeStart=0;//	 SetLeadMeStatus(0);

	//sys_Info_Box.Loitor_yaw=1500;//	 SetYaw(1500);
	//g_thro=1500;//	 set_g_thro(1500);
}

/////如果起飞后 发现处在GUIDe 模式，就开始计算本地的GPS 远端的GPS，滤波，计算方位角，计算提前量，
////set guide mode ,and send the location to coptor in navpoint
void one_key_FollowMe(u16 * keyvaule)
{
    static u32 state=0;
		static u32 lastRCTime=0;
	static u16 sleepKey=0;
  if(systemTickMs-lastRCTime>=_50HZ)
	  {
		  lastRCTime =systemTickMs;
    }
	else
		{
		  return;
	  }
	/////////////////////////////////////////////
	if(Disarm_Arm_check()==ARMED) {	  return;}
	if(*keyvaule!=(1 << Button_KEY11)&&state==0)
		return;
	else if(*keyvaule==(1 << Button_KEY11)&&state==0)
	{
		*keyvaule=0;
		if(gps_status.num_sats>6&&gps_status.hdop<260)
		     state=1;
		sys_Info_Box.Holdalt=sys_Info_Box.alt;
							setParam("WP_YAW_BEHAVIOR",2);//
   }
	switch(state)
   {   case 1:
		        _send_MISSION_CLEAR();
		        state++;
	     break;
	     case 20:
	       if(checkMISSION_ACK()!=0)
	         {
	          _send_MISSION_CLEAR();
	          state=40;
	         }
            state++;
	    break;
		 case 40:
			     _send_SET_MODE(01,MODE_GUIDED);
		       state++;
	   break;
		 case 50:
					 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			        _send_SET_MODE(01,MODE_GUIDED);
					 state++;
		 break;
      case 60:
		        _send_guide(gps_status.lat,gps_status.lng,sys_Info_Box.Holdalt/10);
		       state++;
	    break;
			case 80:
				setParam("WP_YAW_BEHAVIOR",2);//
			   state++;
		 break;
      case 100:
         if(checkMISSION_ACK()!=0)
         	{           
           	 _send_guide(gps_status.lat ,gps_status.lng,sys_Info_Box.Holdalt/10);
         	}
             state=0;
             sys_Info_Box.FllowMeStart=1;
					SetLeadMeStatus(0);
					setConditionYaw(0);

	    break;
			
	    default:
	    state++;
	    break;
	//_send_TAKEOFF(2.0,10.0,112.0,115.0,5.0);
	 }
}
 

void GPS_filter()
{
static GPS_RAW Filter[8];
static u8 i=0;
	     u8 j=0;
	//ground speed m*100.0 /s
	if(Flygps.vel<3000)
	{  
		 Filter[i].lat=Flygps.lat;
	   Filter[i].lon=Flygps.lon;
	    i++;
      i=i&0x07;
	}
// if(Flygps.vel>20) 
//      RemoteGPS
//RemoteGPS
}


#define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
#define PI FAST_ATAN2_PIBY2_FLOAT*2.0
// fast_atan2 - faster version of atan2
//      126 us on AVR cpu vs 199 for regular atan2
//      absolute error is < 0.005 radians or 0.28 degrees
//      origin source: https://gist.github.com/volkansalma/2972237/raw/
float fast_atan2(float x, float y)
{
   if (x == 0.0f) {
       if (y > 0.0f) {
           return FAST_ATAN2_PIBY2_FLOAT;
       }
       if (y == 0.0f) {
           return 0.0f;
       }
       return -FAST_ATAN2_PIBY2_FLOAT;
   }
   float atan;
   float z = y/x;
   if (fabs( z ) < 1.0f) {
       atan = z / (1.0f + 0.28f * z * z);
       if (x < 0.0f) {
           if (y < 0.0f) {
               return atan - PI;
           }
           return atan + PI;
       }
   } else {
       atan = FAST_ATAN2_PIBY2_FLOAT - (z / (z * z + 0.28f));
       if (y < 0.0f) {
           return atan - PI;
       }
   }
   return atan;
}

float GetBearingOf_CoptorRelativeUser(Gps_Status * LocalGps,mavlink_gps_raw_int_t  *RemoteGps)
{
	float bearing = -9000 + fast_atan2(-(float)(RemoteGps->lon -LocalGps->lng), (float)(RemoteGps->lat-LocalGps->lat)) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
	return  bearing/100.0;
}



void predict_New_Positon(u32 Xoffset,u32 Yoffset)
{///v m/s
	char txt[50];
  static Gps_Status  LastLocalGps;
  static u32 lastGPSTime=0;
	float acclX=0;
	float acclY=0;
  float Dtime=0;
	float absTempX=fabs(gps_status.velned_x);
	float absTempY=fabs(gps_status.velned_y);
	float Pfactorlat = (90.1/cos(gps_status.lat/1.0e7*Deg2Rad));     
  float Pfactorlon =90.1;
	if(LastLocalGps.velned_x==0){
		   acclX=0;
		   acclY=0;
    }
		else
		{
			acclX=gps_status.velned_x-LastLocalGps.velned_x;
	    acclY=gps_status.velned_y-LastLocalGps.velned_y;
    }
   Dtime=(float)(systemTickMs-lastGPSTime)/1000.0;
   lastGPSTime =systemTickMs;
//	 SendGPS.lon=(s32)(((gps_status.velned_y+acclY)*Dtime)*Pfactorlat)+gps_status.lng+Yoffset;
//   SendGPS.lat=(s32)(((gps_status.velned_x+acclX)*Dtime)*Pfactorlon)+gps_status.lat+Xoffset;
		Dtime=1.0;
		if(gps_status.velned_y<=4.0)
		{
		SendGPS.lon=(s32)(gps_status.velned_y*absTempY*Dtime*Pfactorlat)+gps_status.lng+Yoffset;
    SendGPS.lat=(s32)(gps_status.velned_x*absTempX*Dtime*Pfactorlon)+gps_status.lat+Xoffset;
		}
		else if(gps_status.velned_y<=8.0)
		{
    SendGPS.lon=(s32)(gps_status.velned_y*absTempY*Dtime*Pfactorlat)*0.75+gps_status.lng+Yoffset;
			 SendGPS.lat=(s32)(gps_status.velned_x*absTempX*Dtime*Pfactorlon)*0.75+gps_status.lat+Xoffset;
		}
		else
		{
		SendGPS.lon=(s32)(gps_status.velned_y*absTempY*Dtime*Pfactorlat)*0.5+gps_status.lng+Yoffset;
    SendGPS.lat=(s32)(gps_status.velned_x*absTempX*Dtime*Pfactorlon)*0.5+gps_status.lat+Xoffset;
    }
//		SendGPS.lon=gps_status.lng+Yoffset;
//    SendGPS.lat=gps_status.lat+Xoffset;
	 LastLocalGps.ground_speed=gps_status.ground_speed;
//	 sprintf(txt,"X%5d,Y%5d",(s32)(gps_status.velned_y*absTempY*Pfactorlon),(s32)(gps_status.velned_x*absTempX*Pfactorlat));//
//   LCD_ShowString(0,100,128,128,12,txt,WHITE,BLACK);
//	 sprintf(txt,"X%2.2f,Y%3.7f",gps_status.velned_y,gps_status.velned_x);//
//   LCD_ShowString(0,112,128,128,12,txt,WHITE,BLACK);
	 LastLocalGps.velned_x=gps_status.velned_x;
	 LastLocalGps.velned_y=gps_status.velned_y;
//			  sprintf(txt,"Dt=%1.4f",Dtime);//(float)Flygps.lat*1.0/1e7,(float)Flygps.lon*1.0/1e7);//gps_status.num_sats,gps_status.hdop);
//        LCD_ShowString(0,88,128,128,12,txt,WHITE,BLACK);
}


//1e7 in m
// __inline void _send_CHANGE_YAW(float angel,float direction);
// void _send_CHANGE_SPEED(float Velocitym);
u32 check_Distance_Remote_Local(mavlink_gps_raw_int_t *RemoteGps,Gps_Status * LocalGps)
{
	char txt[10];
    u32 temp=0;
    u32 dltax=abs(LocalGps->lat  -RemoteGps->lat )/90;
	  u32 dltay=(u32)((float)(abs(LocalGps->lng  -RemoteGps->lon))*cos(RemoteGps->lat/1.0e7*Deg2Rad)/90.1);
       temp=sqrt(dltax*dltax+dltay*dltay);
	 if(temp>9000) temp = 9000;
	
	
	//if(gps_status.num_sats<4) return 0;
//	    if(LocalGps->ground_speed<40)
//				temp=0;
//			if(LocalGps->fix<2) return 0;
    return (u32)temp;
}

void update_GPS_20HZ(void)
{  u8 txt[50];
	static  u32 lastGpsTick=0; 
	static  float last_Coptor_User_bear=0.0;
		if(systemTickMs - lastGpsTick >=_20HZ ){
//			Update_Gps();
			lastGpsTick = systemTickMs;
			if(gps_status.gps_new_data)
			{
				gps_status.gps_new_data = false;
			if(gps_status.num_sats>=6)
				{
					Coptor_User_bear=GetBearingOf_CoptorRelativeUser(&gps_status,&Flygps);//+180.0;
					//Coptor_User_bear= last_Coptor_User_bear*0.2+last_Coptor_User_bear*0.8;
					//last_Coptor_User_bear=Coptor_User_bear;
					sys_Info_Box.distance=check_Distance_Remote_Local(&Flygps,&gps_status);
			    sprintf(txt,"L%7d,L%7d",gps_status.lat%10000000,gps_status.lng%10000000);//Coptor_User_bear,gps_status.heading_2d/100000);
//					LCD_ShowString(0,62,128,128,12,txt,BLUE,BLACK);
				  sprintf(txt,"F%7d,F%10d",(Flygps.lat)%10000000,Flygps.lon%10000000);//sys_Info_Box.distance/100.0,gps_status.num_sats);
//          LCD_ShowString(0,74,128,128,12,txt,WHITE,BLACK);
				}
      }
		}
}

void _update_GPS_20HZ(void)
{  u8 txt[50];
	static  u32 lastGpsTick=0; 
	static  float last_Coptor_User_bear=0.0;
		if(systemTickMs - lastGpsTick >=_20HZ ){
//			Update_Gps();
			lastGpsTick = systemTickMs;
			if(gps_status.gps_new_data)
			{
				gps_status.gps_new_data = false;
				
				LocalGPS_info.Hdop=gps_status.hdop;
		    LocalGPS_info.ground_speed=gps_status.ground_speed;
			  LocalGPS_info.lat=gps_status.lat;
			  LocalGPS_info.lon=gps_status.lng;
			  LocalGPS_info.velned_x=gps_status.velned_x;
			  LocalGPS_info.velned_y=gps_status.velned_y;
			  LocalGPS_info.GPS_Staus=((gps_status.fix&0x07)|(gps_status.num_sats<<3));
			  LocalGPS_info.Hdg=gps_status.heading_2d;

			if(gps_status.num_sats>=6)
				{
					//no need for RF24l01 by local//Coptor_User_bear=GetBearingOf_CoptorRelativeUser(&gps_status,&Flygps);//+180.0;
					//Coptor_User_bear= last_Coptor_User_bear*0.2+last_Coptor_User_bear*0.8;
					//last_Coptor_User_bear=Coptor_User_bear;
					sys_Info_Box.distance=check_Distance_Remote_Local(&Flygps,&gps_status);
//			    sprintf(txt,"L%7d,L%7d",gps_status.lat%10000000,gps_status.lng%10000000);//Coptor_User_bear,gps_status.heading_2d/100000);
//					LCD_ShowString(0,62,128,128,12,txt,BLUE,BLACK);
//				  sprintf(txt,"F%7d,F%10d",(Flygps.lat)%10000000,Flygps.lon%10000000);//sys_Info_Box.distance/100.0,gps_status.num_sats);
//          LCD_ShowString(0,74,128,128,12,txt,WHITE,BLACK);
				}
      }
		}
}



__inline void setConditionYaw(u8 status)
{
sys_Info_Box.yaw_start_stop=status;
}

__inline u8 Cordination_Shift(float * xcor_lat,float * ycor_lon,float heading)
{
float newcorx,newcory;
	  newcorx = (*xcor_lat)*cos(heading*Deg2Rad)-(*ycor_lon)*sin(heading*Deg2Rad);
    newcory = (*ycor_lon)*cos(heading*Deg2Rad)+(*xcor_lat)*sin(heading*Deg2Rad);
	  *xcor_lat =newcory;
	  *ycor_lon =newcorx;
	return 1;
}
#define LEADTHR 0.5
////////first predict new position .
__inline void Cordination_Left_front(s32 distancem)
{
  float Left_X,Left_Y;
	static float lastHeading=0;
	float heading=0;
	char txt[50];

	if(gps_status.ground_speed>LEADTHR)//sys_Info_Box.vel>0.4)
	{
		setConditionYaw(1);
		heading=(float)gps_status.heading_2d/100000.0;
	
	}
	else
	{
			setConditionYaw(0);
  }
		Left_X = (float)distancem* (0.70710678);//cos135deg
	  Left_Y = (float)distancem* -0.70710678;//sin135deg
	  Cordination_Shift(&Left_X,&Left_Y,lastHeading);//(float)gps_status.heading_2d/100000.0);
		SendGPS.lat+=Left_Y*90.1;///gps_status.lat
	  SendGPS.lon+=Left_X*90.1/cos(gps_status.lat/1.0e7*Deg2Rad);////gps_status.lng+
		sprintf(txt,"X%7d,Y%7d",SendGPS.lat%10000000,SendGPS.lon%10000000);//
//    LCD_ShowString(0,100,128,128,12,txt,WHITE,BLACK);
//	  sprintf(txt,"X%2.2f,Y%3.7f",Left_X*90.1/cos(gps_status.lat/1.0e7*Deg2Rad),Left_Y*90.1);//
//    LCD_ShowString(0,112,128,128,12,txt,WHITE,BLACK);
		//setConditionYaw(1);
}

__inline void Predict_direct_LatAndLon(float distance)
{
	float Pfactorlat = (90.1/cos(gps_status.lat/1.0e7*Deg2Rad));     
  float Pfactorlon =90.1;
	float Dtime=0;
	float Vfactor=0;
	char txt[30];
	s32  Vletghgnd=0;
	static u32 lastGPSTime=0;
	Dtime=(float)(systemTickMs-lastGPSTime)/1000.0;
  lastGPSTime =systemTickMs;
	if(distance==LEADDISTANCE){
		if(gps_status.velned_y<=4.0)      { Vfactor=1.0;}
		else if(gps_status.velned_y<=8.0) {Vfactor=0.8;}
		else                              {Vfactor=0.7;}
	}
	else{
    if(gps_status.velned_y<=4.0)      { Vfactor=0.8;}
		else if(gps_status.velned_y<=8.0) {Vfactor=0.6;}
		else                              {Vfactor=0.4;}
}
//  if(distance<2.0) setConditionYaw(0);
//   else {
//		   if(fabs(Coptor_User_bear-gps_status.heading_2d/100000.0)<90.0)
//           Dtime=0;
//         else			 
//				 Vfactor=2.5;
//        }
	Vletghgnd= gps_status.ground_speed*gps_status.ground_speed*Vfactor;
	
  SendGPS.lat=gps_status.lat+(distance*cos(((float)gps_status.heading_2d/100000.0 -45.0)*Deg2Rad)+Vletghgnd*cos(((float)gps_status.heading_2d/100000.0)*Deg2Rad))*90.1;
	SendGPS.lon=gps_status.lng+(distance*sin(((float)gps_status.heading_2d/100000.0 -45.0)*Deg2Rad)+Vletghgnd*sin(((float)gps_status.heading_2d/100000.0)*Deg2Rad))*Pfactorlat;
  sprintf(txt,"X%7d,Y%7d",SendGPS.lat%10000000,SendGPS.lon%10000000);//
   // LCD_ShowString(0,100,128,128,12,txt,WHITE,BLACK);   
    
}

__inline u8 BearingCompHeading(float FLYHd, float Usrrelative )
{
   u8 temp=0;
	if(FLYHd+180.0==Usrrelative || FLYHd==Usrrelative+180.0)
		return 1;
	else
		return 0;
}

__inline void Condition_Yaw_thread(void)
{
	char text[50];
static u32 state=1;
		static u32 lastRCTime=0;
	float angle=0.0;
	float direct=1.0;
	if(sys_Info_Box.yaw_start_stop==0) return;
	if(Disarm_Arm_check()==ARMED) { return;}
  if(systemTickMs-lastRCTime>=ConditionYawHZ)
	  {
		lastRCTime =systemTickMs;
    }
	else
		return;
	   if(Coptor_User_bear<=179.9&&Coptor_User_bear>=0)
	   {
		   angle=180.0+Coptor_User_bear;
     }else if(Coptor_User_bear<=359.9&&Coptor_User_bear>180.0)
	   {
		   angle=Coptor_User_bear-180.0;
     }
		 if(fabs(angle-((float)sys_Info_Box.hdg)/100.0)<10.0)
		 {
			 state=1;
   			 return;
		 }
		 float deltaYaw =angle -((float)sys_Info_Box.hdg)/100.0;
		 if(deltaYaw<0) deltaYaw= deltaYaw+360.0;
		 if(deltaYaw<180.0)
			 direct=1.0;
		 else
			 direct=-1.0;
   switch(state)
   { 
     case 1:
			// if(0==BearingCompHeading(sys_Info_Box.hdg/100,Coptor_User_bear))
		// if((Coptor_User_bear>0&&Coptor_User_bear<gps_status.heading_2d/100000+90)||(Coptor_User_bear<359&&Coptor_User_bear>gps_status.heading_2d/100000+270))
		   _send_CHANGE_YAW(angle,direct) ;
		
//       LCD_ShowString(64,12,128,128,12,"SetYaw",RED,GREEN);
		   // _send_guide(gps_status.lat,gps_status.lng,500);
		 state++;
	   break;
		 case 2:
			 state=3;
		 break;
     case 3:
					if(0!=checkCMD_ACK(MAV_CMD_CONDITION_YAW))
         	{           
						_send_CHANGE_YAW(angle,direct) ;
         	}
				 state=1;
	   break;
		default:
			state++;
			break;
	 } 
}
	
void SetLeadMeStatus(u8 status)
{
sys_Info_Box.LeadMeStart=status;
}

void Guide_Condition_Yaw_Loop(u16 * keyvaule)
{
    static u32 state=0;
		static u32 lastRCTime=0;

  if(systemTickMs-lastRCTime>=_50HZ)
	  {
		lastRCTime =systemTickMs;
    }
	else
		return;
	/////////////////////////////////////////////
	if(Disarm_Arm_check()==ARMED) {	 return;}  // 
	if(*keyvaule!=(1 << Button_KEY11|(1<<LONGPRESS))&&state==0)
		return;
	else if(*keyvaule==(1 << Button_KEY11|(1<<LONGPRESS))&&state==0)
	{
		*keyvaule=0;
		   if(gps_status.num_sats>6&&gps_status.hdop<260)
		     state=1;
			 sys_Info_Box.Holdalt=sys_Info_Box.alt;
  }
	switch(state)
   { 
		 case 1:
		_send_MISSION_CLEAR();
		state++;
	   break;
	   case 20:
	    if(checkMISSION_ACK()!=0)
	    {
	     _send_MISSION_CLEAR();
	    }
         state++;
	   break;
		case 30:
			_send_SET_MODE(01,MODE_LOITER);
		       state++;
			break;
		case 50:
			if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			        _send_SET_MODE(01,MODE_LOITER);
			 state++;
			break;
		 case 65:
			     _send_SET_MODE(01,MODE_GUIDED);
		       state++;
	   break;
		 case 75:
					 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			        _send_SET_MODE(01,MODE_GUIDED);
					 state++;
		 break;
     case 90:
		     //_send_CHANGE_YAW(Coptor_User_bear,-1.0) ;
		     setConditionYaw(1);//auto aim to user.
		// _send_guide(gps_status.lat,gps_status.lng,500);
		     state++;
	   break;
     case 100:
         if(0!=checkCMD_ACK(MAV_CMD_CONDITION_YAW))
         	{           
         	//_send_guide(gps_status.lat ,gps_status.lng,500);
						_send_CHANGE_YAW(Coptor_User_bear,-1.0) ;
         	}
          state=0;
					sys_Info_Box.FllowMeStart=0;
					SetLeadMeStatus(1);
						setParam("WP_YAW_BEHAVIOR",0);//
	   break;
	   default:
	       state++;
	   break;
	 }
}

//back follow
void FollowMe_GUIDING_Loop(void)
{
 //get the current local GPS position
  static u32 state=1;
  static u32 lastRCTime=0;
	u32 tempflyGndV=0;
	float VgndVSFly=0.0;
  if(sys_Info_Box.FllowMeStart==0) return;
  if(systemTickMs-lastRCTime>=_1HZ)
	  {
		lastRCTime =systemTickMs;
    }
	else
	    return;
   if(check_Distance_Remote_Local(&Flygps,&gps_status)==0) return;
	switch(state)
	{
		case 1://send new positon
			//predict_New_Positon(0,0);//predict cordination
			 if(gps_status.ground_speed>2.0)
			 {
				if(Flygps.vel==0xffff) tempflyGndV=0;
		     else  tempflyGndV=Flygps.vel;
			  VgndVSFly=(float)tempflyGndV/100.0/gps_status.ground_speed;
			   if( VgndVSFly<0.5&&VgndVSFly>0)
		       { setParam("WPNAV_ACCEL",200);////set ACCL to 1}
           } 
	       else if(VgndVSFly<0.7&&VgndVSFly>=0.5)
            {setParam("WPNAV_ACCEL",150);}//set accl to 2
		     else if(VgndVSFly>=0.7)
			      { setParam("WPNAV_ACCEL",100);}//set accl to 2
       }
        else
          setParam("WPNAV_ACCEL",100);					
      if(gps_status.ground_speed>0.3)	//LEADTHR;		 
			{  Predict_direct_LatAndLon(0);
				_send_guide(SendGPS.lat,SendGPS.lon,sys_Info_Box.Holdalt/10);
			}
		//_send_guide(gps_status.lat,gps_status.lng,500);
		state=1;
		break;
		default:
			break;
  }
}

typedef enum{
SEND_GUID=1,
CHECK_DISTANCE,
SEND_COND_YAW,
CHECK_HEADING,
MODIFY_SPEED,
MODIFY_ACCE,
	
}FOLLOW_ME_STATE;

__inline void FollowMe_GUIDING_LeftFront_Loop(void)
{
 //get the current local GPS position
  static u32 state=1;
  static u32 lastRCTime=0;
  if(sys_Info_Box.LeadMeStart==0) return;
  if(systemTickMs-lastRCTime>=_1HZ)
	  {lastRCTime =systemTickMs;}
	else
	    return;
  if(check_Distance_Remote_Local(&Flygps,&gps_status)==0) return;
	switch(state)
	{
		case SEND_GUID:
	//	if(((Coptor_User_bear>0)&&Coptor_User_bear<(gps_status.heading_2d/100000.0+90.0))||(Coptor_User_bear<359.0&&Coptor_User_bear>(gps_status.heading_2d/100000.0+270.0)))
     if(fabs(Coptor_User_bear-gps_status.heading_2d/100000.0)<90.0)		// in the front
		{ setParam("WPNAV_ACCEL",100);setConditionYaw(1); ConditionYawHZ=_1HZ;//LCD_ShowString(0,12,128,128,12,"seYaw",WHITE,BLACK);
    } ////set ACCL to 1}
		else {setParam("WPNAV_ACCEL",250);setConditionYaw(1);ConditionYawHZ=_Dot5HZ;}//set accl to 2}
		if(gps_status.ground_speed>LEADTHR)
		{
			Predict_direct_LatAndLon(LEADDISTANCE);//5m to user at 45 degree
			_send_guide(SendGPS.lat,SendGPS.lon,sys_Info_Box.Holdalt/10);
    }
		state=SEND_GUID;
		break;
		default:
			break;
  }
}

void stick_key_Sevro(u16 *keyvaule)
{
	static u32 state=0;
	static u32 lastRCTime=0;
  if(systemTickMs-lastRCTime>=_10HZ)
	{lastRCTime =systemTickMs;}
	else{ return;}
	/////////////////////////////////////////////
	if(*keyvaule==((1 << Button_KEY1)|0x1000)) // left
	{
     ch1_Roll=1300;
  }else if(*keyvaule==((1 << Button_KEY4)|0x1000)) //right
	{
    ch1_Roll=1700;
  }else if(*keyvaule!=((1 << Button_KEY1)|0x1000)&&*keyvaule!=((1 << Button_KEY4)|0x1000))
	{
    ch1_Roll =1500;
  }
	if(*keyvaule==((1 << Button_KEY2)|0x1000)) //back
	{
    ch2_Pitch=1700;
  }else	if(*keyvaule==((1 << Button_KEY3)|0x1000))//forward
	{
     ch2_Pitch=1300;
  }else if(*keyvaule!=((1 << Button_KEY2)|0x1000)&&*keyvaule!=((1 << Button_KEY3)|0x1000))
	{
     ch2_Pitch=1500;
  }
}
u32 check_Distance_Remote_Local_New(mavlink_gps_raw_int_t *RemoteGps,Gps_Status * LocalGps)
{
	char txt[50];
	float Halt =((float)sys_Info_Box.alt)/1000.0;
    float temp=0;
    float dltax=((float)LocalGps->lat  -(float)RemoteGps->lat)/90.1 ;
	  float dltay=((float)LocalGps->lng  -(float)RemoteGps->lon)*cos(gps_status.lat/1.0e7*Deg2Rad)/90.1;
       temp=sqrt(dltax*dltax+dltay*dltay+Halt*Halt);
	#ifdef FAKE_TEST
	if(temp>100&&temp<200) temp =110*temp/100;
	else if(temp>=200&&temp<300) temp =120*temp/100;
	else if(temp>=300&&temp<400) temp =130*temp/100;
	else if(temp>=400&&temp<500) temp =140*temp/100;
	else if(temp>=500&&temp<600) temp =150*temp/100;
	else if(temp>=600) temp =160*temp/100; 
	#endif
	if(temp>9000) temp=9000;
	if(RemoteGps->fix_type<=2) return 0;
    return (u32)temp;
}

void UpdateDistance(void)
{
 if(Flygps.fix_type>=2)
 {
	// char txt[20];
   sys_Info_Box.distance=check_Distance_Remote_Local_New(&Flygps,&gps_status);
//	  sprintf(txt,"L%7d,L%7d",gps_status.lat%10000000,gps_status.lng%10000000);//Coptor_User_bear,gps_status.heading_2d/100000);
//		LCD_ShowString(0,32,128,128,12,txt,BLUE,BLACK);
//		sprintf(txt,"F%7d,F%10d",(Flygps.lat)%10000000,Flygps.lon%10000000);//sys_Info_Box.distance/100.0,gps_status.num_sats);
//    LCD_ShowString(0,64,128,128,12,txt,WHITE,BLACK);
 }
		  
}

///////////here we need a superSimple mode check 
//void check_set_SuperMode(void)
//{
// static u32 lastcheckTime=0;
//if(systemTickMs-lastcheckTime>_20HZ)
//{
//	lastcheckTime=systemTickMs;
//}
//else
//	return;
////
//if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
//{
//   //if in pro mode ,we deny in the super mode 
//	sys_Info_Box.CH8_Mode_Super=1100;
//}
//else
//{



//}


//}


void SetHomeLocation(void)
{
   gps_status.lat=Flygps.lat;
   gps_status.lng=Flygps.lon;
}

void CheckUsrMode(void)
{
 static u8 ARMED_request=100;
		// on ground we need to check the mode in different user mode.
		// in PROFESSIONAL_MODE ,we need change loitor  to stablize
		// in Junior mode  ,we deny the stablize ,so need to switch to loitor mode 
//if(sys_Info_Box.StickMode==LEFT_THRO_MODE)
//{
//	if(RCx_BatInfo.RC[2].ch>1870 && RCx_BatInfo.RC[3].ch<1130)// user request
//	{  ARMED_request=1; SetHomeLocation(); sys_Info_Box.oneKeyFly=0;sys_Info_Box.CH8_Mode_Super=1100;
//	
//     if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
//	      _send_SET_MODE(01,MODE_ALT_HOLD);//_send_SET_MODE(1,MODE_ALT_HOLD);
//     else
//		    _send_SET_MODE(01,MODE_LOITER);  
//	}
//	else
//		 return;
//}
//else if(sys_Info_Box.StickMode==RIGHT_THRO_MODE)
//{
//	if(RCx_BatInfo.RC[1].ch<1130 && RCx_BatInfo.RC[3].ch<1130)// user request
//	{ ARMED_request=1;SetHomeLocation(); sys_Info_Box.oneKeyFly=0;sys_Info_Box.CH8_Mode_Super=1100;
//     if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
//	      _send_SET_MODE(01,MODE_ALT_HOLD);//_send_SET_MODE(1,MODE_ALT_HOLD);
//     else
//		    _send_SET_MODE(01,MODE_LOITER);
//	}
//	else
//		return;
//}
//if(ARMED_request==100) return;
//switch(ARMED_request)
//{
// case 1:
//	 if(ARMED_request<3){
//     if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
//	      _send_SET_MODE(01,MODE_ALT_HOLD);//_send_SET_MODE(1,MODE_ALT_HOLD);
//     else
//		    _send_SET_MODE(01,MODE_LOITER);  
//		 
//        ARMED_request=2; 
//        return;
//     }
//	 break;
// case 60:
//	 if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
//	  {
//			if( CheckANDcompMODE(MODE_ALT_HOLD)==0)
//			  {_send_SET_MODE(01,MODE_ALT_HOLD);
//					ARMED_request=2;
//         }
//			else
//				 ARMED_request=100;
//		}
//		else
//    {	 
//		  if( CheckANDcompMODE(MODE_LOITER)==0)
//		   {   _send_SET_MODE(01,MODE_LOITER);  
//			     ARMED_request=2;
//		    }
//		   else
//				   ARMED_request=100;
//		}
//	 break;
//   default:
//		 ARMED_request++;
//	 break;
//}
}


__inline void Set_mode_Once(void)
{
	static u8 SetOnce=0;
	if(SetOnce) return;
	   SetOnce=1;
  if(sys_Info_Box.user_mode ==PROFESSIONAL_MODE)
	{
   _send_SET_MODE(1,MODE_ALT_HOLD);
  } else
	{
   _send_SET_MODE(1,MODE_LOITER);
  }
}


__inline void Circle_radius_rution(void)
{
	static u32 lastCircleTime=0;
	static u8 state=0;
	static u16 radius=500;
	u16 Left_riht=0;
	if(systemTickMs-lastCircleTime>=_2HZ)
	{
		lastCircleTime=systemTickMs;
  }
	else
		return;
// if in circle mode 
	if(sys_Info_Box.custom_mode==MODE_CIRCLE)
	{
     if(Left_riht>1560)
		 {
			 if(state=0) {state=1;radius+=100;}
     }
		 else if(Left_riht<1420)
		 {
			 if(state==0) {state=1; if(radius>100) radius-=100; }
     }
		 if(state==0) return;
	switch(state)
   { case 1:
		 setParam("CIRCLE_RADIUS",radius);
		  state++;
		 break;
		 case 3:
			 ////need check the param set ok
		   if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==radius) 
						 state=40;
			     else
					 {
						 setParam("CIRCLE_RADIUS",radius);
						 state++;
           }
				}else
				  state=1;
		 break;
		}
  }
/*
	
		 
		 case 40:
			 if(strcmp(Flyparam.param_id,"CIRCLE_RADIUS")!=0)
			 { if(Flyparam.param_value==500) 
						 state=50;
			     else
					 {
						 setParam("CIRCLE_RADIUS",500);
						 state++;
           }
				}
				else
					state++;
			break; 
		 case 50:
		 //_send_CIRCLE(0,5.0,0.0,Flygps.lat,Flygps.lon,5.0);
		 _send_SET_MODE(1,MODE_CIRCLE);
		 state++;
		 break;
		 case 80:
			 if(0!=checkCMD_ACK(MAVLINK_MSG_ID_SET_MODE))
			     _send_SET_MODE(01,MODE_CIRCLE);
		  { state=0; return;}
		 break;
	   default:
		 state++;
		 break;
	 }
}	
*/
}


__inline void Buzzer_interrupt()
{			
	if(state!=0&&sys_Info_Box.voltage_battery>10900&&sys_Info_Box.connect_state!=0&&times==0)  //&&sys_Info_Box.oneKeyFly==0//only low voltage and disconnection can keep buzzing
	{
		state=0;
	}
		switch(state)
	{
   	 case LOWVOL_STATE:
		 if(systemTickMs-lastBuzzerTime>ContinuTimes){lastBuzzerTime=systemTickMs; BUZZER=!BUZZER;}
		 break;
   case DISCON_STATE: 
		if(systemTickMs-lastBuzzerTime>ContinuTimes){lastBuzzerTime=systemTickMs; BUZZER=!BUZZER;}
		break;
	case TIMES_STATE: 
		if(systemTickMs-lastBuzzerTime>ContinuTimes&&times>0){lastBuzzerTime=systemTickMs; BUZZER=!BUZZER;times--;}
	    break;
	default :
		if(key_flag==0){BUZZER=0;}
		else if(key_flag==1){BUZZER=1;}
		break;

  } 
}


__inline void Buzzer_loop(u16 keyValue)
{
static u32 BuzzerloopTime=0;
 static u8 led_flag=0;
  static u32 LastMode=0;
	static u8 Arm_Flag=1;     //0表示解锁状态，1表示锁定状态
	static u8 RTL_Flag=0;
	static u8 Land_Flag=0;
	static u32 voltage=11500;
	static u32 LastLowVolTime=0;
if(keyValue==0) return;

//	LCD_ShowNum(0,50,state,1,16,WHITE,BLACK);
//	LCD_ShowNum(0,66,times,1,16,WHITE,BLACK);
	voltage=(sys_Info_Box.voltage_battery*1+voltage*9)/10;
if(Disarm_Arm_check()==ARMED) //locked
{
 //stop buzeer
//	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_1))
//       BUZZER=0;
//	state=0;
 	if(Arm_Flag!=1)
 	{
		ContinuTimes=300;
		state=TIMES_STATE;
		times=1;
		Arm_Flag=1;
		lastBuzzerTime=systemTickMs;
		BUZZER=1;
	}
//	if(keyValue)  
//	{
//		BUZZER=1;
//		state=ARMED_KEY_STATE;
//		lastBuzzerTime=systemTickMs;
//	}
		return;
}
else
{	
	if(sys_Info_Box.custom_mode!=MODE_RTL)RTL_Flag=0;
	if(sys_Info_Box.custom_mode!=MODE_LAND)Land_Flag=0;
  //if low batt // continue 
	if(sys_Info_Box.connect_state==0)  // if lost connection // continue	优先级0
	{
		if(state!=DISCON_STATE)
		{
	    	ContinuTimes=200;
			state=DISCON_STATE;
			times=0;                              //continuing buzzer
			lastBuzzerTime=systemTickMs;
			BUZZER=1;
		}
 	}
	else if(voltage<10900) //if lowvoltage  // continue     优先级1
	{
		if(state!=LOWVOL_STATE)
		{
			ContinuTimes=200;
			state=LOWVOL_STATE;
			times=0;                              //continuing buzzer
			lastBuzzerTime=systemTickMs;
			BUZZER=1;
		}
	}
//	else if(sys_Info_Box.custom_mode==MODE_RTL&&RTL_Flag!=1)// if RTL     // 3 times     优先级2
//	{
//			ContinuTimes=500;
//			state=RTL_STATE;
//			lastBuzzerTime=systemTickMs;
//			times=5;
//			BUZZER=1;
//			RTL_Flag=1;		
//    }
//	else if(sys_Info_Box.custom_mode==MODE_LAND&&Land_Flag!=1)// if LAND    // 2 times    优先级2
//	{
//			ContinuTimes=300;
//			state=LAND_STATE;
//			lastBuzzerTime=systemTickMs;
//			times=3;
//			BUZZER=1;
//			Land_Flag=1;
//    }
	else if(Disarm_Arm_check()==DISARMED&&Arm_Flag!=0&&state!=TIMES_STATE)            //其余优先级为2
	{
		ContinuTimes=300;
		state=TIMES_STATE;
		Arm_Flag=0;
		times=1;
		lastBuzzerTime=systemTickMs;
		BUZZER=1;
	}
	else if(LastMode!=sys_Info_Box.custom_mode&&state!=TIMES_STATE)            
	{
		LastMode=sys_Info_Box.custom_mode;
		if(sys_Info_Box.custom_mode==MODE_RTL&&RTL_Flag!=1)      // if RTL     // 3 times  
		{
			ContinuTimes=500;
			state=TIMES_STATE;
			lastBuzzerTime=systemTickMs;
			times=5;
			BUZZER=1;
			RTL_Flag=1;				
		}
		else if(sys_Info_Box.custom_mode==MODE_LAND&&Land_Flag!=1)// if LAND    // 2 times   
		{
			ContinuTimes=300;
			state=TIMES_STATE;
			lastBuzzerTime=systemTickMs;
			times=3;
			BUZZER=1;
			Land_Flag=1;
		}
		else
		{
			ContinuTimes=300;
			state=TIMES_STATE;
			times=1;
			lastBuzzerTime=systemTickMs;
			BUZZER=1;
		}
	}
//	else if(sys_Info_Box.oneKeyFly>0&&state!=TAKEOFF_STATE)       //if oneKeyFly>0      
//	{
////		ContinuTimes=600;
//		state=TAKEOFF_STATE;
////		lastBuzzerTime=systemTickMs;
////		times=1;
////		BUZZER=1;
//	}

	else if(sys_Info_Box.ch6_led_on_off==1900&&led_flag!=1&&state!=TIMES_STATE)       //if led_on      
	{
		ContinuTimes=300;
		state=TIMES_STATE;
		times=1;
		lastBuzzerTime=systemTickMs;
		led_flag=1;
		BUZZER=1;
		
	}
	else if(sys_Info_Box.ch6_led_on_off==1100&&led_flag!=0&&state!=TIMES_STATE)       //if led_off      
	{
		ContinuTimes=300;
		state=TIMES_STATE;
		times=1;
		lastBuzzerTime=systemTickMs;
		led_flag=0;
		BUZZER=1;
	}
	else if(voltage<11000&&state!=TIMES_STATE) //if lowvoltage
	{
		if(systemTickMs-LastLowVolTime>10000)
		{
			ContinuTimes=200;
			state=TIMES_STATE;
			times=3;                              //2 times
			LastLowVolTime=systemTickMs;
			lastBuzzerTime=systemTickMs;
			BUZZER=1;
		}
	}
}


  // buzzer start 



}

////////we need wait for the param of flycontrolor 
u8 check_Radioparam_processor(void)
{
   u8 text[50];
	u8 lastcunt=0;
	u8 i=0;
	u32 lastTime=systemTickMs;
	for(i=0;i<100;i++)
	{
		 KEY_Scan();
		delay_ms(10);
  }
  while(1)
	{
    STM_update(downstream_channel, downstream_handler);
		if(KEY_Scan()) return 0;
		if(systemTickMs-lastTime>1000)
		{
//			sprintf(text,"Local=%d",sys_Info_Box.rssi);
//			LCD_ShowString(0,32,128,128,12,text,RED,BLACK);
//			sprintf(text,"Remote=%d",sys_Info_Box.remrssi);
//			LCD_ShowString(0,48,128,128,12,text,RED,BLACK);
      lastTime=systemTickMs;
			update_heartBeat();
    }
  }
   return 1;
}

