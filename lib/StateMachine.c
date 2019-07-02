
#include "Mavlink_GCS.h"
#include "STM_GCS.h"
//#include <stdbool.h>
#include "../lib/includeMav.h"
#include "StateMachine.h"
#include "gps.h"
#include "Stick.h"
#include "config.h"
#include "flash_config.h"
#include "24l01.h"
//extern RC_BATT_INFO RCx_BatInfo;

  /* Scheduling the on_loop periodic updater. */
  uint32_t _last_run_millis;
  uint32_t _loop_period;
  uint32_t _last_vehicle_hb_millis;

  int8_t  _vehicle_mode;
  bool    _vehicle_armed;
  uint8_t _vehicle_gps_fix;
  int32_t _vehicle_lat;
  int32_t _vehicle_lon;
  int32_t _vehicle_altitude;

  uint8_t _target_system=1;
  uint8_t _target_component=1;

  bool _local_gps_valid;
  int32_t _local_gps_lat;
  int32_t _local_gps_lon;
  int32_t _local_gps_altitude;

  int32_t _offs_lat;
  int32_t _offs_lon;
  int32_t _offs_altitude;
	bool _guiding;
	
////////////////////////////////////////////////////////////////////////////////
////////////you can switch the port by the ENUM  MAVLINK_COMM_1 or MAVLINK_COMM_0
////////////
 mavlink_channel_t upstream_channel=MAVLINK_COMM_0;
 mavlink_channel_t downstream_channel=MAVLINK_COMM_0;
 
 u16 ch5=1300;
 ///mavlink_system.type = 2; to GCS
 ///ID =1
void on_upstream_command_long(mavlink_command_long_t* pkt) {
  switch(pkt->command) {
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_MISSION_START:
      /* clear out FM control of vehicle */
      _on_user_override();
    break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      /* i guess do nothing? */
    break;
  }
}

void on_upstream_set_mode(mavlink_set_mode_t* pkt) {
  /* mode is set in pkt->custom_mode */
  _vehicle_mode = (int8_t) pkt->custom_mode;
  /* clear out FM control of vehicle */
  _on_user_override();
}

void on_downstream_heartbeat(mavlink_heartbeat_t* pkt) {
  /* if mode has changed from last set_mode, the user has triggered a change
   * via RC switch.
   * clear out FM control of vehicle */
  bool pktarmed = ((pkt->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0);
  int8_t pktmode = (int8_t) pkt->custom_mode;
  if ((pktarmed != _vehicle_armed) || (pktmode != _vehicle_mode)) {
    _on_user_override();
  }
  /* update heartbeat millis */
  _last_vehicle_hb_millis =88888;// hal.scheduler->millis();
  /* update local state */ 
  _vehicle_armed = pktarmed;
  _vehicle_mode = pktmode;
}

void on_downstream_gps_raw_int(mavlink_gps_raw_int_t* pkt) {
  /* Keep track of vehicle's latest lat, lon, altitude */
  _vehicle_lat     = pkt->lat;
  _vehicle_lon     = pkt->lon;
  _vehicle_altitude = pkt->alt;
  _vehicle_gps_fix = pkt->fix_type;
}

void on_button_activate() {
  if (_guiding) return;
  /* This action is allowed to swing the state to start guide mode. */
  if (_check_guide_valid()) {
    _set_guide_offset();
    _send_guide(0,1,1000);
    _guiding = true;
    _vehicle_mode = MODE_GUIDED;
   // hal.console->println_P(PSTR("Button activated, entering guided mode"));
  } else {
//    hal.console->println_P(PSTR("Button activated but insufficient conditions "
//          "for entering guided mode"));
  }
}

void on_button_cancel() {
  if (!_guiding) return;
  _send_loiter();
  _guiding = false;
}

//void on_loop(INTERGEINFO* gps) {
// uint32_t now = hal.scheduler->millis();
// if ((_last_run_millis + _loop_period) > now) return;
// _last_run_millis = now;

// if (gps != NULL) {
//   _update_local_gps(gps);
// }

// if (_guiding) {
//   _send_guide();
// }
//}

bool _check_guide_valid() {
  uint32_t now = 888;//hal.scheduler->millis();

  bool vehicle_gps_valid = (_vehicle_gps_fix == 3);
  bool vehicle_hb_valid = (now - _last_vehicle_hb_millis) < 2000;

  bool vehicle_mode_valid = _vehicle_armed 
                          && ( (_vehicle_mode == MODE_LOITER)
                             ||(_vehicle_mode == MODE_ALT_HOLD)
                             ||(_vehicle_mode == MODE_AUTO)
                             ||(_vehicle_mode == MODE_GUIDED)
                             );
#define DEBUG 0
#if DEBUG
  if (!_local_gps_valid) {
   //hal.console->println_P(PSTR("need valid local gps"));
  }
  if (!vehicle_gps_valid) {
   // hal.console->println_P(PSTR("need valid vehicle gps"));
  }
  if (!vehicle_hb_valid) {
    //hal.console->println_P(PSTR("need valid vehicle hb"));
  }
  if (!vehicle_mode_valid) {
   // hal.console->println_P(PSTR("need valid vehicle mode"));
  }
#endif
  return _local_gps_valid
      && vehicle_gps_valid
      && vehicle_hb_valid
      && vehicle_mode_valid;
}

void on_fault_cancel(){
    if (_guiding) { 
       // hal.console->println_P(PSTR("FollowMe: Fault Cancel"));
        _send_loiter();
        _guiding = false;
    }
}

void _update_local_gps(Gps_Status* pStatus) {
	/* Cause an on_fault_cancel if when local gps has transitioned form 
	 * valid to invalid. */
	if (_local_gps_valid && !(pStatus->fix>=GPS_OK_FIX_2D)) {
		on_fault_cancel();
	} 

	_local_gps_valid = pStatus->fix>=GPS_OK_FIX_2D;
	if (pStatus->gps_new_data) {
		_local_gps_lat      = pStatus->lat;
		_local_gps_lon      = pStatus->lng;
		_local_gps_altitude = pStatus->alt;
		pStatus->gps_new_data = false;
	}
}

void _set_guide_offset() {
  _offs_lat = 0;
  _offs_lon = 0;
  _offs_altitude = 1200; /* 12m in centimeters */
}

void _on_fault_cancel() {
    if (_guiding) { 
       // hal.console->println_P(PSTR("FollowMe: Fault Cancel"));
        _send_loiter();
        _guiding = false;
    }
}

void _on_user_override() {
    if (_guiding) {
       // hal.console->println_P(PSTR("FollowMe: User GCS or RC override"));
        _guiding = false;
    }
}

__inline void _send_guide(int32_t lat,int32_t lon,int32_t alt) {
  //hal.console->println_P(PSTR("FollowMe: Sending guide waypoint packet"));

//  int32_t lat = _local_gps_lat + _offs_lat;
//  int32_t lon = _local_gps_lon + _offs_lon;
//  // int32_t alt = _local_gps_altitude + _offs_altitude;
//  int32_t alt = _offs_altitude; /* assume above ground. (ArduCopter bug.) */

  float x = (float) lat / (float) 1e7; /* lat, lon in deg * 10,000,000 */
  float y = (float) lon / (float) 1e7;
  float z = (float) alt / (float) 100; /* alt in cm */
  
//  hal.console->printf_P(
//      PSTR("FollowMe: guide x: %f y: %f z: %f\r\n"),
//      x, y, z);

  mavlink_msg_mission_item_send(
      upstream_channel, /* mavlink_channel_t chan*/
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      0, /* uint16_t seq: always 0, unknown why. */
      MAV_FRAME_GLOBAL, /* uint8_t frame: arducopter uninterpreted */
      MAV_CMD_NAV_WAYPOINT, /* uint16_t command: arducopter specific */
      2, /* uint8_t current: 2 indicates guided mode waypoint */
      0, /* uint8_t autocontinue: always 0 */
      0, /* float param1 : hold time in seconds */
      5, /* float param2 : acceptance radius in meters */
      0, /* float param3 : pass through waypoint */
      0, /* float param4 : desired yaw angle at waypoint */
      x, /* float x : lat degrees */
      y, /* float y : lon degrees */
      z  /* float z : alt meters */
      );
}

__inline void _send_loiter() {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_LOITER_UNLIM, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 */
      0, /* float param2 */
      0, /* float param3 */
      0, /* float param4 */
      0, /* float param5 */ 
      0, /* float param6 */
      0  /* float param7 */
      );
}
//////////////////////////////////for navigation way point mode 
__inline void _send_CIRCLE(float turns,float Radiusm,float yaw,float Lat,float Lon,float Alt) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      7, /* uint8_t target_system _target_system*/
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_LOITER_TURNS, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      turns, /* float param1 turns */
      0, /* float param2 empty*/
      Radiusm, /* float param3 Rdius meters ,- + change the counter-clockwise*/
      0, /* float param4 desire yaw */
      Lat, /* float param5 La*/ 
      Lon, /* float param6 Lon*/
      Alt  /* float param7 Alt*/
      );
}


__inline void _send_ALT_CHANGED(float AIMALT) {

  mavlink_msg_mission_item_send(
      upstream_channel, /* mavlink_channel_t chan*/
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      0, /* uint16_t seq: always 0, unknown why. */
      MAV_FRAME_GLOBAL_RELATIVE_ALT,//MAV_FRAME_GLOBAL, /* uint8_t frame: arducopter uninterpreted */
      MAV_CMD_CONDITION_CHANGE_ALT, /* uint16_t command: arducopter specific */
      3, /* uint8_t current: 3 indicates in a mission alt change  2,indicates guided mode waypoint */
      0, /* uint8_t autocontinue: always 0 */
      0.5, /* float param1 : hold time in seconds climbe rate m/s*/
      5, /* float param2 : acceptance radius in meters */
      0, /* float param3 : pass through waypoint */
      0, /* float param4 : desired yaw angle at waypoint */
      0, /* float x : lat degrees */
      0, /* float y : lon degrees */
      AIMALT  /* float z : alt meters */
      );

}

//MAV_CMD_DO_SET_MODE  Mission Param #1 Mode, as defined by ENUM MAV_MODE
//MAV_CMD_DO_CHANGE_SPEED  Para #1speed type(0=airspeed 1=gndSpeed),Para#2(Speed m/s , -1 = no change),para#3(Throttle %, -1 =no change),4
//mavlink_msg_mission_item_send(
//      upstream_channel, /* mavlink_channel_t chan*/
//      _target_system, /* uint8_t target_system */
//      _target_component, /* uint8_t target_component */
//      0, /* uint16_t seq: always 0, unknown why. */
//      MAV_FRAME_GLOBAL, /* uint8_t frame: arducopter uninterpreted */
//      MAV_CMD_NAV_WAYPOINT, /* uint16_t command: arducopter specific */
//      2, /* uint8_t current: 2 indicates guided mode waypoint */
//      0, /* uint8_t autocontinue: always 0 */
//      0, /* float param1 : hold time in seconds */
//      5, /* float param2 : acceptance radius in meters */
//      0, /* float param3 : pass through waypoint */
//      0, /* float param4 : desired yaw angle at waypoint */
//      x, /* float x : lat degrees */
//      y, /* float y : lon degrees */
//      z  /* float z : alt meters */
//      );

__inline void _send_TAKEOFF(float alt) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      01, /* uint8_t target_system */
      0xfa, /* uint8_t target_component */
      MAV_CMD_NAV_TAKEOFF, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 min climbrate m/s*/
      0, /* float param2 empty*/
      0, /* float param3  empty*/
      0, /* float param4 yaw angle*/
      0, /* float param5 latitude*/
      0, /* float param6 longitude*/
      alt  /* float param7 alttitude*/
      );
}

__inline void _send_TAKEOFF_MISSION(float pitch,float yaw,float lat,float lon,float alt)
{
mavlink_msg_mission_item_send(
      upstream_channel, /* mavlink_channel_t chan*/
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      0, /* uint16_t seq: always 0, unknown why. */
      MAV_FRAME_GLOBAL, /* uint8_t frame: arducopter uninterpreted */
      MAV_CMD_NAV_TAKEOFF, /* uint16_t command: arducopter specific */
      2, /* uint8_t current: 2 indicates guided mode waypoint */
      0, /* uint8_t autocontinue: always 0 */
      pitch, /* float param1 min climbrate m/s*/
      0, /* float param2 empty*/
      0, /* float param3  empty*/
      yaw, /* float param4 yaw angle*/
      lat, /* float param5 latitude*/
      lon, /* float param6 longitude*/
      alt  /* float param7 alttitude*/
      );

}

__inline void _send_LAND(float yaw,float lat,float lon,float alt) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_LAND, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 */
      0, /* float param2 */
      0, /* float param3 */
      yaw, /* float param4 desire YAW*/
      lat, /* float param5 Lat*/
      lon, /* float param6 Lon*/
      alt  /* float param7 Alt*/
      );
}

__inline void _send_RTL() {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_RETURN_TO_LAUNCH, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 empty*/
      0, /* float param2 empty*/
      0, /* float param3 empty*/
      0, /* float param4 empty*/
      0, /* float param5 empty*/
      0, /* float param6 empty*/
      0  /* float param7 empty*/
      );
}

__inline void _send_CHANGE_SPEED(float Velocitym) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_DO_CHANGE_SPEED, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 */
      Velocitym, /* float param2 m/s*/
      0, /* float param3 */
      0, /* float param4 */
      0, /* float param5 */
      0, /* float param6 */
      0  /* float param7 */
      );

}

__inline void _send_SET_ROI(float lat,float lon,float alt) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_ROI, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      MAV_ROI_LOCATION, /* float param1  region of interest mode  MAV_ROI_LOCATION*/
      0, /* float param2, Mission index/target ID   ref MOV_ROI enum */
      0, /* float param3, ROI index*/
      0, /* float param4, empty*/
      lat, /* float param5  X location of the fixed ROI( MAV_FRAME)*/
      lon, /* float param6  Y*/
      alt  /* float param7  Z*/
      );
}
//////////////////////////debug ok
__inline void _send_DISARM_ARM(float arm) 
{
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component,//MAV_COMP_ID_SYSTEM_CONTROL,// _target_component, /* uint8_t target_component */
      MAV_CMD_COMPONENT_ARM_DISARM, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      arm, /* float param1 */// 1=enable or 0=disable ARM 
      0, /* float param2 */
      0, /* float param3 */
      0, /* float param4 */
      0, /* float param5 */
      0, /* float param6 */
      0  /* float param7 */
      );
}

__inline void _send_SET_FlY_MODE(float MODE) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_DO_SET_MODE, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      MODE, /* float param1  MAV_MODE enum*/
      0, /* float param2, empty */
      0, /* float param3, empty*/
      0, /* float param4, empty*/
      0, /* float param5  empty*/
      0, /* float param6  empty*/
      0  /* float param7  empty*/
      );
}
//void _send_CIRCLE(int lat,int lon) {
//  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
//  mavlink_msg_command_long_send(
//      upstream_channel, /* mavlink_channel_t chan */
//      _target_system, /* uint8_t target_system */
//      _target_component, /* uint8_t target_component */
//      MAV_CMD_NAV_LAND, /* uint16_t command: arducopter specific */
//      0, /* uint8_t confirmation */
//      0, /* float param1 */
//      0, /* float param2 */
//      0, /* float param3 */
//      0, /* float param4 */
//      lat, /* float param5 */
//      lon, /* float param6 */
//      alt  /* float param7 */
//      );
//}

__inline void _send_MissionPlanner(void) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_heartbeat_send(
      upstream_channel, /* mavlink_channel_t chan */
      255, /* type */
      06,/* autopilot,*/
      8, /* base_mode */
      0, /* custom_mode */
      0 /* system_status */
      );
}

__inline void _send_DATA_STREAM_REQUEST(u16 StreamID,u16 req_rate) {
  //hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_request_data_stream_send(
      upstream_channel, /* mavlink_channel_t chan */
       0x1, /* target_system */
      0x1,/* target_component,*/
      StreamID, /* req_stream_id */
      req_rate, /* req_message_rate */
      1 /* start_stop */
      );
}

//mavlink_msg_param_request_read_send(
//	upstream_channel,//mavlink_channel_t chan, 
//	0x1,//uint8_t target_system, 
//	0x1,//uint8_t target_component, 
//	,//const char *param_id, 
//	,//int16_t param_index
//	);


__inline void _send_PARAM_REUEST_READ(void)
{
	 mavlink_msg_param_request_list_send(
	upstream_channel,//mavlink_channel_t chan, 
	0x01,//uint8_t target_system, 
	0x01//uint8_t target_component
	);
}


__inline void _send_PARAM_SET(const char *param_id,float param_value,uint8_t param_type)
{
  mavlink_msg_param_set_send(
	upstream_channel,//mavlink_channel_t chan, 
	_target_system,//uint8_t target_system, 
	_target_component,//uint8_t target_component, 
	param_id,//const char *param_id, 
	param_value,//float param_value, 
	param_type//uint8_t param_type
	);
}



__inline void _send_SET_MODE(u8 base_mode,u32 cust_mode)
{  
//   mavlink_msg_set_mode_send(
//	upstream_channel,/*mavlink chan  mavlink_msg_set_nav_mode_send */
//	_target_system,  /*_target_system*/
//	base_mode,       /* new base mode*/
//	cust_mode        /*new autopilot-specific mode ,can ignore by autopilot*/
//	);
	SendRc_CHS_CMD.SetFlightMode=cust_mode;
	
}

__inline void _send_RC_OVERRIDE(u16 ch6,vs32 altmm,u16 ch8_mode)
{
//	if(altmm >LimitHigh)
//	{if(1400>=RCx_BatInfo.RC[2].ch)
//		  RCx_BatInfo.RC[2].ch =1500;
//	}
//  mavlink_msg_rc_channels_override_send(
//	upstream_channel,/*mavlink_channel_t chan,*/
//	_target_system,/*uint8_t target_system, */
//	_target_component,/*uint8_t target_component,*/ 
//	RCx_BatInfo.RC[0].ch,/*uint16_t chan1_raw, */
//	3000-RCx_BatInfo.RC[1].ch,/*uint16_t chan2_raw, */
//	3000-RCx_BatInfo.RC[2].ch,/*uint16_t chan3_raw, */
//	3000-RCx_BatInfo.RC[3].ch,/*uint16_t chan4_raw, */
//	1500,/*uint16_t chan5_raw, *///ch51300放下        1800收起
//	ch6,/*uint16_t chan6_raw, */
//	RCx_BatInfo.RC[4].ch,//RCx_BatInfo.RC[0].ch,/*uint16_t chan7_raw, */
//	RCx_BatInfo.RC[5].ch//ch8_mode//RCx_BatInfo.RC[0].ch,/*uint16_t chan8_raw  */
//	);
//	SendRc_CHS_CMD.rc[0]=RCx_BatInfo.RC[0].ch;
//	SendRc_CHS_CMD.rc[1]=3000-RCx_BatInfo.RC[1].ch;
//	SendRc_CHS_CMD.rc[2]=3000-RCx_BatInfo.RC[2].ch;
//	SendRc_CHS_CMD.rc[3]=3000-RCx_BatInfo.RC[3].ch;
//	SendRc_CHS_CMD.rc[4]=1500;
//	SendRc_CHS_CMD.rc[5]=ch6;
//	SendRc_CHS_CMD.rc[6]=RCx_BatInfo.RC[4].ch;
//	SendRc_CHS_CMD.rc[7]=RCx_BatInfo.RC[5].ch;
	
	
}


__inline void _send_RC_OVERRIDE_RIGHT_MODE(u16 ch6,vs32 altmm,u16 ch8_mode)
{
//	if(altmm >LimitHigh)
//	{
//		if(RCx_BatInfo.RC[1].ch>=1600)
//		  RCx_BatInfo.RC[1].ch =1500;
//	}
////  mavlink_msg_rc_channels_override_send(
////	upstream_channel,/*mavlink_channel_t chan,*/
////	_target_system,/*uint8_t target_system, */
////	_target_component,/*uint8_t target_component,*/ 
////	RCx_BatInfo.RC[0].ch,/*uint16_t chan1_raw, */
////	RCx_BatInfo.RC[2].ch,/*uint16_t chan2_raw, */
////	RCx_BatInfo.RC[1].ch,/*uint16_t chan3_raw, */
////	3000-RCx_BatInfo.RC[3].ch,/*uint16_t chan4_raw, */
////	1500,/*uint16_t chan5_raw,ch5 */
////	ch6,/*uint16_t chan6_raw, */
////	RCx_BatInfo.RC[4].ch,//RCx_BatInfo.RC[0].ch,/*uint16_t chan7_raw, */
////	RCx_BatInfo.RC[5].ch//ch8_mode//RCx_BatInfo.RC[0].ch,/*uint16_t chan8_raw  */
////	);

//	SendRc_CHS_CMD.rc[0]=RCx_BatInfo.RC[0].ch;
//	SendRc_CHS_CMD.rc[1]=RCx_BatInfo.RC[2].ch;
//	SendRc_CHS_CMD.rc[2]=RCx_BatInfo.RC[1].ch;
//	SendRc_CHS_CMD.rc[3]=3000-RCx_BatInfo.RC[3].ch;
//	SendRc_CHS_CMD.rc[4]=1500;
//	SendRc_CHS_CMD.rc[5]=ch6;
//	SendRc_CHS_CMD.rc[6]=RCx_BatInfo.RC[4].ch;
//	SendRc_CHS_CMD.rc[7]=RCx_BatInfo.RC[5].ch;
//	
}

__inline void _send_RC_OVERRIDE_Dummy(u16 thro,u16 ch1_roll,u16 ch2_pitch,u16 Yaw,u16 Gimble,u16 LedOnOff)
{
//  mavlink_msg_rc_channels_override_send(
//	upstream_channel,/*mavlink_channel_t chan,*/
//	_target_system,/*uint8_t target_system, */
//	_target_component,/*uint8_t target_component,*/ 
//	ch1_roll,/*uint16_t chan1_raw, */
//	ch2_pitch,/*uint16_t chan2_raw, */
//	thro,/*uint16_t chan3_raw, */
//	Yaw,/*uint16_t chan4_raw, */
//	1500,/*uint16_t chan5_raw, */
//	LedOnOff,/*uint16_t chan6_raw, */
//	Gimble,//RCx_BatInfo.RC[0].ch,/*uint16_t chan7_raw, */
//	1500//RCx_BatInfo.RC[0].ch,/*uint16_t chan8_raw  */
//	);
	
	SendRc_CHS_CMD.rc[0]=ch1_roll;
	SendRc_CHS_CMD.rc[1]=ch2_pitch;
	SendRc_CHS_CMD.rc[2]=thro;
	SendRc_CHS_CMD.rc[3]=Yaw;
	SendRc_CHS_CMD.rc[4]=1500;
	SendRc_CHS_CMD.rc[5]=LedOnOff;
	SendRc_CHS_CMD.rc[6]=Gimble;
	SendRc_CHS_CMD.rc[7]=1500;

}

__inline void _send_MISSION_CLEAR(void)
{
    mavlink_msg_mission_clear_all_send(
	   upstream_channel,
	   1,
	   0
	);
}

__inline void _send_beat_heart(u8 mode)
{
    mavlink_msg_heartbeat_send(
	upstream_channel,//mavlink_channel_t chan, 
	81,//uint8_t type, 
	MAV_AUTOPILOT_GENERIC,//uint8_t autopilot, 
	0,//uint8_t base_mode, 
	0,//uint32_t custom_mode,
	0);//uint8_t system_status);
}

// param1 : target angle [0-360]
// param2 : speed during change [deg per second]
// param3 : direction (-1:ccw, +1:cw)
// param4 : relative offset (1) or absolute angle (0)
__inline void _send_CHANGE_YAW(float angel,float direction) {
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_CONDITION_YAW, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      angel, /* float param1 */
      30, /* float param2  ignore by fC*/
      direction, /* float param3 */
      0, /* float param4 */
      0, /* float param5 */
      0, /* float param6 */
      0  /* float param7 */
      );
}
