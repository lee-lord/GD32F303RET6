#ifndef _STM_GCS_H_
#define _STM_GCS_H_
#include "includeMav.h"
#define LEFT_THRO_MODE  1
#define RIGHT_THRO_MODE 2
#define PROFESSIONAL_MODE 1
#define JUNIOR_MODE       2

#if 1  //english program
#define GPS_SATS   10    //for stick english
#define GPS_HDOP   220  //for stick english
#else
#define GPS_SATS   10    //for stick chinese
#define GPS_HDOP   220  //for stick chinese
#endif
typedef struct{
	uint32_t heartbeatCount;
	uint32_t connectedTime;
	uint32_t heartbeatTime;
	mavlink_heartbeat_t heartbeat;
	mavlink_gps_raw_int_t remoteGps;
	mavlink_attitude_t attitude;
	mavlink_raw_imu_t imu;
	mavlink_sensor_offsets_t sensor_offset;
	mavlink_battery_status_t battery;
}RemoteStatus;



extern uint32_t lastMavlinkTick;
extern RemoteStatus remoteStatus;

typedef void(*HANDLER) (mavlink_channel_t,mavlink_message_t*);


void STM_update(mavlink_channel_t chan, HANDLER handler);
void upstream_handler(mavlink_channel_t from, mavlink_message_t* msg);
void downstream_handler(mavlink_channel_t from, mavlink_message_t* msg);

void Mavlink_Init(void);
void reset_mavlink(void);
void update_RC_20HZ(void);
void update_heartBeat(void);
void set_g_thro(u16 thro);
void _oneKeyFly(u16 * keyvaule);
void _oneKeyLand(u16 *keyvaule);
void _oneChangeAlt(u16 keyvaule);
void Disarm_Arm(void);
u8 check_param_processor(void);
void SiMToo_Update_Loop(void);
void one_key_Circle(u16 * keyvaule);
void one_key_rotate_self(u16 * keyvaule);
void stick_key_Sevro(u16 *keyvaule);
void one_key_FollowMe(u16 * keyvaule);
void FollowMe_GUIDING_Loop(void);
void StopFollowme(void);
void predict_New_Positon(u32 Xoffset,u32 Yoffset);
void SetLeadMeStatus(u8 status);
void SetYaw(u16 ch4yaw);
__inline void sendDisArm(u16 * keyvaule);
__inline void Condition_Yaw_thread(void);
__inline void Guide_Condition_Yaw_Loop(u16 * keyvaule);
__inline void setConditionYaw(u8 status);
__inline void FollowMe_GUIDING_LeftFront_Loop(void);
void SetLeadMeStatus(u8 status);
void clearFlyModeFlag(void);
void setLedOnOff(u16 * keyvaule);
void set_OPERATOR_mode(u16 mode);
void SetProfessionalMode(u8 pmode);
void CheckUsrMode(void);
void Set_Reset_SuperMode(void);
void resetSuperMode(void);
__inline void Set_mode_Once(void);
__inline void Buzzer_loop(u16 keyValue);
__inline void Buzzer_interrupt();
u8 check_Radioparam_processor(void);
float fast_atan2(float x, float y);
void SiMToo_Update_Loop1(void);
#define _100HZ 10
#define _50HZ 20
#define _40HZ 25
#define _30HZ 33
#define _20HZ 50
#define _10HZ 100
#define _5HZ  200
#define _2HZ  500
#define _1HZ 1000
#define _Dot5HZ 2000


typedef __packed struct{
 int32_t  lat; ///< Latitude, expressed as * 1E7
 int32_t  lon; ///< Longitude, expressed as * 1E7
 int32_t  alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t  Holdalt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL 	
 int32_t  relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
 int16_t  vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 int16_t  vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 int16_t  vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 uint16_t eph; ///< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 uint16_t vel; ///< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 uint16_t cog; ///< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 uint8_t  fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 uint8_t  satellites_visible; ///< Number of satellites visible. If unknown, set to 255
 //uint16_t load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 uint16_t voltage_battery; ///< Battery voltage, in millivolts (1 = 1 millivolt)
 //int16_t  current_battery; ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 //uint16_t drop_rate_comm; ///< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)	
 uint8_t  target_system; ///< System ID
 uint8_t  target_component; ///< Component ID
 uint8_t  rssi; ///< local signal strength
 uint8_t  remrssi; ///< remote signal strength
 //uint16_t rxerrors; ///< receive errors
// uint16_t fixed; ///< count of error corrected packets
  uint8_t cells; ///< battery cells number
// uint8_t noise; ///< background noise level
// uint8_t remnoise; ///< remote background noise level
 float press_abs; ///< Absolute pressure (hectopascal)
 float press_diff; ///< Differential pressure 1 (hectopascal)
 int16_t temperature; ///< Temperature measurement (0.01 degrees celsius)
 uint32_t custom_mode; ///< A bitfield for use for autopilot-specific flags.
 uint8_t type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 uint8_t autopilot; ///< Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 uint8_t base_mode; ///< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 uint8_t system_status; ///< System status flag, see MAV_STATE ENUM
 uint64_t UNIX_time;
 u8    connect_state;
 u8    FllowMeStart;
 u8    LeadMeStart;
 u8    yaw_start_stop;
 u16   localBatt;
 u16    ch6_led_on_off;
 u16 ch6_Land_on_off;
 u16    Loitor_yaw;
 uint32_t  distance;  
 uint32_t  StickMode;
 uint32_t  user_mode;
 u16      oneKeyFly;
 u16     CH8_Mode_Super;
 u16    warring;
  u16    SYS_SHOW;
}MSGBOX;

enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables

        // simulation
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // relay object
        k_param_relay,

        // EPM object
        k_param_epm,

        // BoardConfig object
        k_param_BoardConfig,

        // GPS object
        k_param_gps,

        // Parachute object
        k_param_parachute,	// 17

        // Misc
        //
        k_param_log_bitmask_old = 20,           // Deprecated
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                   // deprecated - remove
        k_param_crosstrack_min_distance,	// deprecated - remove with next eeprom number change
        k_param_rssi_pin,
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_velocity_z_max,
        k_param_circle_rate,                // deprecated - remove
        k_param_sonar_gain,
        k_param_ch8_option,
        k_param_arming_check,
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,
        k_param_angle_rate_max,         // remove
        k_param_rssi_range,
        k_param_rc_feel_rp,
        k_param_NavEKF,                 // Extended Kalman Filter Inertial Navigation Group
        k_param_mission,                // mission library
        k_param_rc_13,
        k_param_rc_14,
        k_param_rally,
        k_param_poshold_brake_rate,
        k_param_poshold_brake_angle_max,
        k_param_pilot_accel_z,
        k_param_serial0_baud,
        k_param_serial1_baud,
        k_param_serial2_baud,
        k_param_land_repositioning,
        k_param_sonar, // sonar object
        k_param_ekfcheck_thresh,
        k_param_terrain,
        k_param_acro_expo,
        k_param_throttle_deadzone,
        k_param_optflow,
        k_param_dcmcheck_thresh,        // 59
        k_param_log_bitmask,

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence,
        k_param_gps_glitch,
        k_param_baro_glitch,            // 71

        //
        // 75: Singlecopter, CoaxCopter
        //
        k_param_single_servo_1 = 75,
        k_param_single_servo_2,
        k_param_single_servo_3,
        k_param_single_servo_4, // 78

        //
        // 80: Heli
        //
        k_param_heli_servo_1 = 80,
        k_param_heli_servo_2,
        k_param_heli_servo_3,
        k_param_heli_servo_4,
        k_param_heli_pitch_ff,      // remove
        k_param_heli_roll_ff,       // remove
        k_param_heli_yaw_ff,        // remove
        k_param_heli_stab_col_min,
        k_param_heli_stab_col_max,  // 88

        //
        // 90: Motors
        //
        k_param_motors = 90,

        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100,
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,     // 104

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud_old, // deprecated
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial2_baud_old, // deprecated
        k_param_serial2_protocol,

        //
        // 140: Sensor parameters
        //
        k_param_imu = 140, // deprecated - can be deleted
        k_param_battery_monitoring = 141,   // deprecated - can be deleted
        k_param_volt_div_ratio, // deprecated - can be deleted
        k_param_curr_amp_per_volt,  // deprecated - can be deleted
        k_param_input_voltage,  // deprecated - can be deleted
        k_param_pack_capacity,  // deprecated - can be deleted
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled_old, // deprecated
        k_param_frame_orientation,
        k_param_optflow_enabled,
        k_param_fs_batt_voltage,
        k_param_ch7_option,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_sonar_type_old,     // deprecated
        k_param_super_simple = 155,
        k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
        k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
        k_param_ahrs, // AHRS group // 159

        //
        // 160: Navigation parameters
        //
        k_param_rtl_altitude = 160,
        k_param_crosstrack_gain,	// deprecated - remove with next eeprom number change
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,
        k_param_tilt_comp, 	//164	deprecated - remove with next eeprom number change


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,
        k_param_camera_mount2,

        //
        // Batery monitoring parameters
        //
        k_param_battery_volt_pin = 168, // deprecated - can be deleted
        k_param_battery_curr_pin,   // 169 deprecated - can be deleted

        //
        // 170: Radio settings
        //
        k_param_rc_1 = 170,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_10,
        k_param_rc_11,
        k_param_throttle_min,
        k_param_throttle_max,
        k_param_failsafe_throttle,
        k_param_throttle_fs_action,     // remove
        k_param_failsafe_throttle_value,
        k_param_throttle_cruise,
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed = 192,
        k_param_failsafe_battery_enabled,
        k_param_throttle_mid,
        k_param_failsafe_gps_enabled,
        k_param_rc_9,
        k_param_rc_12,
        k_param_failsafe_gcs,           // 198
        k_param_rcmap,

        //
        // 200: flight modes
        //
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_simple_modes,

        //
        // 210: Waypoint data
        //
        k_param_waypoint_mode = 210, // remove
        k_param_command_total,       // remove
        k_param_command_index,       // remove
        k_param_command_nav_index,   // remove
        k_param_waypoint_radius,     // remove
        k_param_circle_radius,       // remove
        k_param_waypoint_speed_max,  // remove
        k_param_land_speed,
        k_param_auto_velocity_z_min, // remove
        k_param_auto_velocity_z_max, // remove - 219

        //
        // 220: PI/D Controllers
        //
        k_param_acro_rp_p = 221,
        k_param_axis_lock_p,    // remove
        k_param_pid_rate_roll,
        k_param_pid_rate_pitch,
        k_param_pid_rate_yaw,
        k_param_p_stabilize_roll,
        k_param_p_stabilize_pitch,
        k_param_p_stabilize_yaw,
        k_param_p_loiter_pos,
        k_param_p_loiter_lon,       // remove
        k_param_pid_loiter_rate_lat,
        k_param_pid_loiter_rate_lon,
        k_param_pid_nav_lat,        // 233 - remove
        k_param_pid_nav_lon,        // 234 - remove
        k_param_p_alt_hold,
        k_param_p_throttle_rate,
        k_param_pid_optflow_roll,
        k_param_pid_optflow_pitch,
        k_param_acro_balance_roll_old,  // 239 - remove
        k_param_acro_balance_pitch_old, // 240 - remove
        k_param_pid_throttle_accel,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p, // 244

        // 254,255: reserved
    };

typedef  __packed struct {
 float    param_value; ///< Onboard parameter value
 uint16_t param_index; ///< Index of this onboard parameter
 char     param_id[16]; ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 uint8_t  param_type;
}FLY_SET_PARA;

extern FLY_SET_PARA g_flight_Param[];
#endif
