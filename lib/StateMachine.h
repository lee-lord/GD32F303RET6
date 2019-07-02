#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_
#include <stdbool.h>
/* These have been taken from the ArduCopter/defines.h. Kinda wish they were
 * globally accessible...
 * prefixed with MODE_ for namespacing. */

// Auto Pilot modes
// ----------------
#define MODE_STABILIZE 0                     // hold level position
#define MODE_ACRO 1                          // rate control
#define MODE_ALT_HOLD 2                      // AUTO control
#define MODE_AUTO 3                          // AUTO control
#define MODE_GUIDED 4                        // AUTO control
#define MODE_LOITER 5                        // Hold a single location
#define MODE_RTL 6                           // AUTO control
#define MODE_CIRCLE 7                        // AUTO control
#define MODE_POSITION 8                      // AUTO control
#define MODE_LAND 9                          // AUTO control
#define MODE_OF_LOITER 10            // Hold a single location using optical flow
                                // sensor
#define MODE_TOY_A 11                                // THOR Enum for Toy mode
#define MODE_TOY_M 12                                // THOR Enum for Toy mode
#define MODE_NUM_MODES 13
#define MODE_LOST 66
#define ARMED 1
#define DISARMED 0
__inline bool _check_guide_valid(void);
__inline void _set_guide_offset(void);
__inline void _on_user_override(void);
__inline void _send_loiter(void); 
__inline void _send_DISARM_ARM(float arm);
__inline void _send_TAKEOFF(float alt);
__inline void _send_SET_FlY_MODE(float MODE);
__inline void _send_MissionPlanner(void);
__inline void _send_DATA_STREAM_REQUEST(u16 StreamID,u16 req_rate);
//__inline void _send_RC_OVERRIDE(u16 ch6,u16 altmm);
__inline void _send_RC_OVERRIDE(u16 ch6,vs32 altmm,u16 ch8_mode);
//__inline void _send_RC_OVERRIDE_RIGHT_MODE(u16 ch6,u16 altmm);
__inline void _send_RC_OVERRIDE_RIGHT_MODE(u16 ch6,vs32 altmm,u16 ch8_mode);
__inline void _send_SET_MODE(u8 base_mode,u32 cust_mode);
__inline void _send_TAKEOFF_MISSION(float pitch,float yaw,float lat,float lon,float alt);
__inline void _send_MISSION_CLEAR(void);
__inline void _send_beat_heart(u8 mode);
__inline void _send_CIRCLE(float turns,float Radiusm,float yaw,float Lat,float Lon,float Alt);
__inline void _send_PARAM_REUEST_READ(void);
__inline void _send_PARAM_SET(const char *param_id,float param_value,uint8_t param_type);
__inline void _send_guide(int32_t lat,int32_t lon,int32_t alt);
__inline void _send_ALT_CHANGED(float AIMALT);
__inline void _send_RTL(void);
__inline void one_key_FollowMe(u16 * keyvaule);
//__inline void _send_RC_OVERRIDE_Dummy(u16 thro,u16 ch1_roll,u16 ch2_pitch,u16 Yaw,u16 Gimble);
__inline void _send_RC_OVERRIDE_Dummy(u16 thro,u16 ch1_roll,u16 ch2_pitch,u16 Yaw,u16 Gimble,u16 LedOnOff);

__inline void _send_CHANGE_YAW(float angel,float direction);
__inline void _send_CHANGE_SPEED(float Velocitym);

#endif
