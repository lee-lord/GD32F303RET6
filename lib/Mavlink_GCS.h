#ifndef _MAVLINK_GCS_H_H
#define _MAVLINK_GCS_H_H
#include <stdbool.h>
#include "../lib/includeMav.h"
//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_RAW,
    MSG_SYSTEM_TIME,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_LIMITS_STATUS,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_TERRAIN,
    MSG_BATTERY2,
    MSG_RETRY_DEFERRED // this must be last
};
typedef enum{
eROLL =0,
ePITCH =1,
eYAW =2,
eLAT   ,
eLON   ,
eALT  
} ARHS;

extern void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);
extern uint16_t comm_get_available(mavlink_channel_t chan);
uint8_t comm_receive_ch(mavlink_channel_t chan);
#endif
