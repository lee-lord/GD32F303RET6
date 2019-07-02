#ifndef _INCLUDEMAV_H_
#define _INCLUDEMAV_H_
#include "usart.h"
//#include "include/mavlink/v1.0/ardupilotmega/version.h"
//#include "include/mavlink/v1.0/mavlink_types.h"
//#include "include/mavlink/v1.0/ardupilotmega/mavlink.h"
#define MAVLINK_SEPARATE_HELPERS
#include "include/mavlink/v1.0/ardupilotmega/version.h"

#define MAVLINK_COMM_NUM_BUFFERS 2
#define MAVLINK_MAX_PAYLOAD_LEN 255
#include "include/mavlink/v1.0/mavlink_types.h"

static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
switch(chan) {
	case MAVLINK_COMM_0:
		USART1Write(ch);
		break;
	case MAVLINK_COMM_1:
		USART2Write(ch);
		break;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	case MAVLINK_COMM_2:
		USART3Write(ch);
		break;
#endif
	default:
		break;
	}
}
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v1.0/ardupilotmega/mavlink.h"
// severity levels used in STATUSTEXT messages
enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL,
    SEVERITY_USER_RESPONSE
};

#include <stdbool.h>
#endif

