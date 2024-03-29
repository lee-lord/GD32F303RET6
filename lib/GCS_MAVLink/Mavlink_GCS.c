// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/
#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "../lib/includeMav.h"
#include <stdbool.h>


mavlink_system_t mavlink_system = {255,190,0,0};
#ifdef MAVLINK_SEPARATE_HELPERS
#include "../include/mavlink/v1.0/mavlink_helpers.h"
#endif

mavlink_message_t MyLink;
//USART1	*mavlink_comm_0_port;
//AP_HAL::BetterStream	*mavlink_comm_1_port;
//#if MAVLINK_COMM_NUM_BUFFERS > 2
//AP_HAL::BetterStream	*mavlink_comm_2_port;
//#endif



// mask of serial ports disabled to allow for SERIAL_CONTROL
static uint8_t mavlink_locked_mask;

/*
  lock a channel, preventing use by MAVLink
 */
void lock_channel(mavlink_channel_t _chan, bool lock)
{
    if (_chan >= MAVLINK_COMM_NUM_BUFFERS) {
        return;
    }
    if (lock) {
        mavlink_locked_mask |= (1U<<(unsigned)_chan);
    } else {
        mavlink_locked_mask &= ~(1U<<(unsigned)_chan);
    }
}

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
        return 1;
    // Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
    // If it is addressed to our system ID we assume it is for us
    return 0; // no error
}

// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAVLINK_TYPE_INT8_T;
    }
    if (t == AP_PARAM_INT16) {
	    return MAVLINK_TYPE_INT16_T;
    }
    if (t == AP_PARAM_INT32) {
	    return MAVLINK_TYPE_INT32_T;
    }
    // treat any others as float
    return MAVLINK_TYPE_FLOAT;
}



/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		data = USART3Read();
		break;
	case MAVLINK_COMM_1:
		data = USART2Read();
		break;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	case MAVLINK_COMM_2:
		data = USART1Read();
		break;
#endif
	default:
		break;
	}
    return data;
}

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    if ((1U<<chan) & mavlink_locked_mask) {
        return 0;
    }
	int16_t ret = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		ret = USART3Clear();
		break;
	case MAVLINK_COMM_1:
		ret = USART2Clear();
		break;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	case MAVLINK_COMM_2:
		ret = USART1Clear();
		break;
#endif
	default:
		break;
	}
	if (ret < 0) {
		ret = 0;
	}
    return (uint16_t)ret;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan)
{
    if ((1U<<chan) & mavlink_locked_mask) {
        return 0;
    }
    int16_t bytes = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		bytes = USART3available();
		break;
	case MAVLINK_COMM_1:
		bytes = USART2available();
		break;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	case MAVLINK_COMM_2:
		bytes = USART1available();
		break;
#endif
	default:
		break;
	}
	if (bytes == -1) {
		return 0;
	}
    return (uint16_t)bytes;
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
   switch(chan) {
	case MAVLINK_COMM_0:
  USART3Writes(buf, len);
		break;
	case MAVLINK_COMM_1:
		USART2Writes(buf, len);
		break;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	case MAVLINK_COMM_2:
				USART1Writes(buf, len);
		break;
#endif
	default:
		break;
	}
}

static const uint8_t mavlink_message_crc_progmem[256]  = MAVLINK_MESSAGE_CRCS;

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid)
{
	return  *(const uint8_t * )&mavlink_message_crc_progmem[msgid];//pgm_read_byte(&mavlink_message_crc_progmem[msgid]);
}


/*
  return true if the MAVLink parser is idle, so there is no partly parsed
  MAVLink message being processed
 */
bool comm_is_idle(mavlink_channel_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	return status == NULL || status->parse_state <= MAVLINK_PARSE_STATE_IDLE;
}
