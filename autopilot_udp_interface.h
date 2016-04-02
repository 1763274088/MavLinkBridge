//
//  autopilot_udp_interface.h
//  
//
//  Created by Mohamed A AbdKader on 1/18/16.
//
//

#ifndef AUTOPILOT_UDP_INTERFACE_H_
#define AUTOPILOT_UDP_INTERFACE_H_

//#include <stdio.h>



/****************************************************************************
 *
 *   Copyright (c) 2016 Mohamed Abdelkader. All rights reserved.
 *   Author:  Mohamed Abdelkader <mohamedashraf123@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_udp_interface.h
 *
 * @brief Autopilot interface definition using UDP sockets
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink, using UDP socket
 *
 * @author Mohamed Abdelkader, <mohamedashraf123@gmail.com>
  *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>               // For cout and cerr
#include <cstdlib>                // For atoi()
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>"


#include "PracticalSocket.h"      // For UDPSocket and SocketException
// need to include mavlink types
#include "mavlink_types.h"

#include <common/mavlink.h>


#include "structs.h"
// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */


/*
// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
*/

// This is intended to be more general, not only restriced to position set points. Instead, it's intended for more Mavlink messages


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
/*
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);
*/

void* start_Autopilot_UDP_Interface_read_thread(void *args);
void* start_Autopilot_UDP_Interface_write_thread(void *args);

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------
/*
struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }
    
    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t setpoint;
    uint64_t com;
    
    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
        setpoint=0;
        com=0;
    }
    
};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {
    
    int sysid;
    int compid;
    
    // Heartbeat
    mavlink_heartbeat_t heartbeat;
    
    // System Status
    mavlink_sys_status_t sys_status;
    
    // Battery Status
    mavlink_battery_status_t battery_status;
    
    // Radio Status
    mavlink_radio_status_t radio_status;
    
    // Local Position
    mavlink_local_position_ned_t local_position_ned;
    
    // Global Position
    mavlink_global_position_int_t global_position_int;
    
    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;
    
    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;
    
    // HiRes IMU
    mavlink_highres_imu_t highres_imu;
    
    // Attitude
    mavlink_attitude_t attitude;
    
    // commanding message: for arm/ offboard toggling
    mavlink_command_long_t com;
    
    // setpoints
    mavlink_set_position_target_local_ned_t setpoint;
    
    // System Parameters?
    
    
    // Time Stamps
    Time_Stamps time_stamps;
    
    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }
    
};
*/

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 
 * *****MODIFICATION: for this UDP interface class, the write thread also sends more Mavlink messages, to have more complete interfacing.
 */
class Autopilot_UDP_Interface
{
    
public:
    
    Autopilot_UDP_Interface();
    Autopilot_UDP_Interface(UDPSocket *udp_socket_);
    ~Autopilot_UDP_Interface();
    
    //for udp
    string destAddr;
    unsigned short destPort;
    
    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;
    
    int system_id;
    int autopilot_id;
    int companion_id;
    float offb_flag;
    float arm_flag;
    
    int mavId;
    int mavCompID;
    
    Mavlink_Messages current_messages;
    Mavlink_Messages current_messages_to_read;
    Mavlink_Messages current_messages_to_write;
    mavlink_set_position_target_local_ned_t initial_position;
    
    void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
    void read_messages(void);
    void read_messages_raw(void);
    int  write_message(mavlink_message_t message);
    
    //void enable_offboard_control();
    //void disable_offboard_control();
    
    void start();
    void stop();
    
    void start_read_thread();
    void start_write_thread(void);
    
    void handle_quit( int sig );
    
    bool debug;
    
    
private:
    
    UDPSocket *udp_socket;
    
    const int inBuffLen=1024; // maximum input buffer length
    const int outBuffLen=1024;// maximum output buffer length
    
    // define I/O buffers
    uint8_t inBuff[1024]; // get from UDP, send to uart
    uint8_t outBuff[1024]; // get from uart, send to UDP
    
    bool time_to_exit;
    
    pthread_t read_tid;
    pthread_t write_tid;
    
    mavlink_set_position_target_local_ned_t current_setpoint;
    
    void read_thread();
    void write_thread(void);
    int read_mavlink_messages(mavlink_message_t &message);
    int write_mavlink_message(mavlink_message_t &message);

    
    int toggle_offboard_control( bool flag );
    void write_setpoint();
    void write_to_GC(void);
    void write_raw_mavlink(void);
    uint64_t get_time_usec(void);
    
    mavlink_status_t lastStatus;
    
};



//#endif // AUTOPILOT_UDP_INTERFACE__H_
#endif /* defined(____autopilot_udp_interface__) */

