//
//  autopilot_udp_interface.cpp
//  
//
//  Created by Mohamed A AbdKader on 1/18/16.
//
//

/****************************************************************************
 *
 *   Copyright (c) 2014 Mohamed Abdlekader. All rights reserved.
 *   Author: Mohamed Abdelkader <mohamedashraf123@gmail.com>
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
 * @file autopilot_udp_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink, using UDP sockets
 *
 * @author Mohamed Abdelkader, <mohamedashraf123@gmail.com>

 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "autopilot_udp_interface.h"
//#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
Autopilot_UDP_Interface::
get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
/*
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    
    sp.x   = x;
    sp.y   = y;
    sp.z   = z;
    
    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
    
}
*/
/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
/*
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask =
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;
    
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    
    sp.vx  = vx;
    sp.vy  = vy;
    sp.vz  = vz;
    
    //printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
    
}
*/
/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
/*
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{
    
    // NOT IMPLEMENTED
    fprintf(stderr,"set_acceleration doesn't work yet \n");
    throw 1;
    
    
    sp.type_mask =
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;
    
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    
    sp.afx  = ax;
    sp.afy  = ay;
    sp.afz  = az;
}
*/
// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
/*
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
    
    sp.yaw  = yaw;
    
    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
    
}
*/
/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
/*
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &=
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;
    
    sp.yaw_rate  = yaw_rate;
}
*/


// ----------------------------------------------------------------------------------
//   Autopilot UDP Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_UDP_Interface::
Autopilot_UDP_Interface(UDPSocket *udp_socket_)
{
    // initialize attributes
    write_count = 0;
    
    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit   = false;  // flag to signal thread exit
    
    read_tid  = 0; // read thread id
    write_tid = 0; // write thread id
    
    system_id    = 0; // system id
    autopilot_id = 0; // autopilot component id
    companion_id = 0; // companion computer component id
    mavId=0;
    mavCompID=0;
    
    offb_flag=0;// disable offboard flag
    
    arm_flag=0;
    
    current_messages.sysid  = system_id;
    current_messages.compid = autopilot_id;
    
    current_messages_to_read.sysid=0;
    current_messages_to_read.compid=0;
    
    current_messages_to_write.sysid=0;
    current_messages_to_write.compid=0;
    
    //serial_port = serial_port_; // serial port management object
    udp_socket = udp_socket_; // udp socket management object
    destAddr="127.0.0.1"; // destination Address
    destPort=2007;        // destination port number
    
    debug=0;
    
}

Autopilot_UDP_Interface::
~Autopilot_UDP_Interface()
{}

/*
// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
    current_setpoint = setpoint;
}
*/

// ------------------------------------------------------------------------------
//   Build  Mavlink Messages from received UDP packtes
// ------------------------------------------------------------------------------
int
Autopilot_UDP_Interface::
read_mavlink_messages(mavlink_message_t &message)
{
    uint8_t          cp;// this is intended for serial port, not UDP
    mavlink_status_t status;
    uint8_t          msgReceived = false;
    int result;
    
    
    // --------------------------------------------------------------------------
    //   READ FROM PORT
    // --------------------------------------------------------------------------
    
    // this function locks the port during read
    //int result = _read_port(cp);
    try {
     result = udp_socket->recv(inBuff, inBuffLen);// is it '->' or just '.'
    }catch(exception& e)
    {
        cout << "udp error: "<<e.what() << endl;
    }
    
    // --------------------------------------------------------------------------
    //   PARSE MESSAGE
    // --------------------------------------------------------------------------
    if (result > 0)
    {
        for (int i=0; i<result; i++){
            // the parsing
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, inBuff[i], &message, &status);
            
            // check for dropped packets
            if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
            {
                printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
                unsigned char v=cp;
                fprintf(stderr,"%02x ", v);
            }
            lastStatus = status;
            if (msgReceived)
                break;
        }
    }
    
    // Couldn't read from port
    else
    {
        //fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
        cerr << "Could not read from port." << endl;
    }
    
    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if(msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
        
        fprintf(stderr,"Received serial data: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        
        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
        
        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }
        
        // print out the buffer
        else
        {
            for (i=0; i<messageLength; i++)
            {
                unsigned char v=buffer[i];
                fprintf(stderr,"%02x ", v);
            }
            fprintf(stderr,"\n");
        }
    }
    
    // Done!
    return msgReceived;
}

/*
* ------------------------------------------------------------------------------
*   Read Messages
* this expected to read from Ground Station
* so expected messages:
* heartbeat
* arm/disarm
* toggle offboard, command_long
* set points
* set mode
 * mocap?
* ------------------------------------------------------------------------------
*/
void
Autopilot_UDP_Interface::
read_messages()
{
    bool success;               // receive success flag
    bool received_all = false;  // receive only one message
    Time_Stamps this_timestamps;
    
    // Blocking wait for new data
    //while ( not received_all and not time_to_exit )
    unsigned int counter=0;
    
    current_messages_to_read.reset_timestamps();
    cout << "time stamp... "<<current_messages_to_read.time_stamps.heartbeat << endl;
    //loop 5 times?
    while (not time_to_exit && counter < 10)
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        success = this->read_mavlink_messages(message); // 'this' means the current class object
        
        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if( success )
        {
            
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages_to_read.sysid  = message.sysid;
            current_messages_to_read.compid = message.compid;
            
            // Handle Message ID
            switch (message.msgid)
            {
                    
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages_to_read.heartbeat));
                    current_messages_to_read.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = current_messages_to_read.time_stamps.heartbeat;
                    break;
                }
                    
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(current_messages_to_read.sys_status));
                    current_messages_to_read.time_stamps.sys_status = get_time_usec();
                    this_timestamps.sys_status = current_messages_to_read.time_stamps.sys_status;
                    break;
                }
                    
                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(current_messages_to_read.battery_status));
                    current_messages_to_read.time_stamps.battery_status = get_time_usec();
                    this_timestamps.battery_status = current_messages_to_read.time_stamps.battery_status;
                    break;
                }
                    
                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(current_messages_to_read.radio_status));
                    current_messages_to_read.time_stamps.radio_status = get_time_usec();
                    this_timestamps.radio_status = current_messages_to_read.time_stamps.radio_status;
                    break;
                }
                    
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(current_messages_to_read.local_position_ned));
                    current_messages_to_read.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned = current_messages_to_read.time_stamps.local_position_ned;
                    break;
                }
                    
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                    current_messages_to_read.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int = current_messages_to_read.time_stamps.global_position_int;
                    break;
                }
                    
                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(current_messages_to_read.position_target_local_ned));
                    current_messages_to_read.time_stamps.position_target_local_ned = get_time_usec();
                    this_timestamps.position_target_local_ned = current_messages_to_read.time_stamps.position_target_local_ned;
                    break;
                }
                    
                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(current_messages_to_read.position_target_global_int));
                    current_messages_to_read.time_stamps.position_target_global_int = get_time_usec();
                    this_timestamps.position_target_global_int = current_messages_to_read.time_stamps.position_target_global_int;
                    break;
                }
                    
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(current_messages_to_read.highres_imu));
                    current_messages_to_read.time_stamps.highres_imu = get_time_usec();
                    this_timestamps.highres_imu = current_messages_to_read.time_stamps.highres_imu;
                    break;
                }
                    
                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages_to_read.attitude));
                    current_messages_to_read.time_stamps.attitude = get_time_usec();
                    this_timestamps.attitude = current_messages_to_read.time_stamps.attitude;
                    break;
                }
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                {
                    cout << "Got Set position message" <<endl;
                    mavlink_msg_set_position_target_local_ned_decode(&message, &(current_messages_to_read.setpoint));
                    current_messages_to_read.time_stamps.setpoint = get_time_usec();
                    this_timestamps.setpoint = current_messages_to_read.time_stamps.setpoint;
                }
                case MAVLINK_MSG_ID_COMMAND_LONG:
                {
                    mavlink_msg_command_long_decode(&message, &(current_messages_to_read.com));
                    current_messages_to_read.time_stamps.com = get_time_usec();
                    this_timestamps.com = current_messages_to_read.time_stamps.com;
                    
                    if (current_messages_to_read.com.command == MAV_CMD_NAV_GUIDED_ENABLE && current_messages_to_read.com.param1>0.5)
                    {
                        cout << "Got offboard command" <<endl;
                        offb_flag=1;
                    }
                    if (current_messages_to_read.com.command == MAV_CMD_NAV_GUIDED_ENABLE && current_messages_to_read.com.param1<0.5)
                    {
                        offb_flag=0;
                    }
                    // arm/disarm?
                    if (current_messages_to_read.com.command == MAV_CMD_COMPONENT_ARM_DISARM && current_messages_to_read.com.param1>0.5)
                        cout << "got arm/disarm command from GC..."<<endl;
                    {
                        
                        arm_flag=1;
                    }
                    if (current_messages_to_read.com.command == MAV_CMD_COMPONENT_ARM_DISARM && current_messages_to_read.com.param1<0.5)
                    {
                        arm_flag=0;
                    }
                }
                case MAVLINK_MSG_ID_ATT_POS_MOCAP:
                {
                    mavlink_msg_att_pos_mocap_decode(&message, &(current_messages_to_read.mocap));
                    current_messages_to_read.time_stamps.mocap = get_time_usec();
                    this_timestamps.mocap = current_messages_to_read.time_stamps.mocap;
                }
                    
                default:
                {
                    // printf("Warning, did not handle message id %i\n",message.msgid);
                    break;
                }
                    
                    
            } // end: switch msgid
            
        } // end: if read message
        
        // Check for receipt of all items
        /*
        received_all =
        this_timestamps.heartbeat                  &&
        this_timestamps.sys_status                 &&
        //				this_timestamps.battery_status             &&
        //				this_timestamps.radio_status               &&
        this_timestamps.local_position_ned         &&
        //				this_timestamps.global_position_int        &&
        //				this_timestamps.position_target_local_ned  &&
        this_timestamps.position_target_global_int &&
        this_timestamps.highres_imu                &&
        this_timestamps.attitude                   ;
        */
        
        // give the write thread time to use the port
        if ( writing_status > false )
            usleep(100); // look for components of batches at 10kHz
        
        counter++;
        
    } // end: while not received all
    return;
}


// ------------------------------------------------------------------------------
//   Write  Mavlink Message via UDP
// ------------------------------------------------------------------------------

int
Autopilot_UDP_Interface::
write_mavlink_message(mavlink_message_t &message)
{
    uint8_t buf[300];
    
    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);
    
    // Write buffer to UDP socket
    //_write_port(buf,len);
    //cout << "dest port: " <<destPort<<endl;
    udp_socket->sendTo(buf, len, destAddr, destPort);
    
    return len;
}


// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_UDP_Interface::
write_message(mavlink_message_t message)
{
    // do the write
    int len = write_mavlink_message(message);
    
    // book keep
    write_count++;
    
    // Done!
    return len;
}
/*
// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    
    // pull from position target
    mavlink_set_position_target_local_ned_t sp = current_setpoint;
    
    // double check some system parameters
    if ( not sp.time_boot_ms )
        sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
    sp.target_system    = system_id;
    sp.target_component = autopilot_id;
    
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
    
    
    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    
    // do the write
    int len = write_message(message);
    
    // check the write
    if ( not len > 0 )
        fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);
    
    return;
}
*/
/*
// ------------------------------------------------------------------------------
//   Write Messages to ground control station
// ------------------------------------------------------------------------------
*/
void
Autopilot_UDP_Interface::
write_to_GC()
{
    // TODO: prepare current_messages_to_write
    mavlink_message_t message;
    //cout << "I am trying to write to GC..........." << endl;
    // alwyas check if the meassages are valid before writing...
    //----write heartbeat
    if (current_messages_to_write.time_stamps.heartbeat)
    {
        current_messages_to_write.time_stamps.heartbeat=0;//reset timestamp untile we get a new one
        mavlink_msg_heartbeat_encode(current_messages_to_write.sysid, current_messages_to_write.compid, &message, &(current_messages_to_write.heartbeat));
        // do the write
        int len = write_message(message);
        
        // check the write
        if ( not len > 0 )
            fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    }
    
    //----write attitude
    if(current_messages_to_write.time_stamps.attitude)
    {
        //cout << "roll angle....."<<current_messages_to_write.attitude.roll<<endl;
        //cout << "writing attitude to GC....."<<endl;
        current_messages_to_write.time_stamps.attitude=0;//reset timestamp untile we get a new one
        mavlink_msg_attitude_encode(current_messages_to_write.sysid, current_messages_to_write.compid, &message, &(current_messages_to_write.attitude) );
        // do the write
        int len = write_message(message);
        
        // check the write
        if ( not len > 0 )
            fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    }
    
    //----write local ned position
    if (current_messages_to_write.time_stamps.local_position_ned)
    {
        current_messages_to_write.time_stamps.local_position_ned=0;//reset timestamp untile we get a new one
        mavlink_msg_local_position_ned_encode(current_messages_to_write.sysid, current_messages_to_write.compid, &message, &(current_messages_to_write.local_position_ned) );
        // do the write
        int len = write_message(message);
        
        // check the write
        if ( not len > 0 )
            fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    }
    
    //---- write system status (includes battery information)
    if (current_messages_to_write.time_stamps.sys_status)
    {
        current_messages_to_write.time_stamps.sys_status=0;//reset timestamp untile we get a new one
        mavlink_msg_sys_status_encode(current_messages_to_write.sysid, current_messages_to_write.compid, &message, &(current_messages_to_write.sys_status) );
        // do the write
        int len = write_message(message);
        
        // check the write
        if ( not len > 0 )
            fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
    }
    
    //----write others ??
    
    return;
}

/*
 **** there is no need for offboard toggling in this class, because it's designed for communication between 
 **** GCS and companion computer.
 **** Offboard toggling is done in the serial class, between the companion computer and the autopilot.
 **** If the toggling comes from GCS, then it will be passed to the serial interface class to be handled.
// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
enable_offboard_control()
{
    // Should only send this command once
    if ( control_status == false )
    {
        printf("ENABLE OFFBOARD MODE\n");
        
        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------
        
        // Sends the command to go off-board
        int success = toggle_offboard_control( true );
        
        // Check the command was written
        if ( success )
            control_status = true;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }
        
        printf("\n");
        
    } // end: if not offboard_status
    
}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
disable_offboard_control()
{
    
    // Should only send this command once
    if ( control_status == true )
    {
        printf("DISABLE OFFBOARD MODE\n");
        
        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------
        
        // Sends the command to stop off-board
        int success = toggle_offboard_control( false );
        
        // Check the command was written
        if ( success )
            control_status = false;
        else
        {
            fprintf(stderr,"Error: off-board mode not set, could not write message\n");
            //throw EXIT_FAILURE;
        }
        
        printf("\n");
        
    } // end: if offboard_status
    
}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_UDP_Interface::
toggle_offboard_control( bool flag )
{
    // Prepare command for off-board mode
    mavlink_command_long_t com;
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation     = true;
    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
    
    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
    
    // Send the message
    int len = serial_port->write_message(message);
    
    // Done!
    return len;
}
*/

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
start()
{
    int result;
    /*
    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------
    
    if ( not serial_port->status == 1 ) // SERIAL_PORT_OPEN
    {
        fprintf(stderr,"ERROR: serial port not open\n");
        throw 1;
    }
    */
    
    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------
    
    printf("START UDP READ THREAD \n");
    
    result = pthread_create( &read_tid, NULL, &start_Autopilot_UDP_Interface_read_thread, this );
    if ( result ) throw result;
    
    // now we're reading messages
    printf("\n");
    
    
    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------
    
    printf("CHECK FOR GCS MESSAGES\n");
    
    while ( not current_messages_to_read.sysid )
    {
        if ( time_to_exit )
            return;
        usleep(500000); // check at 2Hz
    }
    
    cout << "Found GCS with ID: " << current_messages_to_read.sysid << endl;
    
    // now we know autopilot is sending messages
    printf("\n");
    
    
    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------
    
    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.
    
    // System ID
    if ( not system_id )
    {
        system_id = current_messages_to_read.sysid;
        printf("GOT GCS SYSTEM ID: %i\n", system_id );
    }
    
    /*
    // Component ID
    if ( not autopilot_id )
    {
        autopilot_id = current_messages.compid;
        printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
        printf("\n");
    }
    */
    
    // --------------------------------------------------------------------------
    //   GET INITIAL POSITION
    // --------------------------------------------------------------------------
    
    // Wait for initial position ned
    /*
    while ( not ( current_messages_to_write.time_stamps.local_position_ned &&
                 current_messages_to_write.time_stamps.attitude            )  )
    {
        if ( time_to_exit )
            return;
        usleep(500000); // 2Hz
    }
    */
    /*
    
    // copy initial position ned
    Mavlink_Messages local_data = current_messages;
    initial_position.x        = local_data.local_position_ned.x;
    initial_position.y        = local_data.local_position_ned.y;
    initial_position.z        = local_data.local_position_ned.z;
    initial_position.vx       = local_data.local_position_ned.vx;
    initial_position.vy       = local_data.local_position_ned.vy;
    initial_position.vz       = local_data.local_position_ned.vz;
    initial_position.yaw      = local_data.attitude.yaw;
    initial_position.yaw_rate = local_data.attitude.yawspeed;
    
    
    printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
    printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    printf("\n");
    */
    // we need this before starting the write thread
    
    cout << "Received initial position and attitude messages. I will start writing to the GCS"<< endl;
    
    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    printf("START UDP WRITE THREAD \n");
    
    result = pthread_create( &write_tid, NULL, &start_Autopilot_UDP_Interface_write_thread, this );
    if ( result ) throw result;
    
    // wait for it to be started
    while ( not writing_status )
        usleep(100000); // 10Hz
    
    // now we're streaming setpoint commands
    printf("\n");
    
    
    // Done!
    return;
    
}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    printf("CLOSE UDP THREADS\n");
    
    // signal exit
    time_to_exit = true;
    
    // wait for exit
    pthread_join(read_tid ,NULL);
    pthread_join(write_tid,NULL);
    
    // now the read and write threads are closed
    printf("\n");
    
    
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
start_read_thread()
{
    
    if ( reading_status != 0 )
    {
        fprintf(stderr,"read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }
    
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
start_write_thread(void)
{
    if ( not writing_status == false )
    {
        fprintf(stderr,"write thread already running\n");
        return;
    }
    
    else
    {
        write_thread();
        return;
    }
    
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
handle_quit( int sig )
{
    
    //disable_offboard_control();// this is done in the serial interface
    
    try {
        stop();
        
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop autopilot interface\n");
    }
    
}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_UDP_Interface::
read_thread()
{
    reading_status = true;
    
    while ( not time_to_exit )
    {
        read_messages();
        usleep(0.02*1000000); // Read batches at 50Hz
    }
    
    reading_status = false;
    
    return;
}

/*
// ------------------------------------------------------------------------------
*  Write Thread
*  This is supposed to write messages to the ground station, logically not setpoints!
* Instead, it should write messages that are read from the serail port (from autopilot)
* Examples:
* Attitude, heartbeat, global/local positions, IMU, battery status, system status, mode...etc
// ------------------------------------------------------------------------------
 */
void
Autopilot_UDP_Interface::
write_thread(void)
{
    // signal startup
    writing_status = 2;
    /*
    // prepare an initial setpoint, just stay put
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx       = 0.0;
    sp.vy       = 0.0;
    sp.vz       = 0.0;
    sp.yaw_rate = 0.0;
    
    // set position target
    current_setpoint = sp;
    */
    // write a message and signal writing
    //write_setpoint();
    writing_status = true;
    
    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while ( not time_to_exit )
    {
        usleep(0.02*1000000);   // Stream at 50Hz
        //write_setpoint();
        write_to_GC();
    }
    
    // signal end
    writing_status = false;
    
    return;
    
}

// End Autopilot_UDP_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_Autopilot_UDP_Interface_read_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_UDP_Interface *autopilot_interface = (Autopilot_UDP_Interface *)args;
    
    // run the object's read thread
    autopilot_interface->start_read_thread();
    
    // done!
    return NULL;
}

void*
start_Autopilot_UDP_Interface_write_thread(void *args)
{
    // takes an autopilot object argument
    Autopilot_UDP_Interface *autopilot_interface = (Autopilot_UDP_Interface *)args;
    
    // run the object's read thread
    autopilot_interface->start_write_thread();
    
    // done!
    return NULL;
}

