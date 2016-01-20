//
//  mavlink_bridge.cpp
//  
//
//  Created by Mohamed A AbdKader on 1/19/16.
//
//

#include "mavlink_bridge.h"


int main(int argc, char *argv[])
{
    if (argc <6){
        cerr << "Usage: " << argv[0]
        << "  <local port> <remote IP> <remote port> <serial_port> <baudrate>" <<endl;
        exit(1);
    }
    
    // the argument have to obey the following order
    unsigned short localPort= atoi(argv[1]); // 1st arg: local port
    string remoteAddrs=argv[2];             // 2nd arg: remote address (e.g. 127.0.0.1)
    unsigned short remotePort= atoi(argv[3]); // 3rd arg: remote port
    char *uart_name = (char*)argv[4];       // 4th arg: serial port name
    int baudrate = atoi(argv[5]);           // 5th arg: baudrate
    
    
    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);
    
    /*
     * start the UDP socket
     */
    UDPSocket sock(localPort);
    
    /*
     * Instantiate an autopilot interface object (for serial)
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
     *
     */
    Autopilot_Interface autopilot_serial_interface(&serial_port);
    
    /*
     * Instantiate an autopilot interface object (for udp)
     
     */
    Autopilot_UDP_Interface autopilot_udp_interface(&sock);
    
    autopilot_udp_interface.destAddr=remoteAddrs;
    autopilot_udp_interface.destPort= remotePort;
    
    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = &serial_port;
    autopilot_interface_quit = &autopilot_serial_interface;
    autopilot_udp_interface_quit = &autopilot_udp_interface;
    signal(SIGINT,quit_handler);
    
    
    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    autopilot_serial_interface.start();
    autopilot_udp_interface.start();
    
    /*
     * here we should get the current_messages_to_read from serial
     * and assign it to current_messages_to_write in udp
     
     * get the current_messages_to_read from udp
     * and assign it to current_messages_to_write in serial
    */
    
    for (;;)// run forever
    {
        //cout << "i am here.." << endl;
        autopilot_udp_interface.current_messages_to_write=autopilot_serial_interface.current_messages_to_read;
        usleep(1000); //give it some time to sink
        autopilot_udp_interface.current_messages_to_write.reset_timestamps();
        autopilot_serial_interface.current_messages_to_write=autopilot_udp_interface.current_messages_to_read;
        usleep(1000); // to sink
        autopilot_serial_interface.current_messages_to_write.reset_timestamps();
        cout << "moc time........." <<autopilot_serial_interface.current_messages_to_write.time_stamps.moc<<endl;
        usleep(0.02*1000000); // 50Hz
    }
    
    
    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_serial_interface.stop();
    autopilot_udp_interface.stop();
    serial_port.stop();
    return 0;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");
    
    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error){}
    
    // autopilot udp interface
    try {
        autopilot_udp_interface_quit->handle_quit(sig);
    }
    catch (int error){}
    
    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}
    
    // end program here
    exit(0);
    
}