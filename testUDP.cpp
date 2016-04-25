//
//  testUDP.cpp
//
//
//  Created by Mohamed A AbdKader on 4/25/16.
//
//

#include "testUDP.h"


int main(int argc, char *argv[])
{
    if (argc <4){
        cerr << "Usage: " << argv[0]
        << "  <local port> <remote IP> <remote port>" <<endl;
        exit(1);
    }
    
    // the argument have to obey the following order
    unsigned short localPort= atoi(argv[1]); // 1st arg: local udp port
    string remoteAddrs=argv[2];             // 2nd arg: remote ip address (e.g. 127.0.0.1)
    unsigned short remotePort= atoi(argv[3]); // 3rd arg: remote port
    
    
    /*
     * start the UDP socket
     */
    UDPSocket sock(localPort);
    
    
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
    autopilot_udp_interface_quit = &autopilot_udp_interface;
    signal(SIGINT,quit_handler);
    
    
    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    autopilot_udp_interface.start();
    
    /*
     * here we should get the current_messages_to_read from serial
     * and assign it to current_messages_to_write in udp
     
     * get the current_messages_to_read from udp
     * and assign it to current_messages_to_write in serial
     */
    const float dt=0.02;
    const int t = 1; // time in seconds
    int T = (int) (t/dt); // number of iterations needed to complete (t) seconds with delay (dt)
    
    for (int i=0;i<T;i++)
    {
        //cout << "i am here.." << endl;
        cout << autopilot_udp_interface.current_messages_to_read.time_stamps.attitude << endl;
        usleep(dt*1000000); // (1/dt) Hz
        
        //autopilot_udp_interface.current_messages_to_read.reset_timestamps();
    }
    
    
    /*
     * Now that we are done we can stop the threads and close the port
     */
    cout << "CLOSING PORT..." << endl;
    autopilot_udp_interface.stop();
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
    
    // autopilot udp interface
    try {
        autopilot_udp_interface_quit->handle_quit(sig);
    }
    catch (int error){}
    
    
    // end program here
    exit(0);
    
}