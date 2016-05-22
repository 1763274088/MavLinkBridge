//
//  mavlink_gateway.cpp
//
//
//  Created by Mohamed A AbdKader on 5/21/16.
//
//

#include "mavlink_gateway.h"


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

    try{
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


    // signal handler
    serial_port_quit         = &serial_port;
    signal(SIGINT,quit_handler);


    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();


    mavlink_message_t message;
    mavlink_status_t status;
    
    int result;
    bool success;
    
    const int inBuffLen=1024*2; // maximum input buffer length
    
    // define I/O buffers
    uint8_t buf[300];
    uint8_t inBuff[1024]; //
        
    uint8_t foundmsg = 0;

    while(1)// run forever
    {
        // 1---------------read serial port, forward to udp port
        
        success = serial_port.read_message(message);
        if (success) {
            
            
            // Translate message to buffer
            unsigned len = mavlink_msg_to_send_buffer(buf, &message);
            
            // Write buffer to UDP socket
            //_write_port(buf,len);
            //cout << "dest port: " <<destPort<<endl;
            sock.sendTo(buf, len, remoteAddrs, remotePort);
        }
        
        // 2----------- read udp port, forward to serial port
        try {
            result=0;
            result = sock.recv(inBuff, inBuffLen);// is it '->' or just '.'
        }catch(exception& e)
        {
            cout << "udp error: "<<e.what() << endl;
        }
        if (result > 0)
        {
            cout << "got udp, forwarding to serial.."<<endl;
            for (int i=0; i<result; i++){
                // the parsing
                foundmsg = mavlink_parse_char(MAVLINK_COMM_1, inBuff[i], &message, &status);

                if (foundmsg>0)
                {
                    // maybe we should assign mavlink messages here
                    serial_port.write_message(message);
                    break;
                }
            }
        }




       // usleep(0.01*1000000); // 50Hz
    }


    /*
     * Now that we are done we can close the port
     */
    serial_port.stop();

    }catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
    }
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


    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}
    // end program here
    exit(0);

}
