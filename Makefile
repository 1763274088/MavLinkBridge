all: mavlink_bridge

mavlink_bridge: mavlink_bridge.cpp serial_port.h PracticalSocket.h autopilot_interface.h autopilot_udp_interface.h
	g++ -w -I include/mavlink  mavlink_bridge.cpp serial_port.cpp PracticalSocket.cpp autopilot_interface.cpp autopilot_udp_interface.cpp  -o mavlink_bridge -lpthread

clean:
	 rm -rf *o mavlink_bridge