all: testUDP

testUDP: testUDP.cpp PracticalSocket.h autopilot_udp_interface.h
	g++ -w -I include/mavlink  testUDP.cpp PracticalSocket.cpp autopilot_udp_interface.cpp  -o testUDP -lpthread

clean:
	 rm -rf *o testUDP