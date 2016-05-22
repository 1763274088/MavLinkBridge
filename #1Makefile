all: mavlink_gateway

mavlink_gateway: mavlink_gateway.cpp serial_port.h PracticalSocket.h
	g++ -w -I include/mavlink  mavlink_gateway.cpp serial_port.cpp PracticalSocket.cpp -o mavlink_gateway

clean:
	 rm -rf *o mavlink_gateway