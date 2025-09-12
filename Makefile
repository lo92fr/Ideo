ideoGateway: main.cpp cc1100/cc1100_raspi.cpp cc1100/cc1100_raspi.h
	g++ main.cpp cc1100/cc1100_raspi.cpp  -I cc1100 -l wiringPi -o ideoGateway -fpermissive -l mosquitto

run:
	./ideoGateway
