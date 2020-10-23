CC = g++
CFLAGS = -g -Wall
OPENCV = pkg-config opencv --cflags --libs
CVFLAGS = $(shell pkg-config opencv --cflags --libs)
#LIBS = -lopencv_video #-nostartfiles

all : cam io i2c i2cmotor pSensor pigpiotest

cam : cam.o
	$(CC) $(CFLAGS) $(CVFLAGS) $< -o $@  $(LIBS)
	
cam.o : cam.cpp
	$(CC) $(CFLAGS) -c $< -o $@

io : io.c
	gcc -Wall -o $@ $< -lwiringPi -lpthread

pSensor: pSensor.cpp
	g++ -Wall -lgrovepicpp $< -o $@

i2c: i2c.c
	gcc -Wall -o $@ $< -lwiringPi -lpthread

i2cmotor: i2cmotor.c
	gcc -Wall -o $@ $< -lwiringPi -lpthread

pigpiotest: pigpiotest.c
	gcc -Wall -o $@ $< -lpigpio -lrt -lpthread

clear:
	rm *.o cam io i2c i2cmotor pSensor pigpiotest

