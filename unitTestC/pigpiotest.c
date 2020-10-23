#include <pigpio.h>
#include <stdio.h>


#define EVER (;;)

#define MDRIVER 0x14
#define STOP 0x00 
#define BRAKE 0x01
#define CW 0x02 
#define CCW 0x03

int main(){

	int fd;
	gpioInitialise();

	fd = i2cOpen(1,MDRIVER,0);

	for EVER{
            char com;
		scanf("%c",&com);

		switch(com){
		case 'd':
			i2cWriteWordData(fd,CW,0xFF00);
			break;
		case 'g':
			i2cWriteWordData(fd,CCW,0xFF00);
			break;
		case 'b':
			i2cWriteByteData(fd,BRAKE,0x00);
			break;
		case 's':
			i2cWriteByteData(fd,STOP,0x00);
			break;
		default:
			printf("d -> CW \ng -> CCW \nb -> BRAKE \ns -> STOP\n");

		}
	}
}













