#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>


#define EVER (;;)

#define MDRIVER 0x14
#define STOP 0x00 
#define BRAKE 0x01
#define CW 0x02 
#define CCW 0x03

int main(){

	int fd;

	fd = wiringPiI2CSetup(MDRIVER);

	for EVER{
            char com;
		scanf("%c",&com);

		switch(com){
		case 'd':
			wiringPiI2CWriteReg16(fd,CW,0xFF00);
			//printf("CW\r");
			break;
		case 'g':
			wiringPiI2CWriteReg16(fd,CCW,0xFF00);
			//printf("CCW\r");
			break;
		case 'b':
			wiringPiI2CWriteReg8(fd,BRAKE,0x00);
			//printf("BRAKE\r");
			break;
		case 's':
			wiringPiI2CWriteReg8(fd,STOP,0x00);
			//printf("STOP\r");
			break;
		default:
			printf("d -> CW \ng -> CCW \nb -> BRAKE \ns -> STOP\n");

		}
	}
}
//ret = wiringPiI2CRead(fd);
//ret = wiringPiI2CWriteReg16(fd,CW,0xFF00);
//delay(500);		
//ret = wiringPiI2CWriteReg16(fd,CW,0x0000);
//delay(500);		
