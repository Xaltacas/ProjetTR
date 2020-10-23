#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>


#define EVER (;;)

#define GROVE 0x04
#define OFFSET 0x30

int pin = 1;

int main(){

	int fd;

	fd = wiringPiI2CSetup(GROVE);

	int ret;

	for EVER{

		//ret = wiringPiI2CRead(fd);
		ret = wiringPiI2CReadReg16(fd,OFFSET + pin);

		printf("lu : %d \n",ret);
		delay(500);		

	}



}
