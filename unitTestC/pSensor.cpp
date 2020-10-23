#include "grovepi.h"

#define EVER (;;)

using namespace GrovePi;

int main(){

	int pin = 1;
	int incoming;

	try{
		initGrovePi();
		setMaxI2CRetries(20);
		pinMode(pin, INPUT);

		for EVER{
			incoming = analogRead(pin);
			printf("pin %d reads %d \n",pin,incoming);
			delay(10);
		}
	}
	catch (I2CError &error){
		printf(error.detail());
		return -1;

	}
	return 0;
}
