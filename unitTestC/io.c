#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

#define PIN 0

#define EVER (;;)

int main(){
	wiringPiSetup();
	pinMode(PIN, OUTPUT);
	digitalWrite(PIN, LOW);

	if(softPwmCreate(PIN,0,100)){
		printf("ca marche pas\n");
		return 0;
	}

	
	for EVER{
		softPwmWrite(PIN,10);
		printf("pin 17 pwm 10 \n");
 		delay(1500);
		softPwmWrite(PIN,20);
		printf("pin 17 pwm 20 \n");
 		delay(1500);
	}
	
	return 0;

}
