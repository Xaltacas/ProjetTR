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

	int pos = 0;
	for EVER{
		scanf("%d",&pos);
		softPwmWrite(PIN,pos);

	}
	
	return 0;

}
