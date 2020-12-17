#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <softPwm.h>

#include "driver_util.h"

#define EVER (;;)

#define MDRIVER 0x14
#define STOP 0x00 
#define BRAKE 0x01
#define CW 0x02 
#define CCW 0x03

#define SPEED 0x4F00

#define GROVE 0x04
#define OFFSET 0x30

#define SERVOPIN 0


#define SIZE 10
#define UPDATERATE 0.5


int pin = 1;

static pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;

static int driver;
static int grove;



static int run = 1;

static int values[SIZE];

int map(int x, int in_min, int in_max, int out_min, int out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int argmax(int* values, int size){
	int res = 0;
	int val = values[0];
	for(int i =1; i< size; i++){
		if(values[i] > val){
			val = values[i];
			res = i;
		}
	}
	return res;
}


void quitHandler(int s){
	
	wiringPiI2CWriteReg8(driver,BRAKE,0x00);
	printf("quit\n");
	exit(s);

}



void *telemetre(void *arg) {
	int sens = -1;
	int dist;
	int enc;
	long count=0;


	if((enc = open("/dev/motor", O_RDWR))==-1){
		perror("open failed\n");
		return 0;
	}
	printf("start loop\n");

	wiringPiI2CWriteReg16(driver,CW,SPEED);


	while(run) {
		if(ioctl(enc,IOCTL_GET_COUNT, &count)==-1){
			perror("read failed\n");
			break;
		}
		else{
			dist = wiringPiI2CReadReg16(grove,OFFSET+pin);
			int index = map(count, -120, 120, 0, SIZE);
			pthread_mutex_lock(&m);
			values[index] =+ UPDATERATE*(dist-values[index]);
			
			printf("[%03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d %03.d ]\n",values[ 0 ], values[ 1 ], values[ 2 ], values[ 3 ], values[ 4 ], values[ 5 ], values[ 6 ], values[ 7 ], values[ 8 ], values[ 9 ], values[ 10 ], values[ 11 ], values[ 12 ], values[ 13 ], values[ 14 ], values[ 15 ], values[ 16 ], values[ 17 ], values[ 18 ], values[ 19 ], values[ 20 ], values[ 21 ], values[ 22 ], values[ 23 ], values[ 24 ], values[ 25 ], values[ 26 ], values[ 27 ], values[ 28 ], values[ 29 ]);
			pthread_mutex_unlock(&m);


		}
		if(count < -100 && sens == -1){
			sens = 1;
			wiringPiI2CWriteReg16(driver,CCW,SPEED);
		}
		else if( count > 100 && sens == 1){
			sens = -1;
			wiringPiI2CWriteReg16(driver,CW,SPEED);
		}

	}
	wiringPiI2CWriteReg8(driver,BRAKE,0x00);
	return 0;
}

int main(int argc, char **argv) {
	driver = wiringPiI2CSetup(MDRIVER);
	grove = wiringPiI2CSetup(GROVE);

	wiringPiSetup();
	pinMode(SERVOPIN,OUTPUT);
	digitalWrite(SERVOPIN,LOW);
	
	if(softPwmCreate(SERVOPIN,0,100)){
		printf("error creating softpwm\n");
		return 0;

	}

	signal(SIGINT,quitHandler);

	for(int i = 0; i< SIZE ; i++)
		values[i] =-1;

	if(pthread_mutex_init(&m,NULL)!=0){
		printf("error initializing the mutex \n");
		return 0;
	}


	
	pthread_t id;
	pthread_create(&id, NULL, telemetre, NULL);

	int pos = 0;
	int pwm = 13;
	for EVER{
		usleep(300);
		pthread_mutex_lock(&m);
		pos = argmax(values,SIZE);
		pthread_mutex_unlock(&m);
		int desired = 18 - map(pos,0,SIZE,0,10);
		if(desired < pwm) pwm--;
		else if(desired > pwm) pwm++;
		//printf("pos: %d  pwm: %d  \n",pos,pwm);

		softPwmWrite(SERVOPIN,pwm);
	}

	pthread_join(id, NULL);


	return 0;
}
