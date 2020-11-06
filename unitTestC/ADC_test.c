#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <pigpio.h>

#include "ADC.h"


bool volatile run = true;

void stop(int signum){
   run = false;
    printf("stop\n");
}

int main(int argc, char* argv[]){
    if (gpioInitialise()<0){
        perror("pigpio init failed\n");
    }
    gpioSetSignalFunc(SIGINT, stop);
    if (initADC()<0) {
        perror("init failed\n");
    }
    int value;
    while(run && (value = readRatio1())>0){
        printf("\r\r\r\rvalue %04d",value);
    }
    closeADC();
    gpioTerminate();
    printf("end\n");
    return 0;
}

// gcc -Wall ADC_test.c ADC.c -o ADC_test -lpigpio -lrt -pthread

