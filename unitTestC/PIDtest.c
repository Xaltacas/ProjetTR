#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>

#include <sys/ioctl.h>

#include "PID.h"
#include <wiringPiI2C.h>
#include <wiringPi.h>

volatile sig_atomic_t ok = 1;

void sighandler(int signum) {
    ok = 0;
    printf("stop\n");
}

int main(int agc, char* argv[]){
    printf("start\n");
    struct sigaction act; // Signal
    act.sa_handler = sighandler;
    sigaction(SIGINT, &act, NULL);
    
  if (initPID()==-1){
    perror("PID init failed");
    return 1;
  }
    int target=100;
  time_t last=time(NULL), now;
  while(ok){
    now = time(NULL);
    if (stepPID(target)==-1){
      printf("ERROR step");
      break;
    }
    else if (now - last >0) {
      target = -target;
      last = now;
    }
    usleep(1);
  }
  deinitPID();
    printf("end\n");
    return 0;
}





