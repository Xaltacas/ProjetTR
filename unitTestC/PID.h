#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define MAJOR_NUM 100
#define IOCTL_GET_COUNT _IOR(MAJOR_NUM, 0, long *)

#define MDRIVER 0x14
#define STOP 0x00
#define BRAKE 0x01
#define CW 0x02
#define CCW 0x03

int target=0, integral=0, e_prev=0, Kp=16, Ki=0, Kd=4;
int wph, fd, directions[2]={CW, CCW};
long offset;
double last_time;

static double get_seconds(void) { // https://stackoverflow.com/a/36095407
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec  + ts.tv_nsec*0.000000001;
}

/*return the offset of the motor*/
int initPID(){
  long pos, last, limits[2];
  if((fd = open("/dev/motor", O_RDWR))==-1 ){ // open the device created by the module
      perror("open failed\n");
      return -1;
  }
  if ((wph = wiringPiI2CSetup(MDRIVER))==-1){ // init I2C
    perror("error init i2c");
    return -1;
  }
  for (int i=0; i<2; i++) { // turn right then left
    if (wiringPiI2CWriteReg16(wph,directions[i],0xEE00)==-1){ // turn
      perror("error write i2c");
      return -1;
    }
    last_time=get_seconds();
    while (last!=pos || get_seconds()-last_time<0.1){
      last = pos;
      if(ioctl(fd,IOCTL_GET_COUNT, &pos)==-1){
          perror("read failed\n");
          return -1;
      }
      usleep(1000); // let the motor rotate otherwise it reads too fast
    }
    limits[i]=pos;
  }
  offset = (limits[0]+limits[1])/2;
  printf("d=%ld g=%ld offset=%ld\n", limits[0],limits[1],offset);
  last_time=get_seconds();
  return 1;
}



int stepPID(int new_target){
  target = new_target;
  // get the current position
  int e,U,direction,sign, speed;
  long pos;
  if(ioctl(fd,IOCTL_GET_COUNT, &pos)==-1){
      perror("read failed\n");
      return -1;
  }
  // PID
  double now = get_seconds(), Te = now - last_time;
  e = target - (pos-offset);
  integral += e;
  U = Kp*(e + Ki*Te*integral + Kd*(e - e_prev)/Te);
  e_prev = e;
  
  // send the command
  sign = (U>0) - (U<0);
  if(U>255) U=255; // clip
  else if (U<-255) U=-255;
  direction = directions[(U>0)];
  speed = (sign*U) << 8; // it has to be 0x(0-255)00
  if(wiringPiI2CWriteReg16(wph,direction,speed)==-1){
    perror("error write i2c");
    return -1;
  }
  return 1;
}



int deinitPID(){
  return close(fd);
}

#endif
