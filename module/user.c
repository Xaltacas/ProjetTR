#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>

#include <sys/ioctl.h>

#include "driver_util.h"

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
    
    int fd;
    long count=0;
    if((fd = open("/dev/motor", O_RDWR))==-1){
        perror("open failed\n");
        return 1;
    }
    printf("start loop\n");
    while(ok){
        //if(read(fd,count,4)==-1){
        if(ioctl(fd,IOCTL_GET_COUNT, &count)==-1){
            perror("read failed\n");
            break;
        }
        else
            //printf("\33[2K\rcount : %d", count);
            printf("\r\r\r\r\r\rcount : %06d", count);
            //printf("count : %d\n", count);
    }
    close(fd);
    printf("end\n");
    return 0;
}





