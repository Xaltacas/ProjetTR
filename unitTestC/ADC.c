
#include "ADC.h"

int handle=-1;

#define READRAWGEN(X) int readRaw         ## X (void) {return i2cReadWordData(handle, 0x1 ## X );}

#define READVOLTAGEGEN(X) int readVoltage ## X (void) {return i2cReadWordData(handle, 0x2 ## X );}

#define READRATIOGEN(X) int readRatio     ## X (void) {return i2cReadWordData(handle, 0x3 ## X );}

int initADC(){
    return (handle = i2cOpen(BUS,0x04,0));
}

READRAWGEN(0)
READRAWGEN(1)
READRAWGEN(2)
READRAWGEN(3)
READRAWGEN(4)
READRAWGEN(5)
READRAWGEN(6)
READRAWGEN(7)

READVOLTAGEGEN(0)
READVOLTAGEGEN(1)
READVOLTAGEGEN(2)
READVOLTAGEGEN(3)
READVOLTAGEGEN(4)
READVOLTAGEGEN(5)
READVOLTAGEGEN(6)
READVOLTAGEGEN(7)


READRATIOGEN(0)
READRATIOGEN(1)
READRATIOGEN(2)
READRATIOGEN(3)
READRATIOGEN(4)
READRATIOGEN(5)
READRATIOGEN(6)
READRATIOGEN(7)

int closeADC(){
    return i2cClose(handle);
}
