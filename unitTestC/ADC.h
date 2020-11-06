#ifndef ADC_H
#define ADC_H
#include <pigpio.h>

#define BUS 1 // bus for raspberry 2 and 3

#define READRAWGENH(X) int readRaw ## X(void);
#define READVOLTAGEGENH(X) int readVoltage ## X(void);
#define READRATIOGENH(X) int readRatio ## X(void);


int initADC();

READRAWGENH(0)
READRAWGENH(1)
READRAWGENH(2)
READRAWGENH(3)
READRAWGENH(4)
READRAWGENH(5)
READRAWGENH(6)
READRAWGENH(7)

READVOLTAGEGENH(0)
READVOLTAGEGENH(1)
READVOLTAGEGENH(2)
READVOLTAGEGENH(3)
READVOLTAGEGENH(4)
READVOLTAGEGENH(5)
READVOLTAGEGENH(6)
READVOLTAGEGENH(7)


READRATIOGENH(0)
READRATIOGENH(1)
READRATIOGENH(2)
READRATIOGENH(3)
READRATIOGENH(4)
READRATIOGENH(5)
READRATIOGENH(6)
READRATIOGENH(7)


int closeADC();

#undef READRAWGENH
#undef READVOLTAGEGENH
#undef READRATIOGENH

#endif