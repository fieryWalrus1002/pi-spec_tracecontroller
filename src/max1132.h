#ifndef _MAX1132_H_
#define _MAX1132_H_

#include "Arduino.h"
#include <Wire.h>

#define EXT 0 // External VREF
#define INT 1 // Internal VREF

// MAX1132 16bit ADC
const int ADC_CS_PIN = 28;
const int ADC_RST_PIN = 29;
const int ADC_SSTRB_PIN = 30;
// 11011000
const byte CALBYTE = 0xC8;// calibration control byte
const byte READBYTE = 0xD8; // acquire unipolar in long acquistion mode (~150 ksps with ext clock)

const int ADC_DELAY = 0; // number of microseconds to wait before pulling data from ADC. 


class MAX1132
{  

public:    
    void init(); // activates spi1 and calibrates the adc
    uint16_t read();    
};


#endif