#ifndef _MAX1132_H_
#define _MAX1132_H_

#include "Arduino.h"
#include <SPI.h>

#define EXT 0 // External VREF
#define INT 1 // Internal VREF

//forward declaration for PCF8575 gpio expander class
class PCF8575;

// 11011000

const int ADC_DELAY = 0; // number of microseconds to wait before pulling data from ADC. 


class MAX1132
{  

public:
    MAX1132(uint8_t max_aq, uint8_t cs_pin, uint8_t rst_pin, uint8_t sstrb_pin); // activates spi1 and calibrates the adc
    // void init(uint8_t max_aq, uint8_t cs_pin, uint8_t rst_pin, uint8_t sstrb_pin); // activates spi1 and calibrates the adc
    uint16_t read();
    uint8_t m_preaq;
    uint8_t m_aq;
    uint8_t m_cs_pin;
    uint8_t m_rst_pin;
    uint8_t m_sstrb_pin;
private:
    void set_aquisition_points(uint8_t max_aq);
    void init_pins();
    void calibrate();
    
    int m_max_aq;
    const byte m_calbyte = 0xC8;// calibration control byte
    const byte m_readbyte = 0xD8; // acquire unipolar in long acquistion mode (~150 ksps with ext clock) 216
    const byte m_transfer_byte = 0x00; // dummy byte to send to adc to get data
    SPISettings adcSettings = SPISettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz clock, MSB first, SPI mode 0


};


#endif