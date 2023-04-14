#ifndef _LED_INT_H_
#define _LED_INT_H_

#include "Arduino.h"
#include <vector>
#include <iostream>
#include <string>
#include <mcp41010.h>
#include <PCF8575.h>

// #include <Wire>

// MAX1132 16bit ADC
// const int ADC_CS_PIN = 28;
// const int ADC_RST_PIN = 29;
// const int ADC_SSTRB_PIN = 30;

// 11011000
// const byte CALBYTE = 0xC8;// calibration control byte
// const byte READBYTE = 0xD8; // acquire unipolar in long acquistion mode (~150 ksps with ext clock)

// const int ADC_DELAY = 0; // number of microseconds to wait before pulling data from ADC. 

/// @brief Class to hold LED implementation details
/// @details This class is used to hold the implementation details of the LED class. It can turn on the led and turn it off.
/// @param led_name The name of the LED. Usuually wavelength in nm.
/// @param led_pin The microcontroller pin that the LED gate is connected to. 
/// @param max_source_current The maximum current that the LED can source, in mA.
/// @param max_constant_current The maximum current that the LED can source in constant current mode, in mA.
/// @param max_surge_current The maximum current that the LED can source in pulse mode, in mA.
/// @param mcp41010 The MCP41010 digital potentiometer used to set the LED current.
/// @param pcf8575 The PCF8575 I/O expander used to control the LED gate.
class LED {
public:
    LED(std::string led_name, uint8_t led_pin, uint16_t max_source_current, uint16_t max_constant_current,
        uint16_t max_surge_current, MCP41010& mcp41010, uint8_t led_intensity=0)
        : led_name_(led_name), led_pin_(led_pin), max_source_current_(max_source_current),
          max_constant_current_(max_constant_current), max_surge_current_(max_surge_current),
          mcp41010_(mcp41010), m_current_intensity(led_intensity) {pinMode(led_pin_, OUTPUT);}
    void toggle(bool state);
    void calibrate_intensity(uint8_t value);
    void set_intensity(uint8_t value, const uint8_t mode);
    uint8_t get_resistance_value(uint8_t value);
    const char* get_name();
    uint8_t get_intensity(){return m_current_intensity;};
    
    private:
    std::string led_name_;
    uint8_t led_pin_;
    uint8_t m_current_intensity; 
    uint16_t max_source_current_;
    uint16_t max_constant_current_;
    uint16_t max_surge_current_;
    MCP41010& mcp41010_;
    
    
};

#endif