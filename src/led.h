#ifndef _LED_INT_H_
#define _LED_INT_H_

#include "Arduino.h"
#include <vector>
#include <iostream>
#include <string>

// #include <Wire>

// MAX1132 16bit ADC
// const int ADC_CS_PIN = 28;
// const int ADC_RST_PIN = 29;
// const int ADC_SSTRB_PIN = 30;

// 11011000
// const byte CALBYTE = 0xC8;// calibration control byte
// const byte READBYTE = 0xD8; // acquire unipolar in long acquistion mode (~150 ksps with ext clock)

// const int ADC_DELAY = 0; // number of microseconds to wait before pulling data from ADC. 


class MCP41010
{  

public:
    MCP41010(uint8_t cs_pin) {m_cs_pin = cs_pin; }
    uint8_t set_value(uint8_t value);    
    uint8_t get_value();
private:
    uint8_t m_cs_pin;
    uint8_t m_command_byte = 0b00010001;
    uint8_t m_value = 0;
};

class LED
{  
public:
    LED(std::string led_name, uint8_t led_pin, uint8_t mcp_cs_pin, uint8_t shunt_pin, uint16_t max_source_current, uint16_t max_constant_current, uint16_t max_surge_current);
    int set_intensity(uint8_t value, char mode);   
    void on();
    void off();
private:
    uint8_t m_led_pin;
    uint8_t m_cs_pin;
    uint8_t m_shunt_pin;
    int m_max_source;
    int m_max_constant;
    int m_max_surge;
    MCP41010 mcp41010 = MCP41010(m_cs_pin);
    uint8_t get_resistance_value(uint8_t value);
    void calibrate_intensity(uint8_t value);
};

class LED_ARRAY
{
public:
    void add(LED led);
    void change_led_state(uint8_t led_num, bool state);
    void set_meas_led(uint8_t led_num);
    void set_act_led(uint8_t led_num);
    void set_intensity(uint8_t led_num, uint8_t value);
private:
    std::vector<LED> m_leds;
    uint8_t m_meas_led;
    uint8_t m_act_led;
};






#endif