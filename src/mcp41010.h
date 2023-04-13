#ifndef _MCP41010_H
#define _MCP41010_H
#include <Arduino.h>
#include <SPI.h>
#include <PCF8575.h>

///

//* MCP41010 digital potentiometer
// 8-bit digital potentiometer
// 0-255 resistance values
// 0x11 command byte
// 0x00-0xFF data byte
// CS low to high to execute command


/// @brief Class to interface with MCP41010 digital potentiometer
/// @details Uses SPI1 to interface with the MCP41010 digital potentiometer
/// 8-bit digital potentiometer
/// 0-255 resistance values, from 0-10k ohms
/// 00010001 command byte writes data and executes command on P0.
class MCP41010
{
public:
    MCP41010(const uint8_t cs_pin, PCF8575& pcf8575) : m_cs_pin(cs_pin), m_pcf8575(pcf8575) {};
    uint8_t set_value(uint8_t value);
    uint8_t get_value();
    void begin();
private:    
    uint8_t m_cs_pin;
    uint8_t m_value;
    void csToggle(bool state);
    static const uint8_t m_command_byte{0x11};
    PCF8575 &m_pcf8575;
};


#endif


// convert 00010001 to hex
// 0x11