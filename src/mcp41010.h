/**
 * @file mcp41010.h
 * @brief Contains definitions for functions to control a digital potentiometer.
 *
 * @note
 * Pinout for MCP41010
 *
 * CS    1  +----+  Vcc
 * SCK   2  |    |  PB0
 * MOSI  3  |    |  PW0
 * Vss   4  +----+  PA0
 *
 */

#ifndef _MCP41010_H_
#define _MCP41010_H_

#include <string>
#include <Arduino.h>
#include <SPI.h>
// #include <PCF8575.h>

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
    MCP41010(const uint8_t cs_pin) : m_cs_pin(cs_pin){};
    uint8_t setValue(uint8_t value);
    uint8_t getValue();
    void begin();
    void csToggle(bool state);
    uint8_t m_cs_pin;
    uint8_t m_value;    
    static const uint8_t m_command_byte{0x11};
    // PCF8575 &m_pcf8575;
};
#endif