/*
 Digitally programmable LED intensity module. 
 Created by: 2023-02-27 by Magnus Wood
 ------------------------------------------------------------------------------
 A library to set the intensity of an LED, based on the max value given for the LED. 
 THe vlaue is used to calculate the needed current, and the current is set using a
 MCP41010 digital potentiometer.
 
------------------------------------------------------------------------------    
 Library for Microchip MCP41010 Digital Potentiometer
 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/11195c.pdf
 Created by: 2023-02-27 by Magnus Wood
 ------------------------------------------------------------------------------
 A library to easily set the resistance of the MCP41010 digital potentiometer.
 This chip is a 10k potentiometer with a digital interface, with a 256 position
 resolution.

------------------------------------------------------------------------------    
 Chip Wiring
 All connections necessary unless specifically mentionned.
 
 PIN Name Description
 1 - CS - Chip select. Drive CS LOW to enable serial interface.
 2 - SCLK - Serial data clock input.
 3 - MOSI - Serial data input. Data latched on rising edge of SCLK.
    4 - Vss - Ground
    5 - PA0 - Potiometer terminal .Code 00h brings wiper close to PA0.
    6 - PW0 - Potentiometer wiper.
    7 - PB0 - Potentiometer terminal B. Higher code brings wiper closer to B.
    8 - Vdd - 5V supply. 

------------------------------------------------------------------------------    
 Calculating Resistance
 Rw - wiper resistance
 Rab - resistance between A and B terminals. 10k ohm for this chip.
 Dn - wiper position, 8-bit value. 0-255

 Rwa(Dn) = ((Rab)(256 - Dn) / 256) + Rw
 Rwb(Dn) = ((Rab)(Dn) / 256) + Rw

 For using these equations to calculate feedback for amplifier circuits, the wiper
 resistance can be omitted due to the high impedance input of the amplifier.

------------------------------------------------------------------------------    
Function list
 in header

------------------------------------------------------------------------------    
Executing Commands
Set CS low to enable the serial interface. Then clock in a command byte followed by a 
data byte into the 16-bit register. The command is executed when CS is raised.

The command byte to write to the potentiometer is 0b00010001.
*/

#include "Arduino.h"
#include "SPI.h"
#include <led.h>

// LED::LED(std::string led_name, uint8_t led_pin, uint16_t max_source_current, uint16_t max_constant_current, uint16_t max_surge_current, MCP41010& mcp41010, PCF8575& pcf8575){
//     m_led_pin = led_pin;
//     m_max_source = max_source_current;
//     m_max_constant = max_constant_current;
//     m_max_surge = max_surge_current;
//     m_mcp41010 = mcp41010;
//     m_pcf8575 = pcf8575;

//     // m_pcf8575.pinMode(m_led_pin, OUTPUT);
//     // m_pcf8575.digitalWrite(m_led_pin, LOW);
// }

void LED::calibrate_intensity(uint8_t value){
    // Calibrate the intensity of the LED, as a fraction of the max current allowed
    // by the mode selected.

    // calculate the resistance needed to set the LED to the desired current (in mA)
    uint8_t resistance = get_resistance_value(value);
    // calculate the current needed to set the LED to the given value

    // set the current using the digital potentiometer
    mcp41010_.set_value(resistance);
}

void LED::toggle(bool state){
    digitalWrite(led_pin_, state);
}

const char* LED::get_name(){
    return led_name_.c_str();
}

void LED::set_intensity(uint8_t value, const uint8_t mode){
// Set the intensity of the LED, as a fraction of the max current allowed
    // by the mode selected.
    // value: 0-255, fraction of max current
    // mode: 1 for constant illumination, 0 for pulse mode
    
    int max_current = 0;
 
    if (mode == 0){
        max_current = max_surge_current_;
    } else {
        max_current = max_constant_current_;
    }

    int desired_current = (value / 255) * max_current;

    // calculate the resistance needed to set the LED to the desired current (in mA)
    
    // calculate the current needed to set the LED to the given value
    int resistance = get_resistance_value(desired_current);

    // set the current using the digital potentiometer
    mcp41010_.set_value(value);
    m_current_intensity = value;
};

uint8_t LED::get_resistance_value(uint8_t value){
    // calculate the resistance needed to set the LED to the given current (in mA)
    // given the 0-255 desired intensity value
    int resistance = 0;
    return resistance;
}
