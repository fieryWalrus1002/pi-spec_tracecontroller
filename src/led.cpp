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



uint8_t MCP41010::set_value(uint8_t value){
    // set the value of the potentiometer
    // also returns the value set
    // and sets the m_value variable to equal value
    
    digitalWrite(m_cs_pin, LOW);
    SPI.transfer(m_command_byte);
    SPI.transfer(value);
    digitalWrite(m_cs_pin, HIGH);
    
    m_value = value;
    
    return value;
}

void LED_ARRAY::change_led_state(uint8_t led_num, bool state){
    if (state){
        m_leds[led_num].on();
    }
    else{
        m_leds[led_num].off();
    }
}

void LED_ARRAY::add(LED led){
    m_leds.push_back(led);
}


uint8_t MCP41010::get_value(){
    // returns the current 0-255 value of the potentiometer
    return m_value;
}


LED::LED(std::string led_name, uint8_t led_pin, uint8_t mcp_cs_pin, uint8_t shunt_pin, uint16_t max_source_current, uint16_t max_constant_current, uint16_t max_surge_current){
    // initialize the LED object
    // led_pin: the pin the LED is connected to
    // mcp_cs_pin: the chip select pin for the MCP41010 digital potentiometer
    // max_source_current: the max current of the LED source in mA
    // max_constant_current: the max current of the LED in constant illumination mode in mA
    // max_surge_current: the max current of the LED in pulse mode in mA
    m_led_pin = led_pin;
    m_cs_pin = mcp_cs_pin;
    m_shunt_pin = shunt_pin;
    m_max_source = max_source_current;
    m_max_constant = max_constant_current;
    m_max_surge = max_surge_current;
    mcp41010 = MCP41010(m_cs_pin);
    pinMode(m_led_pin, OUTPUT);
}

void LED::calibrate_intensity(uint8_t value){
    // Calibrate the intensity of the LED, as a fraction of the max current allowed
    // by the mode selected.

    
    // calculate the resistance needed to set the LED to the desired current (in mA)
    uint8_t resistance = get_resistance_value(value);
    // calculate the current needed to set the LED to the given value

    // set the current using the digital potentiometer
    mcp41010.set_value(resistance);
}

int LED::set_intensity(uint8_t value, char mode){
    // Set the intensity of the LED, as a fraction of the max current allowed
    // by the mode selected.
    // value: 0-255, fraction of max current
    // mode: 'c' for constant illumination, 'p' for pulse mode
    
    int max_current = 0;

    if (mode == 'p'){
        max_current = m_max_surge;
    } else {
        max_current = m_max_constant;
    }

    int desired_current = (value / 255) * max_current;

    // calculate the resistance needed to set the LED to the desired current (in mA)
    
    // calculate the current needed to set the LED to the given value
    int resistance = get_resistance_value(desired_current);

    // set the current using the digital potentiometer
    mcp41010.set_value(resistance);
    
    return value;
}

void LED::on(){
    // turn the LED on
   digitalWrite(m_led_pin, HIGH);
}

void LED::off(){
    // turn the LED off.
    // intensity will be at whatever the current source is set to.
    digitalWrite(m_led_pin, LOW);
}

uint8_t LED::get_resistance_value(uint8_t value){
    // calculate the resistance needed to set the LED to the given current (in mA)
    // given the 0-255 desired intensity value
    int resistance = 0;
    return resistance;
}

// void write_act_intensity(int value)
// {
//     // sets the current act intensity by changing MCP resistance 0-255/0-10k Ohm
//     digitalWrite(pins.MCP_ACT_CS_PIN, LOW);
//     SPI.transfer(B00010001);
//     SPI.transfer(value);
//     digitalWrite(pins.MCP_ACT_CS_PIN, HIGH);
//     current_act_intensity = value;
// }