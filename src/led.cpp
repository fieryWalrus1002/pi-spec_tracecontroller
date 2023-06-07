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

void LED::calibrateIntensity(uint8_t value)
{
    // // Calibrate the intensity of the LED, as a fraction of the max current allowed
    // // by the mode selected.

    // // calculate the resistance needed to set the LED to the desired current (in mA)
    // uint8_t resistance = getResistanceValue(value);
    // // calculate the current needed to set the LED to the given value

    // // set the current using the digital potentiometer
    // mcp41010->setValue(resistance);
}

void LED::toggle(bool state)
{
    digitalWrite(led_pin, state);
}

void LED::turnOn()
{
    digitalWrite(led_pin, HIGH);
}

void LED::turnOff()
{
    digitalWrite(led_pin, LOW);
}

const char *LED::getName()
{
    return led_name.c_str();
}

/**
 * @brief Takes a desired intensity value from zero to max allowed intensity based
 * on the LED's capabilites. It will scale the value to the max intensity allowed.
 *
 * @param mode 0 is pulsed, constant is 1.
 * @param value 0-255, fraction of max intensity
 * @return intensity the 0-255 value to give to the MCP41010 digital pot
 */
uint8_t LED::setIntensity(const uint8_t value, const uint8_t mode)
{
    m_current_intensity = mcp41010->setValue(getResistanceValue(value, mode));
    return m_current_intensity;
};

/**
 * @brief Takes a desired intensity value from zero to max allowed intensity based
 * on the LED's capabilites.
 * @param mode 0 is pulsed, constant is 1.
 * @param value 0-255, fraction of max intensity
 * @return resistance the 0-255 value to give to the MCP41010 digital pot
 * in order to get the desired intensity.
 */
uint8_t LED::getResistanceValue(const uint8_t value, const uint8_t mode)
{
    uint8_t clipped_value = static_cast<uint8_t>(value * m_max_intensity / 255);

    // If the value is greater than 0, make sure the returned value is at least 1. This
    // is for the LEDs with very low max intensity. If we are dividing a max_intensity of
    // 10 by 255, a significant portion of the range will be 0.
    if (value > 0 && clipped_value <= 1)
    {
        return 1;
    }
    else
    {
        return clipped_value;
    }
}

/**
 * @brief return the uint8_t led_pin as a c string
 */
const char *LED::ledPinToString()
{
    static char str[4];
    snprintf(str, sizeof(str), "%d", led_pin);
    return str;
}

/**
 * @brief takes a reference to an LED object and then calls an analogRead
 * on the shunt_pin associated with that LED object. Returns the int value
 * it gets from the microcontroller ADC
 */
int LED::getShuntVoltage()
{
    return analogRead(shunt_pin);
};

/**
 * @brief instantiates a vector of LED objects from an array of LedData defined in main.h
 * @return std::shared_ptr to the vector of leds
 */
std::shared_ptr<std::vector<LED>> getLedArray(const std::vector<LedData> &ledData)
{
    auto leds = std::make_shared<std::vector<LED>>();
    // for each struct in the ledData, instantiate an LED object in the leds vector
    for (const auto &data : ledData)
    {
        leds->push_back(LED(data));
    }

    // now return the shared ptr
    return leds;
}

/**
 * @brief Take a string in the form of "625" representing the wavelenght of an LED. Find the
 * corresponding LED in the vector of LED objects and return the index of that LED in the vector.
 * @return uint8_t index of the LED in the vector if present, else 0
 */
uint8_t getLedNum(std::string ledNm, std::shared_ptr<std::vector<LED>> leds)
{
    for (size_t i = 0; i < leds->size(); ++i)
    {
        if ((*leds)[i].led_name == ledNm)
        {
            return static_cast<uint8_t>(i);
        }
    }
    return 0; // no matching LED found
}