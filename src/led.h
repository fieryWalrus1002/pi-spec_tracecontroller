#include "Arduino.h"
#include <vector>
#include <iostream>
#include <string>
#include <mcp41010.h>
#include <memory>

/// @param led_name The name of the LED, usually wavelength in nm.
/// @param led_pin The microcontroller pin that the LED gate is connected to.
/// @param led_shunt_pin The pin that will be read to get shunt voltage
/// @param max_source_current The maximum current that the LED can source, in mA.
/// @param max_constant_current The maximum current that the LED can source in constant current mode, in mA.
/// @param max_surge_current The maximum current that the LED can source in pulse mode, in mA.
/// @param pot_cs_pin The pin for the appropriate CS MCP41010
struct LedData
{
    std::string led_name;
    int led_pin;
    int shunt_pin;
    int max_source_current;
    int max_constant_current;
    int max_surge_current;
    int pot_cs_pin;
};

/// @brief Class to hold LED implementation details
/// @details This class is used to hold the implementation details of the LED class.
/// It can turn on the led and turn it off.
/// @param LedData A struct containing data to instantiate an object of class LED
class LED
{
public:
    LED(const LedData &ledData) : led_pin(ledData.led_pin),
                                  shunt_pin(ledData.shunt_pin),
                                  max_source_current(ledData.max_source_current),
                                  max_constant_current(ledData.max_constant_current),
                                  max_surge_current(ledData.max_surge_current),
                                  mcp41010(std::make_shared<MCP41010>(ledData.pot_cs_pin))
    {
        pinMode(led_pin, OUTPUT);
    }

    void toggle(bool state);
    void calibrate_intensity(uint8_t value);
    void set_intensity(uint8_t value, const uint8_t mode);
    uint8_t get_resistance_value(uint8_t value);
    const char *get_name();
    uint8_t get_intensity() { return m_current_intensity; };
    const char *ledPinToString();
    int getShuntVoltage();

    std::string led_name;
    uint8_t led_pin;
    uint8_t shunt_pin;
    uint16_t max_source_current;
    uint16_t max_constant_current;
    uint16_t max_surge_current;
    uint8_t m_current_intensity;

private:
    std::shared_ptr<MCP41010> mcp41010;
};

std::shared_ptr<std::vector<LED>> getLedArray(const std::vector<LedData> &ledData);