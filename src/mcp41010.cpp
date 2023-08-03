#include <mcp41010.h>

void MCP41010::begin()
{
    // begin the SPI1 interface
    // m_pcf8575.pinMode(m_cs_pin, OUTPUT);
    // m_pcf8575.digitalWrite(m_cs_pin, HIGH);
    pinMode(m_cs_pin, OUTPUT);
    csToggle(HIGH);
    setValue(0);
}

uint8_t MCP41010::getValue()
{
    // returns the current 0-255 value of the potentiometer
    return m_value;
}

uint8_t MCP41010::setValue(uint8_t value)
{
    // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    csToggle(LOW);
    SPI.transfer(m_command_byte);
    SPI.transfer(value);
    csToggle(HIGH);
    // SPI.endTransaction();

    m_value = value;

    return value;
}

void MCP41010::csToggle(bool state)
{
    // toggle the chip select pin to the state specified
    // m_pcf8575.digitalWrite(m_cs_pin, state);
    digitalWrite(m_cs_pin, state);
}