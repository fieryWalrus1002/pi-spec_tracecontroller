#include <mcp41010.h>

void MCP41010::begin(){
    // begin the SPI1 interface
    // m_pcf8575.pinMode(m_cs_pin, OUTPUT);
    // m_pcf8575.digitalWrite(m_cs_pin, HIGH);
    pinMode(m_cs_pin, OUTPUT);
    digitalWrite(m_cs_pin, HIGH);
    SPI.begin();
    set_value(0);
}

uint8_t MCP41010::get_value(){
    // returns the current 0-255 value of the potentiometer
    return m_value;
}

uint8_t MCP41010::set_value(uint8_t value){
    // set the value of the potentiometer
    // also returns the value set
    // and sets the m_value variable to equal value
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(m_cs_pin, LOW);
    SPI.transfer(m_command_byte);
    SPI.transfer(value);
    digitalWrite(m_cs_pin, HIGH);
    SPI.endTransaction();
    
    m_value = value;
    
    return value;
}

void MCP41010::csToggle(bool state){
    // toggle the chip select pin to the state specified
    // m_pcf8575.digitalWrite(m_cs_pin, state);   
    digitalWrite(m_cs_pin, state);
}