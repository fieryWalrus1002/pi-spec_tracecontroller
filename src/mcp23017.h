#ifndef _MCP23017_H_
#define _MCP23017_H_
#include <Arduino.h>
#include <i2c_device.h>

/// @brief  Class to interface with MCP23017 I/O expander
/// @details Uses I2C to interface with the MCP23017 I/O expander
/// @param i2c_address - I2C address of the MCP23017
class Mcp23017
{
public:
    Mcp23017(byte i2c_address, I2CMaster &master) : i2c_address(i2c_address), wire(I2CDevice(master, i2c_address, 1)), master(master){};
    ~Mcp23017() { wire.~I2CDevice(); };
    void begin(byte port_mask_a, byte port_mask_b);
    bool write_register(byte reg, byte data);
    byte read_register(byte reg);
    uint8_t digitalRead(uint8_t pin);
    void digitalWrite(uint8_t pin, bool value);

    uint8_t mcp_port(uint8_t pin_num);
    // MCP23017 register addresses
    const byte iodir_a = 0x00;    // I/O direction register port a
    const byte iodir_b = 0x01;    // I/O direction register port b
    const byte iopol_a = 0x02;    // I/O polarity register port a
    const byte iopol_b = 0x03;    // I/O polarity register port b
    const byte gpinten_a = 0x04;  // interrupt on change register port a
    const byte gpinten_b = 0x05;  // interrupt on change register port b
    const byte defval_a = 0x06;   // default value register port a
    const byte defval_b = 0x07;   // default value register port b
    const byte intcon_a = 0x08;   // interrupt control register port a
    const byte intcon_b = 0x09;   // interrupt control register port b
    const byte iocon = 0x0A;      // io configuration register
    const byte gppu_a = 0x0C;     // gpio pullup resistor register port a
    const byte gppu_b = 0x0D;     // gpio pullup resistor register port b
    const byte intf_a = 0x0E;     // interrupt flag register port a
    const byte intf_b = 0x0F;     // interrupt flag register port b
    const byte intcap_a = 0x10;   // interrupt capture register port a
    const byte intcap_b = 0x11;   // interrupt capture register port b
    const byte gpio_a = 0x12;     // general purpose I/O port register port a
    const byte gpio_b = 0x13;     // general purpose I/O port register port b
    const byte olata = 0x14;      // output latch register port a
    const byte olatb = 0x15;      // output latch register port b
    const int i2c_address = 0x27; // default i2c address
    const int i2c_speed = 100000; // 1.7 MHz
    I2CDevice wire;
    I2CMaster &master;
    bool configured; // is the device configured successfully
private:
    bool configure_mcp23017();

    // control register bytes
    // IOCON.BANK = 0, registers associated with each port are alternating
    // IOCON.MIRROR = 0, int pins are not connected
    // IOCON.SEQOP = 1, sequential operation disabled, address pointer does not increment
    // IOCON.DISSLW = 0, slew rate enabled
    // IOCON.HAEN = 0, don't care on MCP23017
    // IOCON.ODR = 0, active driver output (intpol sets polarity)
    // IOCON.INTPOL = 0, sets polarity of INT output pin. 0 = active low, 1 = active high
    // IOCON.UNUSED = Unimplemented, read as 0
    const byte iocon_byte_mode = 0b00100000;
};

#endif