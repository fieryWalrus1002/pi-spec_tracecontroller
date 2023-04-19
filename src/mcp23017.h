#ifndef _MCP23017_H_
#define _MCP23017_H_
#include <Arduino.h>
#include <Wire.h>



/// @brief  Class to interface with MCP23017 I/O expander
/// @details Uses I2C to interface with the MCP23017 I/O expander
/// @param i2c_address - I2C address of the MCP23017
class Mcp23017
{
public:
    Mcp23017(int i2c_address) : i2c_address_(i2c_address) {};
    void setup(bool port_a = false, bool port_b = false);
    void transmit(uint8_t port, byte data);
    byte receive(uint8_t port);
private:
    // MCP23017 register addresses
    const byte IODIRA = 0x00; // I/O direction register port A
    const byte IODIRB = 0x01; // I/O direction register port B
    const byte GPIOA = 0x12; // GPIO port A
    const byte GPIOB = 0x13; // GPIO port B
    const int i2c_address_;
};

/// @brief Class to interface with MCP23017 I/O expander
/// @param port_a - true if port A is INPUT, false if OUTPUT
/// @param port_b - true if port B is INPUT, false if OUTPUT
void Mcp23017::setup(bool port_a = false, bool port_b = false) {
  Wire.begin(); // Initialize the I2C bus
  Wire.setClock(1700000); // Set the I2C bus speed to 1.7 MHz
  Wire.beginTransmission(0x20); // Connect to MCP23017 at address 0x20
  Wire.write(IODIRA); // Configure I/O direction of port A as outputs
  Wire.write(0x00);
  Wire.write(IODIRB); // Configure I/O direction of port B as outputs
  Wire.write(0x00);
  Wire.endTransmission();
}

/// @brief transmit byte to MCP23017 port of choice
/// @param port - 0 for port A, 1 for port B
/// @param data - byte to transmit
void Mcp23017::transmit(uint8_t port, byte data) {
    Wire.beginTransmission(0x20); // Connect to MCP23017
  if (port == 0) {
    Wire.write(GPIOA); // Set output pins of port A high
  }
  else if (port == 1) {
    Wire.write(GPIOB); // Set output pins of port A high
  }
  Wire.write(data);
  Wire.endTransmission();
}

/// @brief receive byte from MCP23017 port of choice
/// @param port - 0 for port A, 1 for port B
/// @return byte received
byte Mcp23017::receive(uint8_t port) {
  Wire.beginTransmission(0x20); // Connect to MCP23017
  if (port == 0) {
    Wire.write(GPIOA);
  }
  else if (port == 1) {
    Wire.write(GPIOB);
  }
  Wire.requestFrom(0x20, 1); // Request 1 byte of data
  while (Wire.available() == 0); // Wait for data to become available
  byte data = Wire.read(); // Read the received data
  Wire.endTransmission();
  return data;
}

/// @brief speed test to determine I2C bus speed
/// @details 1.7 MHz is the fastest speed that the MCP23017 can handle
/// @param 

#endif