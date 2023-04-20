#include <mcp23017.h>

/// @brief Class to interface with MCP23017 I/O expander
/// @param port_mask_a - bit mask for port A, 0 for output, 1 for input
/// @param port_mask_b - bit mask for port B, 0 for output, 1 for input
void Mcp23017::begin(byte port_mask_a, byte port_mask_b) {
    wire.begin();
    // wire.setClock(i2c_speed);
    write_register(iocon, iocon_byte_mode); // set iocon to byte mode, non-sequential
    write_register(iodir_a, port_mask_a); // set port mask
    write_register(iodir_b, port_mask_b); // set port mask
    // write_register(gppu_a, 0x00); // disable pullup resistors
    // write_register(gppu_b, 0x00); // disable pullup resistors
    // write_register(iopol_a, 0x00); // set polarity to normal
    // write_register(iopol_b, 0x00); // set polarity to normal
    // write_register(gpinten_a, 0x00); // disable interrupts
    // write_register(gpinten_b, 0x00); // disable interrupts
    // write_register(intcon_a, 0x00); // set interrupt control to compare against previous value
    // write_register(intcon_b, 0x00); // set interrupt control to compare against previous value
    // write_register(defval_a, 0x00); // set default value to 0
    // write_register(defval_b, 0x00); // set default value to 0
}

uint8_t Mcp23017::write_register(byte reg, byte value) {
    wire.beginTransmission(i2c_address);
    wire.write(reg);
    wire.write(value);
    Wire.endTransmission();
    return 0;
}

uint8_t Mcp23017::read_register(uint8_t reg) {
    wire.beginTransmission(i2c_address);
    wire.write(reg);
    wire.endTransmission();
    wire.requestFrom(i2c_address, (uint8_t)1);
    return (wire.read());
}

void Mcp23017::digitalWrite(uint8_t pin, uint8_t value) {
    byte port = mcp_port(pin);
    byte reg = (port == 0) ? gpio_a : gpio_b;
    byte bit_mask = read_register(reg) & ~(value << (pin % 8));
    Serial.print("bit_mask: ");
    Serial.println(bit_mask);
    write_register(reg, bit_mask);
}

uint8_t Mcp23017::digitalRead(uint8_t pin) {
    byte port = mcp_port(pin);
    byte reg = (port == 0) ? gpio_a : gpio_b;
    return (read_register(reg) & (1 << (pin % 8)));
}


/// @brief return the mcp port for a given pin number
/// @param pin_num - pin number 0-15
/// @return 0 for port A, 1 for port B
byte Mcp23017::mcp_port(uint8_t pin_num){
    return ((pin_num < 8) ? 0 : 1);
};