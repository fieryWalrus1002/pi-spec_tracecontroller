#include <mcp23017.h>

/// @brief Class to interface with MCP23017 I/O expander
/// @param port_mask_a - bit mask for port A, 0 for output, 1 for input
/// @param port_mask_b - bit mask for port B, 0 for output, 1 for input
void Mcp23017::begin(byte port_mask_a, byte port_mask_b)
{
    master.begin(1000000);
    if (configure_mcp23017())
    {
        write_register(iodir_a, port_mask_a); // set port mask
        write_register(iodir_b, port_mask_b); // set port mask
        write_register(gppu_a, 0x00);         // disable pullup resistors
        write_register(gppu_b, 0x00);         // disable pullup resistors
        write_register(iopol_a, 0x00);        // set polarity to normal
        write_register(iopol_b, 0x00);        // set polarity to normal
        write_register(gpinten_a, 0x00);      // disable interrupts
        write_register(gpinten_b, 0x00);      // disable interrupts
        write_register(intcon_a, 0x00);       // set interrupt control to compare against previous value
        write_register(intcon_b, 0x00);       // set interrupt control to compare against previous value
        write_register(defval_a, 0x00);       // set default value to 0
        write_register(defval_b, 0x00);       // set default value to 0
        write_register(gpio_a, 0x00);
        write_register(gpio_b, 0x00);
    }
    else
    {
        Serial.println("ERROR: cannot configure mcp in begin()");
    }
}

bool Mcp23017::write_register(byte reg, byte value)
{
    if (!wire.write(reg, value, true))
    {
        Serial.println("ERROR: Failed to set Mask/Enable Register");
        return false;
    }
    return true;
}

byte Mcp23017::read_register(byte reg)
{
    byte response = 0;
    wire.read(reg, &response, false);
    return response;
}

void Mcp23017::digitalWrite(uint8_t pin, bool value)
{
    byte port = mcp_port(pin);
    byte reg = (port == 0) ? gpio_a : gpio_b;
    byte mask = (1 << (pin % 8));

    byte current_state = read_register(reg);

    write_register(reg, mask);
    Serial.print(current_state);
    Serial.print(" : ");
    Serial.println(mask);
    delay(250);
}

uint8_t Mcp23017::digitalRead(uint8_t pin)
{
    byte port = mcp_port(pin);
    byte reg = (port == 0) ? gpio_a : gpio_b;
    return (read_register(reg) & (1 << (pin % 8)));
}

/// @brief return the mcp port for a given pin number
/// @param pin_num - pin number 0-15
/// @return 0 for port A, 1 for port B
byte Mcp23017::mcp_port(uint8_t pin_num)
{
    return ((pin_num < 8) ? 0 : 1);
};

/// @brief confiugre the mcp23017
bool Mcp23017::configure_mcp23017()
{
    if (!wire.write(iocon, iocon_byte_mode, false))
    {
        Serial.println("ERROR: Failed to set configuration register");
        return false;
    }
    Serial.println("Configured mcp23017 successfully.");
    return true;
}
