#include "teensy_i2c.h" // Include the header file containing the low-level I2C functions

class MCP23017 {
public:
    static constexpr uint8_t MCP23017_ADDRESS = 0x20; // Default I2C address
    static constexpr uint8_t IODIRA = 0x00;
    static constexpr uint8_t IODIRB = 0x01;
    static constexpr uint8_t GPIOA = 0x12;
    static constexpr uint8_t GPIOB = 0x13;
    static constexpr uint8_t OLATA = 0x14;
    static constexpr uint8_t OLATB = 0x15;

    MCP23017(uint8_t address = MCP23017_ADDRESS) : _address(address) {}

    void begin() {
        writeRegister(IODIRA, 0xFF); // Set all pins as inputs by default
        writeRegister(IODIRB, 0xFF);
    }

    void pinMode(uint8_t pin, bool mode) {
        uint8_t registerAddr = (pin < 8) ? IODIRA : IODIRB;
        uint8_t bit = pin % 8;
        uint8_t iodir = readRegister(registerAddr);
        iodir = mode ? (iodir | (1 << bit)) : (iodir & ~(1 << bit));
        writeRegister(registerAddr, iodir);
    }

    void digitalWrite(uint8_t pin, bool value) {
        uint8_t registerAddr = (pin < 8) ? GPIOA : GPIOB;
        uint8_t bit = pin % 8;
        uint8_t gpio = readRegister(registerAddr);
        gpio = value ? (gpio | (1 << bit)) : (gpio & ~(1 << bit));
        writeRegister(registerAddr, gpio);
    }

    bool digitalRead(uint8_t pin) {
        uint8_t registerAddr = (pin < 8) ? GPIOA : GPIOB;
        uint8_t bit = pin % 8;
        uint8_t gpio = readRegister(registerAddr);
        return (gpio & (1 << bit)) != 0;
    }

private:
    uint8_t _address;

    void writeRegister(uint8_t reg, uint8_t value) {
        i2c_start();
        i2c_write((_address << 1) | I2C_WRITE);
        i2c_write(reg);
        i2c_write(value);
        i2c_stop();
    }

    uint8_t readRegister(uint8_t reg) {
        i2c_start();
        i2c_write((_address << 1) | I2C_WRITE);
        i2c_write(reg);
        i2c_start(); // Repeated start
        i2c_write((_address << 1) | I2C_READ);
        uint8_t value = i2c_read(false);
        i2c_stop();
        return value;
    }
};
