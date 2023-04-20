#ifndef TEENSY_I2C_H
#define TEENSY_I2C_H

#include <Wire.h>

// I2C communication constants
constexpr uint8_t I2C_WRITE = 0;
constexpr uint8_t I2C_READ = 1;

// I2C initialization function
void i2c_init(uint32_t frequency = 100000) {
    Wire.begin();
    Wire.setClock(frequency);
}

// I2C start function
void i2c_start() {
    // The start condition is handled by the Wire library
}

// I2C write function
void i2c_write(uint8_t data) {
    Wire.write(data);
}

// I2C read function
uint8_t i2c_read(bool ack) {
    return Wire.read();
}

// I2C stop function
void i2c_stop() {
    // The stop condition is handled by the Wire library
}

// I2C transmit function
void i2c_transmit(uint8_t address, uint8_t *data, uint16_t length, bool stop = true) {
    Wire.beginTransmission(address);
    for (uint16_t i = 0; i < length; ++i) {
        Wire.write(data[i]);
    }
    Wire.endTransmission(stop);
}

// I2C request function
void i2c_request(uint8_t address, uint8_t *data, uint16_t length, bool stop = true) {
    Wire.requestFrom(address, length, stop);
    for (uint16_t i = 0; i < length; ++i) {
        data[i] = Wire.read();
    }
}

#endif // TEENSY_I2C_H
