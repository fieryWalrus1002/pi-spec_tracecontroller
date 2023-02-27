#include <Arduino.h>

// new pins
const int 


class Pins {
    public:
        // define pins
        // 30  - ADC_CS_PINT
        // 29 - ADC_RST_PIN
        // 28 - ADC_SSTRB_PIN for max1132 input signal of adc ready state
        const int MCP_MEAS_CS_PIN = 23;
        const int MCP_ACT_CS_PIN = 10;
        const int POWER_GATE_PIN = 21;
        const int STO_FLASH_PIN = 20;
        // 19 - SCL - SSD1306 IC2
        // 18 - SDA - SSD1306 IC2
        // 13 - SCK
        // 12 - MISO
        // 11 - MOSI 
        const int meas_led_array[9] {0, 
        2, //520nm
        3, 
        4, 
        5, 
        6, 
        7,  // 940nm
        8, // 820nm
        9}; // 0 will not flash anything
        const int pin_setup_array[6] {SAT_PULSE_GATE, DETECTOR_GATE_PIN, MCP_MEAS_CS_PIN, MCP_ACT_CS_PIN, ACT_GATE_PIN, POWER_GATE_PIN, STO_FLASH_PIN};

        void init();
        void meas_led_cleanup();
};
