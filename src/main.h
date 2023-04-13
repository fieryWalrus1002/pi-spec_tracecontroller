#ifndef _MAIN_H_
#define _MAIN_H_

#include <Arduino.h>


// MAX1132 16bit ADC
const int ADC_CS_PIN = 28;
const int ADC_RST_PIN = 29;
const int ADC_SSTRB_PIN = 30;

// MCP41010 8bit DAC
// on gpio expander
const int POT1_CS_PIN = 40;
const int POT2_CS_PIN = 41;

// SPI1
const int SPI1_MOSI_PIN = 26;
const int SPI1_MISO_PIN = 39;
const int SPI1_SCK_PIN = 27;

// other pin numbers
const int POWER_GATE_PIN = 50;
const int STO_GATE_PIN = 51;

// other variables
const int GPIO_I2C_ADDR = 0x20;
const int MAX_INT_VAL = 32768;
const int INT_SAFETY_VAL = 3276;
const int MAX_AQ = 5;
const int MAX_DATA = 1000;
bool DEBUG_MODE = false; // set this to true to send responses to commands, for debug purposes. Disable before real measurements, as serial output lags the program beyond acceptable time delays.

typedef enum
{
    NONE,
    GOT_A,
    GOT_B,
    GOT_C,
    GOT_D,
    GOT_E,
    GOT_F,
    GOT_G,
    GOT_H,
    GOT_I,
    GOT_J,
    GOT_K,
    GOT_L,
    GOT_M,
    GOT_N,
    GOT_O,
    GOT_P,
    GOT_R,
    GOT_S,
    GOT_T,
    GOT_U,
    GOT_V,
    GOT_W,
    GOT_X,
    GOT_Y,
    GOT_Z
} states;
states state = NONE;

// struc that contains all of our trace variables, to remove global variables as much as possible
struct TraceParameters
{

} traceParams;



struct Point
{
    long time_us[3]; // begin of measurement, time of pulse turned on, time of pulse turned off
    uint16_t aq[MAX_AQ];
};

struct TraceBuffer
{
    Point data[MAX_DATA]; // 
};

TraceBuffer traceData[1];
TraceBuffer *ptr_buffer;

// void send_data_point(int, int, TraceBuffer*);
void send_response(auto, auto);
void send_data_point(int, int);
// void write_act_intensity(int);
// void switch_act_gate(int);
void pinTest(int);
// void testMeasLedPins(int);
void handle_act_phase(int);
#endif