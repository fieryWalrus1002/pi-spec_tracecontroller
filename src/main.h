#ifndef _MAIN_H_
#define _MAIN_H_

#include <Arduino.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
// #include <map>
// #include <vector>
#include <utility>
#include <max1132.h>
#include <led.h>
// #include <mcp23017.h>
// #include <string>
#include <IntervalTimer.h>
#include <sstream>
#include <memory>
#include <string>
#include <util.h>

enum CURRENT_MODE
{
    CONSTANT,
    PULSE
};

const int MAX_DATA = 5000;
struct Point
{
    uint32_t time_us[2]; // begin of measurement, time of pulse turned on, time of pulse turned off
    uint16_t aq[9];      // analog values
};

struct TraceBuffer
{
    Point data[MAX_DATA]; //
};

bool act_phase_trigger_state[3] = {false, false, false};
uint8_t meas_aq_num = 9;
bool testMode = false; // if test mode is on or off?
uint32_t zeroTime = 0;
bool measureState = false; // state for measurements
int traceNumber = 0;       // the current trace number, index in traceData for current trace data to be placed
int counter = 0;
int satPulseBegin = 200;
int satPulseEnd = 300;
uint32_t pulseLength = 33;         // in uS
unsigned int pulseInterval = 1000; // in uS, so 1000 is 1ms between measurement pulses
int pulseMode = 1;                 // 0 just actinic setting, 1 sat pulse, 2 single turnover
int measLedNum = 0;
uint8_t actinicIntensity = 0;
int numPoints = 700;
uint32_t triggerDelay = 0;
bool powerState = false; // status of the power switch
int tracePhase = 0;
uint8_t actIntPhase[] = {0, 250, 0}; // holds the actinic intensity values for phases 0-2 in 0-255 values
int currentValue;

/**
 * 5-4-23 Teensy 4.1 pin diagram
 *
 *                 GND +-----+  5V
 * LED_PIN_NONE      0 |     | GND
 *                   1 |     | 3.3V
 *                   2 |     | 23   LED_PIN_800
 *                   3 |     | 22   LED_PIN_900
 *                   4 |     | 21   LED_PIN_740
 *                   5 |     | 20   LED_PIN_545
 *                   6 |     | 19   SCL
 *                   7 |     | 18   SDA
 *                   8 |     | 17   LED_PIN_554
 *                   9 |     | 16   LED_PIN_563
 *                  10 |     | 15   LED_PIN_572
 * MOSI             11 |     | 14
 * MISO             12 |     | 13   SCK
 *                3.3V |     | GND
 * POT1_SHUNT_PIN   24 |     | 41   POT1_CS_PIN
 * POT2_SHUNT_PIN   25 |     | 40   POT2_CS_PIN
 * POT3_SHUNT_PIN   26 |     | 39   POT3_CS_PIN
 *                  27 |     | 38
 * ADC_CS_PIN       28 |     | 37
 * ADC_RST_PIN      29 |     | 36
 * ADC_SSTRB_PIN    30 |     | 35   LED_PIN_520
 *                  31 |     | 34   LED_PIN_625
 *                  32 +-----+ 33
 */

// led pins
const int LED_PIN_NONE = 0;
const int LED_PIN_520 = 35;
const int LED_PIN_545 = 20;
const int LED_PIN_554 = 17;
const int LED_PIN_563 = 16;
const int LED_PIN_572 = 15;
const int LED_PIN_740 = 21;
const int LED_PIN_800 = 23;
const int LED_PIN_900 = 22;
const int LED_PIN_625 = 34;

// Potentiometer Pins
// Pot1 is low current, pot2 is high current, pot 3 is IR LED current
const int POT1_SHUNT_PIN = 24;
const int POT1_CS_PIN = 41;
const int POT2_SHUNT_PIN = 25;
const int POT2_CS_PIN = 40;
const int POT3_CS_PIN = 39;
const int POT3_SHUNT_PIN = 26;

// MAX1132 16bit ADC
const int ADC_CS_PIN = 28;
const int ADC_RST_PIN = 29;
const int ADC_SSTRB_PIN = 30;

// other variables
const int GPIO_I2C_ADDR = 0x20;
const int MAX_INT_VAL = 32768;
const int INT_SAFETY_VAL = 3276;

// LCD module
const int LCD_I2C_ADDR = 0x3C;
const int LCD_RESET_PIN = -1;
const int LCD_WIDTH = 128;
const int LCD_HEIGHT = 32;

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
    GOT_Q,
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

TraceBuffer traceData[1];
TraceBuffer *pBuffer;

// void send_data_point(int, int, TraceBuffer*);
void toggleTestPulser(bool);
void sendResponse(auto, auto);
void sendDataPoint(int, int);
// void write_act_intensity(int);
// void switch_act_gate(int);
void pinTest(int);
void letThereBeLight(int);
// void testMeasLedPins(int);
void handleActPhase(int);
int testPulse();
Point measurementPulse(TraceBuffer *, int);
void setLedIntensity(int, int);
void setActinicIntensity(int, int);
void calibrateTriggerDelay(int);
#endif