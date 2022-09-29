#ifndef _MAIN_H_
#define _MAIN_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "pins.h"
#include "max1132.h"

const int MAX_INT_VAL = 32768;
const int INT_SAFETY_VAL = 3276;
const int MAX_AQ = 9;
const int MAX_DATA = 16384;
bool DEBUG_MODE = false; // set this to true to send responses to commands, for debug purposes. Disable before real measurements, as serial output lags the program beyond acceptable time delays.

// struc that contains all of our trace variables, to remove global variables as much as possible
struct TraceParameters
{

} traceParams;



struct Point
{
    long time_us[2]; // begging and end time of this point's aq
    uint16_t aq[MAX_AQ];
};

struct TraceBuffer
{
    Point data[MAX_DATA]; // 
};

// todo:
// array of Points is dynam alloc using malloc during runtime
// a pointer to this array is 

TraceBuffer traceData[1];
TraceBuffer *ptr_buffer;

// void send_data_point(int, int, TraceBuffer*);
void send_response(auto, auto);
void send_data_point(int, int);
void write_act_intensity(int);
void switch_act_gate(int);

#endif