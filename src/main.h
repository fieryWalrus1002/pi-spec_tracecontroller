#ifndef _MAIN_H_
#define _MAIN_H_
const int MAX_INT_VAL = 32768;
const int INT_SAFETY_VAL = 3276;
const int MAX_AQ = 9;
const int MAX_DATA = 16384;

// struct to store information about trace paramters, store them and
// maybe a pointer to a data buffer for that trace?

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

void send_data_point(int, int);
void write_act_intensity(int, int);
void set_act_intensity(int, int);
void preset_trace_act_values();

#endif