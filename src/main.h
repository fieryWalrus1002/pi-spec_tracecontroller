#ifndef _MAIN_H_
#define _MAIN_H_

// define pins
const int CS_PIN = 10;
const int ADC_PIN = 15;
const int SAT_PULSE_PIN = 0;
const int STO_FLASH_PIN = 0;
const int meas_led_array[] = {0, 2, 3, 4, 5, 6, 7, 8, 9}; // 0 will not flash anything

void send_data_point(int);
void write_act_intensity(int);
void set_act_intensity(int);
#endif