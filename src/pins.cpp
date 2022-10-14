#include "pins.h"

void Pins::init(){
    pinMode(Pins::SAT_PULSE_GATE,OUTPUT);
    pinMode(Pins::DETECTOR_GATE_PIN,OUTPUT);
    pinMode(Pins::MCP_CS_PIN,OUTPUT);
    pinMode(Pins::ACT_GATE_PIN,OUTPUT);
    pinMode(Pins::POWER_GATE_PIN,OUTPUT);
    pinMode(Pins::STO_FLASH_PIN,OUTPUT);
    pinMode(Pins::meas_led_array[0], OUTPUT);
    pinMode(Pins::meas_led_array[1], OUTPUT);
    pinMode(Pins::meas_led_array[2], OUTPUT);
    pinMode(Pins::meas_led_array[3], OUTPUT);
    pinMode(Pins::meas_led_array[4], OUTPUT);
    pinMode(Pins::meas_led_array[5], OUTPUT);
    pinMode(Pins::meas_led_array[6], OUTPUT);
    pinMode(Pins::meas_led_array[7], OUTPUT);
    pinMode(Pins::meas_led_array[8], OUTPUT);


    digitalWrite(Pins::SAT_PULSE_GATE,LOW);
    digitalWrite(Pins::DETECTOR_GATE_PIN,LOW);
    digitalWrite(Pins::MCP_CS_PIN,LOW);
    digitalWrite(Pins::ACT_GATE_PIN,LOW);
    digitalWrite(Pins::POWER_GATE_PIN,LOW);
    digitalWrite(Pins::STO_FLASH_PIN,LOW);
    digitalWrite(Pins::meas_led_array[0], LOW);
    digitalWrite(Pins::meas_led_array[1], LOW);
    digitalWrite(Pins::meas_led_array[2], LOW);
    digitalWrite(Pins::meas_led_array[3], LOW);
    digitalWrite(Pins::meas_led_array[4], LOW);
    digitalWrite(Pins::meas_led_array[5], LOW);
    digitalWrite(Pins::meas_led_array[6], LOW);
    digitalWrite(Pins::meas_led_array[7], LOW);
    digitalWrite(Pins::meas_led_array[8], LOW);
}

void Pins::meas_led_cleanup(){
    digitalWrite(Pins::meas_led_array[0], LOW);
    digitalWrite(Pins::meas_led_array[1], LOW);
    digitalWrite(Pins::meas_led_array[2], LOW);
    digitalWrite(Pins::meas_led_array[3], LOW);
    digitalWrite(Pins::meas_led_array[4], LOW);
    digitalWrite(Pins::meas_led_array[5], LOW);
    digitalWrite(Pins::meas_led_array[6], LOW);
    digitalWrite(Pins::meas_led_array[7], LOW);
    digitalWrite(Pins::meas_led_array[8], LOW);
}