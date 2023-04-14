/*
 * File: iread.h
 * Description: code for selecting and reading from 8 analog input channels using a Mux08 and outputting the data as analog signals.
 * Author: Magnus Wood
 * Date:4-14-2023
 */

#ifndef _CURRENT_MONITOR_H_
#define _CURRENT_MONITOR_H_
#include <Arduino.h>


/// @brief A class that controls a MUX08, selecting the input channel to route to the
/// output pin.
/// @details The MUX08 is a 8 channel analog multiplexer that routes the analog input   
/// from one of 8 input channels to the output pin. The input channel is selected by
/// setting the 3 control pins to a binary value corresponding to the channel number.
/// The output pin is an analog output pin that outputs the analog signal from the selected
/// input channel.
/// @param a0 The pin number of the a0 control pin
/// @param a1 The pin number of the a1 control pin
/// @param a2 The pin number of the a2 control pin
/// @param output_pin The pin number of the output pin/ Drain pin
/// @note The mux08 pinout is as follows:
/// 1 - A0       16 - A1
/// 2 - Enable   15 - A2
/// 3 - V- (GND) 14 - GND
/// 4 - S0       13 - V+ (5V)
/// 5 - S1       12 - S5
/// 6 - S2       11 - S6
/// 7 - S3       10 - S7
/// 8 - Drain    9 - S8
class Mux08
{
    public:
    // constructor with initialization list
        Mux08(const uint8_t mux_enable_pin, const uint8_t a0, const uint8_t a1, const uint8_t a2, const uint8_t output_pin) : m_enable(mux_enable_pin), m_a0(a0), m_a1(a1), m_a2(a2), m_output_pin(output_pin) {
            pinMode(m_enable, OUTPUT);
            pinMode(m_a0, OUTPUT);
            pinMode(m_a1, OUTPUT);
            pinMode(m_a2, OUTPUT);
            pinMode(m_output_pin, INPUT);
        };
        void select_input_channel(uint8_t channel);
        uint8_t get_channel() const {return m_channel;};
        uint8_t get_output_pin() const {return m_output_pin;};
        void disable(){digitalWrite(m_enable, LOW);};
        void enable(){digitalWrite(m_enable, HIGH);};
    private:
        uint8_t m_channel{0};
        const uint8_t m_enable;
        const uint8_t m_a0;
        const uint8_t m_a1;
        const uint8_t m_a2;
        const uint8_t m_output_pin = 9;
};


/// @brief Class that utilizes a MUX08 to read the input from 8 analog input channels,
/// and outputs the data as a value representing the voltage on the output pin, adjusted
/// for the voltage offset.
class CurrentMonitor
{
    public:
        // constructor with initialzation list. instantians m_mux as a Mux08 object with input pins 2,3,4 and output pin 9
        CurrentMonitor(const uint8_t a0, const uint8_t a1, const uint8_t a2, const uint8_t mux_output_pin, const uint8_t mux_enable_low, const uint8_t mux_enable_high) : m_mux_low(mux_enable_low, a0,a1,a2,mux_output_pin), m_mux_high(mux_enable_high, a0, a1, a2, mux_output_pin), m_mux_input_pin(mux_output_pin) {

        };
        float get_current(uint8_t channel);
    private:
        int read_channel(uint8_t channel);
        void disable_muxes();
        void select_channel(uint8_t channel);
        uint8_t m_current_channel{0};
        uint8_t m_mux_input_pin;
        Mux08 m_mux_low;
        Mux08 m_mux_high;
        uint8_t m_voltage_offset{0};
};

#endif
