#include <current_monitor.h>

void Mux08::select_input_channel(uint8_t channel){
    m_channel = channel;

    // Set the mux control pins to the appropriate values
    digitalWrite(m_a0, m_channel & 0x01);
    digitalWrite(m_a1, m_channel & 0x02);
    digitalWrite(m_a2, m_channel & 0x03);
}

int CurrentMonitor::read_channel(const uint8_t channel){

    if (channel == m_current_channel){return analogRead(m_mux_input_pin);} 
    else
    {
        m_current_channel = channel;
        disable_muxes();
        select_channel(channel);
        return analogRead(m_mux_input_pin);
    }
}

float CurrentMonitor::get_current(const uint8_t channel){
    // return read_channel(channel) * 0.0048828125;
    return read_channel(channel);
}

void CurrentMonitor::disable_muxes(){
    m_mux_low.disable();
    m_mux_high.disable();
}

void CurrentMonitor::select_channel(const uint8_t channel){
    // Select the input channel
    if (channel < 8){
        m_mux_low.select_input_channel(channel);
        m_mux_low.enable();
    }
    else{
        m_mux_high.select_input_channel(channel);
        m_mux_high.enable();
    }
}