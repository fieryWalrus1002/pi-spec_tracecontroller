/*
 Library for Maxim MAX1132BCAP ADC
 Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX1132-MAX1133.pdf
 ------------------------------------------------------------------------------
 This is a basic library to use the MAX1132BCAP ADC. Unipolar only for now, external
 reference. 
 
 ------------------------------------------------------------------------------    
 Chip Wiring
 All connections necessary unless specifically mentionned.
 
 PIN Name Description
 1 - REF - ADC ref input. Bypass to AGND with 0.22 uF. 
 2 - REFADJ - For external ref, bypass to AVdd.
 3 - AGND - Analog ground
 4 - AVdd - Analog supply, 5V. Bypass to AGND with 0.1 uF cap.
 5 - DGND - digital ground
 6 - SHDN - Drive low to put ADC in shutdown mode.
 7 - P2 - User defined output 2
 8 - P1 - User defined output 1
 9 - P0 - User defined output 0
 10 - SSTRB - Serial strobe out. Internal clock, goes low when ADC begins conversion, high when finished.
 11 - DOUT - Serial Data output. MSB first, straight binary for unipolar. each bit clocked out at falling edge of SCLK
 12 - RST - Reset input. Drive low to put device in power on default mode. 
 13 - SCLK - Serial data clock input. 
 14 - DGND. Connect to pin 5. 
 15 - DVdd Digital 5v supply. Bypass DVdd to pin 14 w/ 0.1 uF cap
 16 - DIN - Serial data input. Data latched on rising edge of SCLK.
 17 - CS - Chip select. Drive CS LOW to enable serial interface. 
 18 - CREF - Reference buffer bypass. Bypass CREF to AGND (pin3) with 1 uF cap.
 19 - AGND - Analog Ground. Connect pin 19 to pin 3. 
 20 - AIN - Analog input. 

 - Connect AGND pin 3 to Ground
 - Connect DGND pin 5 to Ground
 - Connect +AVdd pin 4 to 5v
 - Connect SCLK to pin uC SPI
 - connect DIN pin to  uC SPI
 - connect DOUT pin to uC SPI
 - connect CS to pin defined in init()
 - connect SSTRB to pin defined in init()
 - 

 ------------------------------------------------------------------------------    
 Function list
 in header

 ------------------------------------------------------------------------------    



bit 7 = 1; first logic bit, after CS goes low, defines beginning of control byte
bit 6 = 1; 1 = unipolar, 0 = bipolar. 0 - +12 V
bit 5 = 0;  1 for internal clock, 0 for external clock
bit 4 = M1;
bit 3 = M0;  Different combinations mean different things. See below.
bit 2 = 0; p2 state
bit 1 = 0; p1 state
bit 0 = 0; p0 state

M1/M0
00: 24 ext clk per conversion (short acquisition mode)
01: start internal calibration, do on startup.
10: software power-down mode. 
11: 32 external clocks per conversion (long acquisition mode)


example: 
const byte CALBYTE = 0xC8;// calibration control byte

calbye, 200U
11001000
7=1, control bit
6=1, unipolar
5=0, internal clock 
4,3=01, start internal calibration
2=unused
1=unused
0=unused

readbyte, 216
11011000
7=1, control bit
6=1, unipolar
5=0, internal clock 
4l,3 = 11, 32 External clocks per conversion, long aq mode
2,1,0=0, unused


*/

#include "Arduino.h"
#include "max1132.h"
#include "SPI.h"

bool ads_vref_int_enabled = 0;  // default voltage reference is external

// MAX1132::MAX1132(unsigned char _address){
//   ads_address = _address;   // Set ADC i2c address to the one passed to the function
// }

void MAX1132::init_pins(){
    pinMode(m_cs_pin, OUTPUT);
    pinMode(m_rst_pin, OUTPUT);
    pinMode(m_sstrb_pin, INPUT);
    digitalWrite(m_cs_pin, HIGH);
    digitalWrite(m_rst_pin, HIGH); 
}

void MAX1132::calibrate(){
    SPI.beginTransaction(adcSettings);
  // take the chip select low to select the device:
    digitalWrite(m_cs_pin, LOW);

    //set up for unipolar
    SPI.transfer(m_readbyte);

    // send the calibration byte:
    SPI.transfer(m_calbyte);
    
    // take the chip select high to de-select:
    digitalWrite(m_cs_pin, HIGH);
    SPI.endTransaction();
}

void MAX1132::set_aquisition_points(uint8_t max_aq){
    m_max_aq = max_aq;
    m_aq = m_max_aq / 3;
    m_preaq = m_max_aq - m_aq;
}

MAX1132::MAX1132(uint8_t max_aq, uint8_t cs_pin, uint8_t rst_pin, uint8_t sstrb_pin){
    m_cs_pin = cs_pin;
    m_rst_pin = rst_pin;
    m_sstrb_pin = sstrb_pin;
    SPI.begin();
    set_aquisition_points(max_aq);
    init_pins();  // set pins to appropriate modes and values
    calibrate(); // send the calibration byte
    // send the device the acquistion command byte for unipolar conversion long
    
}

uint16_t MAX1132::read()
{
    int16_t reading{0}; //
    // // take the chip select low to select the device:
    digitalWrite(m_cs_pin, LOW);
    
    SPI1.beginTransaction(adcSettings);
    
    // // send the read byte:
    SPI1.transfer(m_readbyte);

    // // // discard first reading
    SPI1.transfer(m_transfer_byte);

    int16_t highByte = SPI1.transfer(m_transfer_byte);
    int16_t lowByte = SPI1.transfer(m_transfer_byte);

    reading |= highByte << 8;
    reading |= lowByte;

    digitalWrite(m_cs_pin, HIGH);
    SPI1.endTransaction();

    return reading;
}