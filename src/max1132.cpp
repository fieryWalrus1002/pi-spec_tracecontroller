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
 - connect DOUTpin to uC SPI
 - connect CS to pin defined in init()
 - connect SSTRB to pin defined in init()
 - 

 ------------------------------------------------------------------------------    
 Function list
 
  init(INT)*                  
  init()*                     
  read(channel, SD|DF)

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
*/

#include "Arduino.h"
#include "max1132.h"
#include "SPI.h"

bool ads_vref_int_enabled = 0;  // default voltage reference is external

// MAX1132::MAX1132(unsigned char _address){
//   ads_address = _address;   // Set ADC i2c address to the one passed to the function
// }

void MAX1132::init(){

  
  // calibrate the ADC
  // take the chip select low to select the device:
    digitalWrite(ADC_CS_PIN, LOW);

    // send the device the register you want to read:
    SPI.transfer(CALBYTE);
    
    // take the chip select high to de-select:
    digitalWrite(ADC_CS_PIN, HIGH);
  
}

uint16_t MAX1132::read()
{
    uint16_t reading;

    // take the chip select low to select the device:
    digitalWrite(ADC_CS_PIN, LOW);

    // send the device the acquistion command byte for unipolar conversion long
    SPI.transfer(READBYTE);
    reading = SPI.transfer(0x00);     		// receive low byte
    reading |= SPI.transfer(0x00) << 8;    // receive high byte
        
    // take the chip select high to de-select:
    digitalWrite(ADC_CS_PIN, HIGH);

    return reading; // return the full 16 bit reading from the ADC channel
}
  