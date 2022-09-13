#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "main.h"
#include "max1132.h"
#define OLED_RESET -1

elapsedMicros tLast;
long zeroTime = 0;
bool debugOut = false; // set if we do any serial output for debugging or not
bool measureState = false; // state for measurements
// I2C display
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);
MAX1132 adc;

// define pins
const int STO_FLASH_PIN = 0;
const int POWER_GATE = 33;
const int MCP_CS_PIN[] = {23, 22, 21};
const int ACT_GATE[] = {17, 16, 15};
const int SAT_PULSE_GATE = 38;
const int meas_led_array[] = {0, 2, 3, 4, 5, 6, 7, 8, 9}; // 0 will not flash anything

Point buffer[10000];

// MCP41XX is connected via SPI
// SCK: 13
// MOSI: 11
// CS: 21
//

// SSD1306 is connected via I2C bus
// SCL: 19
// SDA: 18

int incoming_byte = 0;
int counter = 0;
int sat_pulse_begin = 500;
int sat_pulse_end = 600;
unsigned int pulse_length = 50; // in uS
int post_trig_pulse_length = 0;
unsigned int pulse_interval = 1000; // in uS, so 1000 is 1ms between measurement pulses
int pulse_mode = 1; // 0 just actinic setting, 1 sat pulse, 2 single turnover
int meas_led_vis = 0;
int meas_led_ir = 0;
int num_points = 1000;
int trigger_delay = 15;
int numAq = 5; // number of acquisitions for the adc to perform and data to retrieve

// actinic stuff
/* Actinic intensity is given in a 0-255 range. We take this and calculate the proper
* value to give to our digital potentiometer. 
* For < 2000:
* int value = 8E-18x^5 - 3E-13x^4 + 3E-09x^3 - 2E-05x^2 + 0.047x + 120.52
* where x is desired light intensity
*
*/
int trace_phase = 0;
int act_int_phase[] = {0, 0, 0}; // holds the actinic intenisty values for phases 0-2 in uE
const int ACT_OFF_VAL = 0; // value for all lights off
const int ACT_SAT_VAL = 6000; // value for saturation pulse. Roughly 6000 uE
int zero_offset = 110; //  intercept for actinic poly fit function
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
    GOT_L,
    GOT_M,
    GOT_N,
    GOT_P,
    GOT_Q,
    GOT_R,
    GOT_S,
    GOT_T,
    GOT_V,
    GOT_W,
    GOT_X,
    GOT_Y,
    GOT_Z
} states;
states state = NONE;
int current_value;


////////////////////////////////////////////// functions //////////////////////////////////////////////////////////


void execute_trace()
{
    // pulse length calculation for after trigger, ie second half of pulse
    post_trig_pulse_length = pulse_length - trigger_delay;

    // set trace variables to initial values
    measureState = true;
    trace_phase = 0;
    counter = 0;
    zeroTime = micros();

    // let pispec know
    // Serial.println("trace_triggered;");
    set_act_intensity(act_int_phase[0], MCP_CS_PIN[0]);
    set_act_intensity(act_int_phase[1], MCP_CS_PIN[1]);
    set_act_intensity(act_int_phase[2], MCP_CS_PIN[2]);
}

void oledPrint(char* str){
    // visual feedback
    display.clearDisplay();
    display.setCursor(0, 15);
    display.print(str);
    display.display();
}

void set_act_intensity(int x, int mcp_cs_pin)
{
    // x is desired light intensity
    // mcpVal is the 16-bit value we send to MCP40101 digital potentiometer
    // it has an offset, and we want zero to be absolutely zero so we way undershoot it
    unsigned int mcpVal = 0;
    if (x > 0){
        mcpVal = 2E-05 * pow(x, 2) + 0.047 * x + zero_offset;
    }
    if (mcpVal > 255){
        mcpVal = 255;
    }
    
    if (debugOut == true){
    // serial output for debug
    Serial.print("ue: ");
    Serial.print(x);
    Serial.print(", act: ");
    Serial.println(mcpVal);
        // output to display
    display.clearDisplay();
    display.setCursor(0, 15);
    display.print("ue: ");
    display.print(x);
    display.print(", act: ");
    display.print(mcpVal);
    display.display();
    }

    write_act_intensity(mcpVal, mcp_cs_pin);
}

void write_act_intensity(int value, int mcp_cs_pin)
{
    digitalWrite(mcp_cs_pin, LOW);
    SPI.transfer(B00010001);
    SPI.transfer(value);
    digitalWrite(mcp_cs_pin, HIGH);
}

void calibrate_zero_offset(int value)
{
    // change the value that is subtracted from the MCP val sent to MCP40101 dig pot
    // this represents the value that the LEDs turn on at. At this value, there should
    // be no light at all. 
    zero_offset = value;
}

void set_num_points(const int value)
{
    counter = value + 1;
    num_points = value;
}

void set_pulse_interval(const int value)
{
    pulse_interval = value;
}

void set_vis_led(const int value)
{
    meas_led_vis = meas_led_array[value];
}

void set_ir_led(const int value)
{
    meas_led_ir = meas_led_array[value];
}

void set_pulse_length(const int value)
{
    pulse_length = value;
}

void set_sat_pulse_end(const int value)
{
    sat_pulse_end = value;
}

void set_sat_pulse_begin(const int value)
{
    sat_pulse_begin = value;
}

void set_phase_act_value(const int value, int phase_num)
{
    act_int_phase[phase_num] = value;
}

void meas_pulse_on()
{
    if (meas_led_vis != 0){
        digitalWrite(meas_led_vis, HIGH);
    }
    if (meas_led_ir != 0){
        digitalWrite(meas_led_ir, HIGH);
    }
}

void meas_pulse_off()
{
    if (meas_led_vis != 0){
        digitalWrite(meas_led_vis, LOW);
    }
    if (meas_led_ir != 0){
        digitalWrite(meas_led_ir, LOW);
    }
}

void handle_saturation_pulse(int pulse_mode, int trace_phase, int gate_state)
{
    // 0 for dirk/normal actinic value, 1 for sat pulse, 2 for st flash
    switch (pulse_mode)
    {
    case 0:
        // set_act_intensity(act_int_phase[trace_phase]);
        break;
    case 1:
        // set desired light intensity to a saturating value, say 6000 uE
        // set_act_intensity(ACT_SAT_VAL);
        digitalWrite(SAT_PULSE_GATE, gate_state);
        break;
    case 2:
        // single turnover flash, as quick as we can pulse it
        digitalWrite(SAT_PULSE_GATE, 1);
        delayMicroseconds(1);
        digitalWrite(SAT_PULSE_GATE, 0);
        break;
    default:
        break;
    }
}

void return_params()
{
    Serial.print("counter=");
    Serial.print(counter);
    Serial.print(", sat_pulse_begin=");
    Serial.print(sat_pulse_begin);
    Serial.print(", sat_pulse_end=");
    Serial.print(sat_pulse_end);
    Serial.print(", pulse_length=");
    Serial.print(pulse_length);
    Serial.print(", pulse_interval=");
    Serial.print(pulse_interval);
    Serial.print(", meas_led_vis=");
    Serial.print(meas_led_vis);
    Serial.print(", meas_led_ir=");
    Serial.print(meas_led_ir);
    Serial.print(", num_points=");
    Serial.print(num_points);
    Serial.print(", pulse_mode=");
    Serial.print(pulse_mode);
    Serial.print(", act_int_phase=[");
    Serial.print(act_int_phase[0]);
    Serial.print(",");
    Serial.print(act_int_phase[1]);
    Serial.print(",");
    Serial.print(act_int_phase[2]);
    Serial.println("];");
}

void pushData(){
    for (int i = 0; i <= num_points; i++){
        send_data_point(numAq, i);
    }
}

void toggleLedPower(int val){
    if (val > 0){
        digitalWrite(POWER_GATE, HIGH);
    } else {
        digitalWrite(POWER_GATE, LOW);
    }
}

void handle_action()
{
    // NONE, GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
    switch (state)
    {
    case GOT_A:
        set_act_intensity(current_value, MCP_CS_PIN[trace_phase]);
        break;
    case GOT_B:
        break;
    case GOT_C:
        calibrate_zero_offset(current_value);
        break;
    case GOT_D:
        return_params();
        break;
    case GOT_E:
        trigger_delay = current_value;
        break;
    case GOT_F:
        break;
    case GOT_G:
        pushData();
        break;
    case GOT_H:
        break;
    case GOT_I:
        set_pulse_interval(current_value);
        break;
    case GOT_L:
        break;
    case GOT_M:
        Serial.println("trace_begun");
        execute_trace();
        break;
    case GOT_N:
        set_num_points(current_value);
        break;
    case GOT_P:
        set_pulse_length(current_value);
        break;
    case GOT_Q:
        toggleLedPower(current_value);
        break;
    case GOT_R:
        set_ir_led(current_value);
        break;
    case GOT_S:
        set_sat_pulse_end(current_value);
        break;
    case GOT_T:
        set_sat_pulse_begin(current_value);
        break;
    case GOT_V:
        set_vis_led(current_value);
        break;
    case GOT_W:
        set_phase_act_value(current_value, 0);
        break;
    case GOT_X:
        set_phase_act_value(current_value, 1);
        break;
    case GOT_Y:
        set_phase_act_value(current_value, 2);
        break;
    case GOT_Z:
        pulse_mode = current_value;
        break;
    default:
        break;
    } // end of switch

    current_value = 0; // since we utilized the current_value above, now we reset it to zero for the next variable
    state = NONE;      // set the state to none, as we have used it
}

void process_inc_byte(const byte c)
{
    if (isdigit(c))
    {
        current_value *= 10;
        current_value += c - '0';
    } // end of digit
    else
    {
        // set the new state if we recognize it
        switch (c)
        {
        // GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
        case ';':
            handle_action();
            break;
        case 'a':
            state = GOT_A;
            break;
        case 'b':
            state = GOT_B;
            break;
        case 'c':
            state = GOT_C;
            break;
        case 'd':
            state = GOT_D;
            break;
        case 'e':
            state = GOT_E;
            break;
        case 'f':
            state = GOT_F;
            break;
        case 'g':
            state = GOT_G;
            break;
        case 'h':
            state = GOT_H;
            break;
        case 'i':
            state = GOT_I;
            break;
        case 'l':
            state = GOT_L;
            break;
        case 'm':
            state = GOT_M;
            break;
        case 'n':
            state = GOT_N;
            break;
        case 'p':
            state = GOT_P;
            break;
        case 'q':
            state = GOT_Q;
            break;
        case 'r':
            state = GOT_R;
            break;
        case 's':
            state = GOT_S;
            break;
        case 't':
            state = GOT_T;
            break;
        case 'v':
            state = GOT_V;
            break;
        case 'w':
            state = GOT_W;
            break;
        case 'x':
            state = GOT_X;
            break;
        case 'y':
            state = GOT_Y;
            break;
        case 'z':
            state = GOT_Z;
            break;
        default:
            state = NONE;
            break;
        } // end switch
    }     // end of not digit
}


void measurement_pulse()
{
    // once it triggers, it executes the following sequence of actions:
    // if this is a singleturnover flash trigger point, then we trigger the single turnover flash
    if (counter == sat_pulse_begin && pulse_mode == 2)
    {
        digitalWrite(STO_FLASH_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(STO_FLASH_PIN, LOW);
        delayMicroseconds(8);
    }
    if (counter == 0){
        zeroTime = micros();
    }

    // turn on measureing pulse leds
    meas_pulse_on();
    delayMicroseconds(trigger_delay); // wait to trigger ADC

    Point pnt = adc.read(1, zeroTime);

    while ((micros() - zeroTime)  < pulse_length){
        delayMicroseconds(1); // keep the pulse going for the full length
    }

    meas_pulse_off();

    // handle data storage in buffer
    buffer[counter] = pnt;
}

void init_digital_pins(){
    
    // initialze led output pins
    size_t n = sizeof(meas_led_array) / sizeof(meas_led_array[0]);
    
    for (unsigned int i = 1; i < n; i++)
    {
        pinMode(meas_led_array[i], OUTPUT);
        digitalWrite(meas_led_array[i], LOW);
    }
    // init other pins
    pinMode(ADC_CS_PIN, OUTPUT);
    pinMode(ADC_RST_PIN, OUTPUT);
    pinMode(ADC_SSTRB_PIN, INPUT); 
    pinMode(POWER_GATE, OUTPUT);
    pinMode(SAT_PULSE_GATE, OUTPUT);
    pinMode(MCP_CS_PIN[0], OUTPUT);
    pinMode(MCP_CS_PIN[1], OUTPUT);
    pinMode(MCP_CS_PIN[2], OUTPUT);
    pinMode(ACT_GATE[0], OUTPUT);
    pinMode(ACT_GATE[1], OUTPUT);
    pinMode(ACT_GATE[2], OUTPUT);
}

void cleanupTrace(){
    /// trace done, clean up LED lights
    if (debugOut == true){
        Serial.print("trace finished;");
    }

    //turn off all actinic gates
    for (int i = 0; i < 3; i++){
        digitalWrite(ACT_GATE[0], LOW);
    }

    size_t n = sizeof(meas_led_array) / sizeof(meas_led_array[0]);
    for (unsigned int i=1; i < n; i++)
    {
        digitalWrite(meas_led_array[i], LOW);
    }

    measureState = false;

    display.clearDisplay();
    display.setCursor(0, 15);
    display.print("trace_finished");
    display.display();

}

double getVoltage(uint16_t value){
    // 10.06 V / 55040 = 0.00018277
    double convFactor = 0.00018277;
    double v = value * convFactor;
    return v;
}

void setup()
{
    Serial.begin(115200);
    Serial.print("helllooooo");
    Serial.println(".");
    // initialize digital pins
    init_digital_pins();
        


    //Initialize display by providing the display type and its I2C address.
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setCursor(10, 15);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("init");
    display.display();
    Serial.println("display init");
    SPI.begin();
    Serial.println("spi begin");
    
    // set up MCP41xx digital pot
    digitalWrite(ADC_CS_PIN, HIGH);
    digitalWrite(ADC_RST_PIN, HIGH);
    adc.init();
    
    Serial.println("adc init");
    // digitalWrite(CS_PIN, HIGH);
    // pinMode(CS_PIN, OUTPUT);

    set_act_intensity(ACT_OFF_VAL, MCP_CS_PIN[trace_phase]);
    Serial.println("set act intensity");
    // set counter above threshold so it doesn't execute trace
    counter = num_points + 1;
    Serial.print("counter: ");
    Serial.println(counter);
    
    // visual feedback
    display.clearDisplay();
    display.setCursor(0, 15);
    display.print("ready");
    display.display();
    Serial.println("display ready");
}


void send_data_point(int numAq, int wrt_cnt)
{
    if (wrt_cnt >= num_points)
    {
        Serial.println(";");
    }
    else
    {
        Serial.print(wrt_cnt);
        Serial.print(",");
        Serial.print(buffer[wrt_cnt].time_us);
        
        for (int i = 0; i < numAq; i++){
            Serial.print(", ");    
            Serial.print(buffer[wrt_cnt].data[i]);
        }
        Serial.println(""),
        delayMicroseconds(4);
    }
}

void readAndDisplay(int numAq){
    // // visual feedback
    // adc.setNumAq(numAq);
    Point pnt = adc.read(numAq, micros());
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(pnt.time_us);
    
    for (int i = 0; i < numAq; i++){
        display.setCursor(20, i * 10);
        display.print(pnt.data[i]);
    }
    display.display();

   
    // display.clearDisplay();
    // display.setCursor(0, 0);
    // display.print("val: ");
    // display.print(reading);

    // display.setCursor(0, 10);
    // display.print("idx: ");
    // display.print(idx);

    // display.setCursor(0, 20);
    // display.print("V: ");
    // double voltage = getVoltage(reading);
    // display.print(voltage);

    // display.display();
}



void loop()
{   
    if (measureState == 0){
        readAndDisplay(4);
    }
    // static int idx = 0;
    // if (idx < 10000){
    //     idx++;
    // } else {
    //     idx = 0;
    // }

    // uint16_t reading = adc.read();
    // // reading = adc.read();
        


    // debug stuff
    

    while (Serial.available())
    {
        process_inc_byte(Serial.read());
    }

    if (counter <= num_points)
    {
        if (tLast > pulse_interval){
                // check saturation pulse time points
                if (counter == sat_pulse_begin)
                {
                    // transitiopn act value
                    digitalWrite(ACT_GATE[trace_phase], LOW);
                    trace_phase += 1;
                    digitalWrite(ACT_GATE[trace_phase], HIGH);
                    
                    handle_saturation_pulse(pulse_mode, trace_phase, 1);

                } else if (counter == sat_pulse_end)
                {
                    handle_saturation_pulse(pulse_mode, trace_phase, 0);

                    digitalWrite(ACT_GATE[trace_phase], LOW);
                    trace_phase += 1;
                    digitalWrite(ACT_GATE[trace_phase], HIGH);
                }

            // perform a measurement pulse
            tLast = 0;
            measurement_pulse();
            counter++;
        }
    }

    if (counter > num_points && measureState == true){
        cleanupTrace();
    }
}
