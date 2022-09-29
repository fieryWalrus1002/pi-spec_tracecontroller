#include "main.h"

#define OLED_RESET -1

elapsedMicros tLast;
long zeroTime = 0;
bool debugOut = false; // set if we do any serial output for debugging or not
bool measureState = false; // state for measurements
bool aqFlag = false; // flag indication of adc ready for new reading
int traceNumber = 0; // the current trace number, index in traceData for current trace data to be placed
int current_act_intensity = 0;
// I2C display
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);
MAX1132 adc;


long readInterval = 250000; // time between display read updates for non measuring state fun measurmeents



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
int numAq = 6;
int numPreAq = 3;
bool power_state = false; // status of the power switch

// actinic stuff
/* Actinic intensity is given in a 0-255 range. We take this and calculate the proper
* value to give to our digital potentiometer. 
* For < 2000:
* int value = 8E-18x^5 - 3E-13x^4 + 3E-09x^3 - 2E-05x^2 + 0.047x + 120.52
* where x is desired light intensity
*
*/
int trace_phase = 0;
uint8_t act_int_phase[] = {0, 0, 0}; // holds the actinic intenisty values for phases 0-2 in uE
const int8_t ACT_OFF_VAL = 0; // value for all lights off
const int8_t ACT_SAT_VAL = 140; // value for saturation pulse

typedef enum
{
    NONE,
    GOT_A,
    GOT_B,
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
    GOT_P,
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
int current_value;


////////////////////////////////////////////// functions //////////////////////////////////////////////////////////


void execute_trace()
{
    // set trace variables to initial values
    measureState = true;
    trace_phase = 0;
    counter = 0;
    zeroTime = micros();
    ptr_buffer = &traceData[traceNumber];
    


}

void handle_act_phase(int trace_num){
    if (act_int_phase[trace_num] > 0){
        write_act_intensity(act_int_phase[trace_num]);
        switch_act_gate(1);
    } else
    {
        switch_act_gate(0);
    }
    
}

void oledPrint(char* str){
    // visual feedback
    display.clearDisplay();
    display.setCursor(0, 15);
    display.print(str);
    display.display();
}

// int getMcpVal(int x){
//     // take a desired light intensity in uE, and calculate the the 16-bit value 
//     // that will be send to the MCP40101 digital potentiometer
//     // The zero_offset global variable is used to bring it to a real zero at x=0.

//     unsigned int mcpVal = 0;

//     if (x > 0){
//         mcpVal = 2E-05 * pow(x, 2) + 0.047 * x + zero_offset;
//     }
//     if (mcpVal > 255){
//         mcpVal = 255;
//     }
//     return mcpVal;
// }


void write_act_intensity(int value)
{
    // sets the current act intensity by changing MCP resistance 0-255/0-10k Ohm
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(B00010001);
    SPI.transfer(value);
    digitalWrite(MCP_CS_PIN, HIGH);
    current_act_intensity = value;
}

void set_num_points(const int value)
{
    counter = value + 1;
    num_points = value;
    send_response("num_points",num_points);
}

void set_pulse_interval(const int value)
{
    pulse_interval = value;
    send_response("pulse_interval",pulse_interval);
}

void set_vis_led(const int value)
{
    meas_led_vis = meas_led_array[value];
    send_response("meas_led_vis",meas_led_array[value]);
}

void set_ir_led(const int value)
{
    meas_led_ir = meas_led_array[value];
    send_response("meas_led_ir",meas_led_array[value]);
}

void set_pulse_length(const int value)
{
    pulse_length = value;    
    send_response("pulse_length", pulse_length)
}

void set_sat_pulse_end(const int value)
{
    sat_pulse_end = value;
    send_response("sat_pulse_end", sat_pulse_end);
}

void set_sat_pulse_begin(const int value)
{
    sat_pulse_begin = value;
    send_response("sat_pulse_begun", sat_pulse_begin);
}

void set_phase_act_value(const int value, int phase_num)
{
    act_int_phase[phase_num] = value;
    send_response("act_int_phase", act_int_phase[phase_num]);
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

void handle_saturation_pulse(int pulse_mode, int trace_phase)
{
    // 0 for dirk/normal actinic value, 1 for sat pulse, 2 for st flash
    switch (pulse_mode)
    {
    case 0:
        // set_act_intensity(act_int_phase[trace_phase]);
        break;
    case 1:
        handle_act_phase(trace_phase);
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
    Serial.print(",numAq=");
    Serial.print(numAq);
    Serial.print(",numPreAq=");
    Serial.print(numPreAq);
    Serial.print(",power_state=");
    Serial.print(power_state);
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
    Serial.print("|");
    Serial.print(act_int_phase[1]);
    Serial.print("|");
    Serial.print(act_int_phase[2]);
    Serial.print("]");
    Serial.print(", trigger_delay=");
    Serial.print(trigger_delay);
    
    Serial.println(";");

}

void pushData(int whichTraceBuffer){
    for (int i = 0; i <= num_points; i++){
        send_data_point(i, whichTraceBuffer);
    }
}

void send_data_point(int wrt_cnt, int trace)
{
    
    if (wrt_cnt >= num_points)
    {
        Serial.println(";");
    }
    else
    {
        Serial.print(wrt_cnt);
        Serial.print(",");
        Serial.print(traceData[0].data[wrt_cnt].time_us[0]);
        // Serial.print(",");
        // Serial.print(traceData[0].data[wrt_cnt].time_us[1]);
        
        for (int i = 0; i < (numPreAq + numAq); i++){
            Serial.print(", ");    
            Serial.print(traceData[trace].data[wrt_cnt].aq[i]);
        }
        Serial.println(""),
        delayMicroseconds(1);
    }
}

void switch_act_gate(int state){
    digitalWrite(ACT_GATE_PIN, state);
}

void switch_detector_circuit(int val)
{
    digitalWrite(DETECTOR_GATE_PIN, val);
}

void set_12v_power(int val){
    if (val >= 1){
        // driving the relay switch low closes the switch
        digitalWrite(POWER_GATE_PIN, LOW);
        power_state = false;
    }
    else
    {
        digitalWrite(POWER_GATE_PIN, HIGH);
        power_state = true;
    }
    send_response("power_state", power_state);
    
}

void send_response(auto respcode, auto val){
    /* send a chararacter array and a value back across serial to acknowledge receipt
        of the command. 
    */
    if (DEBUG_MODE == true){
        Serial.print(respcode);
        Serial.print(":");
        Serial.print(val);
        Serial.println(";");
    }
    
}


void handle_action()
{
    // NONE, GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
    switch (state)
    {
    case GOT_A:
        write_act_intensity(current_value);
        break;
    case GOT_B:
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
        // not ready for mulitple buffers yet
        // pushData(current_value);
        pushData(0);
        break;
    case GOT_H:
        break;
    case GOT_I:
        set_pulse_interval(current_value);
        break;
    case GOT_J:
        switch_detector_circuit(current_value);
        break;
    case GOT_K:
        set_12v_power(current_value);
        break;
    case GOT_L:
        break;
    case GOT_M:
        // Serial.println("trace_begun");
        execute_trace();
        break;
    case GOT_N:
        set_num_points(current_value);
        break;
    case GOT_P:
        set_pulse_length(current_value);
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
    case GOT_U:
        switch_act_gate(current_value);
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
        case 'j':
            state = GOT_J;
            break;
        case 'k':
            state = GOT_K;
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
        case 'r':
            state = GOT_R;
            break;
        case 's':
            state = GOT_S;
            break;
        case 't':
            state = GOT_T;
            break;
        case 'u':
            state = GOT_U;
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
void adcInterrupt(){
    aqFlag = true;
}


void measurement_pulse(int numAq, int numPreAq, TraceBuffer *buffer)
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

    Point pnt;
    pnt.time_us[0] = micros() - zeroTime;

    // collect pre-pulse values
    for (int i = 0; i < numPreAq; i++){

        pnt.aq[i] = adc.read();   
    }

    // turn on measureing pulse leds
    meas_pulse_on();
    // activate interrupt
    // attachInterrupt(digitalPinToInterrupt(ADC_SSTRB_PIN), adcInterrupt, RISING);
    
    delayMicroseconds(trigger_delay); // wait to trigger ADC
    for (int i = numPreAq; i < numAq + numPreAq; i++){
        pnt.aq[i] = adc.read();   
    }

    
    pnt.time_us[1] = micros() - zeroTime;

    while ((micros() - pnt.time_us[0])  < pulse_length){
        delayMicroseconds(1); // keep the pulse going for the full length
    }

    meas_pulse_off();

    buffer->data[counter] = pnt;
}

void init_digital_pins(){
    
    // initialze led output pins
    int n = int(sizeof(meas_led_array) / sizeof(meas_led_array[0]));
    for (int i = 0; i < n; i++){
        pinMode(meas_led_array[i], OUTPUT);
        digitalWrite(meas_led_array[i], LOW);
    }

    int m = int(sizeof(pin_setup_array) / sizeof(pin_setup_array[0]));
    for (int i = 0; i < m; i++){
        pinMode(pin_setup_array[i], OUTPUT);
        digitalWrite(pin_setup_array[i], LOW);
    }

}

void cleanupTrace(){
    /// trace done, clean up LED lights

    //turn off actinic gate
    switch_act_gate(0);
    write_act_intensity(0);

    size_t n = sizeof(meas_led_array) / sizeof(meas_led_array[0]);
    
    for (unsigned int i=1; i < n; i++)
    {
        digitalWrite(meas_led_array[i], LOW);
    }

    measureState = false;
    trace_phase = 0;
}

void setup()
{
    Serial.begin(115200);
    // initialize digital pins
    init_digital_pins();
        
    //Initialize display by providing the display type and its I2C address.
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setCursor(10, 15);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("init");
    display.display();
    SPI.begin();
    
    // set up MCP41xx digital pot
    adc.init();


    write_act_intensity(ACT_OFF_VAL);
    
    // set counter above threshold so it doesn't execute trace
    counter = num_points + 1;
    
    // visual feedback
    display.clearDisplay();
    display.setCursor(0, 15);
    display.print("ready");
    display.display();
}

double toVoltage(double val){
    return val / 65536.0 * 10;
}

void getKMeasurements(int k, Point* pnt){
        pnt->time_us[0] = micros();

        for (int i = 0; i < numAq; i++){
            pnt->aq[i] = adc.read();
        }

        pnt->time_us[1] = micros() - pnt->time_us[0];
}

double getMeanMeasVal(int k, Point* pnt){
    // mean Voltage
    double sum{0};

    for (int i = 0; i < numAq; i++){
        sum += pnt->aq[i];
    }
    
    return sum / static_cast<double>(k);
}

double getStdDev(int k, Point* pnt, double meanVal){
    // calculate std dev from 
    double ss = 0;
    for (int i = 0; i < numAq; i++){
        ss += sq(pnt->aq[i] - meanVal);
    }
    return sqrt(ss / numAq);
}

void displayKMeasResults(long time_us, double meanVal, double sd){
    // Format and display results of k measurements on ssd1306
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("/\t: ");
        display.print(time_us);
        display.setCursor(0, 8);
        display.print("meas: ");
        display.print(meanVal);
        display.setCursor(0, 16);
        display.print("sd: ");
        display.print(sd);
        display.display();
}

void readAndDisplayKValues(int k, long interval){
    static long readLast = 0;
    long timeSinceLastRead = micros() - readLast;

    if (timeSinceLastRead > interval){
        
        Point pnt;

        getKMeasurements(numAq, &pnt);
        
        double meanVal = getMeanMeasVal(k, &pnt);

        double sd = getStdDev(k, &pnt, meanVal);

        displayKMeasResults(pnt.time_us[1], meanVal, sd);
 
        readLast = micros();
    }
}



void loop()
{   
    while (Serial.available())
    {
        process_inc_byte(Serial.read());
    }

    if (measureState == 0){
        readAndDisplayKValues(numAq, readInterval);
    }

    if (counter <= num_points)
    {
        if (tLast > pulse_interval){
                // check saturation pulse time points
                if (counter == sat_pulse_begin)
                {
                    // transitiopn act value
                    trace_phase = 1;
                    handle_saturation_pulse(pulse_mode, trace_phase);

                } else if (counter == sat_pulse_end)
                {
                    trace_phase = 2;
                    handle_saturation_pulse(pulse_mode, trace_phase);
                    
                }

            // perform a measurement pulse
            tLast = 0;
            measurement_pulse(numAq, numPreAq, ptr_buffer);
            counter++;
        }
    }

    if (counter > num_points && measureState == true){
        cleanupTrace();
    }
}
