#include "main.h"
#include "pins.h"
#include "max1132.h"
#include "led.h"

LED_ARRAY leds;








#define OLED_RESET -1
uint8_t k_push_delay = 10; // delay between data point pushes in us
elapsedMicros tLast;
const int aqMode {1};
long zeroTime = 0;
bool measureState = false; // state for measurements
int traceNumber = 0; // the current trace number, index in traceData for current trace data to be placed
int current_act_intensity = 0;

MAX1132 adc;
Pins pins; // class to declare and initialize digital pins
long readInterval = 1000000; // time between display read updates for non measuring state fun measurmeents

int incoming_byte = 0;
int counter = 0;
int sat_pulse_begin = 500;
int sat_pulse_end = 600;
unsigned int pulse_length = 75; // in uS
int post_trig_pulse_length = 0;
unsigned int pulse_interval = 1000; // in uS, so 1000 is 1ms between measurement pulses
int pulse_mode = 1; // 0 just actinic setting, 1 sat pulse, 2 single turnover
int meas_led_num = 0;
int act_led_num = 4;
int num_points = 1000;
int trigger_delay = 35;
bool power_state = false; // status of the power switch

int trace_phase = 0;
uint8_t act_int_phase[] = {0, 0, 0}; // holds the actinic intenisty values for phases 0-2 in uE

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
    GOT_J,
    GOT_K,
    GOT_L,
    GOT_M,
    GOT_N,
    GOT_O,
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
    handle_act_phase(trace_phase);
}

void handle_act_phase(int trace_num){
    uint8_t desired_value = act_int_phase[trace_num];
    if (desired_value == 0){
        leds.set_intensity(act_led_num, 0, 1);
        leds.change_led_state(act_led_num, 0);
    }
    else{
        leds.set_intensity(act_led_num, desired_value, 1 );
        leds.change_led_state(act_led_num, 1);
    }
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
    meas_led_num = pins.meas_led_array[value];
    send_response("meas_led_num", meas_led_num);
}

void set_pulse_length(const int value)
{
    pulse_length = value;    
    send_response("pulse_length", pulse_length);
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

int get_numPreAq()
{
    int result = MAX_AQ / 3;
    return(result);
}

int get_numAq()
{
    int result = MAX_AQ - (MAX_AQ / 3);
    return result;
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
        digitalWrite(pins.STO_FLASH_PIN, 1);
        delayMicroseconds(1);
        digitalWrite(pins.STO_FLASH_PIN, 0);
        break;
    default:
        break;
    }
}

void return_params()
{
    Serial.print("counter=");
    Serial.print(counter);
    Serial.print(",max_aq=");
    Serial.print(MAX_AQ);
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
    Serial.print(", meas_led_num=");
    Serial.print(meas_led_num);
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
        delayMicroseconds(k_push_delay);
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

        for (int i = 0; i < MAX_AQ; i++){
            Serial.print(", ");    
            Serial.print(traceData[trace].data[wrt_cnt].aq[i]);
        }
        Serial.println("");
    }
}

void set_12v_power(int val){
    if (val >= 1){
        // driving the relay switch low closes the switch
        digitalWrite(pins.POWER_GATE_PIN, LOW);
        power_state = false;
    }
    else
    {
        digitalWrite(pins.POWER_GATE_PIN, HIGH);
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

void set_debug(){
    if (DEBUG_MODE == false){DEBUG_MODE = true;}else {DEBUG_MODE = false;};
    send_response("DEBUG_MODE", DEBUG_MODE);
}

void handle_action()
{
    switch (state)
    {
    case GOT_B:
        break;
    case GOT_C:
        pinTest(current_value);
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
        pushData(0);
        break;
    case GOT_H:
        break;
    case GOT_I:
        set_pulse_interval(current_value);
        break;
    case GOT_K:
        set_12v_power(current_value);
        break;
    case GOT_L:
        break;
    case GOT_M:
        execute_trace();
        break;
    case GOT_N:
        set_num_points(current_value);
        break;
    case GOT_O:
        set_debug();
        break;
    case GOT_P:
        set_pulse_length(current_value);
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
        switch (c)
        {
        case '?':
            state = GOT_O;
            break;
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
        case 'o':
            state = GOT_O;
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

void measurement_pulse(TraceBuffer *buffer, int meas_led)
{
    Point pnt;
    pnt.time_us[0] = micros() - zeroTime;

    for (int i = 0; i < adc.preaq; i++){
        pnt.aq[i] = adc.read();   
    }

    pnt.time_us[1] = micros() - zeroTime;

    leds.change_led_state(meas_led, 1);

    for (int i = adc.preaq; i < adc.aq + adc.preaq; i++){
        pnt.aq[i] = adc.read();   
    }

    pnt.time_us[2] = micros() - zeroTime;

    while ((micros() - pnt.time_us[0])  < pulse_length){
        delayMicroseconds(1);
    }

    leds.change_led_state(meas_led, 0);

    buffer->data[counter] = pnt;
}

void cleanupTrace(){
    pins.meas_led_cleanup();
    measureState = false;
    trace_phase = 0;
}

void setup()
{
    Serial.begin(115200);
    pins.init();
    leds.add(LED("520", 5, 6, 7, 1000, 1000, 1000));
    leds.add(LED("545", 8, 9, 10, 1000, 1000, 1000));
    leds.add(LED("554", 11, 12, 13, 1000, 1000, 1000));
    leds.add(LED("563", 14, 15, 16, 1000, 1000, 1000));
    leds.add(LED("563", 14, 15, 16, 1000, 1000, 1000));
    leds.add(LED("572", 17, 18, 19, 1000, 1000, 1000));
    leds.add(LED("630", 2, 3, 4, 1000, 1000, 1000));
    leds.add(LED("740", 23, 24, 25, 1000, 1000, 1000));
    leds.add(LED("810", 20, 21, 22, 1000, 1000, 1000));
    leds.add(LED("940", 23, 24, 25, 1000, 1000, 1000));
    adc.init(MAX_AQ);
    counter = num_points + 1;
}

void pinTest(int pinNumber){
    for (int i = 0; i < num_points; i++){
        digitalWrite(pinNumber, HIGH);
        delayMicroseconds(pulse_length);
        digitalWrite(pinNumber, LOW);
        delayMicroseconds(pulse_interval);
    }
}

void loop()
{   
    while (Serial.available())
    {
        process_inc_byte(Serial.read());
    }

    if (counter <= num_points)
    {
        if (tLast > pulse_interval){
                if ((counter == sat_pulse_begin) | (counter == sat_pulse_end))
                {
                    trace_phase++;
                    handle_act_phase(trace_phase);
                }

            tLast = 0;
            measurement_pulse(ptr_buffer, meas_led_num);
            counter++;
        }
    }

    if (counter > num_points && measureState == true){
        cleanupTrace();
    }
}
