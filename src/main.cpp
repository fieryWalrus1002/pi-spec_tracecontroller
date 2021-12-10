#define btn 7

#define REFRACT_PRD 10000
#include <Arduino.h>

////////////////////////////////// input state machine ////////////////////////////////////////
// adapted from http://gammon.com.au/serial state machine example
// the states of our input state-machine
typedef enum
{
    NONE,
    GOT_A,
    GOT_C,
    GOT_D,
    GOT_G,
    GOT_H,
    GOT_I,
    GOT_L,
    GOT_M,
    GOT_N,
    GOT_P,
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

/////////////////////////////////// trace variables /////////////////////////////////////////////
// trace variables that are modified by experimentControl
// format for trace values will be something like this:
// testing d n6000 i248 g0 h0 v1 r5 p40 a128 s500 t900 w1 x2 y4 z1 m
// this would define a measurement trace with
// n6000 = 6000 data points,
// i248 = pulse_interval of 248 us,
// g0 = gain_vis = 0,
// h0 = gain_ir = 0,
// v1 = meas_led_vis = 1,
// r5 = meas_led_ir = 5,
// p40 = pulse_length = 40us
// a128 = actininc light intensity = 128, this turns on the actinic light during measure_state = false, not during a trace
// s500 = sat_pulse_length = 500 in milliseconds
// t900 = sat_pulse_begin = 900
// w1 = act_int_phase[0] = 1/255
// x2 = act_int_phase[1] = 2/255
// y4 = act_int_phase[2] = 4/255
// z1 = pulse_mode = 1; will trigger a saturation pulse. 0 is dirk. 2 is single-turnover flash.
// m would then execute the trace by changing measure_state from false to true, which would stop listening for the duration of the trace

int num_points = 1000;              // number of times that measurement pulse will be activated, set by n
unsigned long pulse_interval = 248; // pulse interval in time units chosen, set by i
int gain_vis = 0;                   // gain setting, from 0-7. int value will be interpreted to a binary output to detector by switch case, set by g
int gain_ir = 0;                    // gain setting, from 0-7. int value will be interpreted to a binary output to detector by switch case, set by h
int meas_led_vis = 0;               // holds the green led pin number, set by v
int meas_led_ir = 0;                // holds the IR led pin number, set by r
int pulse_length = 25;              // the length that the measurement LEDs will be on during a measurement pulse in microseconds

int sat_pulse_begin = 100;         // the data point at which the sat pulse will trigger
int sat_pulse_length = 50;         // length in points for the saturation pulse
int sat_pulse_end;                 // calculated in trace: sat_pulse_end = sat_pulse_begin + sat_pulse_length
int act_int_phase[] = {10, 0, 10}; // holds the actinic intenisty values for phases 0-2
int pulse_mode = 1;                // 0 is normal actinic/dirk, 1 is a sat pulse, 0, 2 is a single turnover flash

//////////////////////////// internal variables not modified externally /////////////////////////////////////////////////////////
long prev_time = 0;

int cycles = 10; // count the number of cycles between two button presses
int counter = 0; // counter that will be used to count the number of measurement points and trigger the end of measure_state
bool measure_state = false;
int trace_phase = 0; // trace phase will be 0 for before sat pulse, 1 for sat pulse, 2 for post sat pulse.
int trace_length = 0;

long trace_begin_time;
int test_value = 0; // just an int for doing test feedback stuff

// 7-20-21 are we using this? changing how sat pulse triggers, not in phases but point based for consistency
// long sat_trigger_time = 0;
// int sat_pulse_trigger_time = 0;

//////////////////////////////////// board specific pin assignments ////////////////////////////////////////////////////////////
// using the Cerebot MX3ck board instead of UNO, so output pins will be for that board

// cerebot JA port, used for status indicator LEDs
// upper row, looking at it is 3.3V, GND, 3, 2, 1, 0
// lower row, looking at it is 3.3V, GND, 7, 6, 5, 4
// Low power LEDs on front of case used for status indication.
int ledArray[] = {0, 1, 2, 3, 4, 5, 6, 7};

// cerebot JB port, used for output to the head
// upper row, looking at it is 3.3V, GND, 11, 10, 9, 8
// lower row, looking at it is 3.3V, GND, 15, 14, 13, 12
// there are LEDs hooked up to this port. HIGH signal will turn on the optical isolators and turn on the LEDs

// cerebot JC port, used for actinic board inputs to set intensity of the actinic light
// upper row, looking at it is 3.3V, GND, 19, 18, 17, 16
// lower row, looking at it is 3.3V, GND, 23, 22, 21, 20
int act_int_pins[] = {16, 17, 18, 19, 20, 21, 22, 23};

// cerebot JD port, used for output to detector for SH, output to IR detector gain and vis detector gain
// upper row, looking at it is 3.3V, GND, 27, 26, 25, 24
// lower row, looking at it is 3.3V, GND, 31, 30, 29, 28
int adc_pin = 31; // the adc_tri pinout, will trigger the adc when it goes HIGH as a pseudo clock

// cerebot JE port, used for output to ADC and to the actinic board to trigger saturation pulse and the single turnover flash
// upper row, looking at it is 3.3V, GND, 35 34 33 32
// lower row, looking at it is 3.3V, GND, 39 38 37 36
int stoFlash_pin = 35;
int blueAct_pin = 27;
int satPulse_pin = 26;
int farRed_pin = 25;
int powerLED = 39; // will be HIGH whenever the board has power

////////////////////////////////////////////// functions //////////////////////////////////////////////////////////
void measurement_pulse()
{
    // once it triggers, it executes the following sequence of actions:
    // if this is a singleturnover flash trigger point, then we trigger the single turnover flash
    if (counter == sat_pulse_begin && pulse_mode == 2)
    {
        digitalWrite(stoFlash_pin, HIGH);
        delayMicroseconds(1);
        digitalWrite(stoFlash_pin, LOW);
        delayMicroseconds(8);
    }

    digitalWrite(meas_led_vis, HIGH); // set green LED to HIGH
    digitalWrite(meas_led_ir, HIGH);  // set IR LED to HIGH
    digitalWrite(adc_pin, HIGH);      // turn on trigger for ADC conversion
    delayMicroseconds(pulse_length);  // wait the rest of the pulse width
    digitalWrite(adc_pin, LOW);       // adc trigger off
    digitalWrite(meas_led_vis, LOW);  // set green LED to LOW
    digitalWrite(meas_led_ir, LOW);   // set IR LED to LOW
}

//  1 <<  0  ==    1
//  1 <<  1  ==    2
//  1 <<  2  ==    4
//  ...
//  1 <<  8  ==  256
// length of actinic light period is set in the python experimentControl, not here. We just turn it on and off.
void setActIntensity(byte value, int intensity_array[])
{
    for (int i = 0, mask = 1; i < 8; i++, mask = mask << 1)
    {
        if (value & mask)
        {
            // bit is on
            digitalWrite(intensity_array[i], HIGH);
        }
        else
        {
            // bit is off
            digitalWrite(intensity_array[i], LOW);
        }
    }
    // Serial.print("actinic intensity is now: ");
    // // Serial.println(value & 255);
}

void checkLED(byte value)
{
    // Serial.print("pulsing LED#: ");
    // // Serial.println(value);
    int test_LED = ledArray[value];

    for (int i = 0; i < 5000; i++)
    {
        digitalWrite(test_LED, HIGH);
        delayMicroseconds(100); // LED pulse is set by the variable pulse_length
        digitalWrite(test_LED, LOW);
        delayMicroseconds(300);
    }

    // // Serial.println("LED check finished.");
}

int set_gain(byte value) {}

void setupPins(int lastPin)
{
    for (int pinNum = 0; pinNum <= lastPin; pinNum++)
    {
        pinMode(pinNum, OUTPUT);
        digitalWrite(pinNum, LOW);
    }
}

void setNumPoints(const int value)
{
    num_points = value;
    // Serial.print("num_points = ");
    // // Serial.println(num_points);
}

void setPulseInterval(const int value)
{
    pulse_interval = value;
    // Serial.print("pulse_interval = ");
    // // Serial.println(pulse_interval);
}

void setVisLED(const int value)
{
    // meas_led_vis = value;
    //  Serial.print("value == ");
    //  // Serial.println(value);
    meas_led_vis = ledArray[value];
    // Serial.print("meas_led_vis = ");
    // // Serial.println(meas_led_vis);
}

void setIRLED(const int value)
{
    meas_led_ir = value;
    // Serial.print("meas_led_ir = ");
    // // Serial.println(meas_led_ir);
}

void setPulseLength(const int value)
{
    pulse_length = value;
    // Serial.print("pulse_length = ");
    // // Serial.println(pulse_length);
}

void setSatPulseLength(const int value)
{
    sat_pulse_length = value;
    // Serial.print("sat_pulse_length (ms) = ");
    // // Serial.println(sat_pulse_length);
}

void setSatTriggerPoint(const int value)
{
    sat_pulse_begin = value;
    // Serial.print("sat_pulse_begin = ");
    // // Serial.println(sat_pulse_begin);
}

void executeTrace()
{
    measure_state = true;
    // // Serial.println("Start measurement");
}

void sendStatus()
{
    //  // // Serial.println("Arduino traceController is listening.");
    // Serial.print("num_points = ");
    // // Serial.println(num_points);

    // Serial.print("sat_pulse_begin = ");
    // // Serial.println(sat_pulse_begin);

    // Serial.print("sat_pulse_length = ");
    // // Serial.println(sat_pulse_length);
}

void setPhaseActValue(const int value, int phase_num)
{
    // act_int_phase[]act_int_phase
    act_int_phase[phase_num] = value;
    // Serial.print("phase[");
    // Serial.print(phase_num);
    // Serial.print("] actinic intensity = ");
    // // Serial.println(act_int_phase[phase_num]);
} // end setPhaseActValue

void respondState()
{

    // if (measure_state == false)
    // {
    //     // // Serial.println("listening");

    //     for (int i = 0; i < 6; i++)
    //     {
    //         digitalWrite(listenLED, LOW);
    //         delay(150);
    //         digitalWrite(listenLED, HIGH);
    //         delay(150);
    //     }
    // }
}

void handle_action()
{
    // NONE, GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
    switch (state)
    {
    case GOT_A:
        setActIntensity(current_value, act_int_pins);
        break;
    case GOT_C:
        checkLED(current_value);
        break;
    case GOT_D:
        // return diagnostic info
        sendStatus();
        break;
    case GOT_G:
        set_gain(current_value);
        break;
    case GOT_H:
        set_gain(current_value);
        break;
    case GOT_I:
        setPulseInterval(current_value);
        break;
    case GOT_L:
        respondState();
        break;
    case GOT_M:
        executeTrace();
        break;
    case GOT_N:
        setNumPoints(current_value);
        break;
    case GOT_P:
        setPulseLength(current_value);
        break;
    case GOT_R:
        setIRLED(current_value);
        break;
    case GOT_S:
        setSatPulseLength(current_value);
        break;
    case GOT_T:
        setSatTriggerPoint(current_value);
        break;
    case GOT_V:
        setVisLED(current_value);
        break;
    case GOT_W:
        setPhaseActValue(current_value, 0);
        break;
    case GOT_X:
        setPhaseActValue(current_value, 1);
        break;
    case GOT_Y:
        setPhaseActValue(current_value, 2);
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
        case 'c':
            state = GOT_C;
            break;
        case 'd':
            state = GOT_D;
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

void handle_saturation_pulse(int pulse_mode, int trace_phase)
{
    // 0 for dirk/normal actinic value, 1 for sat pulse, 2 for st flash
    switch (pulse_mode)
    {
    case 0:
        setActIntensity(act_int_phase[trace_phase], act_int_pins);
        break;
    case 1:
        // saturation pulse is changed to whatever it wasn't beforej
        // If it was LOW, it is now HIGH. If it was HIGH, it is now LOW.
        digitalWrite(satPulse_pin, !digitalRead(satPulse_pin));
        break;
    case 2:
        // single turnover flash
        digitalWrite(stoFlash_pin, !digitalRead(stoFlash_pin));
        break;
    default:
        break;
    }
}

bool debounce()
{
    static uint16_t state = 0;
    state = (state << 1) | digitalRead(btn) | 0xfe00;
    return (state == 0xff00);
}

bool button_pressed()
{
    static uint16_t state = 0;
    state = digitalRead(btn);
    return (state == 1);
}

void trace_summary()
{
    long total_trace_time = micros() - trace_begin_time;
    Serial.print("Trace completed, num_points = ");
    Serial.print(counter);
    Serial.print(", total_time = ");
    Serial.println(total_trace_time);
}

////////////////////////////////////// void setup //////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(9600);

    // setup pins
    pinMode(LED_BUILTIN, OUTPUT); //  set the builtin LED mode
    pinMode(btn, INPUT);
    pinMode(adc_pin, OUTPUT);
    pinMode(satPulse_pin, OUTPUT);
    pinMode(stoFlash_pin, OUTPUT);
    pinMode(farRed_pin, OUTPUT);
    pinMode(meas_led_vis, OUTPUT);
    pinMode(meas_led_ir, OUTPUT);

    for (int i = 0; i < 39; i++)
    {
        digitalWrite(i, LOW);
    }
    digitalWrite(LED_BUILTIN, HIGH);

    prev_time = micros(); // set our previous time to zero
    state = NONE;         // set initial state

    //  Serial.println("tracecontroller ready");
}

///////////////////////////// BEGIN THE LOOOOOOOP ///////////////////////////////////////////////////
void loop()
{

    ////////////////// serial interpreter /////////////////////
    while (Serial.available())
    {
        process_inc_byte(Serial.read());
    }

    if (measure_state == true)
    {
        /////////// trace logic below /////////////////////////////
        ///////////////////////// BEGIN PHASE 0: PRE PULSE MEASUREMENT /////////////////////////////////////
        trace_phase = 0;
        sat_pulse_end = sat_pulse_begin + sat_pulse_length; // beginning and end of phase 1 determined
        trace_begin_time = micros();

        //////////////////////////////// BEGIN TRACE LOOP ///////////////////////////////////////////////////
        while (counter < num_points)
        {

            ///////////////////////////////// SAT PULSE TRIGGER ///////////////////////////////////////////////
            if (counter == sat_pulse_begin)
            {
                trace_phase += 1;
                handle_saturation_pulse(pulse_mode, trace_phase);
            }
            ///////////////////////////////END SAT PULSE TRIGGER //////////////////////////////////////////////

            ///////////////////////////////// SAT PULSE OFF //////////////////////////////////////////////////
            if (counter == sat_pulse_end)
            {
                trace_phase += 1;
                handle_saturation_pulse(pulse_mode, trace_phase);
            }
            ///////////////////////////////END SAT PULSE OFF //////////////////////////////////////////////////

            ///////////////////////////////// MEASUREMENT PULSE ////////////////////////////////////////////////////
            // check the time elapsed since the last loop, and if it has, execute a measurement pulse
            if ((micros() - prev_time) >= pulse_interval)
            {
                counter += 1;
                prev_time = micros(); // reset the pulse timer at the beginning of the pulse
                measurement_pulse();
            }
            ////////////////////////////////END MEASUREMENT PULSE //////////////////////////////////////////////////
        }
        //////////////////////////////// END TRACE LOOP //////////////////////////////////////////////////////////
        // trace_summary();

        ///////////////////////////////// begin trace cleanup ///////////////////////////////////////////////////
        // LED safety check, turn off everything just to be sure
        digitalWrite(satPulse_pin, LOW); // ensure saturation pulse is off
        digitalWrite(stoFlash_pin, LOW); // ensure st flash is off

        setActIntensity(0, act_int_pins); // set actinic to zero

        measure_state = false; // return to listen state

        counter = 0; // reset counter
        ///////////////////////////////// end trace cleanup ///////////////////////////////////////////////////

        Serial.println("trace completed");
    }

} // end main loop