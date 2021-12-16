#include <Arduino.h>
#define ADC_PIN 31
#define SAT_PULSE_PIN 26
#define STO_FLASH_PIN 35
#define POWER_LED_PIN 39

int incoming_byte = 0;
int counter = 0;
int sat_pulse_begin = 500;
int sat_pulse_end = 600;
int pulse_length = 10;
int pulse_interval = 25;
int pulse_mode = 1;
int meas_led_vis = 1;
int meas_led_ir = 5;
long prev_time = 0;
int num_points = 1000;
int trace_phase = 0;
int act_int_phase[] = {10, 0, 10}; // holds the actinic intenisty values for phases 0-2
int act_int_pins[] = {16, 17, 18, 19, 20, 21, 22, 23};
int meas_led_array[] = {0, 1, 2, 3, 4, 5, 6, 7};

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


////////////////////////////////////////////// functions //////////////////////////////////////////////////////////
void execute_trace()
{
    Serial.println("trace_triggered;");
    counter = 0;
}

void set_act_intensity(byte value, int intensity_array[])
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

void set_num_points(const int value)
{
    counter = value + 1;
    num_points = value;
    // Serial.print("num_points = ");
    // // Serial.println(num_points);
}

void set_pulse_interval(const int value)
{
    pulse_interval = value;
    // Serial.print("pulse_interval = ");
    // // Serial.println(pulse_interval);
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
    // Serial.print("pulse_length = ");
    // // Serial.println(pulse_length);
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

void handle_saturation_pulse(int pulse_mode, int trace_phase)
{
    // 0 for dirk/normal actinic value, 1 for sat pulse, 2 for st flash
    switch (pulse_mode)
    {
    case 0:
        set_act_intensity(act_int_phase[trace_phase], act_int_pins);
        break;
    case 1:
        // saturation pulse is changed to whatever it wasn't beforej
        // If it was LOW, it is now HIGH. If it was HIGH, it is now LOW.
        digitalWrite(SAT_PULSE_PIN, !digitalRead(SAT_PULSE_PIN));
        break;
    case 2:
        // single turnover flash
        digitalWrite(STO_FLASH_PIN, !digitalRead(STO_FLASH_PIN));
        break;
    default:
        break;
    }
}

void return_params()
{
    Serial.print("counter=");
    Serial.print(counter);
    Serial.print(",");
    Serial.print("sat_pulse_begin=");
    Serial.print(sat_pulse_begin);
    Serial.print(",");
    Serial.print("sat_pulse_end=");
    Serial.print(sat_pulse_end);
    Serial.print(",");
    Serial.print("pulse_length=");
    Serial.print(pulse_length);
    Serial.print(",");
    Serial.print("pulse_interval=");
    Serial.print(pulse_interval);
    Serial.print(",");
    Serial.print("meas_led_vis=");
    Serial.print(meas_led_vis);
    Serial.print(",");
    Serial.print("meas_led_ir=");
    Serial.print(meas_led_ir);
    Serial.print(",");
    Serial.print("num_points=");
    Serial.print(num_points);
    Serial.print(",");
    Serial.print("act_int_phase=[");
    Serial.print(act_int_phase[0]);
    Serial.print(",");
    Serial.print(act_int_phase[1]);
    Serial.print(",");
    Serial.print(act_int_phase[2]);
    Serial.println("];");
}


void handle_action()
{
    // NONE, GOT_M, GOT_N, GOT_I, GOT_G, GOT_H, GOT_V, GOT_R, GOT_P
    switch (state)
    {
    case GOT_A:
        set_act_intensity(current_value, act_int_pins);
        break;
    case GOT_C:
        // checkLED(current_value);
        break;
    case GOT_D:
        // return diagnostic info
        // sendStatus();
        return_params();
        break;
    case GOT_G:
        break;
    case GOT_H:
        break;
    case GOT_I:
        set_pulse_interval(current_value);
        break;
    case GOT_L:
        /// respondState();
        break;
    case GOT_M:
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


void measurement_pulse()
{
    // once it triggers, it executes the following sequence of actions:
    // if this is a singleturnover flash trigger point, then we trigger the single turnover flash
    if (counter == sat_pulse_begin && pulse_mode == 2)
    {
        digitalWrite(STO_FLASH_PIN, HIGH);
        delayMicroseconds(1);
        prev_time = micros();
        digitalWrite(STO_FLASH_PIN, LOW);
        delayMicroseconds(8);
    }

    digitalWrite(meas_led_vis, HIGH); // set green LED to HIGH
    digitalWrite(meas_led_ir, HIGH);  // set IR LED to HIGH
    digitalWrite(ADC_PIN, HIGH);      // turn on trigger for ADC conversion
    delayMicroseconds(pulse_length);  // wait the rest of the pulse width
    digitalWrite(ADC_PIN, LOW);       // adc trigger off
    digitalWrite(meas_led_vis, LOW);  // set green LED to LOW
    digitalWrite(meas_led_ir, LOW);   // set IR LED to LOW
}


void setup()
{
    Serial.begin(9600);
    // set up all pins for ouput
    pinMode(ADC_PIN, OUTPUT);
    pinMode(SAT_PULSE_PIN, OUTPUT);
    pinMode(STO_FLASH_PIN, OUTPUT);
    pinMode(POWER_LED_PIN, OUTPUT);
    pinMode(POWER_LED_PIN, HIGH);

    for (int i = 0; i < 8; i++)
    {
        pinMode(i, OUTPUT);
    }

    for (int i = 16; i < 24; i++)
    {
        pinMode(i, OUTPUT);
    }
    prev_time = micros();
    counter = num_points + 1;

    for (int i = 0; i < 40; i++)
    {
        digitalWrite(i, LOW);
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
        if ((micros() - prev_time) >= pulse_interval)
        {
                // check saturation pulse time points
                if (counter == sat_pulse_begin)
                {
                    trace_phase += 1;
                    handle_saturation_pulse(pulse_mode, trace_phase);
                } else if (counter == sat_pulse_end)
                {
                    trace_phase += 1;
                    handle_saturation_pulse(pulse_mode, trace_phase);
    
                }

            // perform a measurement pulse
            prev_time = micros();
            measurement_pulse();
            counter++;
            if (counter > num_points)
            {
                Serial.println("trace_finished;");
            }
        }
    }
    if (counter >= num_points){
        digitalWrite(SAT_PULSE_PIN, LOW);
    }
}
