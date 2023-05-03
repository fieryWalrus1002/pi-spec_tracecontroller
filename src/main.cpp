
#include <main.h>
#include <iomanip>
#include <cmath>

IntervalTimer timer;

// interval between display updates in us.
// int DISPLAY_INTERVAL = 250000;
int DISPLAY_INTERVAL = 250000;

uint8_t k_push_delay = 10; // delay between data point pushes in us

MAX1132 adc(MAX_AQ, ADC_CS_PIN, ADC_RST_PIN, ADC_SSTRB_PIN);
elapsedMicros tLast;
MCP41010 dPotLow(POT1_CS_PIN);
uint8_t dPotLowShuntPin = 2;
MCP41010 dPotHigh(POT2_CS_PIN);
uint8_t dPotHighShuntPin = 2;

Adafruit_SSD1306 display(LCD_WIDTH, LCD_HEIGHT, &Wire, LCD_RESET_PIN);
// CurrentMonitor currentMonitor(MUX_A0_PIN, MUX_A1_PIN, MUX_A2_PIN, MUX_OUTPUT_PIN, MUX_HIGH_ENABLE_PIN, MUX_LOW_ENABLE_PIN);
I2CMaster &master = Master;
Mcp23017 mcp(0x27, master);

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

std::vector<LedData> ledDataVec{
    {"none", 0, POT1_SHUNT_PIN, 0, 0, 0, POT1_CS_PIN},
    {"520", 35, POT2_SHUNT_PIN, 1000, 300, 750, POT2_CS_PIN},
    {"545", 20, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"554", 17, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"563", 16, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"572", 15, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"625", 34, POT2_SHUNT_PIN, 1400, 300, 1400, POT2_CS_PIN},
    {"740", 21, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
    {"800", 23, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
    {"900", 22, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
};

std::shared_ptr<std::vector<LED>> leds = getLedArray(ledDataVec);

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

void handle_act_phase(int trace_num)
{
    uint8_t desired_value = act_int_phase[trace_num];
    if (desired_value == 0)
    {
        (*leds)[actinic_led_num].set_intensity(0, 0);
    }
    else
    {
        (*leds)[actinic_led_num].set_intensity(desired_value, 0);
    }
}

void set_num_points(const int value)
{
    counter = value + 1;
    num_points = value;
    send_response("num_points", num_points);
}

void set_pulse_interval(const int value)
{
    pulse_interval = value;
    send_response("pulse_interval", pulse_interval);
}

void set_vis_led(const int value)
{
    meas_led_num = value;
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
    return (result);
}

int get_numAq()
{
    int result = MAX_AQ - (MAX_AQ / 3);
    return result;
}

void handle_single_turnover()
{
    // set_act_intensity(act_int_phase[trace_phase]);
    // delay(pulse_length);
    // set_act_intensity(0);
    // delay(pulse_interval - pulse_length);
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
        handle_single_turnover();
        break;
    default:
        break;
    }
}

void return_params()
{
    const char *ledName = (*leds)[meas_led_num].led_name.c_str();

    Serial.print("meas_led: ");
    Serial.print(ledName);
    Serial.print(", led_num: ");
    Serial.print(meas_led_num);
    Serial.print(", pin_num: ");
    Serial.print((*leds)[meas_led_num].led_pin);
    Serial.print(", intensity: ");
    Serial.println((*leds)[meas_led_num].get_intensity());
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

void pushData(int whichTraceBuffer)
{
    for (int i = 0; i <= num_points; i++)
    {
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

        for (int i = 0; i < MAX_AQ; i++)
        {
            Serial.print(", ");
            Serial.print(traceData[trace].data[wrt_cnt].aq[i]);
        }
        Serial.println("");
    }
}

void set_12v_power(int val)
{
    if (val >= 1)
    {
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

void send_response(auto respcode, auto val)
{
    /* send a chararacter array and a value back across serial to acknowledge receipt
        of the command.
    */
    if (DEBUG_MODE == true)
    {
        Serial.print(respcode);
        Serial.print(":");
        Serial.print(val);
        Serial.println(";");
    }
}

void set_debug()
{
    if (DEBUG_MODE == false)
    {
        DEBUG_MODE = true;
    }
    else
    {
        DEBUG_MODE = false;
    };
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
    case GOT_Q:
        (*leds)[meas_led_num].set_intensity(current_value, 0);
        Serial.println(current_value);
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

/** @brief Performs ameasurement pulse sequence.
 * @param Tracebuffer* , default is nullptr. If not null, save data to the buffer.
 * @param meas_led_num The vector position of the LED object to use
 * @return shuntVoltage
 *
 */
int measurement_pulse(TraceBuffer *buffer, int meas_led)
{
    Point pnt;
    pnt.time_us[0] = micros() - zeroTime;

    for (int i = 0; i < adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    };

    pnt.time_us[1] = micros() - zeroTime;

    (*leds)[meas_led].toggle(HIGH);

    while ((micros() - zeroTime) < trigger_delay)
    {
        delayMicroseconds(5);
    }

    for (int i = adc.m_preaq; i < adc.m_aq + adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    }

    pnt.shuntV = (*leds)[meas_led].getShuntVoltage();

    pnt.time_us[2] = micros() - zeroTime;

    while ((micros() - pnt.time_us[0]) < pulse_length)
    {
        delayMicroseconds(1);
    }

    (*leds)[meas_led].toggle(LOW);

    if (buffer != nullptr)
    {
        buffer->data[counter] = pnt;
    }

    return pnt.shuntV;
}

void cleanupTrace()
{
    for (auto &led : *leds)
    {
        led.toggle(LOW);
    }

    measureState = false;
    trace_phase = 0;
}

/// @brief Display helper
void display_clear()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
}

/// @brief display short message
void display_message(String message)
{
    display_clear();
    display.print(message);
    display.display();
}

/// @brief display short message on tft display
void displayMessage(std::string message)
{
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.printf(message.c_str());
}

/// @brief display status of various parameters of interest
void displayStatus()
{
    static bool reset{false};
    static std::string lastStr{"pancakes"};

    if (measureState)
    {
        reset = true;
        return;
    }

    int ledShuntInt = measurement_pulse(nullptr, meas_led_num);
    std::string ledName = (*leds)[meas_led_num].led_name;
    std::string ledPin = (*leds)[meas_led_num].ledPinToString();
    int shuntPin = static_cast<int>((*leds)[meas_led_num].shunt_pin);
    int ledIntensity = static_cast<int>((*leds)[meas_led_num].m_current_intensity);

    float ledShuntVoltage = [](int val)
    {
        return std::round(val * (3.3 / 1024.0) * 100) / 100;
    }(ledShuntInt);

    std::stringstream ss;
    ss << "[" << meas_led_num << "]=" << ledName << "nm\n";
    ss << "pin: " << ledPin << "\n";
    ss << "int:" << ledIntensity << "\n";
    ss << "pts:" << num_points << "\n";
    ss << "shuntPint: " << shuntPin << "\n";
    ss << "Vshunt: " << ledShuntVoltage;

    std::string str = ss.str();

    if (!reset && str == lastStr)
        return;
    else
    {
        reset = false;
        tft.fillScreen(ST7735_BLACK);
        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.println(str.c_str());
        lastStr = str;
    }
}

void pinTest(int pinNumber)
{
    for (int i = 0; i < num_points; i++)
    {
        digitalWrite(pinNumber, HIGH);
        delayMicroseconds(pulse_length);
        digitalWrite(pinNumber, LOW);
        delayMicroseconds(pulse_interval);
    }
}

uint16_t test_measurement(bool measurement = true)
{

    uint16_t value = 1;

    if (measurement)
    {
        value = adc.read();
        return value;
    }
    else
    {
        return value;
    }
}

/// @brief test an KED LED by toggling it on and off a number of times with a given intensity
/// @param led_num the array number of the LED to test
/// @param i the intensity, 0-255 given to thje digital potentiometer
/// @param n the number of times to toggle the LED on and off
void test_led(int led_num, int i, int n)
{
    (*leds)[led_num].set_intensity(i, PULSE);
    delayMicroseconds(50);
    for (int j = 0; j < n; j++)
    {
        (*leds)[led_num].toggle(HIGH);
        delayMicroseconds(50);
        (*leds)[led_num].toggle(LOW);
        delayMicroseconds(250);
    }
}

/** performs a measurement pulse with current variables */
void testPulse()
{
    measurement_pulse(ptr_buffer, meas_led_num);
}

void setup()
{
    pinMode(POT1_SHUNT_PIN, INPUT);
    pinMode(POT2_SHUNT_PIN, INPUT);

    Serial.begin(115200);
    SPI.begin();
    dPotHigh.begin();
    dPotLow.begin();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display_message("Starting up...");

    mcp.begin(0, 0);
    display_message("MCP23017 started");

    tft.initR(INITR_BLACKTAB); // Init ST7735S chip
    tft.setSPISpeed(40000000);
    tft.setTextColor(ST7735_WHITE);
    tft.setRotation(3);
    tft.fillScreen(ST7735_BLACK);
    tft.println("welcome!");
    delay(2000);

    timer.begin(displayStatus, DISPLAY_INTERVAL);

    counter = num_points + 1;
}

void loop()
{

    if (Serial.available())
    {
        process_inc_byte(Serial.read());
    }

    if (counter <= num_points)
    {

        if (tLast > pulse_interval)
        {
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

    if (counter > num_points && measureState == true)
    {
        cleanupTrace();
    }
}