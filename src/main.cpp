
#include <main.h>
#include <iomanip>
#include <cmath>

// set displayInterval to 0.5s in us
uint32_t displayInterval = 500000;
bool woops = false;
int shuntValue = 0;
uint8_t k_push_delay = 10; // delay between data point pushes in us

MAX1132 adc(meas_aq_num, ADC_CS_PIN, ADC_RST_PIN, ADC_SSTRB_PIN);
elapsedMicros tLast;
// elapsedMicros tLastDisplay;
MCP41010 dPotLow(POT1_CS_PIN);
MCP41010 dPotMid(POT2_CS_PIN);
MCP41010 dPotHigh(POT3_CS_PIN);

// Adafruit_SSD1306 display(LCD_WIDTH, LCD_HEIGHT, &Wire, LCD_RESET_PIN);

// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// LED(led_name, led_pin, shunt_pin, max_source_current, max_constant_current, max_surge_current, pot_cs_pin, max_intensity)
std::vector<LedData> ledDataVec{
    {"none", LED_PIN_NONE, POT1_SHUNT_PIN, 0, 0, 0, POT1_CS_PIN, 0, 1},
    {"520", LED_PIN_520, POT2_SHUNT_PIN, 1000, 300, 750, POT3_CS_PIN, 255, 1},
    {"545", LED_PIN_545, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN, 255, 1},
    {"554", LED_PIN_554, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN, 255, 1},
    {"563", LED_PIN_563, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN, 255, 1},
    {"572", LED_PIN_572, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN, 255, 1},
    {"625", LED_PIN_625, POT2_SHUNT_PIN, 1400, 300, 1400, POT3_CS_PIN, 255, 1},
    {"740", LED_PIN_740, POT1_SHUNT_PIN, 500, 300, 1000, POT2_CS_PIN, 255, 10},
    {"800", LED_PIN_800, POT1_SHUNT_PIN, 500, 300, 1000, POT2_CS_PIN, 255, 10},
    {"900", LED_PIN_900, POT1_SHUNT_PIN, 500, 300, 1000, POT2_CS_PIN, 255, 10},
};

std::shared_ptr<std::vector<LED>> leds = getLedArray(ledDataVec);
int actLedNum = getLedNum("625", leds);

////////////////////////////////////////////// functions //////////////////////////////////////////////////////////


void executeTrace()
{
    // set trace variables to initial values
    measureState = true;
    testMode = false;
    tracePhase = 0;
    counter = 0;
    zeroTime = micros();
    pBuffer = &traceData[traceNumber];
    handleActPhase(tracePhase);
    debugPrint("begin trace, int: ", actIntPhase[tracePhase]);
}

/**
 * @brief handles the actinic phase of the trace
 * Turns on or off the actinic LED based on the desired value for a given trace
 * @param trace_num the trace number, used to access the appropriate value from array
 *
 */
void handleActPhase(int trace_num)
{
    letThereBeLight(actIntPhase[trace_num]);
}

void setNumPoints(const int value)
{
    if (value > MAX_DATA)
    {
        numPoints = MAX_DATA - 1;
    }
    else if (value < 0)
    {
        numPoints = 1;
    }
    else
    {
        numPoints = value;
    }
    counter = numPoints + 1;
    debugPrint("numPoints", numPoints);
}

void setPulseInterval(const int value)
{
    pulseInterval = value;
    debugPrint("pulseInterval", pulseInterval);
}

void set_vis_led(const int value)
{

    measLedNum = value;
    debugPrint("measLedNum", measLedNum);
}

void setPulseLength(const int value)
{
    pulseLength = value;
    debugPrint("pulseLength", pulseLength);
}

void set_sat_pulse_end(const int value)
{
    satPulseEnd = value;
    debugPrint("satPulseEnd", satPulseEnd);
}

void set_sat_pulse_begin(const int value)
{
    satPulseBegin = value;
    debugPrint("sat_pulse_begun", satPulseBegin);
}

void set_phase_act_value(const int value, int phase_num)
{
    actIntPhase[phase_num] = value;
    debugPrint("actIntPhase", actIntPhase[phase_num]);
}

int get_numPreAq()
{
    int result = meas_aq_num / 3;
    return (result);
}

int get_numAq()
{
    int result = meas_aq_num - (meas_aq_num / 3);
    return result;
}

void handle_single_turnover()
{
    // set_act_intensity(actIntPhase[tracePhase]);
    // delay(pulseLength);
    // set_act_intensity(0);
    // delay(pulseInterval - pulseLength);
}

void handle_saturation_pulse(int pulseMode, int tracePhase)
{
    // 0 for dirk/normal actinic value, 1 for sat pulse, 2 for st flash
    switch (pulseMode)
    {
    case 0:
        // set_act_intensity(actIntPhase[tracePhase]);
        break;
    case 1:
        handleActPhase(tracePhase);
        break;
    case 2:
        // single turnover flash, as quick as we can pulse it
        handle_single_turnover();
        break;
    default:
        break;
    }
}

void returnParams()
{
    Serial.print("measureState: ");
    Serial.print(measureState);
    Serial.print(", counter=");
    Serial.print(counter);
    Serial.print("testMode: ");
    Serial.print(testMode);
    Serial.print(", meas_led: ");
    Serial.print((*leds)[measLedNum].led_name.c_str());
    Serial.print(", led_num: ");
    Serial.print(measLedNum);
    Serial.print(", pin_num: ");
    Serial.print((*leds)[measLedNum].led_pin);
    Serial.print(", intensity: ");
    Serial.print((*leds)[measLedNum].getIntensity());
    Serial.print(", meas_aq_num=");
    Serial.print(meas_aq_num);
    Serial.print(", powerState=");
    Serial.print(powerState);
    Serial.print(", satPulseBegin=");
    Serial.print(satPulseBegin);
    Serial.print(", satPulseEnd=");
    Serial.print(satPulseEnd);
    Serial.print(", pulseLength=");
    Serial.print(pulseLength);
    Serial.print(", pulseInterval=");
    Serial.print(pulseInterval);
    Serial.print(", numPoints=");
    Serial.print(numPoints);
    Serial.print(", pulseMode=");
    Serial.print(pulseMode);
    Serial.print(", actIntPhase=[");
    Serial.print(actIntPhase[0]);
    Serial.print("|");
    Serial.print(actIntPhase[1]);
    Serial.print("|");
    Serial.print(actIntPhase[2]);
    Serial.print("]");
    Serial.print(", triggerDelay=");
    Serial.print(triggerDelay);
    Serial.print(", actinicIntensity=");
    Serial.print(actinicIntensity);
    Serial.println(';');
}

void pushData(int whichTraceBuffer)
{
    for (int i = 0; i < numPoints; i++)
    {
        sendDataPoint(i, whichTraceBuffer);
        delayMicroseconds(k_push_delay);
    }
    Serial.println(";");
}

void sendDataPoint(int wrt_cnt, int trace)
{
    Serial.print(wrt_cnt);
    Serial.print(",");
    Serial.print(traceData[0].data[wrt_cnt].time_us[0]);

    for (int i = 0; i < meas_aq_num; i++)
    {
        Serial.print(", ");
        Serial.print(traceData[trace].data[wrt_cnt].aq[i]);
    }
    Serial.println("");
}

void setDebug()
{
    if (DEBUG_MODE == false)
    {
        DEBUG_MODE = true;
    }
    else
    {
        DEBUG_MODE = false;
    };
    debugPrint("DEBUG_MODE", DEBUG_MODE);
}
// /** performs a measurement pulse with current variables and returns
//  * the shunt value
//  */
// int testPulse()
// {
//     zeroTime = micros();
//     tracePhase = 0;
//     pBuffer = &traceData[traceNumber];
//     return measurementPulse(pBuffer, measLedNum);
// }

/** sets the test mode to the new mode */
void toggleTestPulser(int value)
{
    testMode = (value >= 1) ? true : false;
}

/**
 * @brief sets the designated LED in the led array to a chosen value. Clips
 * the value to 0-255, then sets the intensity of the LED to that value.
 * THe led functionality is handled by the led class, which may include further
 * modification of the value to prevent LED over-driving.
 */
void setLedIntensity(const int ledArrayNum, const int value)
{
    uint8_t clipped_value = static_cast<uint8_t>(value);
    uint8_t ledValue = (*leds)[ledArrayNum].setIntensity(clipped_value, 0);
    Serial.printf("setLedIntensity: ledArrayNum=%d, clipped_value=%d, value=%d, ledValue=%d\n", ledArrayNum, clipped_value, value, ledValue);
}

/**
 * sets the intensity of the actinic LED. If positive, the LED is turned on, if
 * it is =< 0, the LED is turned off.
 */
void letThereBeLight(int value)
{
    actinicIntensity = value;
    setLedIntensity(actLedNum, value);

    if (value > 0)
    {
        (*leds)[actLedNum].turnOn();
    }
    else
    {
        (*leds)[actLedNum].turnOff();
    }
}

void handleAction()
{
    switch (state)
    {
    case GOT_A:
        toggleTestPulser(currentValue);
        break;
    case GOT_B:
        calibrateTriggerDelay(currentValue);
        break;
    case GOT_C:
        if (currentValue <= 9 && currentValue >= 0)
        {
            meas_aq_num = currentValue;
        }
        else
        {
            meas_aq_num = 9;
        }
        adc.set_acquisition_points(meas_aq_num);

        break;
    case GOT_D:
        returnParams();
        break;
    case GOT_E:
        triggerDelay = currentValue;
        break;
    case GOT_F:
        break;
    case GOT_G:
        pushData(0);
        break;
    case GOT_H:
        break;
    case GOT_I:
        setPulseInterval(currentValue);
        break;
    case GOT_K:
        break;
    case GOT_L:
        letThereBeLight(currentValue);
        break;
    case GOT_M:
        executeTrace();
        break;
    case GOT_N:
        setNumPoints(currentValue);
        break;
    case GOT_O:
        setDebug();
        break;
    case GOT_P:
        setPulseLength(currentValue);
        break;
    case GOT_Q:
        setLedIntensity(measLedNum, currentValue);
        // (*leds)[measLedNum].setIntensity(currentValue, 0);
        break;
    case GOT_S:
        set_sat_pulse_end(currentValue);
        break;
    case GOT_T:
        set_sat_pulse_begin(currentValue);
        break;
    case GOT_V:
        set_vis_led(currentValue);
        break;
    case GOT_W:
        set_phase_act_value(currentValue, 0);
        break;
    case GOT_X:
        set_phase_act_value(currentValue, 1);
        break;
    case GOT_Y:
        set_phase_act_value(currentValue, 2);
        break;
    case GOT_Z:
        pulseMode = currentValue;
        break;
    default:
        break;
    } // end of switch

    currentValue = 0; // since we utilized the currentValue above, now we reset it to zero for the next variable
    state = NONE;     // set the state to none, as we have used it
}

void process_inc_byte(const byte c)
{
    if (isdigit(c))
    {
        currentValue *= 10;
        currentValue += c - '0';
    } // end of digit
    else
    {
        switch (c)
        {
        case '?':
            state = GOT_O;
            break;
        case ';':
            handleAction();
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

// int testPulse(TraceBuffer *buffer, int meas_led)
// {
//     Point pnt;
//     static uint32_t ticksLastPrint = 0;
//     static uint32_t printInterval = 200;
//     uint32_t tPulse = 0;

//     // take pre-pulse measurements
//     for (int i = 0; i < adc.m_preaq; i++)
//     {
//         pnt.aq[i] = adc.read();
//     };

//     // time stamp pulse on
//     pnt.time_us[0] = micros() - zeroTime;

//     (*leds)[meas_led].toggle(HIGH);

//     // take pulse measurements
//     for (int i = adc.m_preaq; i < adc.m_aq + adc.m_preaq; i++)
//     {
//         pnt.aq[i] = adc.read();
//     }

//     while (tPulse < pulseLength)
//     {
//         tPulse = micros() - pnt.time_us[0];
//     }

//     (*leds)[meas_led].toggle(LOW);

//     // time pulse off
//     pnt.time_us[1] = micros() - zeroTime;

//     if (buffer != nullptr)
//     {
//         buffer->data[counter] = pnt;
//     }

//     return 0;
// }

/** @brief Performs measurement pulse sequence.
 * @param buffer* , default is nullptr. If not null, save data to the buffer.
 * @param meas_led The vector position of the LED object to use
 * @return the measurement Point object
 */
Point measurementPulse(TraceBuffer *buffer, int meas_led)
{
    Point pnt;
    uint32_t tPulse{0};
    u_int32_t pulseOn{0};
    uint32_t delayTime{0};

    // take pre-pulse measurements
    for (int i = 0; i < adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    };

    // time stamp before pre-pulse measurements
    pnt.time_us[0] = micros() - zeroTime;

    // time stamp of LED turning on within the measurement pulse
    pulseOn = micros();

    // time stamp of LED on
    (*leds)[meas_led].toggle(HIGH);

    // delay for trigger delay
    while (delayTime < triggerDelay)
    {
        delayTime = micros() - pulseOn; // diff is microseconds since LED turned on
        delayMicroseconds(1);
    }

    // take pulse measurements
    for (int i = adc.m_preaq; i < adc.m_aq + adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    }

    while (tPulse < pulseLength)
    {
        tPulse = micros() - pnt.time_us[0];
    }

    pnt.time_us[1] = micros() - zeroTime;

    (*leds)[meas_led].toggle(LOW);

    if (buffer != nullptr)
    {
        buffer->data[counter] = pnt;
    }

    return pnt;
}

void cleanupTrace()
{
    for (auto &led : *leds)
    {
        led.toggle(LOW);
    }
}

void calibrateTriggerDelay(int howMany = 10)
{
    // set trigger delay to 0, and pulseLength to 100
    triggerDelay = 0;
    Point points[howMany];

    // take howMany measurement pulses, incrementing triggerDelay
    for (int i = 0; i < howMany; i++)
    {
        points[i] = measurementPulse(nullptr, measLedNum);
        delay(1);
        triggerDelay += 1;
    }

    // now send the data to the computer
    for (int i = 0; i < howMany; i++)
    {
        Serial.print(i);
        Serial.print(",");
        Serial.print(points[i].time_us[0]);

        for (int j = 0; j < adc.m_preaq + adc.m_aq; j++)
        {
            Serial.print(", ");
            Serial.print(points[i].aq[j]);
        }
        Serial.println("");
    }
    Serial.println(";");
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();

    for (auto led : *leds)
    {
        led.begin();
    }

    delay(1000);

    counter = numPoints + 1;
    set_vis_led(8);
}



// create a vector to hold the data
std::vector<float> data;
float voltage = 0;
int jMax = 1;
int iMax = 256;
int iMin = 0;

int usDelay = 100;

void loop()
{
    if (!measureState && Serial.available())
        process_inc_byte(Serial.read());
    



    
    for (int i = iMin; i < iMax; i++)
    {
        (*leds)[measLedNum].setIntensity(i, 0);

        
        for (int j = 0; j < jMax; j++)
        {
                digitalWrite(23, HIGH);
                delayMicroseconds(usDelay);
                voltage = (*leds)[measLedNum].getShuntVoltage();
                digitalWrite(23, LOW);
                delayMicroseconds(usDelay);
        }
        
        data.push_back(voltage);
        
    }
        
    
    
    Serial.print("i, voltage\n");
    // print out the data to serial
    for (int i = 0; i < data.size(); i++)
    {
        Serial.print(i);
        Serial.print(",");
        Serial.println(data[i]);
    }

    Serial.print("led shunt pin: ");
    Serial.println((*leds)[measLedNum].getShuntPin());
    Serial.println(";");
    data.clear();
    delay(1000);


    while (counter <= numPoints)
    {
        if (measureState)
        {
            if (tLast > pulseInterval)
            {
                tLast = 0;

                if ((counter == satPulseBegin) | (counter == satPulseEnd))
                {
                    tracePhase++;
                    debugPrint("counter: ", counter, ", tracePhase: ", tracePhase);
                    handleActPhase(tracePhase);
                }

                measurementPulse(pBuffer, measLedNum);
                counter++;
            }

            if (counter > numPoints)
            {
                debugPrint("cleanup");
                cleanupTrace();
                measureState = false;
            }
        }
    }
}