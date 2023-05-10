
#include <main.h>
#include <iomanip>
#include <cmath>

uint32_t displayInterval = 500013; // interval between display updates in us.
int shuntValue = 0;
uint8_t k_push_delay = 10; // delay between data point pushes in us

MAX1132 adc(MAX_AQ, ADC_CS_PIN, ADC_RST_PIN, ADC_SSTRB_PIN);
elapsedMicros tLast;
elapsedMicros tLastDisplay;
MCP41010 dPotLow(POT1_CS_PIN);
MCP41010 dPotHigh(POT2_CS_PIN);

Adafruit_SSD1306 display(LCD_WIDTH, LCD_HEIGHT, &Wire, LCD_RESET_PIN);

// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

std::vector<LedData> ledDataVec{
    {"none", LED_PIN_NONE, POT1_SHUNT_PIN, 0, 0, 0, POT1_CS_PIN},
    {"520", LED_PIN_520, POT2_SHUNT_PIN, 1000, 300, 750, POT2_CS_PIN},
    {"545", LED_PIN_545, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"554", LED_PIN_554, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"563", LED_PIN_563, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"572", LED_PIN_572, POT1_SHUNT_PIN, 500, 300, 750, POT1_CS_PIN},
    {"625", LED_PIN_625, POT2_SHUNT_PIN, 1400, 300, 1400, POT2_CS_PIN},
    {"740", LED_PIN_740, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
    {"800", LED_PIN_800, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
    {"900", LED_PIN_900, POT1_SHUNT_PIN, 500, 300, 50, POT1_CS_PIN},
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
    counter = value + 1;
    numPoints = value;
    sendResponse("numPoints", numPoints);
}

void setPulseInterval(const int value)
{
    pulseInterval = value;
    sendResponse("pulseInterval", pulseInterval);
}

void set_vis_led(const int value)
{

    measLedNum = value;
    // sendResponse("measLedNum", measLedNum);
}

void setPulseLength(const int value)
{
    pulseLength = value;
    sendResponse("pulseLength", pulseLength);
}

void set_sat_pulse_end(const int value)
{
    satPulseEnd = value;
    sendResponse("satPulseEnd", satPulseEnd);
}

void set_sat_pulse_begin(const int value)
{
    satPulseBegin = value;
    sendResponse("sat_pulse_begun", satPulseBegin);
}

void set_phase_act_value(const int value, int phase_num)
{
    actIntPhase[phase_num] = value;
    sendResponse("actIntPhase", actIntPhase[phase_num]);
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
    Serial.print(", counter=");
    Serial.print(counter);
    Serial.print(", max_aq=");
    Serial.print(MAX_AQ);
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
    for (int i = 0; i <= numPoints; i++)
    {
        sendDataPoint(i, whichTraceBuffer);
        delayMicroseconds(k_push_delay);
    }
}

void sendDataPoint(int wrt_cnt, int trace)
{

    if (wrt_cnt >= numPoints)
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

void sendResponse(auto respcode, auto val)
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
    sendResponse("DEBUG_MODE", DEBUG_MODE);
}
/** performs a measurement pulse with current variables and returns
 * the shunt value
 */
int testPulse()
{
    zeroTime = micros();
    tracePhase = 0;
    pBuffer = &traceData[traceNumber];
    return measurementPulse(pBuffer, measLedNum);
}

/** sets the test mode to the new mode */
void toggleTestPulser(int value)
{
    testMode = (value >= 1) ? true : false;
}

void setLedIntensity(int ledArrayNum, int value)
{
    // check to see if the value is in range
    (*leds)[ledArrayNum].setIntensity(value, 0);
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
        break;
    case GOT_C:
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

/** @brief Performs ameasurement pulse sequence.
 * @param Tracebuffer* , default is nullptr. If not null, save data to the buffer.
 * @param measLedNum The vector position of the LED object to use
 * @return shuntVoltage
 *
 */
int measurementPulse(TraceBuffer *buffer, int meas_led)
{
    Point pnt;
    static uint16_t internalCounter;

    // time stamp before prepulse measurements
    pnt.time_us[0] = micros() - zeroTime;

    // take pre-pulse measurements
    for (int i = 0; i < adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    };

    // time stamp of LED on
    pnt.time_us[1] = micros() - zeroTime;
    (*leds)[meas_led].toggle(HIGH);

    // wait for trigger delay period. between pulse on and measurement
    while ((micros() - pnt.time_us[1]) < triggerDelay)
    {
        delayMicroseconds(5);
    }

    // take pulse measurements
    for (int i = adc.m_preaq; i < adc.m_aq + adc.m_preaq; i++)
    {
        pnt.aq[i] = adc.read();
    }

    pnt.shuntV = (*leds)[meas_led].getShuntVoltage();

    // time since pulse on shuold be > pulseLength
    uint32_t tPulse = 0;
    while (tPulse < pulseLength)
    {
        tPulse = micros() - pnt.time_us[1];
        delayMicroseconds(5);
    }

    // time stamp of LED off
    pnt.time_us[2] = micros() - zeroTime;
    (*leds)[meas_led].toggle(LOW);

    // serial.print the time stamps, minus the first point
    if (testMode && internalCounter > 250)
    {
        Serial.print("pulse length: ");
        Serial.print(pnt.time_us[2] - pnt.time_us[1]);
        Serial.print(", Vshunt: ");
        Serial.print(pnt.shuntV / 1023.0 * 5.0);
        Serial.print(", preaq: ");
        Serial.print(pnt.aq[0]);
        Serial.print(", aq: ");
        Serial.println(pnt.aq[1]);
        internalCounter = 0;
    }
    internalCounter++;

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
    tracePhase = 0;
}

/// @brief display status of various parameters of interest on ssd1306
void displayStatus()
{
    static float shuntVoltage = 0.0;
    shuntVoltage = shuntValue / 1023.0 * 3.3 * 1000.0;

    std::stringstream ss;
    ss << "V: " << shuntVoltage;
    std::string str = ss.str();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(str.c_str());
    display.display();
}

void init_display()
{
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Shit is getting real.");
    display.display();
}

void setup()
{
    pinMode(POT1_SHUNT_PIN, INPUT);
    pinMode(POT2_SHUNT_PIN, INPUT);

    Serial.begin(115200);
    SPI.begin();
    dPotHigh.begin();
    dPotLow.begin();
    init_display();
    delay(1000);

    counter = numPoints + 1;
}

void loop()
{
    if (!measureState)
    {
        if (Serial.available())
        {
            process_inc_byte(Serial.read());
        }

        if (testMode)
        {
            if (tLast > pulseInterval)
            {
                shuntValue = testPulse();
            }
        }
        if (tLastDisplay > displayInterval)
        {
            displayStatus();
        }
    }

    if (counter <= numPoints)
    {

        if (tLast > pulseInterval)
        {
            if ((counter == satPulseBegin) | (counter == satPulseEnd))
            {
                tracePhase++;
                handleActPhase(tracePhase);
            }

            tLast = 0;
            measurementPulse(pBuffer, measLedNum);
            counter++;
        }
    }

    if (counter > numPoints && measureState == true)
    {
        cleanupTrace();
    }
}