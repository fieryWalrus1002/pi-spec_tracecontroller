#pragma once
#include <Arduino.h>
bool DEBUG_MODE = false;

void debugPrint(const char *msg)
{
    if (DEBUG_MODE == true)
    {
        Serial.println(msg);
    }
}

void debugPrint(const char *msg, int value)
{
    if (DEBUG_MODE == true)
    {
        Serial.print(msg);
        Serial.print(": ");
        Serial.println(value);
    }
}

void debugPrint(const char *msg1, int value1, const char *msg2, int value2)
{
    if (DEBUG_MODE == true)
    {
        Serial.print(msg1);
        Serial.print(": ");
        Serial.print(value1);
        Serial.print(", ");
        Serial.print(msg2);
        Serial.print(": ");
        Serial.println(value2);
    }
}