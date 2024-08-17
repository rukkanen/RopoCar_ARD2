#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>
#include "logger.h"

// power_management.h
enum PowerState
{
  ACTIVE,
  SLEEP,
  DEEP_SLEEP
};

int BATTERY_PIN;
int BATTERY_THRESHOLD;

PowerState currentPowerState = ACTIVE;

/**
 * Set up the battery manager with the specified pin, threshold, and initial power state
 * @param pin The analog pin to read the battery level from
 * @param threshold The battery level threshold to trigger low battery mode
 * @param state The pointer to the powerstate shared by the main program
 */
void setupBatteryManager(int pin, int threshold, PowerState &state)
{
  pinMode(pin, INPUT);
  BATTERY_PIN = pin;
  BATTERY_THRESHOLD = threshold;
  currentPowerState = state;
}

String powerStateToString(PowerState state)
{
  switch (state)
  {
  case ACTIVE:
    return "ACTIVE";
  case SLEEP:
    return "SLEEP";
  case DEEP_SLEEP:
    return "DEEP_SLEEP";
  default:
    return "UNKNOWN";
  }
}

int readBatteryLevel()
{
  int batteryLevel = analogRead(BATTERY_PIN);
  logMessage(INFO, "Battery level: " + String(batteryLevel));
  return batteryLevel;
}

void checkBatteryLevel()
{
  int batteryLevel = readBatteryLevel();
  if (batteryLevel < BATTERY_THRESHOLD)
  {
    logMessage(WARNING, "Battery level is low, entering deep sleep mode");
    enterDeepSleepMode();
  }
}

void enterSleepMode()
{
  logMessage(INFO, "Entering sleep mode");
  currentPowerState = SLEEP;
  // Turn off peripherals
}

void enterDeepSleepMode()
{
  logMessage(INFO, "Entering deep sleep mode");
  currentPowerState = DEEP_SLEEP;
  // Turn off all peripherals
  // digitalWrite(LED_PIN, LOW);
  // Enter deep sleep mode
  // (Platform-specific deep sleep code here)
}

void wakeUp()
{
  logMessage(INFO, "Waking up from sleep mode");
  currentPowerState = ACTIVE;
  // Re-initialize peripherals
  // (Platform-specific wake-up code here)
}

#endif