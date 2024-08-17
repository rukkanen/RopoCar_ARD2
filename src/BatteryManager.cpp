#include "BatteryManager.h"
#include "logger.h"

BatteryManager::BatteryManager(int computePin, int batteryThreshold)
    : computeBatteryPin(computePin), batteryThreshold(batteryThreshold) {}

int BatteryManager::getBatteryLevel()
{
  int batteryLevel = analogRead(batteryPin);
  logMessage(INFO, "Battery level: " + String(batteryLevel));
  return batteryLevel;
}

void BatteryManager::enterSleepMode()
{
  logMessage(INFO, "Entering sleep mode");
  // Platform-specific sleep code here
}

void BatteryManager::enterDeepSleepMode()
{
  logMessage(INFO, "Entering deep sleep mode");
  // Platform-specific deep sleep code here
}

void BatteryManager::checkBatteryLevel()
{
  // dummy value for motor battery at this point
  // motorBatteryLevel = analogRead(MOTOR_BATTERY_PIN) * (5.0 / 1023.0);
  int level = analogRead(COMPUTE_BATTERY_PIN) * (5.0 / 1023.0);

  if (level < batteryThreshold)
  {
    logMessage(WARNING, "Battery level is low, entering deep sleep mode");
    enterDeepSleepMode();
  }
}

int BatteryManager::readBatteryLevel()
{
  int batteryLevel = analogRead(batteryPin);
  logMessage(INFO, "Battery level: " + String(batteryLevel));
  return batteryLevel;
}
