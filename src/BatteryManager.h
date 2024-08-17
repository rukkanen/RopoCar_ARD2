#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

/**
 * Battery manager class to handle battery level monitoring
 * and low battery mode
 *
 * This class is responsible for monitoring the battery level
 * and entering low battery mode when the battery level falls
 * below a certain threshold.
 *
 * The battery level is read from an analog pin and compared
 * to a threshold value. If the battery level is below the threshold,
 * the system will enter low battery mode.
 *
 * The battery manager is used to monitor the battery level and
 * enter low battery mode when necessary.
 */

class BatteryManager
{
public:
  BatteryManager(int batteryPin, int batteryThreshold);
  void checkBatteryLevel();
  int getComputeBatteryLevel(); // Getter method for compute battery level
  int getMotorBatteryLevel();   // This needs a dummy implementation since the motor level is fetched from ARD1 (tje current project is ARD2
  void enterSleepMode();
  void enterDeepSleepMode();

  int computeBatteryPin;

private:
  int readBatteryLevel();
  int batteryPin;
  int batteryThreshold;
};

#endif // BATTERY_MANAGER_H