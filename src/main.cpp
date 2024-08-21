#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>

#include <SoftwareSerial.h>
#include <pt.h>

#include "logger.h"
#include "YA_OV7670_camera.h"
#include "BatteryManager.h"

// ESP-01 Pins
#define ESP_RX_PIN 14 // ESP-01 RX connected to D14 (CIPO)
#define ESP_TX_PIN 15 // ESP-01 TX connected to D15 (SCK)

// Servo Pins
#define TILT_SERVO_PIN A3 // Tilt Servo connected to A3 (D21)
#define PAN_SERVO_PIN A2  // Pan Servo connected to A2 (D20)

// Other Components
#define LED_PIN A1             // LED connected to A1 (D19)
#define PHOTORESISTOR_PIN A0   // Photoresistor connected to A0 (D18)
#define MOTOR_BATTERY_PIN A6   // Motor Battery Level connected to A6
#define COMPUTE_BATTERY_PIN A7 // Compute Battery Level connected to A7

static struct pt ptPingPong, ptHandleESPComs, ptCaptureAndSendImage, ptUpdateLightLevel, ptUpdateBatteryLevels, ptTestServos;

// Create an instance of BatteryManager
BatteryManager motorBatteryManager = BatteryManager(MOTOR_BATTERY_PIN);
BatteryManager computeBatteryManager = BatteryManager(COMPUTE_BATTERY_PIN);

// Servos
Servo tiltServo;
Servo panServo;

// SoftwareSerial for ESP-01 communication
SoftwareSerial espSerial(ESP_TX_PIN, ESP_RX_PIN); // Micro Arduino RX and TX pins for ESP-01
bool isEspConnected = false;
bool isSetupDone = false;
// ESP01 will send a message "READY" when it's ready to receive commands
bool isEspReady = false;
// Flag to indicate if servos have been tested
bool isServosTested = false;

// Variables
static int initialTiltPosition = 90;
static int initialPanPosition = 90;
static int lightLevel = 0;
static float motorBatteryLevel = 0.0;
static float computeBatteryLevel = 0.0;
// Light Level Threshold
static const int lightLevelThreshold = 500; // Adjust as needed
static const int lowBatteryThreshold = 300; // Adjust as needed

// Variables to manage battery warning timing
const unsigned int waitForESP = 2000;        // 2 seconds
int retryESPConnection = 8;                  // Number of times to retry ESP connection
const unsigned int batteryNotifWait = 25000; // 25 seconds
const unsigned int lightUpdateWait = 5000;   // 5 seconds

/**
 * Business logic functions
 */
// Function to capture an image and send it to ESP-01
PT_THREAD(captureAndSendImage(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  while (isEspConnected)
  {
    logMessage(INFO, "-> captureAndSendImage, check for cam: " + String(isOV7670Connected()));

    const int imageWidth = 640;
    const int imageHeight = 480;

    espSerial.println("picture_start"); // Notify ESP-01 to expect image data

    logMessage(INFO, "Yielding, do other stuff, then come back to capture image");
    PT_YIELD(pt); // Yield to allow ESP-01 to prepare for image data
    logMessage(INFO, "Yielded, here were back!!");

    for (int y = 0; y < imageHeight; y++)
    {
      // Wait for HREF to indicate valid data
      while (!digitalRead(OV7670_HREF))
        ;
      for (int x = 0; x < imageWidth; x++)
      {
        // Read a pixel from the camera's data pins
        uint16_t pixelData = readPixelData();
        // Send the pixel data to the ESP-01
        espSerial.write((uint8_t)(pixelData >> 8)); // Send high byte
        espSerial.write((uint8_t)pixelData);        // Send low byte
      }

      // Wait for HREF to indicate the end of a line (optional depending on your setup)
      while (digitalRead(OV7670_HREF))
        ;
    }
    logMessage(INFO, "camera thread going to sleep");
    startTime = millis();
    PT_YIELD_UNTIL(pt, millis() - startTime >= 5000);
    logMessage(INFO, "camera thread woke up!");
  }
  PT_END(pt);
}

// Function to update battery levels with a timeout for warnings
PT_THREAD(updateBatteryLevels(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  startTime = millis();
  Serial.println("-> BATTERY");
  startTime = millis();

  while (1)
  {
    if (isEspConnected)
    {
      // Read battery levels from the analog pins
      // motorBatteryLevel = analogRead(MOTOR_BATTERY_PIN) * (3.0 / 1023.0);
      // computeBatteryLevel = analogRead(COMPUTE_BATTERY_PIN) * (5.0 / 1023.0);
      // Send battery levels to ESP-01
      espSerial.println("battery_level=" + String(motorBatteryLevel) + "%," + String(computeBatteryLevel) + "%");

      // Check if the battery levels are below the threshold and if it's time to send another warning
      if ((motorBatteryLevel < lowBatteryThreshold || computeBatteryLevel < lowBatteryThreshold))
      {
        // Send the battery warning
        logMessage(ERROR, "Battery low, consider recharging or shutting down.");
      }
    }
    // Serial.println("Battery thread going to sleep");
    PT_WAIT_UNTIL(pt, millis() - startTime >= batteryNotifWait);
    // Serial.println("Battery thread woke up!");
    startTime = millis();
  }
  PT_END(pt);
}

PT_THREAD(updateLightLevel(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  logMessage(INFO, "-> LIGHT");
  startTime = millis();
  while (1)
  {
    if (isEspConnected)
    {

      // Check light levels and control LED
      lightLevel = analogRead(PHOTORESISTOR_PIN);
      logMessage(INFO, "light level: " + String(lightLevel));
      if (lightLevel < lightLevelThreshold)
      {
        logMessage(INFO, "Light level is low, turning on LED");
        digitalWrite(LED_PIN, HIGH);
      }
      else
      {
        logMessage(INFO, "Light level is high, turning off LED");
        digitalWrite(LED_PIN, LOW);
      }

      logMessage(INFO, "Light thread going to sleep");
    }
    PT_YIELD_UNTIL(pt, millis() - startTime >= lightUpdateWait);
    startTime = millis();
    logMessage(INFO, "Light thread woke up!");
  }
  PT_END(pt);
}

static int handleESPComs(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    // Wait for a message from the ESP
    PT_WAIT_UNTIL(pt, espSerial.available());

    String message = espSerial.readStringUntil('\n');
    message.trim();

    logMessage(INFO, "ESP SAYS: " + message);

    if (message == "initWlan:start")
    {
      logMessage(INFO, "Handling initWlan:start");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initWlan:ok")
    {
      logMessage(INFO, "Handling initWlan:ok");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initHttp:ok")
    {
      logMessage(INFO, "Handling initHttp:ok");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initHttp:fail")
    {
      logMessage(INFO, "Handling initHttp:fail");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "READY")
    {
      logMessage(INFO, "Handling READY");
      isEspReady = true;
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "tilt_up")
    {
      logMessage(INFO, "Handling tilt_up");
      initialTiltPosition = min(initialTiltPosition + 10, 180);
      tiltServo.write(initialTiltPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "tilt_down")
    {
      logMessage(INFO, "Handling tilt_down");
      initialTiltPosition = max(initialTiltPosition - 10, 0);
      tiltServo.write(initialTiltPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "pan_left")
    {
      logMessage(INFO, "Handling pan_left");
      initialPanPosition = min(initialPanPosition + 10, 180);
      panServo.write(initialPanPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "pan_right")
    {
      logMessage(INFO, "Handling pan_right");
      initialPanPosition = max(initialPanPosition - 10, 0);
      panServo.write(initialPanPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "mode_change:toy")
    {
      logMessage(INFO, "Switched to Toy/Mapping mode");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "mode_change:guard")
    {
      logMessage(INFO, "Switched to Guard mode");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message.startsWith("msg:"))
    {
      logMessage(INFO, "ESP message: " + message);
      PT_EXIT(pt); // Exit the current iteration
    }
    else
    {
      logMessage(INFO, "Unknown message: " + message);
      PT_EXIT(pt); // Exit the current iteration
    }
  }

  PT_END(pt);
}

PT_THREAD(pingPong(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  startTime = millis();
  while (1)
  {
    logMessage(INFO, "Waiting for ESP-01 connection...");
    espSerial.println("ping");
    espSerial.flush();
    if (espSerial.available())
    {
      String message = espSerial.readStringUntil('\n');
      message.trim();
      if (message == "pong")
      {
        logMessage(INFO, "ESP-01 is OK.");
        isEspConnected = true;
        PT_EXIT(pt);
      }
    }
    // try to ping ESP-01 every 3 seconds
    PT_WAIT_UNTIL(pt, millis() - startTime >= 3000);
    startTime = millis();
  }

  PT_END(pt);
}

PT_THREAD(testServos(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  logMessage(INFO, "-> Setup servos");
  startTime = millis();

  // Initialize Servos
  tiltServo.attach(TILT_SERVO_PIN);
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.write(initialTiltPosition);
  panServo.write(initialPanPosition);

  logMessage(INFO, "tiltServo.write(0)");
  tiltServo.write(0);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  logMessage(INFO, "tiltServo.write(180)");
  tiltServo.write(180);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  logMessage(INFO, "tiltServo.write(90)");
  tiltServo.write(90);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 1800);

  logMessage(INFO, "panServo.write(0)");
  panServo.write(0);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  logMessage(INFO, "panServo.write(180)");
  panServo.write(180);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  logMessage(INFO, "panServo.write(90)");
  panServo.write(90);

  isServosTested = true;
  logMessage(INFO, "<- DONE: setup servos");
  PT_END(pt);
}

void setup()
{
  Serial.begin(115200);
  espSerial.begin(9600);

  // Initialize LEDs and Photoresistor
  pinMode(LED_PIN, OUTPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(MOTOR_BATTERY_PIN, INPUT);
  pinMode(COMPUTE_BATTERY_PIN, INPUT);

  initCamera(); // Initialize Camera

  // Initialize Protothreads
  PT_INIT(&ptPingPong);
  PT_INIT(&ptHandleESPComs);
  PT_INIT(&ptCaptureAndSendImage);
  PT_INIT(&ptUpdateLightLevel);
  PT_INIT(&ptUpdateBatteryLevels);
  PT_INIT(&ptTestServos);
  Serial.println("<- Setup complete");
}

void loop()
{
  // Check battery level periodically
  // checkBatteryLevel();
  if (!isEspConnected)
  {
    PT_SCHEDULE(pingPong(&ptPingPong));
  }
  else
  {
    PT_SCHEDULE(handleESPComs(&ptHandleESPComs));
    if (isEspReady)
    {
      // PT_SCHEDULE(captureAndSendImage(&ptCaptureAndSendImage));
      PT_SCHEDULE(updateLightLevel(&ptUpdateLightLevel));
      PT_SCHEDULE(updateBatteryLevels(&ptUpdateBatteryLevels));
      if (!isServosTested)
      {
        // Test the servos, this blocks
        PT_SCHEDULE(testServos(&ptTestServos));
      }
    }
  }
}