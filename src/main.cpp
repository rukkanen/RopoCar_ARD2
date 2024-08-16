#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>

#include <SoftwareSerial.h>
#include <SerialRedirect.h>
#include <QueueArray.h> // External library for a simple queue implementation
#include <pt.h>

// Pin definitions for the camera module OV7670
#define OV7670_SDA A4   // Camera SDA connected to A4 (D22)
#define OV7670_SCL A5   // Camera SCL connected to A5 (D23)
#define OV7670_D7 7     // Camera D7 connected to D7
#define OV7670_D6 6     // Camera D6 connected to D6
#define OV7670_D5 5     // Camera D5 connected to D5
#define OV7670_D4 4     // Camera D4 connected to D4
#define OV7670_D3 3     // Camera D3 connected to D3
#define OV7670_D2 2     // Camera D2 connected to D2
#define OV7670_D1 8     // Camera D1 connected to D8
#define OV7670_D0 10    // Camera D0 connected to D10
#define OV7670_PCLK 13  // Camera Pixel Clock connected to D13
#define OV7670_VSYNC 12 // Camera VSYNC connected to D12
#define OV7670_HREF 11  // Camera HREF connected to D11
#define OV7670_XCLK 9   // Camera XCLK connected to D9

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

// Camera I2C Address
#define OV7670_I2C_ADDRESS 0x42 // OV7670 I2C address

static struct pt ptPingPong, ptHandleESPComs, ptCaptureAndSendImage, ptUpdateLightLevel, ptUpdateBatteryLevels, ptTestServos;

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
// Function to write to camera registers via I2C
void writeCameraRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(OV7670_I2C_ADDRESS); // OV7670 I2C address
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read a pixel's data (corrected implementation)
uint16_t readPixelData()
{
  uint16_t pixelData = 0;
  pixelData |= (digitalRead(OV7670_D7) << 7); // Read D7 (bit 7)
  pixelData |= (digitalRead(OV7670_D6) << 6); // Read D6 (bit 6)
  pixelData |= (digitalRead(OV7670_D5) << 5); // Read D5 (bit 5)
  pixelData |= (digitalRead(OV7670_D4) << 4); // Read D4 (bit 4)
  pixelData |= (digitalRead(OV7670_D3) << 3); // Read D3 (bit 3)
  pixelData |= (digitalRead(OV7670_D2) << 2); // Read D2 (bit 2)
  pixelData |= (digitalRead(OV7670_D1) << 1); // Read D1 (bit 1)
  pixelData |= digitalRead(OV7670_D0);        // Read D0 (bit 0)
  return pixelData;
}

bool isOV7670Connected()
{
  Wire.beginTransmission(OV7670_I2C_ADDRESS);
  return (Wire.endTransmission() == 0);
}

// Function to capture an image and send it to ESP-01
PT_THREAD(captureAndSendImage(struct pt *pt))
{
  static unsigned long startTime;
  PT_BEGIN(pt);
  while (isEspConnected)
  {
    Serial.println("-> captureAndSendImage, check for cam: " + String(isOV7670Connected()));

    const int imageWidth = 640;
    const int imageHeight = 480;

    espSerial.println("picture_start"); // Notify ESP-01 to expect image data

    Serial.println("Yielding, do other stuff, then come back to capture image");
    PT_YIELD(pt); // Yield to allow ESP-01 to prepare for image data
    Serial.println("Yielded, here were back!!");

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
    Serial.println("camera thread going to sleep");
    startTime = millis();
    PT_YIELD_UNTIL(pt, millis() - startTime >= 5000);
    Serial.println("camera thread woke up!");
  }
  PT_END(pt);
}

// Function to initialize the camera
void initCamera()
{
  Serial.println("-> InitCamera");

  Wire.begin();
  writeCameraRegister(0x12, 0x80); // Reset all registers to default values
  delay(100);                      // Wait for reset to complete
  writeCameraRegister(0x12, 0x00); // Set to VGA resolution
  writeCameraRegister(0x11, 0x01); // Set clock prescaler
  writeCameraRegister(0x6B, 0x4A); // PLL control, set clock
  writeCameraRegister(0x40, 0x10); // RGB565 format
  writeCameraRegister(0x6B, 0x0A); // Set frame rate to 30 fps
  writeCameraRegister(0x13, 0xE0); // Automatic exposure
  writeCameraRegister(0x6A, 0x40); // Automatic white balance
  writeCameraRegister(0x55, 0x00); // Set brightness
  writeCameraRegister(0x56, 0x40); // Set contrast
  writeCameraRegister(0x57, 0x40); // Set saturation

  Serial.println("<- InitCamera");
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
      motorBatteryLevel = analogRead(MOTOR_BATTERY_PIN) * (5.0 / 1023.0);
      computeBatteryLevel = analogRead(COMPUTE_BATTERY_PIN) * (5.0 / 1023.0);
      // Send battery levels to ESP-01
      espSerial.println("battery_level=" + String(motorBatteryLevel) + "," + String(computeBatteryLevel));

      // Check if the battery levels are below the threshold and if it's time to send another warning
      if ((motorBatteryLevel < lowBatteryThreshold || computeBatteryLevel < lowBatteryThreshold))
      {
        // Send the battery warning
        Serial.println("Battery low, consider recharging or shutting down.");
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
  Serial.println("-> LIGHT");
  startTime = millis();
  while (1)
  {
    if (isEspConnected)
    {
      // Check light levels and control LED
      lightLevel = analogRead(PHOTORESISTOR_PIN);
      Serial.println("light level: " + String(lightLevel));
      if (lightLevel < lightLevelThreshold)
      {
        Serial.println("Light level is low, turning on LED");
        digitalWrite(LED_PIN, HIGH);
      }
      else
      {
        Serial.println("Light level is high, turning off LED");
        digitalWrite(LED_PIN, LOW);
      }

      Serial.println("Light thread going to sleep");
    }
    PT_YIELD_UNTIL(pt, millis() - startTime >= lightUpdateWait);
    startTime = millis();
    Serial.println("Light thread wokeup!");
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

    Serial.println("ESP SAYS: " + message);

    if (message == "initWlan:start")
    {
      Serial.println("Handling initWlan:start");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initWlan:ok")
    {
      Serial.println("Handling initWlan:ok");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initHttp:ok")
    {
      Serial.println("Handling initHttp:ok");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "initHttp:fail")
    {
      Serial.println("Handling initHttp:fail");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "READY")
    {
      Serial.println("Handling READY");
      isEspReady = true;
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "tilt_up")
    {
      Serial.println("Handling tilt_up");
      initialTiltPosition = min(initialTiltPosition + 10, 180);
      tiltServo.write(initialTiltPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "tilt_down")
    {
      Serial.println("Handling tilt_down");
      initialTiltPosition = max(initialTiltPosition - 10, 0);
      tiltServo.write(initialTiltPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "pan_left")
    {
      Serial.println("Handling pan_left");
      initialPanPosition = min(initialPanPosition + 10, 180);
      panServo.write(initialPanPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "pan_right")
    {
      Serial.println("Handling pan_right");
      initialPanPosition = max(initialPanPosition - 10, 0);
      panServo.write(initialPanPosition);
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "mode_change:toy")
    {
      Serial.println("Switched to Toy/Mapping mode");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message == "mode_change:guard")
    {
      Serial.println("Switched to Guard mode");
      PT_EXIT(pt); // Exit the current iteration
    }
    else if (message.startsWith("msg:"))
    {
      Serial.println("ESP message: " + message);
      PT_EXIT(pt); // Exit the current iteration
    }
    else
    {
      Serial.println("Unknown message: " + message);
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
    Serial.println("Waiting for ESP-01 connection...");
    espSerial.println("ping");
    espSerial.flush();
    if (espSerial.available())
    {
      String message = espSerial.readStringUntil('\n');
      message.trim();
      if (message == "pong")
      {
        Serial.println("ESP-01 is OK.");
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
  Serial.println("-> Setup servos");
  startTime = millis();

  // Initialize Servos
  tiltServo.attach(TILT_SERVO_PIN);
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.write(initialTiltPosition);
  panServo.write(initialPanPosition);

  Serial.println("tiltServo.write(0)");
  tiltServo.write(0);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  Serial.println("tiltServo.write(180)");
  tiltServo.write(180);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  Serial.println("tiltServo.write(90)");
  tiltServo.write(90);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 1800);

  Serial.println("panServo.write(0)");
  panServo.write(0);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  Serial.println("panServo.write(180)");
  panServo.write(180);
  startTime = millis();
  PT_WAIT_UNTIL(pt, millis() - startTime >= 800);

  Serial.println("panServo.write(90)");
  panServo.write(90);

  isServosTested = true;
  Serial.println("<- DONE: setup servos ");
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