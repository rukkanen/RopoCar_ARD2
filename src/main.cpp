#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>

#include <SoftwareSerial.h>
#include <SerialRedirect.h>
#include <QueueArray.h>  // External library for a simple queue implementation
#include <pt.h>

// Pin definitions for the OV7670
#define OV7670_SDA A4        // Camera SDA connected to A4 (D22)
#define OV7670_SCL A5        // Camera SCL connected to A5 (D23)
#define OV7670_D7 7          // Camera D7 connected to D7
#define OV7670_D6 6          // Camera D6 connected to D6
#define OV7670_D5 5          // Camera D5 connected to D5
#define OV7670_D4 4          // Camera D4 connected to D4
#define OV7670_D3 3          // Camera D3 connected to D3
#define OV7670_D2 2          // Camera D2 connected to D2
#define OV7670_D1 8          // Camera D1 connected to D8
#define OV7670_D0 10         // Camera D0 connected to D10
#define OV7670_PCLK 13       // Camera Pixel Clock connected to D13
#define OV7670_VSYNC 12      // Camera VSYNC connected to D12
#define OV7670_HREF 11       // Camera HREF connected to D11
#define OV7670_XCLK 9        // Camera XCLK connected to D9

// ESP-01 Pins
#define ESP_RX_PIN 14        // ESP-01 RX connected to D14 (CIPO)
#define ESP_TX_PIN 15        // ESP-01 TX connected to D15 (SCK)

// Servo Pins
#define TILT_SERVO_PIN A3    // Tilt Servo connected to A3 (D21)
#define PAN_SERVO_PIN A2     // Pan Servo connected to A2 (D20)

// Other Components
#define LED_PIN A1           // LED connected to A1 (D19)
#define PHOTORESISTOR_PIN A0 // Photoresistor connected to A0 (D18)
#define MOTOR_BATTERY_PIN A6 // Motor Battery Level connected to A6
#define COMPUTE_BATTERY_PIN A7 // Compute Battery Level connected to A7

unsigned long waitForESP = 7000;

static struct pt ptProcessMessageQueue, ptCaptureAndSendImage, ptHandleLightLevel, ptUpdateBatteryLevels;

// Servos
Servo tiltServo;
Servo panServo;

// SoftwareSerial for ESP-01 communication
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);  // Micro Arduino RX and TX pins for ESP-01

// Message queue for outgoing messages to ESP-01
QueueArray<String> messageQueue(20);

// Variables
static int tiltPosition = 90;
static int panPosition = 90;
static int lightLevel = 0;
static float motorBatteryLevel = 0.0;
static float computeBatteryLevel = 0.0;
static const int threshold = 500;  // Adjust as needed
static const int lowBatteryThreshold = 300;  // Adjust as needed

// Variables to manage battery warning timing
unsigned long lastBatteryWarningTime = 0;  // Tracks the last time the warning was sent
const unsigned long batteryWarningInterval = 60000;  // 60 seconds interval



/**
 * Utility functions
 */
void printToSerial(String message) {
    printf("%s\n", message.c_str());
}

/**
 * Business logic functions
 */
PT_THREAD(handleLightLevel(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        // Check light levels and control LED
        lightLevel = analogRead(PHOTORESISTOR_PIN);
        if (lightLevel < threshold) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
        unsigned long startTime = millis();
        PT_WAIT_UNTIL(pt, millis() - startTime >= 4000);

    }
    PT_END(pt);
}

// Function to check if ESP-01 is responsive
PT_Thread(waitForESPConnection(*ptWaitForESPConnection) {
    printToSerial("-> checkESPConnection");
    espSerial.println("ping");
    while (1) {
        if (espSerial.available()) {
            printToSerial("Waiting for ESP-01 response...");
            String response = espSerial.readStringUntil('\n');
            if (response == "pong" || response == "ESP: pong") {
                printToSerial("ESP-01 is OK.");
                printToSerial("<- checkESPConnection");
                return;
            }
        }
    }
    printToSerial("ESP-01 not responding. Please check connections.");
    printToSerial("<- checkESPConnection");
}

// Function to handle commands from ESP-01
void handleCommand(String command) {
    /*
    * Available Commands:
    - "tilt_up": Increase tilt servo position by 10 degrees.
    - "tilt_down": Decrease tilt servo position by 10 degrees.
    - "pan_left": Increase pan servo position by 10 degrees.
    - "pan_right": Decrease pan servo position by 10 degrees.
    - "battery_level": Request the current battery levels from Arduino.
    - "mode_toy": Switch to Toy/Mapping mode.
    - "mode_guard": Switch to Guard mode.
    */
    printToSerial("-> handleCommand: " + command);
    int separatorIndex = command.indexOf(':');
    if (separatorIndex == -1) return;  // Invalid command format

    String id = command.substring(0, separatorIndex);
    String cmd = command.substring(separatorIndex + 1);

    if (cmd == "tilt_up") {
        tiltPosition = min(tiltPosition + 10, 180);
        tiltServo.write(tiltPosition);
    } else if (cmd == "tilt_down") {
        tiltPosition = max(tiltPosition - 10, 0);
        tiltServo.write(tiltPosition);
    } else if (cmd == "pan_left") {
        panPosition = min(panPosition + 10, 180);
        panServo.write(panPosition);
    } else if (cmd == "pan_right") {
        panPosition = max(panPosition - 10, 0);
        panServo.write(panPosition);
    } else if (cmd.startsWith("battery_level")) {
        String response = id + ":battery_level=" + String(motorBatteryLevel) + "," + String(computeBatteryLevel);
        messageQueue.push(response);  // Use correct method for adding to the queue
    } else if (cmd == "mode_toy") {
        printToSerial("Switched to Toy/Mapping mode");
    } else if (cmd == "mode_guard") {
        printToSerial("Switched to Guard mode");
    }
    printToSerial("a message: " + command);
}

// Function to process messages in the queue and send them to ESP-01
PT_THREAD(processMessageQueue(struct pt *pt)) {
    //Serial.println("-> processMessageQueue");
    PT_BEGIN(pt);
    while (1) {
        if (!messageQueue.isEmpty()) {
            String message = messageQueue.pop();  // Use correct method for removing from the queue
            printToSerial("sending message: " + message);
            espSerial.println(message);
        }
        PT_WAIT_UNTIL(pt, !messageQueue.isEmpty());
    }
    PT_END(pt);
    //Serial.println("<- processMessageQueue");
}

// Function to write to camera registers via I2C
void writeCameraRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x42);  // OV7670 I2C address
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Function to read a pixel's data (corrected implementation)
uint16_t readPixelData() {
    uint16_t pixelData = 0;
    pixelData |= (digitalRead(OV7670_D7) << 7);  // Read D7 (bit 7)
    pixelData |= (digitalRead(OV7670_D6) << 6);  // Read D6 (bit 6)
    pixelData |= (digitalRead(OV7670_D5) << 5);  // Read D5 (bit 5)
    pixelData |= (digitalRead(OV7670_D4) << 4);  // Read D4 (bit 4)
    pixelData |= (digitalRead(OV7670_D3) << 3);  // Read D3 (bit 3)
    pixelData |= (digitalRead(OV7670_D2) << 2);  // Read D2 (bit 2)
    pixelData |= (digitalRead(OV7670_D1) << 1);  // Read D1 (bit 1)
    pixelData |= digitalRead(OV7670_D0);         // Read D0 (bit 0)
    return pixelData;
}

// Function to capture an image and send it to ESP-01
PT_THREAD(captureAndSendImage(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1){
        printToSerial("-> captureAndSendImage");

        unsigned long startTime = millis();

        const int imageWidth = 640;
        const int imageHeight = 480;
        // unused, lets keep it further reference
        // const int bytesPerPixel = 2; // RGB565 is 2 bytes per pixel
        uint16_t imageBuffer[imageWidth]; // Buffer to store a line of image data

        for (int y = 0; y < imageHeight; y++) {
            // Wait for HREF to indicate valid data
            while (!digitalRead(OV7670_HREF));

            for (int x = 0; x < imageWidth; x++) {
                // Read a pixel from the camera's data pins
                imageBuffer[x] = readPixelData();
            }

            // Send the line of image data to the ESP-01
            for (int x = 0; x < imageWidth; x++) {
                espSerial.write((uint8_t)(imageBuffer[x] >> 8)); // Send high byte
                espSerial.write((uint8_t)imageBuffer[x]);        // Send low byte
            }
        }
        printToSerial("Image capture and send time: " + String(millis() - startTime) + " ms");
        printToSerial("<- captureAndSendImage");

        unsigned long waitStartTime = millis();
        PT_WAIT_UNTIL(pt, millis() - waitStartTime >= 5000);
    }
    PT_END(pt);
}

// Function to initialize the camera
void initCamera() {
    printToSerial("-> InitCamera");
    /* Initialize camera settings:
       Clock Speed (XCLK) 8 MHz, 12 MHz, 16 MHz, etc
       Resolution QQVGA (160x120), QVGA (320x240), VGA (640x480)
       Color Format: RGB565, YUV422, Grayscale
       Frame Rate 15 fps, 30 fps
       Exposure: Automatic, Manual (with specific values)
       Gain: Automatic, Manual (with specific values)
       White Balance: Automatic, Manual (specific settings like Indoor, Outdoor)
       Brightness: -2 to 2 (or similar range)
       Contrast: -2 to 2 (or similar range)
       Saturation: -2 to 2 (or similar range)
       Sharpness:-2 to 2 (or similar range)
       Mirroring: Enabled, Disabled
       Flip: Enabled, Disabled
       Night Mode: Enabled, Disabled
       Lens Correction: Enabled, Disabled
       Auto-focus: Enabled, Disabled
    */

    Wire.begin();
    writeCameraRegister(0x12, 0x80);  // Reset all registers to default values
    writeCameraRegister(0x12, 0x00);  // Set to VGA resolution
    writeCameraRegister(0x11, 0x01);  // Set clock prescaler
    writeCameraRegister(0x6B, 0x4A);  // PLL control, set clock
    writeCameraRegister(0x40, 0x10);  // RGB565 format
    writeCameraRegister(0x6B, 0x0A);  // Set frame rate to 30 fps
    writeCameraRegister(0x13, 0xE0);  // Automatic exposure
    writeCameraRegister(0x6A, 0x40);  // Automatic white balance
    writeCameraRegister(0x55, 0x00);  // Set brightness
    writeCameraRegister(0x56, 0x40);  // Set contrast
    writeCameraRegister(0x57, 0x40);  // Set saturation

    printToSerial("<- InitCamera");
}

// Function to update battery levels with a timeout for warnings
PT_THREAD(updateBatteryLevels(struct pt *pt)) {
    PT_BEGIN(pt);
    printToSerial("-> updateBatteryLevels");

    while(1) {
        // Read battery levels from the analog pins
        motorBatteryLevel = analogRead(MOTOR_BATTERY_PIN) * (5.0 / 1023.0);
        computeBatteryLevel = analogRead(COMPUTE_BATTERY_PIN) * (5.0 / 1023.0);

        // Get the current time
        unsigned long currentMillis = millis();
        
        // Check if the battery levels are below the threshold and if it's time to send another warning
        if ((motorBatteryLevel < lowBatteryThreshold || computeBatteryLevel < lowBatteryThreshold) &&
            (currentMillis - lastBatteryWarningTime >= batteryWarningInterval)) {

            // Send the battery warning
            printToSerial("Battery low, consider recharging or shutting down.");
            
            // Update the last warning time
            lastBatteryWarningTime = currentMillis;
        }
        unsigned long startTime = millis();
        PT_WAIT_UNTIL(pt, millis() - startTime >= 100000);
    }
    PT_END(pt);
    //Serial.println("<- updateBatteryLevels");
}

void setup() {
    // pipe Std::out and Std::err to Serial
    Serial.begin(9600);  // Initialize Serial communication
    espSerial.begin(9600);  // Initialize SoftwareSerial for ESP-01

    setupSerialRedirect();
    
    delay(4000);
    // Example usage
    printToSerial("------- Setup -------");
    //checkESPConnection();  // Ensure ESP-01 is responsive
    initCamera();  // Initialize Camera

    // Initialize Servos
    tiltServo.attach(TILT_SERVO_PIN);
    panServo.attach(PAN_SERVO_PIN);
    tiltServo.write(tiltPosition);
    panServo.write(panPosition);

    // Initialize LEDs and Photoresistor
    pinMode(LED_PIN, OUTPUT);
    pinMode(PHOTORESISTOR_PIN, INPUT);
    pinMode(MOTOR_BATTERY_PIN, INPUT);
    pinMode(COMPUTE_BATTERY_PIN, INPUT);

    PT_INIT(&ptProcessMessageQueue);
    PT_INIT(&ptCaptureAndSendImage);
    PT_INIT(&ptHandleLightLevel);
    PT_INIT(&ptUpdateBatteryLevels);

    printToSerial("<- Setup complete");
}

void loop() {
    // Capture and send image at intervals
    
    PT_SCHEDULE(captureAndSendImage(&ptCaptureAndSendImage));
    PT_SCHEDULE(handleLightLevel(&ptHandleLightLevel));
    PT_SCHEDULE(updateBatteryLevels(&ptUpdateBatteryLevels));
    PT_SCHEDULE(processMessageQueue(&ptProcessMessageQueue));
    // Process incoming messages from ESP-01
    if (espSerial.available()) {
        String command = espSerial.readStringUntil('\n');
        handleCommand(command);
    }
    
}