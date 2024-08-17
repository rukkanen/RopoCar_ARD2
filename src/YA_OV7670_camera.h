#ifndef YA_OV7679_CAMERA_H
#define YA_OV7679_CAMERA_H

#include <Arduino.h>
#include <Wire.h>
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

// Camera I2C Address
#define OV7670_I2C_ADDRESS 0x42 // OV7670 I2C address

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

#endif
