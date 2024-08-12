# ARD2 spec

## Command Structure

### Servo Commands
- **Tilt Up**: `"tilt_up"`
- **Tilt Down**: `"tilt_down"`
- **Pan Left**: `"pan_left"`
- **Pan Right**: `"pan_right"`

### LED Commands
- **Turn On LED**: `"led_on"`
- **Turn Off LED**: `"led_off"`

## Implementation Steps

1. **SoftwareSerial Setup**
   - Use `SoftwareSerial` on pins D10 (RX) and D11 (TX) for communication with ESP-01.

2. **Command Parsing**
   - Implement a function to read and parse incoming serial data, and execute the appropriate command.

3. **Basic Testing**
   - Test the servo movement commands (e.g., `"tilt_up"`) and LED control commands (e.g., `"led_on"`) to ensure proper communication and functionality.

