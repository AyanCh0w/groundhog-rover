# Rover Control with WiFi, MQTT, and MPU6050

This Arduino Giga-based rover control program connects to WiFi, subscribes to MQTT commands, and drives using motor control with heading stabilization from an MPU6050 gyroscope. It supports forward movement with heading hold, in-place turns, and live command reception.

## Features

- **WiFi Connectivity**: Connects to a specified SSID and shows connection status.
- **MQTT Control**:  
  - Subscribes to `jumpstart/rover_command` for movement commands.
  - Publishes status/location updates to `jumpstart/rover_location`.
- **Motor Control**:
  - Independent control of left and right motors.
  - Forward driving with gyroscope-based heading correction.
  - In-place turning with speed scaling as it approaches target.
- **IMU Integration**:
  - Uses MPU6050 for yaw angle readings.
  - Maintains heading during straight-line driving.
- **LED Status**:
  - Blinks fast while connecting to WiFi.
  - Solid on when connected.

## Hardware Requirements

- **Arduino Giga R1 WiFi** (or compatible)
- **MPU6050** IMU module
- **H-Bridge Motor Driver** (e.g., L298N, TB6612FNG)
- **Two DC motors**
- **LED** (optional, for status indication)
- Power supply appropriate for motors and Arduino

## Pin Assignments

| Pin | Function                |
|-----|------------------------|
| 2   | Left motor forward PWM |
| 3   | Left motor reverse PWM |
| 5   | Right motor forward PWM|
| 4   | Right motor reverse PWM|

**Note:** Adjust pin mapping if using different wiring.

## Software Dependencies

Install the following libraries from the Arduino Library Manager:

- `WiFi`
- `PubSubClient`
- `Wire`
- `MPU6050_light`

## Setup

1. Connect the motors to the motor driver and assign the driver’s input pins to match the `L_IN1`, `L_IN2`, `R_IN1`, and `R_IN2` definitions.
2. Connect the MPU6050 to the Arduino via I2C (`SDA` and `SCL`).
3. Edit the WiFi credentials:
   ```cpp
   char ssid[] = "YOUR_WIFI_SSID";
   char pass[] = "YOUR_WIFI_PASSWORD";
   ```
4. Upload the sketch to your Arduino Giga.

## MQTT Command Format

Commands are CSV-formatted strings. Supported commands:

### Drive Forward
```
drive,<feet>,<baseDuty>,<corrDuty>,<deadbandDeg>,<ctrlHz>,<feetPerSec>
```
**Example:**
```
drive,5,90,255,3,50,0.90
```
- Drives forward 5 feet, base speed 90, correction speed 255, ±3° deadband, 50Hz loop, 0.90 ft/s.

### Turn In Place
```
turn,<degreesRight>,<turnDuty>,<stopThresholdDeg>,<ctrlHz>,<maxSeconds>
```
**Example:**
```
turn,90,140,2,50,6
```
- Turns 90° to the right, duty cycle 140, stop when within 2°, 50Hz loop, max 6 seconds.

### Stop
```
stop
```

## Topics

- **Subscribe:** `jumpstart/rover_command`
- **Publish:** `jumpstart/rover_location`

## Example MQTT Testing

To test commands using `mosquitto_pub`:
```bash
mosquitto_pub -h broker.hivemq.com -t jumpstart/rover_command -m "drive,3"
mosquitto_pub -h broker.hivemq.com -t jumpstart/rover_command -m "turn,90"
mosquitto_pub -h broker.hivemq.com -t jumpstart/rover_command -m "stop"
```

## Safety Notes

- Always test with wheels lifted off the ground first.
- Ensure power supply matches motor requirements.
- Double-check wiring to avoid short circuits.
