# PeripheralController

PeripheralController is the controller code for the microUSV's Arduino Nano which interfaces between the peripheral devices (Qik motor controller) and the main controller board (Raspberry pi zero) at a baud rate of 115200. It receives pairs of integers over serial (USB) which it sends as motor speed commands to the Qik motor controller. 

### Wiring

The Arduino and motor controller must be connected using cable harness 7 (see the [hardware wiki](https://osf.io/k7a8p/wiki/08.%20Wiring/)) attached to the following terminals on each device.

Arduino | qik 2s9v1 | Wire Color
------- | --------- | ----------
Digital Pin 2 | TX | Blue
Digital Pin 3 | RX | Brown
Digital Pin 4 | RESET | Purple

### Message Format

Serial messages are expected to follow this format:

```
{*, *, starboard_motor_speed, port_motor_speed}
```

Each message must be prepended by a pair of __*__ characters which indicate the start of message. If one or both __*__ characters are missing, the application will discard the leading character in its serial buffer and continue looking for a new message. The **starboard_motor_speed** and **port_motor_speed** values must be 16-bit signed integers (e.g. c++ short).
