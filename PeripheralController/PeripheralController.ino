/*
 * PeripheralController receives integer motor speed values over serial and sends 
 * them to the Qik motor controller. 
 *
 * Copyright (C) 2019  CalvinGregory  cgregory@mun.ca
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 */

/*
 * Required connections between Arduino and qik 2s9v1:
 * 
 *       Arduino   qik 2s9v1
 * -------------------------
 *
 * Digital Pin 2 - TX
 * Digital Pin 3 - RX
 * Digital Pin 4 - RESET
 */

#include <PololuQik.h>

PololuQik2s9v1 qik(3, 2, 4);

union intBytes {
  byte b[2];
  int i;
};

void setup() {
  Serial.begin(115200);  
  qik.init();
}

/**
 * Interpret incoming serial messages as motor speed commands. 
 * 
 * Messages are prepended by two '*' characters. 
 * Motor speed values are assumed to be 16-bit integers.
 * 
 * msg format: (*,*,starboardMotorSpeed,portMotorSpeed)
 */
void loop() {
  if(Serial.available() > 5) {
    if (Serial.read() == '*') {
      if (Serial.read() == '*') {
        qik.setSpeeds(boundedMotorSpeed(recvInt()), boundedMotorSpeed(recvInt()));
      }
    }
  }
}

/**
 * Converts the next two bytes in the serial buffer to a 16-bit signed integer.
 */
int recvInt() {
  intBytes val;
  val.b[0] = Serial.read();
  val.b[1] = Serial.read();
  return val.i;
}

/**
 * Constrains an integer motor speed input to a range of -127 to 127: values acceptable by the Qik.
 */
int boundedMotorSpeed(int speed) {
  if (speed > 127) {
    return 127;
  }
  else if (speed < -127) {
    return -127;
  }
  return speed;
}
