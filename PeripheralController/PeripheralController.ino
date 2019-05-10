/*
Required connections between Arduino and qik 2s9v1:

      Arduino   qik 2s9v1
-------------------------

Digital Pin 2 - TX
Digital Pin 3 - RX
Digital Pin 4 - RESET
*/

#include <PololuQik.h>

PololuQik2s9v1 qik(3, 2, 4);
char msg;
int motorSpeed = 110;

void setup() {
  Serial.begin(9600);  
  qik.init();
}

/**
 * Interpret incoming serial messages as speed commands.
 */
 //TODO change hardcoded directional char inputs to pair of ints for motor speeds
void loop() {
 if(recvMsg()) { 
  switch(msg) {
    // Set motor speeds to low
    case '1':
      motorSpeed = 80;
      qik.setCoasts();
      break;
    // Set motor speeds to medium
    case '2':
      motorSpeed = 100;
      qik.setCoasts();
      break;
    // Set motor speeds to high
    case '3':
      motorSpeed = 127;
      qik.setCoasts();
      break;
    // pivot ccw
    case 'a':
      qik.setSpeeds(-motorSpeed,motorSpeed);
      break;
    // pivot cw
    case 'd':
      qik.setSpeeds(motorSpeed,-motorSpeed);
      break;
    // drive forward
    case 'w':
      qik.setSpeeds(motorSpeed,motorSpeed);
      break;
    // drive in reverse
    case 's':
      qik.setSpeeds(-motorSpeed,-motorSpeed);
      break;
    // drive forward-left (starboard motor only)
    case 'q':
      qik.setSpeeds(0,motorSpeed);
      break;
    // drive forward-right (port motor only)
    case 'e':
      qik.setSpeeds(motorSpeed,0);
      break;
    // drive reverse-left (starboard motor only)
    case 'z':
      qik.setSpeeds(0,-motorSpeed);
      break;
    // drive reverse-right (port motor only)
    case 'c':
      qik.setSpeeds(-motorSpeed,0);
      break;
    // stop
    default: 
      qik.setCoasts();
      break;
  }
 }
}

/**
 * Checks if there is a serial message available.
 * If so send the first character in the buffer to the msg variable. 
 */
boolean recvMsg() {
 if (Serial.available() > 0) {
   msg = Serial.read();
   return true;
 }
 return false;
}
