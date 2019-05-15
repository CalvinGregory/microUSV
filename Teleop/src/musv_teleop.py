'''
Created on May 14, 2019

@author: CalvinGregory

Based on a tutorial by Christopher Barnatt.
https://www.explainingcomputers.com/rasp_pi_robotics.html
'''

import serial
import curses
import time
import struct

def sendSpeeds( starboardSpeed, portSpeed ):
    """ Send formated motor speed message to Arduino
    
    Args:
        starboardSpeed (int16): Desired starboard motor speed (range -127 to 127)
        portSpeed (int16):      Desired port motor speed (range -127 to 127)
           
    Messages are prepended by two '*' characters to indicate message start.     
    """
    arduino.write(struct.pack('<cchh', '*', '*', starboardSpeed, portSpeed))
    return

# Connect to the arduino over USB
arduino = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600, timeout = 1)
# Give serial connection time to settle
time.sleep(2)

# Setup terminal window for curses
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

speed = 100

try:
    while True:
        msg = screen.getch()
        
        if msg == 27: # if ESC key: stop motors and end program
            sendSpeeds(0, 0)
            break
        
        # For 1,2,3 key presses change internal motor speed to preset low, medium, or high
        elif msg == ord('1'): 
            speed = 75
        elif msg == ord('2'): 
            speed = 100
        elif msg == ord('3'): 
            speed = 127
        # For w,a,s,d and q,e,z,c key presses send motor speeds to Arduino.
        elif msg == ord('w'): 
            sendSpeeds(speed, speed)
        elif msg == ord('a'):
            sendSpeeds(speed, -speed)
        elif msg == ord('s'):
            sendSpeeds(-speed, -speed)
        elif msg == ord('d'):
            sendSpeeds(-speed, speed)
        elif msg == ord('q'):
            sendSpeeds(speed, 0)
        elif msg == ord('e'):
            sendSpeeds(0, speed)
        elif msg == ord('z'):
            sendSpeeds(-speed, 0)
        elif msg == ord('c'):
            sendSpeeds(0, -speed)
        # If not a control character, set motor speeds to 0.
        else:
            sendSpeeds(0, 0)
            
# Reset terminal window to defaults
finally:
    curses.nocbreak()
    screen.keypad(False)
    curses.echo()
    curses.endwin()