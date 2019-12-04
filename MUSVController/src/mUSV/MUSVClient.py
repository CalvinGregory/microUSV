'''
MUSVClient controls the microUSV. It connects to a host computer running CVSensorSimulator to determine 
its simulated sensor values, then acts on those values, sending instructions to the motor controller.  

Copyright (C) 2019  CalvinGregory  cgregory@mun.ca
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
'''

import serial
import socket
import time
import struct
import sys
import os.path
import musv_msg_pb2
from Config import Config
from PIDController import PIDController
from OrbitalController import OrbitalController
from PurePursuitController import PurePursuitController

port = 8078

def send_speeds( arduino, portSpeed, starboardSpeed ):
    ''' 
    Send formated motor speed message to the Arduino peripheral controller board which forwards
    the message to the microUSV's motor controller. 
    
    Args:   
        arduino (serial.Serial): Serial port variable connecting to the Arduino.
        portSpeed (int16): Desired port motor speed (range -127 to 127).
        starboardSpeed (int16): Desired starboard motor speed (range -127 to 127).
              
    Note: Messages are prepended by two '*' characters to indicate message start.     
    '''
    arduino.write(struct.pack('<cchh', '*', '*', starboardSpeed, portSpeed))
    return

if __name__ == '__main__':
    '''
    MUSVClient controls the microUSV. It connects to a host computer running CVSensorSimulator to determine 
    its simulated sensor values, then acts on those values, sending instructions to the motor controller.
    '''
    time_offset = -1
    msg_timestamp = -1
    
    # Read config file
    if (len(sys.argv) < 2):
        if (os.path.isfile('config.json')):
            config = Config('config.json')
        else: 
            print ('No config file path provided')
            exit()
    else:
        config = Config(sys.argv[1])
    
    server_ip = config.serverIP
    tagID = config.tagID
    
    # Build controller object
    if config.controller_type == 'orbital':
        controller = OrbitalController(config)
    elif config.controller_type == 'pursuit':
        controller = PurePursuitController(config)
    else:
        controller = PIDController(config)
    

    if not config.debug_mode:    
        # Connect to the arduino over USB and wait for connection to settle
        arduino = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200, timeout = 1)
        time.sleep(2)
    
    first_request = True
    last_timestamp = -1
    last_update = -1
    motorSpeeds = (0, 0)
    lastSpeeds = (1, 1)
    try:
        while True:
            # Ping host machine running CVSensorSimulator, request sensor data for this microUSV. 
            requestData = musv_msg_pb2.RequestData()
            requestData.tag_id = tagID
            if first_request:
                requestData.request_waypoints = True
                first_request = False
            
            sensorSimulator = socket.socket()
            try:
                sensorSimulator.connect((server_ip, port))
                sensorSimulator.send(requestData.SerializeToString())
                            
                sensorData = musv_msg_pb2.SensorData() 
            
                msg = sensorSimulator.recv(128)
                sensorSimulator.close()
                sensorData.ParseFromString(msg)

                msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
                if msg_timestamp > last_timestamp:
                    motorSpeeds = controller.get_motor_speeds(sensorData)
                    last_timestamp = msg_timestamp
                    last_update = time.time()

            except socket.error as e:
                print 'Error connecting to CVSensorSimulator.'
            finally:
                if time.time() - last_update > 1.0:
                    motorSpeeds = (0, 0)
                
                if motorSpeeds != lastSpeeds:
                    if not config.debug_mode:
                        send_speeds(arduino, motorSpeeds[0], motorSpeeds[1])
                lastSpeeds = motorSpeeds
                
                time.sleep(0.1)

    finally:
        if not config.debug_mode:
            send_speeds(arduino, 0, 0)
