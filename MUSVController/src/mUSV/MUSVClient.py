'''
Created on May 15, 2019

@author: CalvinGregory
'''

import serial
import socket
import time
import struct
import sys
import musv_msg_pb2
from Config import Config
from PIDController import PIDController

port = 8078

def send_speeds( arduino, portSpeed, starboardSpeed ):
    """ Send formated motor speed message to Arduino
       
    @param arduino:                Serial port variable connecting to Arduino
    @param portSpeed (int16):      Desired port motor speed (range -127 to 127)
    @param starboardSpeed (int16): Desired starboard motor speed (range -127 to 127)
              
    Messages are prepended by two '*' characters to indicate message start.     
    """
    arduino.write(struct.pack('<cchh', '*', '*', starboardSpeed, portSpeed))
    return

if __name__ == '__main__':
    if (len(sys.argv) < 2):
        print ('No config file path provided')
        exit()
        
    config = Config(sys.argv[1])
    server_ip = config.serverIP
    tagID = config.tagID
    
    controller = PIDController((config.P_dist, config.I_dist, config.D_dist), (config.P_ang, config.I_ang, config.D_ang), config.propSpin_port, config.propSpin_star)
    controller.set_throttle(0.6)
    
    DebugMode = True
    
    if not DebugMode: 
        # Connect to the arduino over USB
        arduino = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600, timeout = 1)
        # Give serial connection time to settle
        time.sleep(2)
    
    while True:
        requestData = musv_msg_pb2.RequestData()
        requestData.tag_id = tagID
        
        sensorSimulator = socket.socket()
        try:
            sensorSimulator.connect((server_ip, port))
            sensorSimulator.send(requestData.SerializeToString())
                        
            sensorData = musv_msg_pb2.SensorData() 
        
            msg = sensorSimulator.recv(128)
            sensorSimulator.close()
            sensorData.ParseFromString(msg)
            
#             print ("Pose = x:{} y:{} yaw:{}".format(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw))
#             for obstacle in sensorData.obstacle_sensors:
#                 print ("Obstacles: ", obstacle)
#             for puck in sensorData.puck_sensors:
#                 print ("Pucks: ", puck)
#             print("TimeStamp: ", sensorData.last_updated)
            
            motorSpeeds = controller.get_motor_speeds(sensorData)
            
            if not DebugMode:
                send_speeds(arduino, motorSpeeds[0], motorSpeeds[1])
                
        except socket.error as e:
            pass
        finally:
#             time.sleep(0.25)
            pass

            