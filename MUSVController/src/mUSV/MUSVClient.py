'''
Created on May 15, 2019

@author: CalvinGregory
'''

import serial
import socket
import time
import struct
import musv_msg_pb2

tagID = 1
server_ip = '192.168.1.2'
port = 8080

if __name__ == '__main__':
#     # Connect to the arduino over USB
#     arduino = serial.Serial(port = '/dev/ttyUSB0', baudrate = 9600, timeout = 1)
#     # Give serial connection time to settle
#     time.sleep(2)
    
    while True:
        requestData = musv_msg_pb2.RequestData()
        requestData.tag_id = tagID
        
        s = socket.socket()
        s.connect((server_ip, port))
        s.send(requestData.SerializeToString())
        
        sensorData = musv_msg_pb2.SensorData() 
    
        msg = s.recv(128)
        
        sensorData.ParseFromString(msg)
        print ("Pose = x:{} y:{} yaw:{}".format(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw))
        for obstacle in sensorData.obstacle_sensors:
            print ("Obstacles: ", obstacle)
        for puck in sensorData.puck_sensors:
            print ("Pucks: ", puck)
        print("TimeStamp: ", sensorData.last_updated)
        
        s.close()
        time.sleep(0.25)
        
    pass

# def sendSpeeds( portSpeed, starboardSpeed ):
#     """ Send formated motor speed message to Arduino
#     
#     Args:
#         portSpeed (int16):      Desired port motor speed (range -127 to 127)
#         starboardSpeed (int16): Desired starboard motor speed (range -127 to 127)
#            
#     Messages are prepended by two '*' characters to indicate message start.     
#     """
#     arduino.write(struct.pack('<cchh', '*', '*', starboardSpeed, portSpeed))
#     return