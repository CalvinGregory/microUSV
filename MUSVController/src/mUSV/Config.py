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

import json

class Config:
    '''
    Config is a class used to import variables from a JSON config file at runtime.
    
    Attributes:
        serverIP (str): IP address of the host computer running the CVSensorSimulator application. 
        tagID (int): Tag ID number of the AprilTag on this robot. 
        label (str): This robot's display name.
        tagTF_x (float): x-axis translation between the robot's coordinate frame and the AprilTag centroid. 
        tagTF_y (float): y-axis translation between the robot's coordinate frame and the AprilTag centroid. 
        tagTF_z (float): z-axis translation between the robot's coordinate frame and the AprilTag centroid. 
        tagTF_yaw (float): z-axis rotation between the robot's coordinate frame and the AprilTag centroid. 
        propSpin_port (int): coefficient indicating which direction the port propeller should spin. 
                             Acceptable values are +1 or -1
        propSpin_star (int): coefficient indicating which direction the starboard propeller should spin. 
                             Acceptable values are +1 or -1
        bias (float): Correction term to account for inequality between port and starboard motor outputs. Value must be a 
                      percentage between -100.0 and +100.0. Positive bias causes the vehicle to turn more to the left (port) 
                      while negative bias causes a right (starboard) turn. 
        P_dist (float): Distance PID controller proportional gain. 
        I_dist (float): Distance PID controller integral gain. 
        D_dist (float): Distance PID controller derivative gain. 
        P_ang (float): Angular PID controller proportional gain. 
        I_ang (float): Angular PID controller integral gain. 
        D_ang (float): Angular PID controller derivative gain. 
        speed_limit (float): Percentage (0 to 100] of the maximum motor speed to treat as an upper bound on motor speed commands.
        tag_plane_distance (float): Distance from the camera to the water's surface in meters.
    '''

    def __init__(self, filepath):
        '''
        Creates a Config object, parses target JSON config file and stores its data in this object. 
        
        Args:
            filepath (str): Filepath to a JSON config file to be parsed.
        '''
        self.serverIP = "192.168.1.0"
        self.tagID = 1
        self.label = "musv"
        self.tagTF_x = 0
        self.tagTF_y = 0
        self.tagTF_z = 0
        self.tagTF_yaw = 0
        self.propSpin_port = 1
        self.propSpin_star = 1
        self.bias = 0.0
        self.P_dist = 1
        self.I_dist = 0
        self.D_dist = 0
        self.P_ang = 1
        self.I_ang = 0
        self.D_ang = 0
        self.speed_limit = 100
        self.tag_plane_distance = 1
        self.debug_mode = False
        self.parse_configs(filepath)

    def parse_configs(self, filepath):
        '''
        Parses target JSON config file and stores its data in this object.
        
        Args:
            filepath (str): Filepath to a JSON config file to be parsed.
        '''
        with open(filepath, 'r') as jsonFile:
            config = json.load(jsonFile)
            
        self.serverIP = config['serverIP']
        self.tagID = config['tagID']
        self.label = config['label']
        self.tagTF_x = config['tagTransform']['x']
        self.tagTF_y = config['tagTransform']['y']
        self.tagTF_z = config['tagTransform']['z']
        self.tagTF_yaw = config['tagTransform']['yaw']
        self.propSpin_port = config['PropellerSpin']['port']
        self.propSpin_star = config['PropellerSpin']['starboard']
        self.bias = config['bias']
        self.P_dist = config['PIDGains']['P_dist']
        self.I_dist = config['PIDGains']['I_dist']
        self.D_dist = config['PIDGains']['D_dist']
        self.P_ang = config['PIDGains']['P_ang']
        self.I_ang = config['PIDGains']['I_ang']
        self.D_ang = config['PIDGains']['D_ang']
        self.speed_limit = config['speed_limit']
        self.tag_plane_distance = config['tag_plane_distance']
        self.debug_mode = config['debug_mode']