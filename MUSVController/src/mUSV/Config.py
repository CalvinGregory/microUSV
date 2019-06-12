'''
Created on Jun. 4, 2019

@author: CalvinGregory
'''

import json

class Config:
    '''
    classdocs
    '''

    def __init__(self, filepath):
        '''
        Constructor
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
        self.P_dist = 1
        self.I_dist = 0
        self.D_dist = 0
        self.P_ang = 1
        self.I_ang = 0
        self.D_ang = 0
        self.parse_configs(filepath)

    def parse_configs(self, filepath):
        '''
        Parse microUSV json config file at the target location. 
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
        self.P_dist = config['PIDGains']['P_dist']
        self.I_dist = config['PIDGains']['I_dist']
        self.D_dist = config['PIDGains']['D_dist']
        self.P_ang = config['PIDGains']['P_ang']
        self.I_ang = config['PIDGains']['I_ang']
        self.D_ang = config['PIDGains']['D_ang']