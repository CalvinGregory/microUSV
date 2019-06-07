'''
Created on Jun. 4, 2019

@author: CalvinGregory
'''

import json

class Config:
    '''
    classdocs
    '''
    serverIP = "192.168.1.0"
    tagID = 0
    label = "musv"
    tagTF_x = 0
    tagTF_y = 0
    tagTF_z = 0
    tagTF_yaw = 0
    propSpin_port = 1
    propSpin_star = 1
    P = 1
    I = 0
    D = 0

    def __init__(self, filepath):
        '''
        Constructor
        '''
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
        self.P = config['PIDGains']['P']
        self.I = config['PIDGains']['I']
        self.D = config['PIDGains']['D']