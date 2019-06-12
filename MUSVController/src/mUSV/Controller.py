'''
Created on May 15, 2019

@author: CalvinGregory
'''

from pose2D import pose2D

class Controller(object):
    '''
    classdocs
    '''

    def __init__(self, port_propeller_spin, starboard_propeller_spin):
        '''
        Constructor
        @param port_propeller_spin: coefficient indicating which direction the port propeller should spin. Acceptable values are +1 or -1
        @param starboard_propeller_spin: coefficient indicating which direction the starboard propeller should spin. Acceptable values are +1 or -1
        '''
        self._portPropSpin = port_propeller_spin
        self._starPropSpin = starboard_propeller_spin
        self._lastTime = -1
        self._lastPose = pose2D(0,0,0)
        self._waypoints = []
    
    def get_motor_speeds(self, sensorData):
        '''
        @return: integer tuple containing motor speeds (portSpeed, starboardSpeed).
        Motor speeds range from -127 to 127.
        '''
        pass