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

from pose2D import pose2D
import math

class Controller(object):
    '''
    Controller is an abstract class which interprets sensor data and produces motor speed outputs. 
    
    Attributes:
        _portPropSpin (int): coefficient indicating which direction the port propeller should spin. 
                             Acceptable values are +1 or -1
        _starPropSpin (int): coefficient indicating which direction the starboard propeller should spin. 
                             Acceptable values are +1 or -1
        _last_timestamp (float): Attribute tracks the timestamp of the most recently used sensorData message. 
        _lastPose (pose2D): Attribute tracks the most recent robot pose estimate. 
        _waypoints (pose2D): List of waypoints the microUSV should steer towards, in order. 
        _loop_waypoints (bool): Boolean flag indicating the boat should iterate through the waypoint list, 
                                then repeat the same list of waypoints. 
        _tag_offset_x (float): X-axis offset in mm between the AprilTag origin and microUSV origin.
        _tag_offset_y (float): Y-axis offset in mm between the AprilTag origin and microUSV origin.
        _tag_offset_yaw (float): Angular offset in radians between the AprilTag and microUSV coordinate frame. (ccw positive)
    '''

    def __init__(self, config):
        '''
        Builds a Controller object. 
        
        Args:
            config (mUSV/Config): Config file object containing controller initialization constants. 
        '''
        self._bias = config.bias
        self._portPropSpin = config.propSpin_port
        self._starPropSpin = config.propSpin_star
        if config.speed_limit > 0 and config.speed_limit <= 100:
            self._speed_limit_upper = int(round(127*config.speed_limit/100))
        else:
            self._speed_limit_upper = 127
        self._speed_limit_lower = -self._speed_limit_upper
        self._last_timestamp = -1
        self._lastPose = pose2D(0,0,0)
        self._waypoints = []
        self._loop_waypoints = False
        self._tag_offset_x = config.tagTF_x
        self._tag_offset_y = config.tagTF_y
        self._tag_offset_yaw = config.tagTF_yaw
    
    def get_motor_speeds(self, sensorData):
        '''
        Interprets sensor data according to the logic of the controller type implemented in the child class to produce
        motor speed values that move the microUSV toward its current goal. 
        
        Args:
            sensorData (musv_msg_pb2.SensorData): Protocol buffer SensorData message object containing the most 
                                                  recent simulated sensor values from the host machine.
            
        Returns:
            (int, int): integer tuple containing motor speeds (portSpeed, starboardSpeed).
                        Motor speed values range from -127 to 127.
        '''
        pass

    def _bounded_angle(self, angle, upper_bound, lower_bound):
        '''
        Constrains an angle in radians to a given range. 
        
        Args:
            angle (float): The angle to constrain. 
            upper_bound (float): The maximum value the angle can have. 
            lower_bound (float): The minimum value the angle can have.
            
        Returns:
            float: The equivalent angle to the input, constrained within the provided limits.
        '''
        new_angle = angle
        while new_angle > upper_bound:
            new_angle = new_angle - 2*math.pi
        while new_angle <= lower_bound:
            new_angle = new_angle + 2*math.pi
        return new_angle 

    def _bounded_motor_speeds(self, port_speed, starboard_speed):
        '''
        Corrects for desired motor outputs exceeding acceptable motor speed range.
        Wherever possible, the difference between motor values is maintained.
        
        Args:
            port_speed (float): Desired port motor speed, unconstrained. 
            starboard_speed (float): Desired starboard motor speed, unconstrained. 
            
        Returns:
            (int, int): Rounded and constrained motor speed values (port_motor_speed, starboard_motor_speed). 
        '''
        port = int(round(port_speed))
        starboard = int(round(starboard_speed))  
        diff = abs(max(port, starboard) - min(port, starboard))
        
        if diff >= 2*self._speed_limit_upper:
            port = int(math.copysign(self._speed_limit_upper, port))
            starboard = int(math.copysign(self._speed_limit_upper, starboard))
        
        elif port > self._speed_limit_upper or starboard > self._speed_limit_upper:
            if port > starboard:
                port = self._speed_limit_upper
                starboard = port - diff
            else:
                starboard = self._speed_limit_upper
                port = starboard - diff
        
        elif port < self._speed_limit_lower or starboard < self._speed_limit_lower:
            if port < starboard:
                port = self._speed_limit_lower
                starboard = port + diff
            else:
                starboard = self._speed_limit_lower
                port = starboard + diff
            
        return (port, starboard)