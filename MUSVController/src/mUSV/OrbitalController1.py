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

from Controller import Controller
from pose2D import pose2D
import math

class OrbitalController1(Controller):
    '''
    The OrbitalController1 class implements the algorithm described in Orbital Construction: Swarms of Simple Robots Building Enclosures by Andrew Vardy. 
    Each vessel orbits clockwise around a point, gathering nearby targets outside of the orbit into a cluster at that point. 

    Attributes:
        _orbit_threshold (float): Distance from the cluster point at which to orbit.
        _orbit_speed (float): Forward speed at which to orbit.
        _orbit_veer (float): Steering parameter. Controls how much the vessel tries to veer off course if it detects a target. 
        _speed_PI_gains (float, float): Tuple containing the PI gains for the vessel's speed controller (P, I).
    '''
    def __init__(self, config):
        '''
        Initialize an OrbitalController object.

        Args:
            config (mUSV/Config): Config file object containing controller initialization constants. 
        '''
        super(OrbitalController1, self).__init__(config)
        self._speed = 0.0
        self._orbit_threshold = config.orbit_threshold
        self._orbit_speed = config.orbit_speed
        self._orbit_veer = config.orbit_veer
        self._speed_PI_gains = (config.P_speed, config.I_speed)
        self._speed_error_sum = 0
        self._heading_error_tolerance = math.pi/180*5
        # self._speed_limit_lower = 0

    def get_motor_speeds(self, sensorData):
        '''
        Applies orbital and speed control logic to the provided sensor data. Calculates motor speeds
        that will move the microUSV along the orbit's trajectory or towards a nearby target. 
        
        Args:
            sensorData (musv_msg_pb2.SensorData): Protocol buffer SensorData message object containing the most 
                                                  recent simulated sensor values from the host machine.
                                                              
        Returns:
            (int, int): integer tuple containing motor speeds (portSpeed, starboardSpeed).
                        Motor speed values range from -127 to 127.
        '''
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
                                
        # If message includes new pose data
        if msg_timestamp > self._last_timestamp:
            # Apply tag offset transform
            x_offset_tagFrame = math.cos(self._tag_offset_yaw)*self._tag_offset_x + math.sin(self._tag_offset_yaw)*self._tag_offset_y
            y_offset_tagFrame = -math.sin(self._tag_offset_yaw)*self._tag_offset_x + math.cos(self._tag_offset_yaw)*self._tag_offset_y
            # Apply yaw transform
            x_offset_worldFrame = math.cos(sensorData.pose.yaw)*x_offset_tagFrame + math.sin(sensorData.pose.yaw)*y_offset_tagFrame
            y_offset_worldFrame = -math.sin(sensorData.pose.yaw)*x_offset_tagFrame + math.cos(sensorData.pose.yaw)*y_offset_tagFrame
            
            x = sensorData.pose.x + x_offset_worldFrame
            y = sensorData.pose.y + y_offset_worldFrame
            yaw = super(OrbitalController1, self)._bounded_angle(sensorData.pose.yaw - self._tag_offset_yaw, math.pi, -math.pi)
            
            pose = pose2D(x, y, yaw)
            heading_error = super(OrbitalController1,self)._bounded_angle(sensorData.clusterPoint.heading - pose.yaw, 2*math.pi, 0)
            
            port = 0
            starboard = 0

            # If not first message
            if self._last_timestamp > 0:
                dt = msg_timestamp - self._last_timestamp
                self._speed = math.sqrt((pose.x - self._lastPose.x)**2 + (pose.y - self._lastPose.y)**2) / dt

                print('speed', self._speed)
                
                speed_error = self._orbit_speed - self._speed

                print('speed_error', speed_error)

                self._speed_error_sum = 0.9*self._speed_error_sum + speed_error*dt

                # PI Controller
                speed_control_value = self._speed_PI_gains[0]*speed_error + self._speed_PI_gains[1]*self._speed_error_sum
                
                # print ('speed_control_value', speed_control_value)
                
                port = self._speed_limit_upper*(speed_control_value - self._bias/100)
                starboard = self._speed_limit_upper*(speed_control_value + self._bias/100)
                (port, starboard) = super(OrbitalController1, self)._bounded_motor_speeds(port, starboard)
                
                #TODO make puck seeking optional (wrapped in if statement with flag)
                # if aligned
                if heading_error > math.pi/2 - self._heading_error_tolerance and heading_error <= math.pi/2 + self._heading_error_tolerance:
                    if sensorData.clusterPoint.range < self._orbit_threshold:
                        # slight veer left
                        print ('slight veer left')
                        if math.copysign(1,speed_control_value) > 0:
                            port = int(round(0.4*port))
                        else:
                            starboard = int(round(0.4*starboard))
                    else:
                        # slight veer right
                        print ('slight veer right')
                        if math.copysign(1,speed_control_value) > 0:
                            starboard = int(round(0.4*starboard))
                        else:
                            port = int(round(0.4*port))
                # if heading uphill
                elif heading_error <= math.pi/2 - self._heading_error_tolerance:
                    print ('veer left')
                    if math.copysign(1,speed_control_value) > 0:
                        port = 0
                    else:
                        starboard = 0
                # if heading downhill
                else:
                    print('veer right')
                    if math.copysign(1,speed_control_value) > 0:
                        starboard = 0
                    else:
                        port = 0







                
                # # if inside cluster area
                # if sensorData.clusterPoint.range < self._orbit_threshold:
                #     # veer left
                #     port = 0
                # # if target detected to port
                # elif (sensorData.target_sensors[0]):
                #     # veer left
                #     port = 0
                # # if not perpendicular to cluster center point
                # elif heading_error < math.pi/2 - self._heading_error_tolerance:
                #     # veer left
                #     port = 0
                # else:
                #     # veer right
                #     starboard = 0
                # # else go straight
                    
            
            
            
            self._motor_speeds = (self._portPropSpin*port, self._starPropSpin*starboard) 
            self._lastPose = pose
            self._last_timestamp = msg_timestamp

        print('motor speeds', self._motor_speeds)
        return self._motor_speeds
