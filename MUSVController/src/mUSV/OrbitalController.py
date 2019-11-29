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
from PIL import Image
import math

class OrbitalController(Controller):
    '''
    The OrbitalController class implements the algorithm described in OC2: A Swarm of Simple Robots Constructing Planar Shapes by Andrew Vardy. 
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
        super(OrbitalController, self).__init__(config)
        self._speed = 0.0
        self._orbit_threshold = config.orbit_threshold
        self._orbit_speed = config.orbit_speed
        self._orbit_veer = config.orbit_veer
        self._cluster_point = (0.0, 0.0)
        #TODO add if statement to select 720p vs 1080p
        scalar_field = Image.open('simple_field_720p.png', 'r')
        self._x_res = scalar_field.size[0]
        self._y_res = scalar_field.size[1]
        self._scalar_field_values = scalar_field.load()

        self._puck_mask = 13
        self._align_mask = 18
        
        FoV_deg = 78
        FoV_diag_hyp = config.tag_plane_distance*1000 / 4.66755 / math.cos(FoV_deg/2)
        FoV_diag_in_plane = FoV_diag_hyp * math.sin(FoV_deg/2)
        alpha = math.atan(720.0/1280.0)
        # Since origin is at center of the frame, max values are 1/2 of frame width. Full measurement range is [-max, +max].
        x_max_measurement = FoV_diag_in_plane * math.cos(alpha)
        # y_max_measurement = FoV_diag_in_plane * math.sin(alpha)
        self._px_per_mm = x_max_measurement/1280.0/2

        # left and right scalar field sensors are each X mm away from the vessel's apriltag center point
        self._scalar_field_LR_sensor_offsets = 30
        # the center scalar field sensor is X mm ahead of the vessel's apriltag center point
        self._scalar_field_C_sensor_offset = 30

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
            port = 0
            starboard = 0

            # Calculate scalar field sensor values
            #C
            xpx = sensorData.pose.xpx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.sin(sensorData.pose.yaw)))
            if xpx > self._x_res - 1:
                xpx = self._x_res -1
            elif xpx < 0:
                xpx = 0
            ypx = sensorData.pose.ypx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.cos(sensorData.pose.yaw)))
            if ypx > self._y_res - 1:
                ypx = self._y_res -1
            elif ypx < 0:
                ypx = 0
            C = self._scalar_field_values[xpx, ypx]

            #R
            xpx = sensorData.pose.xpx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.sin(sensorData.pose.yaw + math.pi/2)))
            if xpx > self._x_res - 1:
                xpx = self._x_res -1
            elif xpx < 0:
                xpx = 0
            ypx = sensorData.pose.ypx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.cos(sensorData.pose.yaw + math.pi/2)))
            if ypx > self._y_res - 1:
                ypx = self._y_res -1
            elif ypx < 0:
                ypx = 0
            R = self._scalar_field_values[xpx, ypx]

            #L
            xpx = sensorData.pose.xpx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.sin(sensorData.pose.yaw - math.pi/2)))
            if xpx > self._x_res - 1:
                xpx = self._x_res -1
            elif xpx < 0:
                xpx = 0
            ypx = sensorData.pose.ypx + int(round(self._scalar_field_C_sensor_offset*self._px_per_mm*math.cos(sensorData.pose.yaw - math.pi/2)))
            if ypx > self._y_res - 1:
                ypx = self._y_res -1
            elif ypx < 0:
                ypx = 0
            L = self._scalar_field_values[xpx, ypx]

            # Set order as a binary number
            if C >= R and R >= L:
                order = 1
            elif R >= C and C >= L:
                order = 2
            elif C >= L and L >= R:
                order = 4
            elif L >= C and C >= R:
                order = 8
            elif R >= L and L >= C:
                order = 16
            else: # L >= R and R >= C
                order = 32

            # Choose action
            if sensorData.clusterPoint.range < self._orbit_threshold:
                # veer left
                port = 0
                starboard = self._orbit_speed/100*self._speed_limit_upper
            elif (self._puck_mask & order != 0) and sensorData.target_sensors[0]:
                # veer left
                port = 0
                starboard = self._orbit_speed/100*self._speed_limit_upper
            elif (self._align_mask & order != 0):
                # veer left
                port = 0
                starboard = self._orbit_speed/100*self._speed_limit_upper
            else: 
                # veer right
                port = self._orbit_speed/100*self._speed_limit_upper
                starboard = 0

            port = port - self._bias/100
            starboard = starboard + self._bias/100
            (port, starboard) = super(OrbitalController, self)._bounded_motor_speeds(port, starboard)                                  
            
            self._motor_speeds = (self._portPropSpin*port, self._starPropSpin*starboard) 
            self._lastPose = sensorData.pose
            self._last_timestamp = msg_timestamp

        print('motor speeds', self._motor_speeds)
        return self._motor_speeds
