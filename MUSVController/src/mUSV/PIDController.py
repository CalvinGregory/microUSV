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

class PIDController(Controller):
    '''
    The PIDController class implements a pair of PID controllers (distance control and angular control) 
    to steer the microUSV toward its goal position.  
    
    Attributes:
        _bias (float): Correction term to account for inequality between port and starboard motor outputs. Value must be a 
                       percentage between -100.0 and +100.0. Positive bias causes the vehicle to turn more to the left (port) 
                       while negative bias causes a right (starboard) turn. 
        _distance_PID_gains (float, float, float): Tuple containing the PID gains for the distance controller (P, I, D).
        _angle_PID_gains (float, float, float): Tuple containing the PID gains for the angular controller (P, I, D). 
        _last_error (float, float): Tuple containing the position error from the previous timestep (x, y).
        _motor_speeds (int, int): Tuple containing the speed values the microUSV's motors should currently be set to.  
        _speed_limit_upper (int): Maximum motor speed value. 
        _speed_limit_lower (int): Minimum motor speed value.
        _waypoint_threshold (float): Threshold indicating at what distance (mm) to treat the current waypoint as reached.
        _tag_plane_distance (float): Distance from the camera to the water's surface in meters.
    '''

    def __init__(self, config):
        '''
        Initialize a PIDController object. 
        
        Args: 
            config (mUSV/Config): Config file object containing controller initialization constants. 
        '''
        super(PIDController, self).__init__(config)
        self._distance_PID_gains = (config.P_dist, config.I_dist, config.D_dist)
        self._angle_PID_gains = (config.P_ang, config.I_ang, config.D_ang)
        self._last_error = (0, 0)
        self._dist_error_sum = 0
        self._ang_error_sum = 0
        self._waypoint_threshold = 50.0
        self._waypoints = []
        self._loop_waypoints = False
        # Scale factor per meter of height between camera and water's surface
#        self._distance_scale_factor = 0.455 # 1080p
        self._distance_scale_factor = 0.3 # 720p
        self._tag_plane_distance = config.tag_plane_distance
        
    def get_motor_speeds(self, sensorData):
        '''
        Applies distance and angular PID controllers to the provided sensor data. Calculates motor speeds
        that will move the microUSV closer to its current goal position. 
        
        Args:
            sensorData (musv_msg_pb2.SensorData): Protocol buffer SensorData message object containing the most 
                                                  recent simulated sensor values from the host machine.
                                                              
        Returns:
            (int, int): integer tuple containing motor speeds (portSpeed, starboardSpeed).
                        Motor speed values range from -127 to 127.
        '''
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
        
        # If the message contains waypoints, overwrite the current waypoint list and loop flag. 
        if sensorData.waypoints.__len__() > 0:
            self._waypoints = []
            for w in sensorData.waypoints:
                self._waypoints.append(pose2D(w.x, w.y, 0))
            self._loop_waypoints = sensorData.loop_waypoints
        
        # If no more waypoints in waypoint list, stop motors.
        if len(self._waypoints) < 1:
            return (0, 0)
                
        # If message includes new pose data
        if msg_timestamp > self._last_timestamp:
            # Apply tag offset transform
            x_offset_tagFrame = math.cos(self._tag_offset_yaw)*self._tag_offset_x + math.sin(self._tag_offset_yaw)*self._tag_offset_y
            y_offset_tagFrame = -math.sin(self._tag_offset_yaw)*self._tag_offset_x + math.cos(self._tag_offset_yaw)*self._tag_offset_y
            # Apply yaw transform
            x_offset_worldFrame = math.cos(sensorData.pose.yaw)*x_offset_tagFrame + math.sin(sensorData.pose.yaw)*y_offset_tagFrame
            y_offset_worldFrame = -math.sin(sensorData.pose.yaw)*x_offset_tagFrame + math.cos(sensorData.pose.yaw)*y_offset_tagFrame
            
            x = (sensorData.pose.x + x_offset_worldFrame) * self._distance_scale_factor * self._tag_plane_distance
            y = (sensorData.pose.y + y_offset_worldFrame) * self._distance_scale_factor * self._tag_plane_distance
            yaw = super(PIDController, self)._bounded_angle(sensorData.pose.yaw - self._tag_offset_yaw, math.pi, -math.pi)            

            pose = pose2D(x, y, yaw)

            (distance_error, angular_error) = self._get_error(pose, self._waypoints[0])

            # Apply Proportional gains 
            distance_control_value = self._distance_PID_gains[0]*distance_error
            angle_control_value = self._angle_PID_gains[0]*angular_error
            
            # Apply Integral and Derivative gains if not first message
            if self._last_timestamp > 0:
                dt = msg_timestamp - self._last_timestamp
                
                self._dist_error_sum = 0.99*self._dist_error_sum + distance_error*dt
                self._ang_error_sum = 0.99*self._ang_error_sum + angular_error*dt
                
                dist_I_val = self._distance_PID_gains[1]*self._dist_error_sum
                dist_D_val = self._distance_PID_gains[2]*(distance_error - self._last_error[0]) / dt
                distance_control_value = distance_control_value + dist_I_val + dist_D_val
                
                ang_I_val = self._angle_PID_gains[1]*self._ang_error_sum
                ang_D_val = self._angle_PID_gains[2]*(angular_error - self._last_error[1]) / dt
                angle_control_value = angle_control_value + ang_I_val + ang_D_val 
            
            forward_speed = self._speed_limit_upper*distance_control_value
            turn = self._speed_limit_upper*angle_control_value
            
            port = (forward_speed - turn)/2
            port = port - (self._bias)/100
            starboard = (forward_speed + turn)/2
            starboard = starboard + (self._bias)/100
            (port, starboard) = super(PIDController, self)._bounded_motor_speeds(port, starboard)
            
            self._motor_speeds = (self._portPropSpin*port, self._starPropSpin*starboard) 
            self._lastPose = pose
            self._last_timestamp = msg_timestamp
            self._last_error = (distance_error, angular_error)

            if distance_error < self._waypoint_threshold:
                self._dist_error_sum = 0
                self._ang_error_sum = 0
                if self._loop_waypoints:
                    self._waypoints.append(self._waypoints.pop(0))
                else:
                    self._waypoints.pop(0)
        
        return self._motor_speeds

    def _get_error(self, robot_pose, goal_pose):
        '''
        Calculates the distance and angular error between the microUSV and its current goal position. 
        Distance error is the distance between the microUSV and it's goal. 
        Angular error is the difference between the microUSV's heading and the direction toward it's goal.    
        
        Args:
            robot_pose (pose2D): pose2D object containing the most up to date pose estimate from CVSensorSimulator.
            goal_position (pose2D): pose2D object containing the coordinates of the goal. Note: yaw is ignored.
            
        Returns:
            (float, float): Tuple containing the distance and angular errors (distance_error, angular_error).
        '''
        dx = goal_pose.x - robot_pose.x
        dy = goal_pose.y - robot_pose.y
        
        distance_error = math.sqrt(dx**2 + dy**2)
        
        goal_angle = math.atan2(dy, dx)
        angular_error = super(PIDController, self)._bounded_angle(robot_pose.yaw - goal_angle, math.pi, -math.pi)
                
        return (distance_error, angular_error)