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
from time import time
import math

class PIDController(Controller):
    '''
    The PIDController class implements a pair of PID controllers (distance control and angular control) 
    to steer the microUSV toward its goal position.  
    
    Attributes:
        _distance_PID_gains (float, float, float): Tuple containing the PID gains for the distance controller (P, I, D).
        _angle_PID_gains (float, float, float): Tuple containing the PID gans for the angular controller (P, I, D). 
        _last_error (float, float): Tuple containing the position error from the previous timestep (x, y).
        _motor_speeds (int, int): Tuple containing the speed values the microUSV's motors should currently be set to. 
        _throttle (float): Baseline motor output speed. Value should be in range 0 to 1. 
        _speed_limit_upper (int): Maximum motor speed value. 
        _speed_limit_lower (int): Minimum motor speed value.
        _waypoint_threshold (float): Threshold indicating at what distance (mm) to treat the current waypoint as reached. 
        _time_offset (float): Offset between internal clock and SensorData message timestamps. 
    '''

    def __init__(self, distance_PID_gains, angle_PID_gains, port_propeller_spin, starboard_propeller_spin):
        '''
        Initialize a PIDController object. 
        
        Args: 
            distance_PID_gains (float, float, float): tuple (P, I, D) of PID controller gains for distance control
            angle_PID_gains (float, float, float): tuple (P, I, D) of PID controller gains for angle control
            port_propeller_spin (int): coefficient indicating which direction the port propeller should spin. 
                                       Acceptable values are +1 or -1
            starboard_propeller_spin (int): coefficient indicating which direction the starboard propeller should spin. 
                                            Acceptable values are +1 or -1
        '''
        super(PIDController, self).__init__(port_propeller_spin, starboard_propeller_spin)
        self._distance_PID_gains = distance_PID_gains
        self._angle_PID_gains = angle_PID_gains
        self._last_error = (0, 0)
        self._motor_speeds = (0, 0)
        self._throttle = 1
        self._speed_limit_upper = 127
        self._speed_limit_lower = -127
        self._waypoint_threshold = 50.0 #TODO tune me
        self._time_offset = -1
        self._distance_scale_factor = 1.5 # per meter of height between camera and tag plane
        self._tag_plane_distance = 3 # meters
        
    def set_throttle(self, speed):
        '''
        Sets the internal _throttle variable. 
        Function caps the throttle value to a range from 0 to 1. 
       
        Args:
            speed (float): The desired throttle value. 
        '''
        if speed < 0:
            self._throttle = 0
        elif speed > 1:
            self._throttle = 1
        else:
            self._throttle = speed    
        
    def get_motor_speeds(self, sensorData, tag_offset_x, tag_offset_y, tag_offset_yaw):
        '''
        Applies distance and angular PID controllers to the provided sensor data. Calculates motor speeds
        that will move the microUSV closer to its current goal position. 
        
        Args:
            sensorData (musv_msg_pb2.SensorData): Protocol buffer SensorData message object containing the most 
                                                  recent simulated sensor values from the host machine.
            tag_offset_x (float): X-axis offset in mm between the AprilTag origin and microUSV origin.
            tag_offset_y (float): Y-axis offset in mm between the AprilTag origin and microUSV origin.
            tag_offset_yaw (float): Angular offset in radians between the AprilTag and microUSV coordinate frame. (ccw positive)
            
        Returns:
            (int, int): integer tuple containing motor speeds (portSpeed, starboardSpeed).
                        Motor speed values range from -127 to 127.
        '''
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
        # If this is the first message received, set time_offset.
        if self._time_offset < 0:
            self._time_offset = time() - msg_timestamp
        
        # If the message contains waypoints, overwrite the current waypoint list and loop flag. 
        if sensorData.waypoints.__len__() > 0:
            self._waypoints = []
            for w in sensorData.waypoints:
                self._waypoints.append(pose2D(w.x, w.y, 0))
            self._loop_waypoints = sensorData.loop_waypoints
        
        # If no more waypoints in waypoint list, stop motors.
        if len(self._waypoints) < 1:
            return (0, 0)
        
        # If no new pose data received for 2 seconds, stop motors.
        if time() - self._time_offset - msg_timestamp > 2.0:
            return (0, 0)
        
        # If message includes new pose data
        if msg_timestamp > self._last_timestamp:
            # Apply tag offset transform
            x_offset_tagFrame = math.cos(tag_offset_yaw)*tag_offset_x + math.sin(tag_offset_yaw)*tag_offset_y
            y_offset_tagFrame = -math.sin(tag_offset_yaw)*tag_offset_x + math.cos(tag_offset_yaw)*tag_offset_y
            # Apply yaw transform
            x_offset_worldFrame = math.cos(sensorData.pose.yaw)*x_offset_tagFrame + math.sin(sensorData.pose.yaw)*y_offset_tagFrame
            y_offset_worldFrame = -math.sin(sensorData.pose.yaw)*x_offset_tagFrame + math.cos(sensorData.pose.yaw)*y_offset_tagFrame
            
            x = (sensorData.pose.x + x_offset_worldFrame) * self._distance_scale_factor * self._tag_plane_distance
            y = (sensorData.pose.y + y_offset_worldFrame) * self._distance_scale_factor * self._tag_plane_distance
            yaw = self._bounded_angle(sensorData.pose.yaw - tag_offset_yaw, math.pi, -math.pi)
            
            pose = pose2D(x, y, yaw)
            
            #DEBUG
            print("pose: x:{} y:{} yaw:{}".format(pose.x, pose.y, pose.yaw))

            (distance_error, angular_error) = self._get_error(pose, self._waypoints[0])
            # Apply Proportional gains 
            distance_control_value = self._distance_PID_gains[0]*distance_error
            angle_control_value = self._angle_PID_gains[0]*angular_error
            
            # Apply Integral and Derivative gains if not first message
            if self._last_timestamp > 0:
                dt = msg_timestamp - self._last_timestamp
                
                dist_I_val = self._distance_PID_gains[1]*distance_error*dt
                dist_D_val = self._distance_PID_gains[2]*(distance_error - self._last_error[0]) / dt
                distance_control_value = distance_control_value + dist_I_val + dist_D_val
                
                ang_I_val = self._angle_PID_gains[1]*angular_error*dt
                ang_D_val = self._angle_PID_gains[2]*(angular_error - self._last_error[1]) / dt
                angle_control_value = angle_control_value + ang_I_val + ang_D_val 
            
            throttled_speed = self._throttle*self._speed_limit_upper*distance_control_value
            turn = throttled_speed*angle_control_value
            (port, starboard) = self._components_to_speeds(throttled_speed, turn)
            (port, starboard) = self._get_bounded_motor_speeds(port, starboard)
            #DEBUG message
#             print ("motor speeds: ({}, {})".format(port, starboard))
            
            self._motor_speeds = (port, starboard) 
            self._lastPose = pose
            self._last_timestamp = msg_timestamp
            self._last_error = (distance_error, angular_error)
            
            if distance_error < self._waypoint_threshold:
                if self._loop_waypoints:
                    self._waypoints.append(self._waypoints.pop())
                else:
                    self._waypoints.pop()
        
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
        dx = robot_pose.x - goal_pose.x
        dy = robot_pose.y - goal_pose.y
        distance_error = math.sqrt(dx**2 + dy**2)
        
        goal_angle = math.atan2(dy, dx)        
        angular_error = self._bounded_angle(robot_pose.yaw - goal_angle, math.pi, -math.pi)
        
        return (distance_error, angular_error)
    
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
    
    def _components_to_speeds(self, forward_speed, turn_rate):
        '''
        Calculates the desired motor speeds given a desired linear speed and turn rate.
        
        Args:
            forward_speed (float): The desired forward speed. 
            turn_rate (float): The desired turn rate (ccw positive). 
            
        Returns:
            (float, float): Motor speed tuple (port_motor_speed, starboard_motor_speed).
        '''
        port = (forward_speed - turn_rate)/2
        starboard = (forward_speed + turn_rate)/2
        return (port, starboard)
    
    def _get_bounded_motor_speeds(self, port_speed, starboard_speed):
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
        diff = abs(port - starboard)
        if diff >= 2*self._speed_limit_upper:
            port = int(math.copysign(self._speed_limit_upper, port))
            starboard = int(math.copysign(self._speed_limit_upper, starboard))
        
        elif port > self._speed_limit_upper:
            port = self._speed_limit_upper
            starboard = port - diff
        elif starboard > self._speed_limit_upper:
            starboard = self._speed_limit_upper
            port = starboard - diff
        elif port < self._speed_limit_lower:
            port = self._speed_limit_lower
            starboard = port + diff
        elif starboard < self._speed_limit_lower:
            starboard = self._speed_limit_lower
            port = starboard + diff
            
        return (port, starboard)
    