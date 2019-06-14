'''
Created on May 15, 2019

@author: CalvinGregory
'''

from Controller import Controller
from pose2D import pose2D
from time import time
import math
import numpy

class PIDController(Controller):
    '''
    classdocs
    '''

    def __init__(self, distance_PID_gains, angle_PID_gains, port_propeller_spin, starboard_propeller_spin):
        '''
        Constructor
        @param distance_PID_gains: tuple (P, I, D) of PID controller gains for distance control
        @param angle_PID_gains: tuple (P, I, D) of PID controller gains for angle control
        @param port_propeller_spin: coefficient indicating which direction the port propeller should spin. Acceptable values are +1 or -1
        @param starboard_propeller_spin: coefficient indicating which direction the starboard propeller should spin. Acceptable values are +1 or -1
        '''
        super(PIDController, self).__init__(port_propeller_spin, starboard_propeller_spin)
        self._distance_PID_gains = distance_PID_gains
        self._angle_PID_gains = angle_PID_gains
        self._last_error = (0, 0)
        self._motor_speeds = (0, 0)
        self._throttle = 1
        self._speed_limit_upper = 127
        self._speed_limit_lower = -127
        self._waypoint_threshold = 50 #TODO tune me
        self._time_offset = -1
        self._motor_distance = 0.048 # 48mm
        
    def set_throttle(self, speed):
        '''
        '''
        if speed < 0:
            self._throttle = 0
        elif speed > 1:
            self._throttle = 1
        else:
            self._throttle = speed    
        
    def get_motor_speeds(self, sensorData):
        '''
        @param sensorData: 
        @return: integer tuple containing motor speeds (portSpeed, starboardSpeed).
        Motor speeds range from -127 to 127.
        '''
        msg_timestamp = sensorData.last_updated.seconds + sensorData.last_updated.nanos*1e-9
        # If first message received, set time_offset.
        if self._time_offset < 0:
            self._time_offset = time() - msg_timestamp
        
        # If no more waypoints, stop motors.
        if len(self._waypoints) < 1:
            return (0, 0)
        # If no new pose data received for 2 seconds, stop motors.
        if time() - self._time_offset - msg_timestamp > 2.0:
            return (0, 0)
        
        if msg_timestamp > self._last_timestamp:
            pose = pose2D(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw)

            (distance_error, angular_error) = self._get_error(pose, self._waypoints[0])
            # P
            distance_control_value = self._distance_PID_gains[0]*distance_error
            angle_control_value = self._angle_PID_gains[0]*angular_error
            
            # I & D 
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
            (port, starboard) = self._get_bounded_motor_speeds(self._control_vals_to_speeds(throttled_speed, turn))
            
            self._motor_speeds = (port, starboard) 
            self._lastPose = pose2D(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw) 
            self._last_timestamp = msg_timestamp
            self._last_error = (distance_error, angular_error)
            
            if distance_error < self._waypoint_threshold:
                self._waypoints.pop()
        
        return self._motor_speeds
    
    def _get_error(self, robot_pose, goal_position):
        '''
        @param robot_pose: pose2D object containing the most up to date pose estimate from CVSensorSimulator
        @param goal_position: tuple (x, y) containing the coordinates of the goal
        '''
        dx = robot_pose.x - goal_position[0]
        dy = robot_pose.y - goal_position[1]
        distance_error = math.sqrt(dx**2 + dy**2)
        
        if robot_pose.yaw < 0:
            robot_pose.yaw = robot_pose.yaw + 2*math.pi()
        
        goal_angle = math.atan2(dy, dx)
        if goal_angle < 0:
            goal_angle = goal_angle + 2*math.pi()
        
        angular_error = robot_pose.yaw - goal_angle
        
        return (distance_error, angular_error)
    
    def _control_vals_to_speeds(self, forward_speed, turn_rate):
        '''
        
        '''
        port = forward_speed/2 - turn_rate/self._motor_distance
        starboard = forward_speed/2 + turn_rate/self._motor_distance
        return (port, starboard)
    
    def _get_bounded_motor_speeds(self, port_speed, starboard_speed):
        '''
        Corrects for desired motor outputs exceeding acceptable motor speed range.
        Wherever possible, the difference between motor values is maintained.
        '''
        port = int(round(port_speed))
        starboard = int(round(starboard_speed))  
        diff = abs(port - starboard)
        if diff >= 2*self._speed_limit_upper:
            port = self._speed_limit_upper*numpy.sign(port)
            starboard = self._speed_limit_upper*numpy.sign(starboard)
        
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
    