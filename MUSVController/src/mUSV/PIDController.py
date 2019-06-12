'''
Created on May 15, 2019

@author: CalvinGregory
'''

from Controller import Controller
from pose2D import pose2D
import math

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
        self._throttle = 0
        self._speed_limit_upper = 127
        self._speed_limit_lower = -127
        self._waypoint_threshold = 50 #TODO tune me
        
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
        currentTime = sensorData.last_updated.seconds + sensorData.last_updated.nanos *1e-9
        
        if len(self._waypoints) < 1:
            return (0, 0)
        
        if currentTime > self._lastTime:
            pose = pose2D(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw)

            (distance_error, angular_error) = self._get_error(pose, self._waypoints[0])
            # P
            distance_control_value = self._distance_PID_gains[0] * distance_error
            angle_control_value = self._angle_PID_gains[0] * angular_error
            
            # I & D 
            if self._lastTime > 0:
                dt = currentTime - self._lastTime
                
                dist_I_val = self._distance_PID_gains[1] * distance_error * dt
                dist_D_val = self._distance_PID_gains[2] * (distance_error - self._last_error[0]) / dt
                distance_control_value = distance_control_value + dist_I_val + dist_D_val
                
                ang_I_val = self._angle_PID_gains[1] * angular_error*dt
                ang_D_val = self._angle_PID_gains[2] * (angular_error - self._last_error[1]) / dt
                angle_control_value = angle_control_value + ang_I_val + ang_D_val 
            
            
            #TODO more elegant turning solution, accounting for motor speed limits
            throttled_speed = self._throttle * self._speed_limit_upper
            turn = throttled_speed * angle_control_value
            # update motor values
            port = round(self._portPropSpin * throttled_speed * distance_control_value + turn) 
            starboard = round(self._starPropSpin * throttled_speed * distance_control_value - turn)
            
            # DEBUG message
            print('port speed:', port)
            print('starboard speed:', starboard)
            
            self._motor_speeds = (port, starboard) 
            self._lastPose = pose2D(sensorData.pose_x, sensorData.pose_y, sensorData.pose_yaw) 
            self._lastTime = currentTime
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
        distance_error = math.sqrt(dx**2 + dy **2)
        
        if robot_pose.yaw < 0:
            robot_pose.yaw = robot_pose.yaw + 2*math.pi()
        
        goal_angle = math.atan2(dy, dx)
        if goal_angle < 0:
            goal_angle = goal_angle + 2*math.pi()
        
        angular_error = robot_pose.yaw - goal_angle
        
        return (distance_error, angular_error)
        