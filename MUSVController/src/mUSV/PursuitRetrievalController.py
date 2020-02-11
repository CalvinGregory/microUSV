from Controller import Controller
from pose2D import pose2D
import math

class PursuitRetrievalController(Controller):
    def __init__(self, config):
        super(PursuitRetrievalController, self).__init__(config)
        self._distance_PID_gains = (config.P_dist, config.I_dist, config.D_dist)
        self._angle_PID_gains = (config.P_ang, config.I_ang, config.D_ang)
        self._last_error = (0, 0)
        self._dist_error_sum = 0
        self._ang_error_sum = 0
        self._orbit_threshold = config.orbit_threshold
        self._cluster_threshold = config.cluster_threshold
        self._look_ahead_distance = 120.0
        self._lookAheadPoint = pose2D(self._orbit_threshold,0,0)
        self._reverse_maneuver_start_time = -10.0
        self._reverse_maneuver_duration = 2.0
        self._speed_limit_lower = -127
        self._execute_PID = True
        
    def get_motor_speeds(self, sensorData):
        self._execute_PID = True
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
        
        # If message includes new pose data
        if msg_timestamp > self._last_timestamp and (msg_timestamp - self._reverse_maneuver_start_time) > self._reverse_maneuver_duration:
            pose = pose2D(sensorData.pose.x, sensorData.pose.y, sensorData.pose.yaw)

            evade = False
            for nearbyVesselPose in sensorData.nearby_vessel_poses:
                vessel_range = self._getTargetRange(pose, nearbyVesselPose)
                vessel_heading_offset = super(PursuitRetrievalController, self)._bounded_angle(self._getTargetHeading(pose, nearbyVesselPose) - pose.yaw, math.pi, -math.pi)
                if vessel_range < 250 and abs(vessel_heading_offset) < 85*math.pi/180 or vessel_range < 500 and abs(vessel_heading_offset) < 20*math.pi/180:
                    evade = True
            
            if sensorData.target_sensors[-1]: #if carrying target
                if sensorData.clusterPoint.range > self._cluster_threshold:
                    clusterPoint = pose2D(0,0,0)
                    (distance_error, angular_error) = self._get_error(pose, clusterPoint)
                    distance_error = self._look_ahead_distance * 1.5
                else:
                    # full reverse for X seconds
                    port = self._speed_limit_lower
                    starboard = self._speed_limit_lower
                    self._reverse_maneuver_start_time = msg_timestamp
                    self._execute_PID = False
            elif evade:
                port = int(round(self._speed_limit_lower * 2/3))
                starboard = 0
                self._execute_PID = False
            else:
                if sensorData.clusterPoint.range > self._orbit_threshold:
                    tangent_line_heading = sensorData.clusterPoint.heading - math.asin(self._orbit_threshold/sensorData.clusterPoint.range)
                    # tangent_point_dist = math.sqrt(sensorData.clusterPoint.range**2 - self._orbit_threshold**2)
                else:
                    tangent_line_heading = sensorData.clusterPoint.heading - math.pi/180 * 100
                    
                self._lookAheadPoint.x = sensorData.pose.x + self._look_ahead_distance*math.sin(tangent_line_heading)
                self._lookAheadPoint.y = sensorData.pose.y - self._look_ahead_distance*math.cos(tangent_line_heading)
                    
                (distance_error, angular_error) = self._get_error(pose, self._lookAheadPoint)

                cluster_angle_error = super(PursuitRetrievalController, self)._bounded_angle(sensorData.clusterPoint.heading - sensorData.pose.yaw, math.pi, -math.pi)

                if (sensorData.clusterPoint.range > (self._orbit_threshold + self._cluster_threshold)/2) and (cluster_angle_error > math.pi/4 or cluster_angle_error < -math.pi/4): # if not facing cluster center
                # if angular_error < math.pi*7/8 and angular_error > -math.pi*7/8: # if not facing completely wrong way

                    ## OLD VEER COMMAND
                    # # veer left
                    # port = -self._speed_limit_upper / 2
                    # starboard = self._speed_limit_upper * 2/3
                    # port = port - (self._bias)/100*self._speed_limit_upper
                    # starboard = starboard + (self._bias)/100*self._speed_limit_upper
                    # (port, starboard) = super(PursuitRetrievalController, self)._bounded_motor_speeds(port, starboard)
                    # self._execute_PID = False


                    if sensorData.target_sensors[0] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 5
                    elif sensorData.target_sensors[1] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 15
                    elif sensorData.target_sensors[2] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 25
                    elif sensorData.target_sensors[3] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 35
                    elif sensorData.target_sensors[4] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 45
                    elif sensorData.target_sensors[5] > 0:
                        distance_error = self._look_ahead_distance
                        angular_error = -math.pi/180 * 55
            
            if self._execute_PID:
                # Apply Proportional gains 
                distance_control_value = self._distance_PID_gains[0]*distance_error
                angle_control_value = self._angle_PID_gains[0]*angular_error
                
                # Apply Integral and Derivative gains if not first message
                if self._last_timestamp > 0:
                    dt = msg_timestamp - self._last_timestamp
                    
                    self._dist_error_sum = 0.9*self._dist_error_sum + distance_error*dt
                    self._ang_error_sum = 0.9*self._ang_error_sum + angular_error*dt
                    
                    dist_I_val = self._distance_PID_gains[1]*self._dist_error_sum
                    dist_D_val = self._distance_PID_gains[2]*(distance_error - self._last_error[0]) / dt
                    distance_control_value = distance_control_value + dist_I_val + dist_D_val
                    
                    ang_I_val = self._angle_PID_gains[1]*self._ang_error_sum
                    ang_D_val = self._angle_PID_gains[2]*(angular_error - self._last_error[1]) / dt
                    angle_control_value = angle_control_value + ang_I_val + ang_D_val 
                
                forward_speed = self._speed_limit_upper*distance_control_value
                turn = self._speed_limit_upper*angle_control_value
                
                self._last_error = (distance_error, angular_error)

                #TODO Check bias math for negative thruster outputs
                port = (forward_speed + turn)/2
                port = port - (self._bias)/100*self._speed_limit_upper
                starboard = (forward_speed - turn)/2
                starboard = starboard + (self._bias)/100*self._speed_limit_upper
                (port, starboard) = super(PursuitRetrievalController, self)._bounded_motor_speeds(port, starboard)
            
            self._motor_speeds = (self._portPropSpin*port, self._starPropSpin*starboard) 
            # self._lastPose = pose
            self._last_timestamp = msg_timestamp
        
        return self._motor_speeds

    def _get_error(self, robot_pose, goal_pose):
        dx = goal_pose.x - robot_pose.x
        dy = goal_pose.y - robot_pose.y
        
        distance_error = math.sqrt(dx**2 + dy**2)
         
        goal_angle = math.atan2(dx, -dy) 
        angular_error = super(PursuitRetrievalController, self)._bounded_angle(goal_angle - robot_pose.yaw, math.pi, -math.pi)
                
        return (distance_error, angular_error)

    #TODO test me
    def _getTargetRange(self, my_pose, target_pose):
        return math.sqrt((my_pose.x - target_pose.x)**2 + (my_pose.y - target_pose.y)**2)

    #TODO test me
    def _getTargetHeading(self, my_pose, target_pose):
        return math.atan2(target_pose.x - my_pose.x, -(target_pose.y - my_pose.y))