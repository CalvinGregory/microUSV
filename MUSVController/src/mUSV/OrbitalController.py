from Controller import Controller
from pose2D import pose2D
import math

class OrbitalController(Controller):

    def __init__(self, config):
        super(OrbitalController, self).__init__(config)
        self._speed = 0.0
        self._orbit_threshold = config.orbit_threshold
        self._orbit_speed = config.orbit_speed
        self._orbit_veer = config.orbit_veer
        self._speed_PID_gains = (config.P_speed, config.I_speed)
        self._speed_error_sum = 0
        self._heading_error_tolerance = math.pi/90

    def get_motor_speeds(self, sensorData):
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
            yaw = super(OrbitalController, self)._bounded_angle(sensorData.pose.yaw - self._tag_offset_yaw, math.pi, -math.pi)
            
            pose = pose2D(x, y, yaw)
            heading_error = super(OrbitalController,self)._bounded_angle(sensorData.clusterPoint.heading - pose.yaw, 2*math.pi, 0)
            
            port = 0
            starboard = 0

            # If not first message
            if self._last_timestamp > 0:
                dt = msg_timestamp - self._last_timestamp
                self._speed = math.sqrt((pose.x - self._lastPose.x)**2 + (pose.y - self._lastPose.y)**2) / dt
                speed_error = self._orbit_speed - self._speed
                self._speed_error_sum = 0.9*self._speed_error_sum + speed_error*dt

                # print (self._speed_error_sum)

                # PI Controller
                speed_control_value = self._speed_PID_gains[0]*speed_error + self._speed_PID_gains[1]*self._speed_error_sum
                
                # print ('speed_control_value', speed_control_value)
                
                port = self._speed_limit_upper*(speed_control_value - self._bias/100)
                starboard = self._speed_limit_upper*(speed_control_value + self._bias/100)
                
                # if inside cluster area
                if sensorData.clusterPoint.range < self._orbit_threshold:
                    # veer left
                    port = port - self._orbit_veer/100*self._speed_limit_upper
                    starboard = starboard + self._orbit_veer/100*self._speed_limit_upper
                # if target detected to port
                elif (sensorData.target_sensors[0]):
                    # veer left
                    port = port - self._orbit_veer/100*self._speed_limit_upper
                    starboard = starboard + self._orbit_veer/100*self._speed_limit_upper
                # if not perpendicular to cluster center point
                elif heading_error < math.pi/2 - self._heading_error_tolerance:
                    # veer left
                    port = port - self._orbit_veer/100*self._speed_limit_upper
                    starboard = starboard + self._orbit_veer/100*self._speed_limit_upper
                elif heading_error > math.pi/2 + self._heading_error_tolerance:
                    # veer right
                    port = port + self._orbit_veer/100*self._speed_limit_upper
                    starboard = starboard - self._orbit_veer/100*self._speed_limit_upper
                # else go straight
                    
            
            (port, starboard) = super(OrbitalController, self)._bounded_motor_speeds(port, starboard)
            
            self._motor_speeds = (self._portPropSpin*port, self._starPropSpin*starboard) 
            self._lastPose = pose
            self._last_timestamp = msg_timestamp

        # print('motor speeds', self._motor_speeds)
        return self._motor_speeds