from Controller import Controller
from pose2D import pose2D
import math

class OrbitalController(Controller):

    def __init__(self, config):
        super(OrbitalController, self).__init__(config)

    def get_motor_speeds(self, sensorData):
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos*1e-9
        
        # If the message contains waypoints, overwrite the current waypoint list and loop flag. 
        if sensorData.waypoints.__len__() > 0:
            self._waypoints = []
            for w in sensorData.waypoints:
                self._waypoints.append(pose2D(w.x, w.y, 0))
            self._loop_waypoints = sensorData.loop_waypoints
                        
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