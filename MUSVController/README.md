# MUSVController

MUSVController is the control code for the microUSV's onboard Raspberry Pi. It polls a host PC running CVSensorSimulator for simulated sensor data, then uses PID control for waypoint following. 

The application can be configured by modifying the config.json file. When performing a new setup, the config file values must be adjusted. 

Config Field | Type | Value
-------------|------|------
serverIP | String | IP address of the host PC running CVSensorSimulator
tagID | Integer | ID number of the AprilTag on this microUSV
label | String | **Unused field:** display name for this microUSV
tagTransform |  | The translation and rotation between the microUSV's AprilTag and its coordinate frame
tagTransform:x | Float | X position offset in mm
tagTransform:y | Float | Y position offset in mm
tagTransform:yaw | Float | Rotation offset in radians
PropellerSpin |  | Integer coefficients (1 or -1) which indicate if the propellers should spin forward or backward. These should match the coefficients found during the [Teleoperation Test](https://osf.io/k7a8p/wiki/14.%20Teleoperation%20Test/)
PropellerSpin:port | Integer | Port (left) propeller coefficient
PropellerSpin:starboard | Integer | Starboard (right) propeller coefficient
PIDGains |  | Gains for the two PID controllers: distance and angular. 
PIDGains:P_dist | Float | Distance controller proportional gain
PIDGains:I_dist | Float | Distance controller integral gain
PIDGains:D_dist | Float | Distance controller differential gain
PIDGains:P_ang | Float | Angular controller proportional gain
PIDGains:I_ang | Float | Angular controller integral gain
PIDGains:D_ang | Float | Angular controller differential gain
speed_limit | Float | Percentage (0 to 100] of the maximum motor speed to treat as an upper bound on motor speed commands
tag_plane_distance | Float | Distance from the camera to the water's surface in meters

Launch the application by connecting to the microUSV's raspberry pi over ssh and executing the following.

```
python MUSVClient.py [PATH_TO_CONFIG.JSON]
```
