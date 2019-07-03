# MUSVController

MUSVController is the control code for the microUSV's onboard Raspberry Pi. It polls a host PC running CVSensorSimulator for simulated sensor data, then uses PID control for waypoint following. 

The application can be configured by modifying the config.json file. When performing a new setup, the config file values must be adjusted. 

Config Field | Value
------------------|------
serverIP | IP address of the host PC running CVSensorSimulator
tagID | ID number of the AprilTag on this microUSV
label | **Unused field:** display name for this microUSV
tagTransform | The translation and rotation between the microUSV's AprilTag and coordinate frame. 
PropellerSpin | Integer coefficients (1 or -1) which indicate if the propellers should spin forward or backward depending on their helix angles. If unsure, test expected behavior and propeller coefficients using the teleop program. 
PIDGains | Gains for the two PID controllers: distance and angular. 
speed_limit | Percentage (0 to 100] of the maximum motor speed to treat as an upper bound on motor speed commands.

Launch the application by connecting to the microUSV's raspberry pi over ssh and executing the following.

```
python MUSVClient.py [PATH_TO_CONFIG.JSON]
```
