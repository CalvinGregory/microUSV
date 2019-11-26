This directory contains the code for two applications: MUSVController and Teleop. 

# MUSVController

MUSVController is the control code for the microUSV's onboard Raspberry Pi. It polls a host PC running CVSensorSimulator for simulated sensor data, then uses PID control for waypoint following. 

The application can be configured by modifying the config.json file. When performing a new setup, the config file values must be adjusted. The *PropellerSpin* coefficients and *bias* value should match the values found during the [Teleoperation Test](https://osf.io/k7a8p/wiki/14.%20Teleoperation%20Test/).

| Config Field | Type | Value |
|:---          |:---: |:---   |
| serverIP | String | IP address of the host PC running CVSensorSimulator |
| tagID | Integer | ID number of the AprilTag on this microUSV |
| label | String | **Unused field:** display name for this microUSV |
| tagTransform |  | The translation and rotation between the microUSV's AprilTag and its coordinate frame |
| tagTransform:x | Float | X position offset in mm |
| tagTransform:y | Float | Y position offset in mm |
| tagTransform:z | Float | **Unused field:** Z position offset in mm |
| tagTransform:yaw | Float | Rotation offset in radians |
| PropellerSpin |  | Integer coefficients (1 or -1) which indicate if the propellers should spin forward or backward. |
| PropellerSpin:port | Integer | Port (left) propeller coefficient |
| PropellerSpin:starboard | Integer | Starboard (right) propeller coefficient |
| bias | Float | Correction term to account for inequality between port and starboard motor outputs. Value must be a percentage between -100.0 and +100.0. Positive bias causes the vehicle to turn more to the left (port) while negative bias causes a right (starboard) turn. |
| PIDGains |  | Gains for three PID controllers: distance, angular, and speed. |
| PIDGains:P_dist | Float | Distance controller proportional gain |
| PIDGains:I_dist | Float | Distance controller integral gain |
| PIDGains:D_dist | Float | Distance controller differential gain |
| PIDGains:P_ang | Float | Angular controller proportional gain |
| PIDGains:I_ang | Float | Angular controller integral gain |
| PIDGains:D_ang | Float | Angular controller differential gain |
| PIDGains:P_speed | Float | Speed controller proportional gain |
| PIDGains:I_speed | Float | Speed controller integral gain |
| speed_limit | Float | Percentage (0 to 100] of the maximum motor speed to treat as an upper bound on motor speed commands |
| tag_plane_distance | Float | Distance from the camera to the water's surface in meters |
| controller_type | String | Flag indicating the type of controller to use. 'pid' for PID control and 'orbital' for Orbital construction | 
| orbit_threshold | Float | Orbit distance from cluster point |
| orbit_speed | Float | Target forward speed for orbital controller |
| orbit_veer | Float | Orbital controller steering parameter given as a percentage (0 to 100). Controls how much the vessel tries to veer of course if it detects a target nearby. |
| debug_mode | Boolean | Flag enabling debug mode. Debug mode removes calls to Arduino serial port, allows testing in desktop environment |

Launch the application by connecting to the microUSV's raspberry pi over ssh and executing the following.

```
$ python MUSVClient.py [PATH_TO_CONFIG.JSON]
```

# Teleop

Teleop is a basic teleoperation controller for the microUSV. It allows a user to ssh into a microUSV and control the motors using the wasd, qezc, and 123 keys as inputs through a terminal window. It is intended primarily as a quick tuning/debugging tool to ensure a microUSV's motors are behaving as expected. 

## Keyboard Input Mappings
 - W - Drive forward
 - A - Pivot counter clockwise
 - S - Drive backward
 - D - Pivot clockwise
 
 - Q - Drive forward and left
 - E - Drive forward and right
 - Z - Drive backward and left
 - C - Drive Backward and right
 
 - 1 - Set motor speed commands to preset low (~59% power)
 - 2 - Set motor speed commands to preset medium (~79% power)
 - 3 - Set motor speed commands to preset high (100% power)

 - ESC - Exit the Teleop program

The Teleop application reads the same config file format as the MUSVController application. Only the PropellerSpin and bias variables are used. 

To launch the program, ssh into the microUSV's raspberry pi.  Navigate to the directory containing musv_teleop.py and launch the script using python.

```
$ python teleop.py [PATH_TO_CONFIG.JSON]
```

The application will cause the terminal window to go blank and intercept keyboard inputs. The application can be terminated by hitting the Escape key or Ctrl+C.
