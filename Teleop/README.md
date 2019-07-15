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

## Config File
The application can be configured by modifying the config.json file. These parameters should be tuned to accomodate different propeller helix angles, motor wiring configurations, and friction inequalities introduced during construction. 

Config Field | Type | Value
-------------|------|------
PropellerSpin |  | Integer coefficients (1 or -1) which indicate if the propellers should spin forward or backward. 
PropellerSpin:port | Integer | Port (left) propeller coefficient
PropellerSpin:starboard | Integer | Starboard (right) propeller coefficient
bias | Float | Correction term to account for inequality between port and starboard motor outputs. Value must be a percentage between -100.0 and +100.0. Positive bias causes the vehicle to turn more to the left (port) while negative bias causes a right (starboard) turn.

See the [Teleoperation Test](https://osf.io/k7a8p/wiki/14.%20Teleoperation%20Test/) page on the hardware wiki for more information. 

## Launching the Application

To launch the program, ssh into the microUSV's raspberry pi.  Navigate to the directory containing musv_teleop.py and launch the script using python.

```
$ python musv_teleop.py [PATH_TO_CONFIG.JSON]
```

By default, the config file should appear in the same directory as musv_teleop.py so simply entering the file name in place of the full file path will also work. 

```
$ python musv_teleop.py config.json
```

The application will cause the terminal window to go blank and intercept keyboard inputs. The application can be terminated by hitting the Escape key or Ctrl+C.


