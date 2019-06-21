# Teleop

Teleop is a basic teleoperation controller for the microUSV. It allows a user to ssh into a microUSV and control the motors using the wasd, qezc, and 123 keys as inputs through a terminal window. It is intended primarily as a quick testing/debugging tool to ensure a microUSV's motors are behaving as expected. 

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

To launch the program, ssh into the microUSV's raspberry pi.  Navigate to the directory containing musv_teleop.py and launch the script using python.

    $ python musv_teleop.py

The application will cause the terminal window to go blank and intercept keyboard inputs. The application can be terminated by hitting the Escape key or Ctrl+C.
