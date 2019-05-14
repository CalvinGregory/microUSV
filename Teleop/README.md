# Teleop

Basic teleoperation controller for the microUSV. Allows a user to ssh into a microUSV and control the motors using the wasd, qezc, and 123 keys as inputs through a terminal window. 

W - Drive forward
A - Pivot counter clockwise
S - Drive backward
D - Pivot clockwise

Q - Drive forward and left
E - Drive forward and right
Z - Drive backward and left
C - Drive Backward and right

1 - Set motor speed commands to preset low (~59%)
2 - Set motor speed commands to preset medium (~79%)
3 - Set motor speed commands to preset high (100%)

ESC - Exit the Teleop program

To launch the program: 
 - ssh into the microUSV's raspberry pi. 
 - Navigate to the directory containing musv_teleop.py
 - Enter the following command
'''
python musv_teleop.py
'''
