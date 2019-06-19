'''
MUSVClient controls the microUSV. It connects to a host computer running CVSensorSimulator to determine 
its simulated sensor values, then acts on those values, sending instructions to the motor controller.  

Copyright (C) 2019  CalvinGregory  cgregory@mun.ca
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
'''

class pose2D:
    '''
    pose2D is an object containing two dimensional pose data.
    
    Attributes: 
        x (float): Represents the x cartesian coordinate of the pose
        y (float): Represents the y cartesian coordinate of the pose
        yaw (float): Represents the rotation about the z (vertical axis). 
                     Positive rotation is defined as counter-clockwise. 
    
    '''

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw