'''
musv_teleop interprets SSH keyboard inputs as direction commands and sends 
the corresponding motor speed commands to the microUSV's motor controller. 

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

import json

class Config:
    '''
    Config is a class used to import variables from a JSON config file at runtime.
    
    Attributes:
        propSpin_port (int): coefficient indicating which direction the port propeller should spin. 
                             Acceptable values are +1 or -1
        propSpin_star (int): coefficient indicating which direction the starboard propeller should spin. 
                             Acceptable values are +1 or -1
        bias (float): Correction term to account for inequality between port and starboard motor outputs. Value must be a 
                      percentage between -100.0 and +100.0. Positive bias causes the vehicle to turn more to the left (port) 
                      while negative bias causes a right (starboard) turn. 
    '''

    def __init__(self, filepath):
        '''
        Creates a Config object, parses target JSON config file and stores its data in this object. 
        
        Args:
            filepath (str): Filepath to a JSON config file to be parsed.
        '''
        self.propSpin_port = 1
        self.propSpin_star = 1
        self.bias = 0.0
        self.parse_configs(filepath)

    def parse_configs(self, filepath):
        '''
        Parses target JSON config file and stores its data in this object.
        
        Args:
            filepath (str): Filepath to a JSON config file to be parsed.
        '''
        with open(filepath, 'r') as jsonFile:
            config = json.load(jsonFile)
            
        self.propSpin_port = config['PropellerSpin']['port']
        self.propSpin_star = config['PropellerSpin']['starboard']
        self.bias = config['bias']