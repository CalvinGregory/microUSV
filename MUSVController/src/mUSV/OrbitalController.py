from Controller import Controller
from pose2D import pose2D
import math

class OrbitalController(Controller):

    def __init__(self, config):
        super(OrbitalController, self).__init__(config)

    def get_motor_speeds(self, sensorData):
        pass