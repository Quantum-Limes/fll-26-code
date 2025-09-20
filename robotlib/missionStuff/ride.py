from robotlib.robot import *
class Ride:
    def __init__(self, arm: arm.Arm, order):
        self.arm = arm
        self.order = order