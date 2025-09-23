from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from robotlib.robot.arm import Arm
# Robot rewritten as an extension of DriveBase and coded by vojta therefore better code then dan's
class Robot:
    def __init__(self, 
    hub: PrimeHub,
    wheelRadius: float,
    axleTrack: float,
    leftWheel: Motor,
    rightWheel: Motor,
    defaultSpeed: int = 900,
    acceleration: float = 1,
    deceleration: float = 1,
    gear: float = 1,
    extra_task: object = None): 

        self.wheelRadius = wheelRadius
        self.oneRotation = 2 * pi * wheelRadius * gear
        self.axleTrack = axleTrack
        
        self.hub = hub
        self.hub.imu.settings()
        self.Lw = leftWheel
        self.Rw = rightWheel

        self.defaultSpeed = defaultSpeed
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.gear = gear
        
        self.x = 0
        self.y = 0
        self.Lw_angle = 0
        self.Rw_angle = 0
        self.avr_motor_angle = 0
        self.orientation = 0
        self.status_skip = False
        self.interupt = False

        self.extra_task = extra_task

    def set_arm(self, arm: Arm):
        self.arm = arm

    def set_origin(self)
