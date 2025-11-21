from pybricks.hubs import PrimeHub
from pybricks.pupdevices import*
from pybricks.parameters import*
from pybricks.tools import wait
from pybricks.iodevices import XboxController
#from pybricks.robotics import DriveBase

def clamp(value, min_value, max_value):
    """Clamp the value between min_value and max_value."""
    return max(min(value, max_value), min_value)
    
class Drivebase:
    def __init__(self, left_motor, right_motor, speed=700, turn_speed=150):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.speed = speed/100
        self.turn_speed = turn_speed/100


    def drive(self, Rdir, dpad, slow_mode):
        if slow_mode:
            speed = self.speed / 2
            turn_speed = self.turn_speed / 2
        else:
            speed = self.speed
            turn_speed = self.turn_speed

        self.left_motor.run(speed * Rdir[1] + turn_speed * Rdir[0])
        self.right_motor.run(speed * Rdir[1] - turn_speed * Rdir[0])
        

class Arm:
    def __init__(self, turner: Motor, lifter: Motor, turn_speed=200, lift_speed=400, grab_speed=1000, turn_limits=(-90, 90), lift_limits=(0, 90), grab_limits=(0, 90)):
        '''otherwise use arm 1/2 for turner and lifter'''
        self.turner = turner
        self.lifter = lifter
        self.grabber = None
        self.turn_speed = turn_speed
        self.lift_speed = lift_speed
        self.grab_speed = grab_speed
        self.turn_limits = turn_limits
        self.lift_limits = lift_limits
        self.grab_limits = grab_limits
        self.turn_disabled = False
        self.grab_disabled = False
        self.turner.reset_angle(0)
        self.superarm = False #set true when using the best attachment ever made!!!
        self.SA_turn_gear = 1 #set to the gear ratio of the super arm turner

    def super_arm_support(self, Adir):
        rot_speed = self.turn_speed * Adir[0]/100
        lift_speed = self.lift_speed * Adir[1]/100 + self.turn_speed * Adir[0]/100 * self.SA_turn_gear
        return rot_speed, lift_speed


    def arm_turn(self, Adir, Agrab):
        if self.turn_disabled:
            self.turn_disabled = self.turner.angle() != 0
            return
        rot_speed, lift_speed = self.super_arm_support(Adir)
        self.turner.run(rot_speed + self.turn_speed * Agrab[0]/100)
        self.lifter.run(lift_speed + self.lift_speed * Agrab[1]/100)

    def turn_reset(self, pressed):
        if Button.X in pressed:
            self.turn_disabled = True
            self.turner.run_target(self.turn_speed, 0, wait=False)

    def arm_grab(self, Agrab, pressed):
        #if Button.A in pressed:
        #    self.grabber.run_target(self.grab_speed, self.grab_limits[0], wait=False)
        #    self.grab_disabled = True
        #    self.grab_angle = self.grab_limits[0]
        #elif Button.B in pressed:
        #    self.grabber.run_target(self.grab_speed, self.grab_limits[1], wait=False)
        #    self.grab_disabled = True
        #    self.grab_angle = self.grab_limits[1]

        if self.grab_disabled:
            self.grab_disabled = self.grabber.angle() != self.grab_angle
        else:
            self.grabber.run(self.grab_speed * (Agrab[0]-Agrab[1])/100)
        

    def run(self, Adir, Agrab, pressed):
        self.arm_turn(Adir, Agrab)
        self.turn_reset(pressed)
        #self.arm_grab(Agrab, pressed)

class Robot:
    def __init__(self, hub: PrimeHub, drivebase: Drivebase, arm: Arm, controller: XboxController):
        self.hub = hub
        self.drivebase = drivebase
        self.arm = arm
        self.controller = controller
        self.slow_mode = False

    def button_decoder(self, pressed):
        if Button.RB in pressed:
            self.slow_mode = True
        elif Button.LB in pressed:
            self.slow_mode = False
        if Button.A in pressed:
            self.hub.speaker.beep(200, 100)
        if Button.B in pressed:
            self.hub.speaker.beep(300, 100)
            wait(1000)
        return self.slow_mode
            

    def run(self):
        pressed = self.controller.buttons.pressed()
        Rdir = self.controller.joystick_left()
        Adir = self.controller.joystick_right()
        Alift = self.controller.triggers()
        dpad = self.controller.dpad()
        slow_mode = self.button_decoder(pressed)
        self.drivebase.drive(Rdir, dpad, slow_mode)
        self.arm.run(Adir, Alift, pressed)

hub = PrimeHub()
Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.A, Direction.CLOCKWISE)
turnter = Motor(Port.F, Direction.CLOCKWISE)
lifter = Motor(Port.D, Direction.CLOCKWISE)
#grabber = Motor(Port.A, Direction.CLOCKWISE)

controller = XboxController()
arm = Arm(turnter, lifter)
arm.superarm = True #enable super arm support
arm.SA_turn_gear = 7*4/56 #set to the gear ration of input/output (number of teeth) on the superarm rotation system
drive = Drivebase(Lw, Rw)
robot = Robot(hub, drive, arm, controller)

def master():
    while True:
        robot.run()
        

#master()