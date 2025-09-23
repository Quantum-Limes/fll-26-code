from robotlib.tools.math import *

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Icon, Axis
from pybricks.tools import wait, StopWatch, multitask, run_task

class Arm(Motor):
    def __init__(self, port: Port, robot: Robot, positive_direction: Direction=Direction.CLOCKWISE, gears: Optional[Union[Collection[int], Collection[Collection[int]]]]=None, reset_angle: bool=True, profile: Number=None, stress: float =1):
        """
        this is expaniso for motor class

        Parameters:
            - port: Port ... Port to which the motor is connected. 
            - robot: Robot ... robot whose attachment this arm is.
            - positive_direction: Direction ... Which direction the motor should turn when you give a positive speed value or angle. 
            - gears: list ... List of gears linked to the motor. The gear connected to the motor comes first and the gear connected to the output comes last.
                For example: ``[12, 36]`` represents a gear train with a
                12-tooth gear connected to the motor and a 36-tooth gear
                connected to the output. Use a list of lists for multiple
                gear trains, such as ``[[12, 36], [20, 16, 40]]``.
                When you specify a gear train, all motor commands and settings
                are automatically adjusted to account for the resulting gear
                ratio. The motor direction remains unchanged by this.
            - reset_angle: bool ... Choose True to reset the rotation sensor value to the absolute marker angle (between -180 and 179). Choose False to keep the current value, so your program knows where it left off last time. 
            - profile: Number ... deg Precision profile. This is the approximate position tolerance in degrees that is acceptable in your application. A lower value gives more precise but more erratic movement; a higher value gives less precise but smoother movement. If no value is given, a suitable profile for this motor type will be selected automatically (about 11 degrees).
            - stress: float
        """
        super().__init__(port, positive_direction, gears, reset_angle, profile)
        self.robot = robot
        self.stress = clamp(abs(stress), 8, 0.25)
    
    def target(self, angle: int, speed: int = 1000, wait: bool = True):
        """
        run_target but better!!!

        """
        if self.robot.interupt == False:
            self.run_target(speed, angle, wait=False)
            if wait == True:
                while abs(self.angle() - angle) > 5 and not self.robot.interupt:
                    if self.robot.extra_task:
                        self.robot.extra_task()
        else:
            self.stop()

    def align(self, speed: int):
        """
        stops the motor as soon as it meets resistance

        Parameters:
            - speed: Number - deg/s (+ clockwise, - counterclockwise)
            - stress: Number - if the motor is under upnormal stress, change stress <0.25; 8>. More stress means higher resistence of motor.
        """
        if self.robot.interupt == False:
            #limiter
            speed = clamp(abs(speed), 1000, 100)*sign(speed) 
            self.stress = clamp(abs(self.stress), 8, 0.25)
            cons = abs(clamp(800/speed, 6, 1.5))*self.stress
            #cons(constant) = how many times the motor speed has to decrease to stop the motor.
            time = 300 + abs(speed/4)
            #time is time until the motor speeds up

            self.run(speed)
            wait(time)

            while abs(self.speed()) < abs(speed/cons) and not self.robot.interupt:
                if self.robot.extra_task:
                    self.robot.extra_task()
            self.stop()