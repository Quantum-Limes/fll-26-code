from spike_lib.arms import *
from spike_lib.ultralocator import*
from spike_lib.mission_managment import*

hub = PrimeHub(front_side=Axis.Y)
robot = Robot(hub)
Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE) #Lw = BetterMotor(robot, Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE) #Rw = BetterMotor(robot, Port.C, Direction.CLOCKWISE)
Ra = Motor(Port.F, Direction.CLOCKWISE) #Ra = BetterMotor(robot, Port.F, Direction.CLOCKWISE)
La = Motor(Port.D, Direction.CLOCKWISE) #La = BetterMotor(robot, Port.D, Direction.CLOCKWISE)
Ul = UltrasonicSensor(Port.E)
Cs = ColorSensor(Port.A)

drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=96)

superruka = SuperArm(robot, Ra, La, Color.YELLOW)
#zvedaciruka = LiftArm(robot, Ra, La, Color.RED)
mlaticiruka = Mlaticka(robot, Ra, La, Color.BLUE)

ultra = UltraLocator(drive, Ul, 180, vec2(-25, 0))

field = setField(vec2(-180, 0), vec2(2182, 0), vec2(-180, 1144), vec2(2182, 1144))