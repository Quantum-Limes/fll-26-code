from spike_lib.arms import *
from spike_lib.ultralocator import*

hub = PrimeHub(front_side=Axis.Y)
Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE)
Ra = Motor(Port.F, Direction.CLOCKWISE)
La = Motor(Port.D, Direction.CLOCKWISE)
Ul = UltrasonicSensor(Port.E)

robot = Robot(hub)
drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=96)

superruka = SuperArm(Ra, La, Color.YELLOW)
#zvedaciruka = LiftArm(Ra, La, Color.RED)
mlaticiruka = Mlaticka(Ra, La, Color.BLUE)

ultra = UltraLocator(drive, Ul, 180, vec2(-25, 0))

field = ultra.setField(vec2(-180, 0), vec2(2182, 0), vec2(-180, 1144), vec2(2182, 1144))