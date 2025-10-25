from spike_lib.robot import *


Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE)
Ra = Motor(Port.F, Direction.CLOCKWISE)
la = Motor(Port.D, Direction.CLOCKWISE)

robot = Robot(PrimeHub())
drive = Drive(robot, Lw, Rw, 56, 96)


