from spike_lib.robot import *


Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE)
Ra = Motor(Port.F, Direction.CLOCKWISE)
La = Motor(Port.D, Direction.CLOCKWISE)

robot = Robot(PrimeHub())
drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=96)

superruka = SuperArm(Ra, La, Color.YELLOW)


def m1_lol():
    drive.moveToPos(vec2(-10, 0))
    superruka.rotateBy(90, True)

m1 = Mission(vec2(15, 100), 180, m1_lol)

m1.run()


