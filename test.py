from spike_lib.drive import *


Lw = Motor(Port.A, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.E, Direction.CLOCKWISE)

robot = Robot(PrimeHub(front_side=Axis.Y))
drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=160)
#odrive = DriveBase(Lw, Rw, 56, 160)
#odrive.use_gyro(True)
drive.movePolar(100, 30, backwards=True)
#drive.moveToPos(vec2(100, 100), backwards=True, stop=True, wait=True)
#drive.moveToPos(vec2(0, 0), backwards=True, stop=True, wait=True)
#odrive.straight(2000)
#odrive.straight(-2000)