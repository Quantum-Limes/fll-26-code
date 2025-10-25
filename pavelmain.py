from spike_lib.robot import *

rides = {}

hub = PrimeHub()
Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE)
RA = Motor(Port.F, Direction.CLOCKWISE)
LA = Motor(Port.D, Direction.CLOCKWISE)

hub = PrimeHub()

robot = Robot(hub)
drive = Drive(robot, Lw, Rw, wheel_diameter=56, axle_track=96)
# robot = Robot(left_motor_port=Port.A, right_motor_port=Port.E, wheel_diameter=56, axle_track=96)

mlaticka = Mlaticka(robot, LA, RA)

# mlaticka.align()
mlaticka.mlaceni(2)
mlaticka.turnGear(500)
mlaticka.ejectFlag()

