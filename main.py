from spike_lib.robot import *


Lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C, Direction.CLOCKWISE)
Ra = Motor(Port.F, Direction.CLOCKWISE)
La = Motor(Port.D, Direction.CLOCKWISE)

robot = Robot(PrimeHub(front_side=Axis.Y))
drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=96)

superruka = SuperArm(Ra, La, Color.YELLOW)
mlaticiruka = Mlaticka(Ra, La, Color.RED)

mlaticiruka.align()
mlaticiruka.mlaceniAngle(90)
wait(2500)
mlaticiruka.mlaceni(3)
mlaticiruka.turnGear(30)
mlaticiruka.ejectFlag()

# print("Mllaceno")
#drive.turn(90)


#def m1_lol():
#    drive.moveToPos(vec2(100, 0))
#    superruka.rotateBy(90, True)

#m1 = Mission(vec2(15, 100), 180, m1_lol)

#m1.run()
