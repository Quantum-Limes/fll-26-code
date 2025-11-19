from inicialization import *
def ride1():
    drive.setLocation(vec2(85, 27), pi/2)
    superruka.align()
    superruka.speed = 600

    #m1
    superruka.raiseTo(210, False)
    drive.moveToPos(vec2(210, 540))
    drive.turn(90)
    drive.moveDistance(100)
    drive.moveDistance(-150)

    #m2
    #superruka.moveTo()
    #drive.moveToPos()
