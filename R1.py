from inicialization import *
    #m1
def m11_body():
    superruka.moveTo(-90 , 17, False)
    drive.moveToPos(vec2(240, 480))
    drive.turn(90)
    drive.motorsStop()
    superruka.moveTo(-20, 17)
    superruka.moveTo(-90, 17)
    drive.motorsStop()

    superruka.moveTo(-90, -60)
    drive.motorsStop()
    


#m2
def m12_body():
    superruka.moveTo(-90, -60, False)
    drive.moveToPos(vec2(240, 520))
    drive.turn(100)
    drive.motorsStop()
    superruka.moveTo(-10,40, True)
    drive.moveDistance(140)
    drive.turn(110)
    drive.motorsStop()
    superruka.moveBy(14,-8)
    superruka.moveBy(0,30)
    drive.moveDistance(40)
    superruka.moveTo(50, 60)
    drive.moveToPos(vec2(250, 420), True)
    drive.turn(140)
    superruka.moveTo(-50, 60)
    #drive.movePolar(150, 100)

    
def ride1():
    drive.setLocation(vec2(85, 27), 90)
    drive.locate()
    print(drive.orientation)
    superruka.align()
    superruka.align()
    superruka.speed = 800
    m11_body()
    m12_body()