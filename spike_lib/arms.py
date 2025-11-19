from spike_lib.drive import *
class Arm:
    def __init__(self, leftMotor: Motor, rightMotor: Motor, color: Color ):
        self.color = color if color != None or Color.NONE else raiseError("Arm color must be specified!", ValueError)
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.speed = 200

    def align(self):
        pass

class SuperArm(Arm):      #OTESTOVAT CELOU CLASSKU-    Nově rotateTo, raiseTo, moveTo 
    def __init__(self, rotationMotor: Motor, liftMotor: Motor, color: Color ):
        super().__init__(leftMotor=rotationMotor, rightMotor=liftMotor, color=color)
        self.rotationMotor = rotationMotor
        self.liftMotor = liftMotor
        self.rotationalRatio = 60/28

    def align(self):
        self.liftMotor.run_until_stalled(-self.speed, Stop.HOLD, 60)
        self.rotationMotor.run_until_stalled(-self.speed, Stop.HOLD, 60)
        self.liftMotor.reset_angle(0)
        self.rotationMotor.reset_angle(-90 * self.rotationalRatio)

    def rotateBy(self, xAngle: int, wait: bool = False):
        self.rotationMotor.run_angle(self.speed, xAngle/2*-1, Stop.HOLD, wait)
        self.liftMotor.run_angle(1/3, xAngle*3, Stop.HOLD, wait)

    def moveTo(self, xAngle: int, yAngle: int, wait: bool = True):
        yMot = (xAngle - self.rotationMotor.angle()) + 3 * yAngle
        self.liftMotor.run_angle(1/3 * self.speed + self.speed * (yMot / (xAngle - self.rotationMotor.angle())-1), yMot, Stop.HOLD, False)
        self.rotationMotor.run_target(self.speed, xAngle-self.rotationalRatio, Stop.HOLD, wait)

# -self.liftMotor.angle()

    def raiseBy(self, yAngle, wait: bool = True):
        self.liftMotor.run_angle(self.speed, yAngle*3, Stop.HOLD, wait)

    def raiseTo(self, yAngle, wait: bool = False):
        self.liftMotor.run_target(self.speed, yAngle*3, Stop.HOLD, wait)

    def rotateTo(self, xAngle: int, wait: bool = True):
        self.moveTo(xAngle, 0,  wait)

#OLD VERSION
#class SuperArm(Arm):
#    def __init__(self, rotationMotor: Motor, liftMotor: Motor, color: Color ):
#        super().__init__(leftMotor=rotationMotor, rightMotor=liftMotor, color=color)
#        self.rotationMotor = rotationMotor
#        self.liftMotor = liftMotor

#    def align(self):
#        self.liftMotor.run_until_stalled(self.speed, Stop.HOLD, False)
#        self.rotationMotor.run_until_stalled(self.speed, Stop.HOLD, True)
#        self.liftMotor.reset_angle(0)
#        self.rotationMotor.reset_angle(-90)

#    def rotateBy(self, angle, wait: bool = False):
#        self.rotationMotor.run_angle(self.speed, angle/2*-1, Stop.HOLD, wait)
#        self.liftMotor.run_angle(1/3, angle*3, Stop.HOLD, wait)

#    def liftBy(self, angle, wait: bool = False):
#        self.liftMotor.run_angle(self.speed, angle*3, Stop.HOLD, wait)


class LiftArm(Arm):
    def __init__(self, liftMotor: Motor, stuffMotor: Motor, color: Color ):
        super().__init__(self, leftMotor=liftMotor, rightMotor=liftMotor, color=color)
        self.liftMotor = liftMotor
        self.stuffMotor = stuffMotor
        self.liftHeight = 35
        self.stuffGone = False
    
    def align(self):
        self.liftMotor.run_until_stalled(self.speed, Stop.HOLD, False)
        self.stuffMotor.run_until_stalled(self.speed, Stop.HOLD, True)
        self.liftHeight = 35
        self.liftMotor.reset_angle(0)
        self.stuffGone = False
        self.stuffMotor.reset_angle(0)
    
    def liftTo(self, height, wait: bool = True):
        clamped = clamp(height, 35, 140)
        self.liftMotor.run_angle(500, (clamped - self.liftHeight)/25, Stop.HOLD, wait)

    def stuffOut(self, wait: bool = True):
        if self.stuffGone:
            self.stuffMotor.run_angle(self.speed, -45, Stop.HOLD, wait)

class Mlaticka(Arm):
    def __init__(self, holderMotor: Motor, mlaticiMotor: Motor, color: Color):
        super().__init__(holderMotor, mlaticiMotor, color)
        self.mlaticiMotor = mlaticiMotor
        self.holderMotor  = holderMotor

    def align(self):
        self.mlaticiMotor.run_until_stalled(self.speed, Stop.HOLD, 60)
        self.holderMotor.run_until_stalled(-self.speed, Stop.HOLD, 60)
        self.mlaticiMotor.reset_angle(0)
        self.holderMotor.reset_angle(-180*5.5)
    
    def mlaceni(self, count: int):
        self.mlaticiMotor.run_target(self.speed, 0, Stop.HOLD, True)
        for i in range(count):
            self.mlaticiMotor.run_angle(400, -90, Stop.HOLD, True)
            self.mlaticiMotor.run_angle(self.speed, 90, Stop.HOLD, True)

    def mlaceniAngle(self, angle: int, waitBool: bool=True):
       self.mlaticiMotor.run_target(self.speed, -angle, Stop.HOLD, wait=waitBool)
       
    def turnGear(self, angle: int, waitBool: bool=True):
        self.holderMotor.run_target(self.speed, -angle, Stop.HOLD, wait=waitBool)

    def ejectFlag(self):
        self.holderMotor.run_target(self.speed, -30, Stop.HOLD, wait=True)

        # self.liftMotor.run_until_stalled(200, then=Stop.HOLD, wait=False)
        # self.stuffMotor.run_until_stalled(200, then=Stop.HOLD, wait=True)
        # self.liftHeight = 35
        # self.liftMotor.reset_angle(0)
        # self.stuffGone = False
        # self.stuffMotor.reset_angle(0)
