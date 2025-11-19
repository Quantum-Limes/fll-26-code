from spike_lib.drive import *
class Arm:
    def __init__(self, leftMotor: Motor, rightMotor: Motor, color: Color ):
        self.color = color if color != None or Color.NONE else raiseError("Arm color must be specified!", ValueError)
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.speed = 500

    def align(self):
        pass

class SuperArm(Arm): 
    def __init__(self, rotationMotor: Motor, liftMotor: Motor, color: Color ):
        super().__init__(leftMotor=rotationMotor, rightMotor=liftMotor, color=color)
        self.rotationMotor = rotationMotor
        self.liftMotor = liftMotor
        self.rotRatio = 60 / 28 # out / in
        self.liftRatio = 3 / 1

    def align(self):
        """pro zarovnání ruky
        Je časově náročná"""
        self.liftMotor.run_until_stalled(-self.speed, Stop.HOLD, 60)
        self.rotationMotor.run_until_stalled(-self.speed, Stop.HOLD, 60)
        self.liftMotor.reset_angle(0)
        self.rotationMotor.reset_angle(-90 * self.rotRatio)

    def moveBy(self, xAngle: int, yAngle: int, wait: bool = True):
        """Pohne rukou o relativní úhel otočení a naklopení
        Parameters:
        - xAngle: int - relativní úhle otočení
        - yAngle: int - relativní úhle naklopení
        - wait: bool"""
        if fabs(xAngle) > fabs(yAngle):
            self.liftMotor.run_angle(self.speed * (1/self.liftRatio + yAngle*self.liftRatio/xAngle), xAngle + self.liftRatio * yAngle, Stop.HOLD, False)
            self.rotationMotor.run_angle(self.speed, xAngle*self.rotRatio, Stop.HOLD, wait)
        else:
            self.rotationMotor.run_angle(self.speed, xAngle*self.rotRatio, Stop.HOLD, False)
            self.liftMotor.run_angle(self.speed * (1/self.liftRatio + xAngle/yAngle*self.liftRatio), xAngle + self.liftRatio * yAngle, Stop.HOLD, wait)

    def moveTo(self, xAngle: int, yAngle: int, wait: bool = True):
        """Pohne rukou na absolutní úhel otočení a naklopeníParameters:
        - xAngle: int - absolutní úhle otočení
        - yAngle: int - absolutní úhle naklopení
        - wait: bool"""
        self.moveBy(xAngle + self.rotationMotor.angle(), yAngle + self.liftMotor.angle(), wait)

    def rotateBy(self, xAngle: int, wait: bool = False):
        self.moveBy(xAngle, 0, wait)

    def rotateTo(self, xAngle: int, wait: bool = True):
        self.moveBy(xAngle + self.rotationMotor.angle(), 0, wait)

    def raiseBy(self, yAngle, wait: bool = True):
        self.liftMotor.run_angle(self.speed, yAngle*3, Stop.HOLD, wait)

    def raiseTo(self, yAngle, wait: bool = False):
        self.liftMotor.run_target(self.speed, yAngle*3, Stop.HOLD, wait)

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
