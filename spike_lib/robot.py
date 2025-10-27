from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from spike_lib.maths import *
# from spike_lib.sound import Sound


class Robot:
    #task management
    def __init__(self, hub: PrimeHub):
        self.hub = hub
        self.tasks = []

    def isTasksRunning(self, numOfTasks = 0):
        if len(self.tasks) > numOfTasks:
            return True
        return False
    
    def waitForTasks(self, numOfTasks = 0):
        while self.isTasksRunning(numOfTasks = numOfTasks):
            self.runTasks()
    
    def stopTasks(self):
        self.tasks = []
    
    def addTask(self, gen):
        self.tasks.append(gen)
    
    def runTasks(self):
        for task in self.tasks: #[:]
            try:
                next(task)
            except StopIteration:
                self.tasks.remove(task)

class MotorControl: #AI trash (create new)
    def __init__(self, motor: Motor):
        self.motor = motor

    def run(self, speed):
        self.motor.run(speed)

    def stop(self, brake_type=Stop.HOLD):
        self.motor.stop(brake_type)

class Drive:
    def __init__(self, robot: Robot, left_motor: Motor, right_motor: Motor, wheel_diameter, axle_track):
        self.hub = robot.hub
        self.robot = robot
        self.leftMotor = left_motor
        self.rightMotor = right_motor
        self.motors = [self.leftMotor, self.rightMotor]
        self.driveBase = DriveBase(self.leftMotor, self.rightMotor, wheel_diameter, axle_track)
        self.wheelCircumference = wheel_diameter * pi # pls dej to do ajiny, nvm jak se to rekne
        self.updateLocation(vec2(0,0), 0)
        self.setMotorsToDef()
        self.orientationReset()
        self.drive_settings = {
            "default": {"straight_speed": 500, "turning_speed": 200, "min_speed": 50, "boundary_angle": pi/8, "straight_acceleration": 1, "tolerance": 2},
            "fast": {"straight_speed": 1000, "turning_speed": 400, "min_speed": 50,"boundary_angle": pi/8, "straight_acceleration": 1, "tolerance": 10},
            "precise": {"straight_speed": 300, "turning_speed": 200, "min_speed": 50,"boundary_angle": pi/8, "straight_acceleration": 1, "tolerance": 1}
        }
        self.setSettings("default")

    #some kind of motor controler functions?  - move to class above partly...
    def setSettings(self, mode):
        if mode in self.drive_settings:
            self.settings = self.drive_settings[mode]
        else:
            print(f"Drive mode '{mode}' not found.")

    #motor control
    def setMotorsToDef(self):
        """resets motor angle"""
        for motor in self.motors:
            motor.reset_angle()
        self.avrMotorAngle = 0

    def getMotorAngle(self):
        """returns motor angle difference from the last call and resects the average motor angle value"""
        avrMotorAngle = average(*[motor.angle() for motor in self.motors])
        diff = avrMotorAngle - self.avrMotorAngle
        self.avrMotorAngle = avrMotorAngle
        return diff

    def motorsDrive(self, leftSpeed, rightSpeed):
        """is sets driving speed of both motors to given values"""
        self.leftMotor.run(leftSpeed)
        self.rightMotor.run(rightSpeed)

    def motorsStop(self):
        """stops and holds both motors"""
        for motor in self.motors:
            motor.hold()

    #zatím k ničemu
    def turnMotorRad(self, deviceID, angle:float, speed = 1000, background = False, simple = False, time = 0):
        if background:
            self.robot.addTask(self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time))
        else:
            for _ in self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time):
                self.robot.runTasks()
                pass
    
    def turnMotorRadGen(self, deviceID: int, angle:float, speed = 1000, simple = False, time = 0):
        dif = self.angleDiff(radians(self.motors[deviceID].angle()), angle, simple=simple)
        doTime = True
        if time == 0:
            doTime = False
        
        while fabs(dif) > self.tolDiff*2 and (time > 0 or not doTime): #have no faith in this (there will be an error)
            dif = self.angleDiff(radians(self.motors[deviceID].angle()), angle, simple=simple)
            self.motors[deviceID].run(sign(dif) * clamp(speed*abs(dif)*0.5,110,200))
            time -= 1
            yield
        self.motors[deviceID].hold()
        if time == 0:
            print("lol")

    def turnMotor(self, deviceID, angle:float, speed = 1000, background = False, simple = False, time = 0):
        self.turnMotorRad(deviceID, angle/180 * pi, speed=speed, background=background, simple=simple, time=time)
    
    
    #gyro contol
    def orientationReset(self, value: float = 0):
        """Resets the orientation to given value (works with mathematical directoin)"""
        self.hub.imu.reset_heading(-value)

    def getOrientation(self):
        """returns current orientation in degrees (mathematical direction)"""
        return -self.hub.imu.heading() #to make maths work

    #localization centre
    def updateLocation(self, pos: vec2, angle: float):
        '''manual position update
        Parameters:
        - pos: vec2 position in mm!
        - angle: orientation in degrees!'''
        self.pos = pos
        self.orientation = angle

    def locate(self):
        """automatic position update"""
        orientation = self.getOrientation()
        length = self.getMotorAngle()*(self.wheelCircumference/360)
        self.updateLocation(self.pos + vec2_polar(vec2(length,0), radians(orientation)), orientation)

    #speed calculators ats
    def getSpeed(self, length, stop, straightSpeed, minSpeed, brakeDist):
        if stop:
            speed = clamp(length / brakeDist * straightSpeed, minSpeed, straightSpeed)
        else:
            speed = straightSpeed
        return speed
    
    def gyroCorection(self, speed, aimAngle, currentAngle, turningSpeed, boundaryAngle):
        deviation = clamp(angleDiff(aimAngle, currentAngle)/boundaryAngle, -1, 1)
        straightSpeed = (1-fabs(deviation)) * speed
        sideSpeed = deviation * turningSpeed
        L_speed = straightSpeed + sideSpeed
        R_speed = straightSpeed - sideSpeed
        return L_speed, R_speed
    
    def getMotorSpeeds(self, length, aimAngle, currentAngle, stop):
        return self.gyroCorection(
            self.getSpeed(length, stop, self.settings["straight_speed"], self.settings["min_speed"], self.settings["straight_acceleration"]*self.wheelCircumference), 
            aimAngle, currentAngle, self.settings["turning_speed"], self.settings["boundary_angle"])

    #drive base core
    def moveDistance(self, distance, backwards = False, stop = True, wait = True):
        '''Moves the robot a certain distance in mm.
        Parameters:
        - distance: distance in mm
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        self.locate()
        self.movePolar(distance, self.orientation, backwards, stop, wait)

    def movePolar(self, length, orientaton, backwards = False, stop = True, wait = True):
        '''Moves the robot a certain distance in certain direction.
        Parameters:
        - distance: distance in mm
        - orientaton: orientation in degrees
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        self.moveToPos(self.pos + vec2_polar(vec2(length, 0), radians(orientaton)), backwards, stop, wait)

    def moveToPos(self, pos, backwards = False, stop = True, wait = True):
        '''Moves the robot to certain position.
        Parameters:
        - position: vec2 position in mm
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        if wait:
            for _ in self.toPosGen(pos, backwards = backwards, stop = stop):
                self.robot.runTasks()
        else: 
            self.robot.addTask(self.toPosGen(pos, backwards = backwards, stop = stop))

    def toPosGen(self, pos: vec2, backwards, stop):
        if backwards:
            dir = -1
        else:
            dir = 1
        
        onPos = False
        while not onPos:
            self.locate() #not ideal for sensor spaming
            trajectory = (pos - self.pos) * dir
            print(pos, self.pos, trajectory)
            angle = trajectory.orientation()
            length = trajectory.length()

            speeds = self.getMotorSpeeds(length, angle, radians(self.orientation), stop)
        
            self.motorsDrive(speeds[0], speeds[1])
            if fabs(length) <= self.settings["tolerance"]:
                onPos = True
            yield
        if not stop:
            self.motorsStop()

    def circle(self, center, circlePercentage, speed = 1000):
        angle = asin((center - self.robot.pos).normalize().y) - sign(circlePercentage)*pi*0.5
        self.rotateRad(angle)
        finalPos = mat2.rotation(2*pi*circlePercentage)*(self.robot.pos - center) + center
        r = (self.robot.pos - center).length()
        ratio = (r-self.robot.axle*0.5)/(r+self.robot.axle*0.5)
        side = -sign(circlePercentage)
        startPos = self.robot.pos
        
        length = abs(r*self.angleDiff((startPos - center).xAngle(), (finalPos - center).xAngle()))
        apos = length
        while(apos > 0.5):
            apos = abs(r*self.angleDiff((self.robot.pos - center).xAngle(), (finalPos - center).xAngle()))
            aspeed = self.calcSpeed(vec2(length-apos,0), length, speed) + 100
            self.robot.update()
            if side > 0:
                self.robot.setSpeed(aspeed, aspeed*ratio)
            else:
                self.robot.setSpeed(aspeed*ratio, aspeed)
        self.robot.stop(self.brake)

    def rotate(self, angle, speed = 1000, background = False):
        self.rotateRad(angle/180 * pi, speed = speed, background=background)
    
    def rotateRad(self, angle, speed = 1000, background = False):
        if background:
            self.addTask(self.rotateRadGen(angle, speed))
        else:
            for _ in self.rotateRadGen(angle, speed):
                self.runTasks()
                pass
       
    def rotateRadGen(self, angle, speed = 1000):
        angleInit = self.robot.hub.angleRad()
        angleInitD = 0
        angleD = self.angleDiff(self.robot.hub.angleRad(), angle)
        if fabs(angleD) <= self.tolDiff:
            return
        while fabs(angleD) > self.accuracy:
            rspeed = self.calcSpeedR(angleD, speed, angleInitD)
            self.robot.setSpeed(-rspeed*sign(angleD), rspeed*sign(angleD))
            self.update()
            angleInitD = self.angleDiff(self.robot.hub.angleRad(), angleInit)
            angleD = self.angleDiff(self.robot.hub.angleRad(), angle)
            
            yield
        self.robot.stop(self.braker)

    def calcSpeedR(self, angle:float, speed:float, angleInit:float):
        rspeed = fabs(angle) * self.rdeacc + self.defspeed
        aspeed = fabs(angleInit) * self.racc + self.defspeed
        return maxV(maxV(rspeed,aspeed),speed)
    
    def angleDiff(self, angle1:float, angle2:float, simple = False):
        if simple:
            return angle2 - angle1
        a1 = (angle1) % (2*pi)
        a2 = (angle2) % (2*pi)
        return (a2 - a1 + pi) % (2*pi) - pi
    
    
    def circleToPos(self,pos, speed = 1000, connect = [False, False], accuracy = 0.2, backwards = False, background = False):
        if background:
            self.addTask(self.circleToPosGen(pos, speed = speed, connect = connect, accuracy = accuracy, backwards = backwards))
        else:
            for _ in self.circleToPosGen(pos, speed = speed, connect = connect, accuracy = accuracy, backwards = backwards):
                self.runTasks()
                pass
    
    def circleToPosGen(self,pos, speed = 1000, connect = [False, False], accuracy = 0.2, backwards = False):
        startPos = self.robot.pos
        if accuracy == 0.2 and connect[1]:
            accuracy = 13
        while (self.robot.pos - pos).length() > accuracy:
            angle = self.robot.hub.angleRad()
            if backwards:
                angle = self.angleDiff(0, angle + pi)
            
            dir = vec2(cos(angle), sin(angle))
            dx = pos.x - self.robot.pos
            dy = pos.y - self.robot.pos.y
            t=(dx*dir.x + dy*dir.y) / (2*(dy*dir.x - dx*dir.y))
            center = vec2(0.5*(self.robot.pos.x + pos.x) - t*dy, 0.5*(pos.y + self.robot.pos.y) + t*dx)
            radius = (pos - center).length()
            ratio = (radius-self.robot.axle*0.5)/(radius+self.robot.axle*0.5)
            side = sign((mat2.rotation(-angle+ 0.5*pi) * (pos-self.robot.pos)).x)
            
            cSpeed = self.calcSpeedDis(startPos, pos, speed, connect = connect)
            if backwards:
                cSpeed = -cSpeed
                side = -side
            
            if side == 1:
                self.robot.setSpeed(cSpeed, cSpeed*ratio)
            else:
                self.robot.setSpeed(cSpeed*ratio, cSpeed)
            
            self.robot.update()
            yield
        if not connect[1]:
            self.robot.stop(self.brake)

    
    def calcSpeedDis(self, startPos, EndPos, speed, connect = [False, False]):
       
        disToStart = (startPos - self.robot.pos).length()
        disToEnd = (EndPos - self.robot.pos).length()
        rspeed = speed
        if not connect[0]:
            rspeed = disToStart * self.acc + self.defspeed
        if not connect[1]:
            rspeed = maxV(rspeed, disToEnd * self.deacc + self.defspeed)
        else:
            rspeed = maxV(rspeed, disToEnd * self.deacc*3 + self.defspeed)
        rspeed = maxV(rspeed, speed)
        return rspeed
    


class Arm:
    def __init__(self, leftMotor: Motor, rightMotor: Motor, color: Color ):
        self.color = color if color != None or Color.NONE else raiseError("Arm color must be specified!", ValueError)
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.speed = 200

    def align(self):
        pass
    


class SuperArm(Arm):
    def __init__(self, rotationMotor: Motor, liftMotor: Motor, color: Color ):
        super().__init__(leftMotor=rotationMotor, rightMotor=liftMotor, color=color)
        self.rotationMotor = rotationMotor
        self.liftMotor = liftMotor

    def align(self):
        self.liftMotor.run_until_stalled(self.speed, Stop.HOLD, False)
        self.rotationMotor.run_until_stalled(self.speed, Stop.HOLD, True)
        self.liftMotor.reset_angle(0)
        self.rotationMotor.reset_angle(-90)

    def rotateBy(self, angle, wait: bool = False):
        self.rotationMotor.run_angle(self.speed, angle/2*-1, Stop.HOLD, wait)
        self.liftMotor.run_angle(1/3, angle*3, Stop.HOLD, wait)

    def liftBy(self, angle, wait: bool = False):
        self.liftMotor.run_angle(self.speed, angle*3, Stop.HOLD, wait)


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
        self.mlaticiMotor.run_until_stalled(-self.speed, Stop.HOLD, False)
        self.holderMotor.run_until_stalled(self.speed, Stop.HOLD, True)
        self.mlaticiMotor.reset_angle(68)
        self.holderMotor.reset_angle(180)
    
    def mlaceni(self, count: int):
        for i in range(count):
            self.mlaticiMotor.run_angle(400, -90, Stop.HOLD, True)
            self.mlaticiMotor.run_angle(self.speed, 90, Stop.HOLD, True)

    def mlaceniAngle(self, angle: int, waitBool: bool=True):
       self.mlaticiMotor.run_angle(self.speed, angle, Stop.HOLD, wait=waitBool)
       
    def turnGear(self, angle: int, waitBool: bool=True):
        self.holderMotor.run_angle(self.speed, angle, Stop.HOLD, wait=waitBool)

    def ejectFlag(self):
        self.holderMotor.run_angle(self.speed, 50, Stop.HOLD, wait=True)

        # self.liftMotor.run_until_stalled(200, then=Stop.HOLD, wait=False)
        # self.stuffMotor.run_until_stalled(200, then=Stop.HOLD, wait=True)
        # self.liftHeight = 35
        # self.liftMotor.reset_angle(0)
        # self.stuffGone = False
        # self.stuffMotor.reset_angle(0)
        

        
class Mission:
    def __init__(self, pos: vec2, rot: float, run: function):
        self.run = run
        self.pos = pos
        self.rot = rot
        self.completed = False

    def goToMission(self, drive: Drive, checkpoints: list = [vec2]):
        if len(checkpoints):
            for point in checkpoints:
                drive.moveToPos(point)
            if not self.completed:
                drive.moveToPos(self.pos)
                drive.rotate(self.rot)
                self.run(drive)
                self.completed = True
            
            
            
class Ride:
    def __init__(self, arm: Arm):
        self.arm = arm
        arm.align()
        self.color = arm.color
        self.missions = {}
    
    def run(self, drive: Drive):
        for mission in self.missions.values():
            mission.goToMission(drive)

    def appendMission(self, mission: Mission):
        self.missions.append(mission)

def rideManager(rides: set, La, Ra, colorSensor: ColorSensor, hub):
    def scanForArm():
        while 1:
            La.run_angle(400, 45, Stop.HOLD, False)
            Ra.run_angle(400, 45, Stop.HOLD, True)
            color = colorSensor.color() 
            if color != Color.NONE: break
            wait(150)
            La.run_angle(400, 45, Stop.HOLD, False)
            Ra.run_angle(400, 45, Stop.HOLD, True)
            if color != Color.NONE: break
            wait(150)
        for ride in rides:
            if ride.color == color:
                return ride
        return None
        
    while True:
        scan = scanForArm()
        if scan != None:
            confirmed = 0
            while not confirmed: confirmed = True if hub.buttons.pressed()[Button.CENTER] else False
            scan.run()
    

