from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from spike_lib.maths import *


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
            "default": {"straight_speed": 500, "straight_acceleration": 1, "turn_rate": 200, "turn_acceleration": 500},
            "fast": {"straight_speed": 1000, "straight_acceleration": 2, "turn_rate": 400, "turn_acceleration": 1000},
            "precise": {"straight_speed": 200, "straight_acceleration": 0.5, "turn_rate": 100, "turn_acceleration": 200}
        }

    #some kind of motor controler functions?  - move to class above partly...
    def set_drive_settings(self, mode):
        return 1000
#        if mode in self.drive_settings:
#            settings = self.drive_settings[mode]
#            self.driveBase.settings(settings["straight_speed"], settings["straight_acceleration"], settings["turn_rate"], settings["turn_acceleration"])
#        else:
#            print(f"Drive mode '{mode}' not found.")

    def setMotorsToDef(self):
        for motor in self.motors:
            motor.reset_angle()
        self.avrMotorAngle = 0
    
    def getMotorAngle(self):
        avrMotorAngle = average(*[motor.angle() for motor in self.motors])
        diff = avrMotorAngle - self.avrMotorAngle
        self.avrMotorAngle = avrMotorAngle
        return diff

    def motorsDrive(self, leftSpeed, rightSpeed):
        self.leftMotor.run(leftSpeed)
        self.rightMotor.run(rightSpeed)

    def motorsStop(self):
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
    def orientationReset(self):
        self.hub.imu.reset_heading(0)

    def getOrientation(self):
        return -self.hub.imu.heading() #to make maths work

    #localization centre
    def updateLocation(self, pos: vec2, angle: float):
        '''Parameters:
        - pos: vec2 position in mm!
        - angle: orientation in degrees!'''
        self.pos = pos
        self.orientation = angle

    def locate(self):
        orientation = self.getOrientation()
        length = self.getMotorAngle()*(self.wheelCircumference/360)
        self.updateLocation(self.pos + vec2_polar(vec2(length,0), radians(orientation)), orientation)

    #drive base core
    def moveDistance(self, distance, speed = 1000, backwards = False, background = False):
        '''Moves the robot a certain distance in mm.
        Parameters:
        - distance: distance in mm
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - background: if True, runs in background'''
        self.locate()
        self.movePolar(distance, self.orientation, speed, backwards, background)

    def movePolar(self, length, orientaton, speed = 1000, backwards = False, background = False):
        '''Moves the robot a certain distance in certain direction.
        Parameters:
        - distance: distance in mm
        - orientaton: orientation in degrees
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - background: if True, runs in background'''
        self.moveToPos(self.pos + vec2_polar(vec2(length, 0), radians(orientaton)), speed, backwards, background=background)

    def moveToPos(self, pos, speed = 1000, backwards = False, stop = True, turn = True, tolerance = 0, extraDist = 10, background=False, connect = [False, False]):
        '''Moves the robot to certain position.
        Parameters:
        - position: vec2 position in mm
        - speed: speed in deg/s
        - backwards: if True, moves backwards
        - background: if True, runs in background'''
        self.robot.addTask(self.toPosGen(pos, speed = speed, backwards = backwards, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect))
        if not background:
            for _ in self.toPosGen(pos, speed = speed, backwards = backwards, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect):
                self.robot.runTasks()

    def toPosGen(self, pos: vec2, speed = 1000, backwards = False, turn = True, tolerance = 0, extraDist = 0, background = False, connect = [False, False]):
        #work in progress
        
        #nemám tušení co to má dělat (ale vím, že to nedělá)    
        #hej to muzem smazat, nikde to nepouzivame, jen v tom nad tim
        #to nad tim je jízda rovně (důležité), ale tenhle kousek je asi k ničemu
        #no toto vyuzivame na jako ten generator (yield), a to nad tim to proste jen prida do tasklistu
        #jo, ale já mluvím o taditom turn whatever 
        # jo toto, 
        #opraveno
        #taky moznost :)
        
        onPos = False
        tolerance = 5 #mm
        while not onPos:
            self.locate() #not ideal for sensor spaming
            trajectory = pos - self.pos
            print(pos, self.pos, trajectory)
            angle = trajectory.orientation()
            length = trajectory.length()

            speeds = self.calculateMotorDirection(length, angle, radians(self.orientation))

            self.motorsDrive(speeds[0], speeds[1])
            if fabs(length) <= tolerance:
                onPos = True
            yield
        if not connect[1]:
            self.motorsStop()

    def calculateMotorDirection(self, length, aimAngle, currentAngle):
        g_cons = 1000
        print(length)
        c_speed = clamp(length/self.wheelCircumference, 1, -1) * 900
        gCor = self.angleDiff(aimAngle, currentAngle) * g_cons
        L_speed = c_speed + gCor
        R_speed = c_speed - gCor
        return L_speed, R_speed

    def calcDir(self, pos, length, speed, offsetAngle, backwards = False, extraDist = 0): #odpad
        a2 = (self.robot.hub.angleRad()-offsetAngle) % (2*pi)
        pos = vec2(length + extraDist - pos.x, -pos.y)
        a1 = atan2(pos.y, pos.x) % (2*pi)
        angle = (a2 - a1 + pi) % (2*pi) - pi
        speedM = speed * minV(1-(fabs(angle)*self.turnCoeff),-1.0)**1
        if(backwards):
            speed, speedM = -speedM, -speed
        mult = 1/(fabs(angle)*0+1)
        if sign(angle) > 0:
            self.robot.setSpeed(speed*mult, speedM*mult)
        else:
            self.robot.setSpeed(speedM*mult, speed*mult)
    

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


    def calcSpeed(self, pos, length, speed, connect = [False, False]):
        if self.cStart == self.cFinish:
            accSpeed = speed
            deaccSpeed = speed
            
            if not connect[0]:
                accSpeed = fabs(pos.x) * self.acc + self.defspeed
            if not connect[1]:
                deaccSpeed = fabs(length - pos.x) * self.deacc + self.defspeed
        else:
            accSpeed =  (self.robot.pos - self.cStart).length() * self.cAcc + self.defspeed
            deaccSpeed = (self.robot.pos - self.cFinish).length() * self.cDeacc + self.defspeed
        return clamp(fabs(maxV(deaccSpeed,accSpeed)), self.defspeed ,speed)


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
    
    
class SupperArm:
    def __init__(self, length: float, elevation: float, liftGear: float, rotationGear: float, rotationSpeed = 200):
        self.length = length
        self.elevation = elevation
        self.liftGear = liftGear #in / out
        self.rotationGear = rotationGear #in / out

    def calculale(self, height: float, orientation: float):
        rotationAngle = orientation / self.rotationGear
        liftAngle = asin((height - self.elevation)/self.length) / self.liftGear + orientation
        rotationSpeed = 0
        liftSpeed = 0
        return liftAngle, rotationAngle, liftSpeed, rotationSpeed

class SkratArm:
    def __init__(self, liftMotor, rotationMotor):
        self.liftMotor = liftMotor
        self.rotationMotor = rotationMotor
    