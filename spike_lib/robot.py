from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import cos, sin, atan2, pi, sqrt, fabs, asin, radians, degrees

# Helper functions from maths.py

def sign(x):
    if x > 0:
        return 1
    if x == 0:
        return 0
    return -1

def clamp(x, minVal, maxVal):
    return max(minVal, min(x, maxVal))

def minV(*values):
    minValue = values[0]
    for v in values:
        if v < minValue:
            minValue = v
    return minValue

def maxV(*values):
    maxValue = values[0]
    for v in values:
        if v > maxValue:
            maxValue = v
    return maxValue

def average(*values):
    return sum(values) / len(values)

def angleDiff(a, b):
    '''returns difference between angles a and b
    - in radians
    - [(a - b) modulo 2pi] - pi'''
    diff = a - b
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

# vec2 and mat2 classes (basic implementation)
class vec2:
    def __init__(self, x, y):
        '''2D vector class
        Parameters:
        - x: x component
        - y: y component
        '''
        self.x = x
        self.y = y

    def __add__(self, other):
        return vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        return vec2(self.x / scalar, self.y / scalar)

    def length(self):
        return sqrt(self.x**2 + self.y**2)
    
    def orientation(self):
        return atan2(self.y, self.x)

    def normalized(self):
        l = self.length()
        if l == 0:
            return vec2(0, 0)
        return self / l

class mat2:
    @staticmethod
    def rotation(angle_rad):
        c = cos(angle_rad)
        s = sin(angle_rad)
        return [[c, -s], [s, c]]

    @staticmethod
    def multiply(matrix, vector):
        x = matrix[0][0] * vector.x + matrix[0][1] * vector.y
        y = matrix[1][0] * vector.x + matrix[1][1] * vector.y
        return vec2(x, y)

def vec2_polar(length: vec2, orientaton: float):
    '''Create a vec2 from polar coordinates.'''
    return length * mat2.rotation(radians(orientaton))

class MotorControl: #AI trash (create new)
    def __init__(self, motor: Motor):
        self.motor = motor

    def run(self, speed):
        self.motor.run(speed)

    def stop(self, brake_type=Stop.HOLD):
        self.motor.stop(brake_type)

class Drive:
    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter, axle_track):
        self.hub = PrimeHub()
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.motors = [self.left_motor, self.right_motor]
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)
        self.updateLocation(vec2(0,0), 0)
        self.avrMotorAngle = 0
        self.tasks = []
        self.drive_settings = {
            "default": {"straight_speed": 500, "straight_acceleration": 1000, "turn_rate": 200, "turn_acceleration": 500},
            "fast": {"straight_speed": 1000, "straight_acceleration": 2000, "turn_rate": 400, "turn_acceleration": 1000},
            "precise": {"straight_speed": 200, "straight_acceleration": 500, "turn_rate": 100, "turn_acceleration": 200}
        }

    #some kind of motor controler functions?  - move to class above partly...
    def set_drive_settings(self, mode):
        if mode in self.drive_settings:
            settings = self.drive_settings[mode]
            self.drive_base.settings(settings["straight_speed"], settings["straight_acceleration"], settings["turn_rate"], settings["turn_acceleration"])
        else:
            print(f"Drive mode '{mode}' not found.")

    def setMotorsToDef(self): #what is that???
        for motor in self.motors:
            motor.reset_angle()
    
    def getMotorAngle(self):
        avrMotorAngle = average(*[motor.angle() for motor in self.motors])
        diff = avrMotorAngle - self.avrMotorAngle
        self.avrMotorAngle = avrMotorAngle
        return diff

    def turnMotorRad(self, deviceID, angle:float, speed = 1000, background = False, simple = False, time = 0):
        if background:
            self.addTask(self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time))
        else:
            for _ in self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time):
                self.runTasks()
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
    
    #task management
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
        for task in self.tasks[:]:
            try:
                next(task)
            except StopIteration:
                self.tasks.remove(task)
    
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
        length = self.getMotorAngle()*(self.drive_base.wheel_diameter * pi / 360)
        self.updateLocation(self.pos + vec2_polar(length, radians(orientation)), orientation)

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
        if background:
            self.addTask(self.toPosGen(pos, speed = speed, backwards = backwards, stop = stop, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect))
        else:
            for _ in self.toPosGen(pos, speed = speed, backwards = backwards, stop = stop, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect):
                self.runTasks()
                pass

    def toPosGen(self, pos: vec2, speed = 1000, backwards = False, turn = True, tolerance = 0, extraDist = 0, background = False, connect = [False, False]):
        #work in progress
        if backwards:
            angle += (pi)
        if turn and not connect[0]:
            if background:
                self.rotateRad(angle, background=True)
                while self.angleDiff(self.robot.hub.angleRad(), angle) > self.tolDiff:
                    yield
            else:
                self.rotateRad(angle)

        onPos = False
        tolerance = 5 #mm
        while not onPos:
            self.locate() #not ideal for sensor spaming
            trajectory = pos - self.pos
            angle = trajectory.orientation()
            length = trajectory.length()

            #add speed calculations here!

            if fabs(length) <= tolerance:
                onPos = True
            yield
        if not connect[1]:
            for motor in self.motors: #replace by motor contoling function (when done)
                motor.stop()

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
            self.robot.update()
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
            dx = pos.x - self.robot.pos.x
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
    
    
  