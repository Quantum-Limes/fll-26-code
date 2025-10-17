from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import cos, sin, atan2, pi, sqrt, fabs, asin

# Helper functions from maths.py
def generateBezierCurve(p0, p1, p2, p3, num_points=10):
    points = []
    for i in range(num_points + 1):
        t = i / num_points
        points.append(bezier(t, p0, p1, p2, p3))
    return points

def bezier(t, p0, p1, p2, p3):
    u = 1 - t
    p = u**3 * p0
    p += 3 * u**2 * t * p1
    p += 3 * u * t**2 * p2
    p += t**3 * p3
    return p

def sign(x):
    if x > 0:
        return 1
    if x == 0:
        return 0
    return -1

def clamp(x, minVal, maxVal):
    return max(minVal, min(x, maxVal))

def angleDiff(a, b):
    diff = a - b
    while diff > pi:
        diff -= 2 * pi
    while diff < -pi:
        diff += 2 * pi
    return diff

# vec2 and mat2 classes (basic implementation)
class vec2:
    def __init__(self, x, y):
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

class Robot:
    def __init__(self, left_motor_port, right_motor_port, wheel_diameter, axle_track):
        self.hub = PrimeHub()
        self.left_motor = Motor(left_motor_port, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_motor_port, Direction.CLOCKWISE)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)
        self.pos = vec2(0, 0)
        self.angle = 0
        self.tasks = []
        self.drive_settings = {
            "default": {"straight_speed": 500, "straight_acceleration": 1000, "turn_rate": 200, "turn_acceleration": 500},
            "fast": {"straight_speed": 1000, "straight_acceleration": 2000, "turn_rate": 400, "turn_acceleration": 1000},
            "precise": {"straight_speed": 200, "straight_acceleration": 500, "turn_rate": 100, "turn_acceleration": 200}
        }

    def set_drive_settings(self, mode):
        if mode in self.drive_settings:
            settings = self.drive_settings[mode]
            self.drive_base.settings(settings["straight_speed"], settings["straight_acceleration"], settings["turn_rate"], settings["turn_acceleration"])
        else:
            print(f"Drive mode '{mode}' not found.")

    def setMotorsToDef(self):
        self.robot.devices[0].setDefAngle()
        self.robot.devices[1].setDefAngle()
    
    def open(self, background = False, time = 0):
        self.turnMotor(0,0, background=True, simple = True, time = time)
        self.turnMotor(1,0, background=background, simple = True, time = time)

    def close(self, background = False):
        angle = 200
        if background:
            self.turnMotor(0,-angle, background=True, simple = True, time = 400)
            self.turnMotor(1,angle, background=background, simple = True, time = 400)
        else:
            self.turnMotor(0,-angle, background=True, simple = True)
            self.turnMotor(1,angle, background=True, simple = True)
            a = 0
            while a < 700 and self.isTasksRunning():
                self.runTasks()
                a += 1
            if a >= 1000:
                print("Closing motors timed out, stopping tasks")
            self.stopTasks()
            self.robot.devices[0].hold()
            self.robot.devices[1].hold()

    def turnMotorRad(self, deviceID, angle:float, speed = 1000, background = False, simple = False, time = 0):
        if background:
            self.addTask(self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time))
        else:
            for _ in self.turnMotorRadGen(deviceID, angle, speed = speed, simple=simple, time = time):
                self.runTasks()
                pass
    
    def turnMotorRadGen(self, deviceID, angle:float, speed = 1000, simple = False, time = 0):
        dif = self.angleDiff(self.robot.devices[deviceID].angleRad(), angle, simple=simple)
        dif = self.angleDiff(self.robot.devices[deviceID].angleRad(), angle, simple=simple)
        doTime = True
        if time == 0:
            doTime = False
        
        while fabs(dif) > self.tolDiff*2 and (time > 0 or not doTime):
            dif = self.angleDiff(self.robot.devices[deviceID].angleRad(), angle, simple=simple)
            self.robot.devices[deviceID].setSpeed(sign(dif) * clamp(speed*abs(dif)*0.5,110,200))
            time -= 1
            yield
        self.robot.devices[deviceID].hold()
        if time == 0:
            print("lol")

    def turnMotor(self, deviceID, angle:float, speed = 1000, background = False, simple = False, time = 0):
        self.turnMotorRad(deviceID, angle/180 * pi, speed=speed, background=background, simple=simple, time=time)
    
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
    
    def straight(self, length, speed = 1000, backwards = False, background = False):
        self.toPos(self.robot.pos + mat2.rotation(self.robot.hub.angleRad()) * vec2(length,0), speed, backwards, background=background)
    
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

    def toPosGen(self, pos, speed = 1000, backwards = False, stop = True, turn = True, tolerance = 0, extraDist = 0, background = False, connect = [False, False]):
        offset = self.robot.pos
        angle = atan2((pos-offset).y,(pos-offset).x)
        rotMat = mat2.rotation(-angle)
        if backwards:
            angle += (pi)
        if turn and not connect[0]:
            if background:
                self.rotateRad(angle, background=True)
                while self.angleDiff(self.robot.hub.angleRad(), angle) > self.tolDiff:
                    yield
            else:
                self.rotateRad(angle)
        length = rotMat*(pos - offset)
        swap = sign(length.x - (rotMat*self.robot.pos).x)
        swap = 1
        movedPos = rotMat*(self.robot.pos-offset)
        while(movedPos.x*swap < length.x*swap - tolerance):
            self.robot.update()
            self.calcDir(rotMat*(self.robot.pos-offset), length.x, self.calcSpeed(movedPos, length.x, speed, connect = connect),angle, backwards, extraDist)
            movedPos = rotMat*(self.robot.pos-offset)
            yield
        if not stop and not connect[1]:
            self.robot.stop(False)
        elif not connect[1]:
            self.robot.stop(self.brake)
            
    def toPos(self, pos, speed = 1000, backwards = False, stop = True, turn = True, tolerance = 0, extraDist = 10, background=False, connect = [False, False]):
        if background:
            self.addTask(self.toPosGen(pos, speed = speed, backwards = backwards, stop = stop, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect))
        else:
            for _ in self.toPosGen(pos, speed = speed, backwards = backwards, stop = stop, turn = turn, tolerance = tolerance, extraDist = extraDist, background=background, connect=connect):
                self.runTasks()
                pass
    

        
    def calcDir(self, pos, length, speed, offsetAngle, backwards = False, extraDist = 0):
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
    
    
    def bezier(self, p0:vec2, p1:vec2, p2:vec2, p3:vec2, numOfPoints = 10, speed = 500):
        points = generateBezierCurve(p0, p1, p2, p3, numOfPoints)
        self.cStart = points[0]
        self.cFinish = points[len(points)-1]
        for i in range(numOfPoints+1):
            if i == 0:
                self.toPos(points[i], tolerance=self.cTolerance)
            elif i == 1:
                self.toPos(points[i], tolerance=self.cTolerance,stop=False, speed=speed)
            elif i == numOfPoints:
                self.toPos(points[i], turn=False, speed=speed)
            else:
                self.toPos(points[i], turn = False, stop=False, tolerance=self.cTolerance, extraDist=0.0,speed=speed)
        self.cStart = self.cFinish = vec2(0,0)