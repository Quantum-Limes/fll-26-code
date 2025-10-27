#from robot.py

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

def open(self, background = False, time = 0): #not shure
        self.turnMotor(0,0, background=True, simple = True, time = time)
        self.turnMotor(1,0, background=background, simple = True, time = time)

def close(self, background = False): #not shure
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
        for motor in self.motors:
            motor.hold()

    #toposgen
    if turn and not connect[0]:
        if background:
            self.rotateRad(angle, background=True)
            while self.angleDiff(self.robot.hub.angleRad(), angle) > self.tolDiff:
                yield
        else:
            self.rotateRad(angle)

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