from spike_lib.robot import *
class Drive:
    def __init__(self, robot: Robot, left_motor: Motor, right_motor: Motor, wheel_diameter, axle_track):
        self.hub = robot.hub
        self.robot = robot
        self.leftMotor = left_motor
        self.rightMotor = right_motor
        self.motors = [self.leftMotor, self.rightMotor]
        self.driveBase = DriveBase(self.leftMotor, self.rightMotor, wheel_diameter, axle_track)
        self.wheelCircumference = wheel_diameter * pi # pls dej to do ajiny, nvm jak se to rekne
        self.axleTrack = axle_track
        self.setLocation(vec2(0,0), 0)
        self.setMotorsToDef()
        self.drive_settings = {
            "default": {"straight_speed": 1000, "turning_speed": 200, "min_speed": 50, "boundary_angle": pi/8, "straight_acceleration": 1, "tolerance": 1, "angle_tolerance": pi/200},
            "smooth": {"straight_speed": 1000, "turning_speed": 300, "min_speed": 50,"boundary_angle": pi/4, "straight_acceleration": 1, "tolerance": 1, "angle_tolerance": pi/200},
            "precise": {"straight_speed": 300, "turning_speed": 200, "min_speed": 50,"boundary_angle": pi/4, "straight_acceleration": 1, "tolerance": 1, "angle_tolerance": pi/200}
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
            motor.reset_angle(0)
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
    
    #gyro contol
    def orientationReset(self, value: float = 0):
        """Resets the orientation to given value (works with mathematical directoin)"""
        self.hub.imu.reset_heading(-value)

    def getOrientation(self):
        """returns current orientation in radians (mathematical direction)"""
        return radians(-self.hub.imu.heading()) #to make maths work

    #localization centre
    def updateLocation(self, pos: vec2, angle: float):
        '''manual position update
        Parameters:
        - pos: vec2 position in mm!
        - angle: orientation in radians!'''
        self.pos = pos
        self.orientation = angle

    def locate(self):
        """automatic position update"""
        orientation = self.getOrientation()
        length = self.getMotorAngle()*(self.wheelCircumference/360)
        self.updateLocation(self.pos + rotateVec2(vec2(length,0), orientation), orientation)
        #print(self.pos, self.orientation, length)

    def setLocation(self, pos: vec2, orientation: float):
        """Manual locate
        Paramenters:
        - pos: vec2 position in mm!
        - angle: orientation in degres!
        """
        self.orientationReset(orientation)
        self.updateLocation(pos, degrees(orientation))

    #speed calculators ats
    def getSpeed(self, length, stop, straightSpeed, minSpeed, brakeDist):
        if stop:
            speed = clamp(fabs(length) / brakeDist * straightSpeed, minSpeed, straightSpeed)*sign(length)
        else:
            speed = straightSpeed
        return speed
    
    def gyroCorection(self, speed, aimAngle, currentAngle, turningSpeed, boundaryAngle):
        deviation = clamp(angleDiff(aimAngle, currentAngle)/boundaryAngle, -1, 1)
        straightSpeed = (1-fabs(deviation)) * speed
        sideSpeed = deviation * turningSpeed
        L_speed = (straightSpeed - sideSpeed)
        R_speed = (straightSpeed + sideSpeed)
        return L_speed, R_speed
    
    def getMotorSpeeds(self, length, aimAngle, currentAngle, stop):
        return self.gyroCorection(
            self.getSpeed(length, stop, self.settings["straight_speed"], self.settings["min_speed"], self.settings["straight_acceleration"]*self.wheelCircumference), 
            aimAngle, currentAngle, self.settings["turning_speed"], self.settings["boundary_angle"])

    #drive base core
    def moveDistance(self, distance, stop = True, wait = True):
        '''Moves the robot a certain distance in mm.
        Parameters:
        - distance: distance in mm
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        self.locate()
        self.movePolar(distance, degrees(self.orientation), stop, wait)

    def movePolar(self, distance, orientaton, stop = True, wait = True):
        '''Moves the robot a certain distance in certain direction.
        Parameters:
        - distance: distance in mm
        - orientaton: orientation in degrees
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        self.locate()
        pos = self.pos + rotateVec2(vec2(distance, 0), radians(orientaton))
        backwards = True if distance < 0 else False
        #print(f"Moving to pos x: {round(pos.x)}, y: {round(pos.y)}, orientaton: {orientaton}, backwards: {backwards}")
        self.moveToPos(pos, backwards, stop, wait)

    def moveToPos(self, pos, backwards = False, stop = True, wait = True):
        '''Moves the robot to certain position.
        Parameters:
        - position: vec2 position in mm
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
            angleOffset = pi
        else:
            dir = 1
            angleOffset = 0
        
        onPos = False
        while not onPos:
            self.locate() #not ideal for sensor spaming
            trajectory = (pos - self.pos)
            angle = angleDiff(trajectory.orientation(), angleOffset)
            length = trajectory.length() * dir

            speeds = self.getMotorSpeeds(length, angle, self.orientation, stop)
            #print(f"Trajectory x: {round(trajectory.x)}, y: {round(trajectory.y)}, Distance: {round(length)}, Angle: {round(degrees(angle))}, speed L: {round(speeds[0])}, R: {round(speeds[1])}")
        
            self.motorsDrive(speeds[0], speeds[1])
            if fabs(length) <= self.settings["tolerance"]:
                onPos = True
                break
            yield
        if not stop:
            self.motorsStop()

    def turn(self, angle: int, stop = True, wait = True): #work in progress
        '''Turns the robot to certain orientation.
        Parameters:
        - angle: orientation in degrees
        - stop: if True, stops at the end (for connectivity)
        - wait: (backgrond) if True, runs in background'''
        self.turnCurve(angle, radius=0, stop=stop, wait=wait)
    
    def turnCurve(self, angle, radius, stop = True, wait = True):
        """Advancer curved turning (not on spot)"""
        if wait:
            for _ in self.turnCurveGen(angle, radius, stop):
                self.robot.runTasks()
        else:
            self.robot.addTask(self.turnCurveGen(angle, radius, stop))

    def turnCurveGen(self, angle, radius, stop = True):
        lRad = fabs(-radius + self.axleTrack*0.5)
        rRad = fabs(-radius - self.axleTrack*0.5)
        lRatio = lRad / rRad #left/right
        rRatio = rRad / lRad #right/left
        angle = radians(angle)
        while True:
            self.locate()
            angleD = angleDiff(self.orientation, angle)
            lDistance = angleD * lRad
            rDistance = angleD * rRad
            speed = self.getSpeed(lDistance + rDistance, stop, self.settings["turning_speed"], self.settings["min_speed"], self.settings["straight_acceleration"]*self.wheelCircumference/2)
            self.motorsDrive(speed*lRatio, -speed*rRatio)
            #print(f"Orientation: {round(degrees(self.orientation))}, angle: {round(degrees(angle))} AngleD: {round(degrees(angleD))}, lDist: {round(lDistance)}, rDist: {round(rDistance)}, speed {round(speed)}")

            if fabs(angleD) <= self.settings["angle_tolerance"]:
                break
            yield
        pass