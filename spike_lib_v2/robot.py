from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import cos, sin, atan2, pi, sqrt

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

    def __str__(self):
        return f"vec2({self.x}, {self.y})"

    def __repr__(self):
        return self.__str__()

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

# Original rdevice class
class rdevice:
    def __init__(self, port: Port):
        self.port = port

# Original motor class
class motor(rdevice):
    def __init__(self, port: Port):
        super().__init__(port)
        self.reverse = False
        self.offset = 0
        self.switchDir = False
        self.m_motor = Motor(port)
        self.deltaAngle = 0
        self.lastAngle = self.angleRad()

    def setDefAngle(self, angle = 0):
        self.offset = angle/180*pi + self.angleRad()

    def setSpeed(self, speed: float):
        if self.reverse:
            self.m_motor.run(-speed)
        else:
            self.m_motor.run(speed)

    def stop(self):
        self.m_motor.stop()

    def Update(self):
        if self.reverse:
            self.deltaAngle = -self.angleRad() + self.lastAngle
        else:
            self.deltaAngle = self.angleRad() - self.lastAngle
        self.lastAngle = self.angleRad()

    def brake(self):
        self.m_motor.brake()

    def hold(self):
        self.m_motor.hold()

    def angle(self):
        return float(self.m_motor.angle()) - self.offset/pi * 180

    def angleRad(self):
        return float(self.m_motor.angle())/180 * pi - self.offset

# Original Ultrasonic class
class Ultrasonic(rdevice):
    def __init__(self, port: Port):
        super().__init__(port)
        self.m_sensor = UltrasonicSensor(port)

    def distance(self):
        return self.m_sensor.distance()/10

    def angle(self):
        return self.m_sensor.angle()

    def angleRad(self):
        return self.m_sensor.angle()/180 * pi

# Original hub class
class hub:
    def __init__(self):
        self.m_hub = PrimeHub()
        self.angleOffset = 0
        self.resetAngle()
        self.setOffButton(Button.BLUETOOTH)
        self.switch = False

    def addOffset(self, offset):
        self.angleOffset += offset

    def angle(self):
        if self.switch:
            return - (self.m_hub.imu.rotation(Axis.Z) - self.angleOffset)
        return self.m_hub.imu.rotation(Axis.Z) - self.angleOffset

    def angleRad(self):
        if self.switch:
            return -(self.m_hub.imu.rotation(Axis.Z) - self.angleOffset) / 180 * pi
        return (self.m_hub.imu.rotation(Axis.Z) - self.angleOffset) / 180 * pi

    def resetAngle(self):
        self.angleOffset = self.m_hub.imu.rotation(Axis.Z)

    def pixel(self,x,y, brigthness=100):
        self.m_hub.display.pixel(y, x, brigthness)

    def beep(self, freq, duration):
        self.m_hub.speaker.beep(freq, duration)

    def setVolume(self, volume):
        self.m_hub.speaker.volume(volume)

    def notes(self, notes, tempo=120):
        self.m_hub.speaker.play_notes(notes, tempo)

    def isButtonPressed(self, button: Button):
        return True if button in self.m_hub.buttons.pressed() else False

    def setOffButton(self, button: Button):
        self.m_hub.system.set_stop_button(button)

    def color(self, color: Color):
        self.m_hub.light.on(color)

    def colorAnimate(self, colors, duration=100):
        self.m_hub.light.animate(colors, duration)

    def animate(self, animation, delta):
        self.m_hub.display.animate(animation, delta)

    def image(self, image):
        self.m_hub.display.icon(image)

    def clear(self):
        self.m_hub.display.off()

# Original navigate function
def navigate(lM:motor, rM:motor, hub:hub, diameter):
    scalar = (lM.deltaAngle*diameter + rM.deltaAngle*diameter) * 0.25
    vec = scalar * mat2.multiply(mat2.rotation(hub.angleRad()), vec2(1, 0))
    lM.Update()
    rM.Update()
    return vec

# Main Robot class integrating all functionalities
class Robot:
    def __init__(self, left_motor_port, right_motor_port, wheel_diameter, axle_track):
        self.hub = hub()
        self.lM = motor(left_motor_port)
        self.rM = motor(right_motor_port)
        self.devices = []
        self.pos = vec2(0, 0)
        self.Diameter = wheel_diameter
        self.axle = axle_track
        self.drive_base = DriveBase(self.lM.m_motor, self.rM.m_motor, wheel_diameter, axle_track)

        # driveManager attributes
        self.speeds = [0, 0, 0, 0]
        self.modes = {
            "default": [500, 1000, 200, 500, Stop.BRAKE],
            "fast": [1000, 2000, 400, 1000, Stop.BRAKE],
            "precise": [200, 500, 100, 200, Stop.BRAKE],
            "start": [500, 1000, 200, 500, Stop.HOLD],
            "connect": [500, 1000, 200, 500, Stop.COAST],
            "finish": [500, 1000, 200, 500, Stop.HOLD]
        }
        self.current_mode = "default"
        self.setMode("default")
        self.tasks = []
        self.hook_speeds = [1, -1]
        self.hooks = [False, False]
        self.side = 1 # 1 for blue, -1 for red
        self.side_color = Color.BLUE

    # Methods from original robot class
    def setSpeed(self, lSpeed: float, rSpeed: float):
        self.lM.setSpeed(lSpeed)
        self.rM.setSpeed(rSpeed)

    def stop(self, brake = True):
        if brake:
            self.lM.hold()
            self.rM.hold()
            wait(200)
            self.lM.brake()
            self.rM.brake()
        else:
            self.lM.brake()
            self.rM.brake()

    def addDevice(self, device:rdevice):
        self.devices.append(device)

    def update(self):
        self.pos += navigate(self.lM, self.rM, self.hub, self.Diameter)

    # Methods from driveManager
    def setMode(self, mode):
        if mode in self.modes:
            self.current_mode = mode
            s, sa, t, ta, stop = self.modes[mode]
            self.drive_base.settings(s, sa, t, ta)
            self.drive_base.stop_type = stop
        else:
            print(f"Drive mode '{mode}' not found.")

    def setDefaultMode(self):
        self.setMode("default")

    def setFastMode(self):
        self.setMode("fast")

    def setPreciseMode(self):
        self.setMode("precise")

    def setStartMode(self):
        self.setMode("start")

    def setConnectMode(self):
        self.setMode("connect")

    def setFinishMode(self):
        self.setMode("finish")

    def open(self):
        self.lM.setSpeed(500)
        self.rM.setSpeed(-500)

    def close(self):
        self.lM.setSpeed(-500)
        self.rM.setSpeed(500)

    def turnMotorRad(self, motor_obj: motor, angle_rad, speed=500, simple=False):
        if simple:
            motor_obj.m_motor.run_angle(speed, angle_rad * 180 / pi, Stop.HOLD, wait=True)
        else:
            # Original complex logic for turnMotorRad
            pass # This needs to be implemented from driveFunc.py

    def turnMotor(self, motor_obj: motor, angle, speed=500, simple=False):
        self.turnMotorRad(motor_obj, angle * pi / 180, speed, simple)

    def addTask(self, task):
        self.tasks.append(task)

    def runTasks(self):
        for task in self.tasks:
            try:
                next(task)
            except StopIteration:
                self.tasks.remove(task)

    def isTasksRunning(self):
        return len(self.tasks) > 0

    def waitForTasks(self):
        while self.isTasksRunning():
            self.runTasks()
            wait(10)

    def stopTasks(self):
        self.tasks = []

    def straight(self, distance):
        self.drive_base.straight(distance)

    def toPos(self, target_pos, speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        # For now, using simplified version
        current_pos = self.pos
        delta = target_pos - current_pos
        distance = delta.length()
        if distance == 0:
            return

        target_angle = atan2(delta.y, delta.x) * 180 / pi
        turn_angle = target_angle - self.angle
        if backwards:
            turn_angle += 180

        while turn_angle > 180:
            turn_angle -= 360
        while turn_angle < -180:
            turn_angle += 360

        self.drive_base.turn(turn_angle)
        self.drive_base.straight(distance if not backwards else -distance)
        self.pos = target_pos
        self.angle = target_angle

    def rotate(self, angle, speed=None):
        self.drive_base.turn(angle)

    def rotateRad(self, angle_rad, speed=None):
        self.drive_base.turn(angle_rad * 180 / pi)

    def circle(self, radius, angle, connect=[False, False], speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        # For now, using simplified version
        self.drive_base.curve(radius, angle)

    def bezier(self, p1, p2, p3, num_points=20):
        # This needs to be implemented from driveFunc.py
        # For now, using simplified version
        points = generateBezierCurve(self.pos, p1, p2, p3, num_points)
        for point in points:
            self.toPos(point)

    # Helper functions from driveFunc.py
    def calcDir(self, target_pos):
        # This needs to be implemented from driveFunc.py
        pass

    def calcSpeed(self, distance, speed, acceleration, deceleration):
        # This needs to be implemented from driveFunc.py
        pass

    def calcSpeedR(self, angle, speed, acceleration, deceleration):
        # This needs to be implemented from driveFunc.py
        pass

    def angleDiff(self, a, b):
        # This needs to be implemented from driveFunc.py
        pass

    def circleToPos(self, target_pos, connect=[False, False], speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def circleToAngle(self, angle, connect=[False, False], speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def circleToAngleRad(self, angle_rad, connect=[False, False], speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def circleToPosRad(self, target_pos, connect=[False, False], speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def bezierToPos(self, p1, p2, p3, num_points=20, speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def bezierToAngle(self, p1, p2, p3, num_points=20, speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def bezierToAngleRad(self, p1, p2, p3, num_points=20, speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def bezierToPosRad(self, p1, p2, p3, num_points=20, speed=None, backwards=False):
        # This needs to be implemented from driveFunc.py
        pass

    def setMotorsToDef(self):
        # This needs to be implemented from driveFunc.py
        pass

    def hook_align(self, speed = 500):
        # This needs to be implemented from driveFunc.py
        pass

    def hook_setup(self, angle = 130, speed = -500):
        # This needs to be implemented from driveFunc.py
        pass

    def hook_pickup(self):
        # This needs to be implemented from driveFunc.py
        pass

    def hook_drop(self, N: int):
        # This needs to be implemented from driveFunc.py
        pass

    def battery_pickup(self, pickup_pos):
        # This needs to be implemented from driveFunc.py
        pass

    def battery_delivery(self, car_number, Starting_pos = vec2(80, 35), car_distance = 20, hook_shift = 2, both = False):
        # This needs to be implemented from driveFunc.py
        pass

    def enemy_sken(self, ang = 180, val = 150, patience = 5, sample = 500):
        # This needs to be implemented from driveFunc.py
        pass

    def car_sken(self, pos, distance = 20):
        # This needs to be implemented from driveFunc.py
        pass

    def ultra_align(self, shift = 5):
        # This needs to be implemented from driveFunc.py
        pass

    def side_decider(self):
        # This needs to be implemented from driveFunc.py
        pass

    def roadside_setup(self, side):
        # This needs to be implemented from driveFunc.py
        pass

    def m1(self):
        # This needs to be implemented from driveFunc.py
        pass

    def m2(self):
        # This needs to be implemented from driveFunc.py
        pass

    def m22(self):
        # This needs to be implemented from driveFunc.py
        pass

    def m3(self):
        # This needs to be implemented from driveFunc.py
        pass

    def mF(self):
        # This needs to be implemented from driveFunc.py
        pass

    def Roadmain1(self):
        # This needs to be implemented from driveFunc.py
        pass

    def shortroad(self):
        # This needs to be implemented from driveFunc.py
        pass

    def Roadmain(self):
        # This needs to be implemented from driveFunc.py
        pass

    def ultraroad(self):
        # This needs to be implemented from driveFunc.py
        pass
