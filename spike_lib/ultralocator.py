from spike_lib.drive import *
class Wall:
    def __init__(self, a: vec2, b: vec2):
        self.a = a
        self.b = b
        self.dirVec = a - b
        self.orientation = angleDiff((a - b).orientation(), 0)
        self.normalVec = normalizeVec2(rotateVec2(a - b, pi/2))

    def move(self, shift: vec2):
        self.a += shift
        self.b += shift

def setField(a: vec2, b: vec2, c: vec2, d: vec2):
    """sets the field object from corners
    named from down left to right than up"""
    return [Wall(a, b), Wall(b, c), Wall(c, d), Wall(d, a)]

class UltraLocator:
    def __init__(self, drive: Drive, sensor: UltrasonicSensor, direction: float, offset: vec2):
        """This is advanced aligning tool sutiated for robots operating in enclosed 2D enviroment with ultrasonic sensor in the plane
        drive extension
        Parameters:
            - drive: Drive
            - sensor: UltrasonicSesnor
            - direction: float in degreas, the orientation of the sensor relative to the robot (the sensor points forward => direction = 0°) (when the robot points at 0° the relative direction of sensor is equal to the absolute)
            - offset: vec2 in mm, distance from the sensor centre to the centre of rotation
            - filed: mat2 of vec2"""
        self.drive = drive
        self.sensor = sensor
        self.direction = radians(direction)  # in degrees
        self.offset = offset  # vec2 in mm
        self.field = setField(vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 1))
        self.settings = {"locate_tolerance": 50, "update_tolerance": 10}

    def moveField(self, shift: vec2):
        for wall in self.field:
            wall.move(shift)

    def read(self, orientation): #in radians
        distance = self.sensor.distance()  # in mm
        if distance == 2000:  # no reading
            print("Ultralocator: ERROR no obsticles found?")
            return None
        return rotateVec2(rotateVec2(vec2(distance, 0), self.direction) + self.offset, orientation)   # in mm
    
    def calculateToWall(self, pos: vec2, wall: Wall):
        return self.toWall(wall.a - pos, wall)

    def toWall(self, shift: vec2, wall: Wall):
        """from orientation and wall direction vector calculates normal vector of posible location line"""
        if shift is None:
            return None
        return shift.split(wall.orientation + pi/2)

    def getPosOffset(self, wall: Wall):
        self.drive.locate()
        shift = self.read(self.drive.orientation)
        if shift:
            return self.toWall(shift, wall) - self.calculateToWall(self.drive.pos, wall)
        else:
            return vec2(0, 0)

    def locateGeneral(self, wall: Wall):
        """This function works known with wall and accordingly ajusts the robot location"""
        shift = self.getPosOffset(wall)
        if shift.length() < self.settings["locate_tolerance"]:
            self.drive.pos -= shift
        else:
            print("ultraLocate failed due unexpected size of shift")    

    def locate(self, wallID: int):
        """This function works with known wall and accordingly ajusts the robot location"""
        shift = self.getPosOffset(self.field[wallID])
        if shift.length() < self.settings["locate_tolerance"]:
            self.drive.pos -= shift
        else:
            print("ultraLocate failed due unexpected size of shift")

    def updateWall(self, wall: Wall):
        """This function works with known location and accordingly ajusts the wall"""
        wall.move(self.getPosOffset(wall))

    def updateField(self, wallID: int, recalculate: bool = True):
        """This function works with known location and accordingly ajusts the field wall list"""
        if recalculate:
            self.moveField(self.getPosOffset(self.field[wallID]))
        else:
            self.updateWall(self.field[wallID])

    
        