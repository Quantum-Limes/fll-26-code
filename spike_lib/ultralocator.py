from spike_lib.drive import *
class UltraLocator:
    def __init__(self, drive: Drive, sensor: UltrasonicSensor, direction: float, offset: vec2, field: mat2):
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
        self.field = self.setField(vec2(0, 0), vec2(1, 0), vec2(0, 1), vec2(1, 1))

    def getDirVec(self, wall: mat2):
        return vec2(wall.m[0][0], wall.m[0][1]) - vec2(wall.m[0][0], wall.m[0][1])
    
    def getNormalVec(self, dirVec: vec2):
        return normalizeVec2(rotateVec2(dirVec, pi/2))

    def setField(self, a: vec2, b: vec2, c: vec2, d: vec2):
        """sets the field object from corners
        named from down left to right than up"""
        return [mat2(a.x, a.y, b.x, b.y), mat2(b.x, b.y, c.x, c.y), mat2(c.x, c.y, d.x, d.y), mat2(d.x, d.y, a.x, a.y)]

    def moveField(self, shift: vec2):
        field = []
        for wall in self.field:
            field.append(mat2(wall.m[0][0] + shift[0], wall.m[0][1] + shift[1], wall.m[1][0] + shift[0], wall.m[1][1] + shift[1]))
        self.field = field

    def read(self, orientation): #in radians
        distance = self.sensor.distance()  # in mm
        if distance == 2000:  # no reading
            print("Ultralocator: ERROR no obsticles found?")
            return None
        return rotateVec2(rotateVec2(vec2(distance, 0), self.direction) + self.offset, orientation)   # in mm
    
    def calculateToWall(self, pos: vec2, wall: mat2):
        return self.toWall(pos - vec2(wall.m[0][0], wall.m[0], [1])  - vec2(wall.m[1][0], wall.m[1], [1]), wall)

    def toWall(self, shift: vec2, wall: mat2):
        """from orientation and wall direction vector calculates normal vector of posible location line"""
        if shift is None:
            return None
        wallNormal = self.getNormalVec(self.getDirVec(wall))
        return vec2(shift.x * wallNormal.x, shift.y * wallNormal.y)

    def updateField(self, recalculate: bool = True):
        """This function works with known location and accordingly ajusts the field wall matrix"""
        self.drive.locate()
        wall = self.field[1]
        offset = self.toWall(self.read(self.drive.orientation), wall) - self.calculateToWall(self.drive.pos, wall)
        self.drive.pos += offset #not shure, not tested
                

    def ultraLocate(self):
        """This function works with known field wall matrix and accordingly ajusts the robot location"""
        self.drive.locate()
        wall = self.field[1]
        offset = self.toWall(self.read(self.drive.orientation), wall) - self.calculateToWall(self.drive.pos, wall)
        self.moveField(-offset) #not shure, not tested
        
        