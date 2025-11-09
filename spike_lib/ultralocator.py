from spike_lib.drive import *
class UltraLocator:
    def __init__(self, drive: Drive, sensor: UltrasonicSensor, direction: float, offset: vec2, field: mat2):
        self.drive = drive
        self.sensor = sensor
        self.direction = radians(direction)  # in degrees
        self.offset = offset  # vec2 in mm
        self.field = field

    def read(self, orientation): #in radians
        distance = self.sensor.distance()  # in mm
        if distance == 2000:  # no reading
            return None
        return ((vec2(distance, 0)).rotated(self.direction) + self.offset).rotated(orientation)   # in mm
    
    def toWall(self, shift: vec2, wallDirVec2: vec2):
        """from orientation and wall direction vector calculates normal vector of posible location line"""
        if shift is None:
            return None
        wallNormal = (wallDirVec2.rotated(pi/2)).normalized()
        return vec2(shift.x * wallNormal.x, shift.y * wallNormal.y)
        
    def updateField(self, recalculate: bool = True):
        """This function works with known location and accordingly ajusts the field wall matrix"""
        shift = self.read(self.drive.orientation)
        if shift is None:
            return

    def ultraLocate(self):
        """This function works with known field wall matrix and accordingly ajusts the robot location"""
        orientation = self.drive.orientation
        shift = self.read(orientation)
        if shift is None:
            return