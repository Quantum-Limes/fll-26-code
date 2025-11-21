from spike_lib.arms import *
from spike_lib.ultralocator import*
from spike_lib.mission_managment import*

hub = PrimeHub(front_side=Axis.Y)
Lw = Motor(Port.A, Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.E, Direction.CLOCKWISE)
Ul = UltrasonicSensor(Port.C)
La = Motor(Port.B)
Ra = Motor(Port.F)

robot = Robot(hub)
drive = Drive(robot, left_motor=Lw, right_motor=Rw, wheel_diameter=56, axle_track=160)
ultra = UltraLocator(drive, Ul, 180, vec2(-130, 0))
#drive.updateLocation(vec2(0, 0), pi/2)
# wall1 = Wall(vec2(0, 200), vec2(200, 200))
# wall2 = Wall(vec2(0, 0), vec2(0, 200))

# while True:
#     if drive.orientation > radians(-45):
#         wall = wall2
#     else:
#         wall = wall1
#     ultra.ultraLocate(wall)
#     print(f"pos: x: {round(drive.pos.x)}, y: {round(drive.pos.y)}, orientation:{round(degrees(drive.orientation))}")
#     wait(200)


# #odrive = DriveBase(Lw, Rw, 56, 160)
# #odrive.use_gyro(True)
# #drive.movePolar(100, 30, backwards=True)
# #drive.moveToPos(vec2(100, 100), backwards=True, stop=True, wait=True)
# #drive.moveToPos(vec2(0, 0), backwards=True, stop=True, wait=True)
# #odrive.straight(2000)
# #odrive.straight(-2000)