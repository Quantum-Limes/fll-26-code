#you can run whatever is there by F5
from spike_lib.robot import PrimeHub, Motor, Port, Direction, Robot, Drive
from spike_lib.maths import vec2
hub = PrimeHub()
Lw = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
Rw = Motor(Port.C)
robot = Robot(hub)
drive = Drive(robot, Lw, Rw, 56, 96)

hub.speaker.beep()

drive.moveToPos(vec2(500,0))

drive.moveToPos(vec2(10,0))
#drive.driveBase.use_gyro(True)
#robot.hub.imu.reset_heading(0)
#drive.driveBase.straight(1000)
#drive.driveBase.straight(-1000)


#drive.moveDistance(200)
