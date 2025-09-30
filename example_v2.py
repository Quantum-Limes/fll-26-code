from spike_lib_v2.robot import Robot
from spike_lib_v2.arm import Arm
from spike_lib_v2.ride import Ride
from spike_lib_v2.mission import Mission
from pybricks.parameters import Port, Color

# 1. Initialize the Robot
# PLEASE ADJUST THE PORTS AND DIMENSIONS TO MATCH YOUR ROBOT
robot = Robot(left_motor_port=Port.A, right_motor_port=Port.E, wheel_diameter=56, axle_track=112)

# 2. Create an Arm
arm = Arm(color=Color.BLUE, motors=[])

# 3. Create Mission instances
mission1 = Mission(name="My First Mission")
mission2 = Mission(name="My Second Mission")

# 4. Define the run logic for each mission using decorators
@mission1.run
def run_mission_1(robot):
    print("Executing MyFirstMission")
    robot.straight(200)
    robot.turn(90)
    return robot.pos

@mission2.run
def run_mission_2(robot):
    print("Executing MySecondMission")
    robot.circle(100, 360)
    return robot.pos

# 5. Create a Ride
my_ride = Ride(arm)

# 6. Add the missions to the ride
my_ride.add_mission(mission1)
my_ride.add_mission(mission2)

# 7. Run the ride
my_ride.run(robot)

print("Example finished.")
