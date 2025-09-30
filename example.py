from spike_lib.robot import Robot
from spike_lib.ui import UI
from spike_lib.sound import Sound
from spike_lib.ride import RideManager
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the robot
# PLEASE ADJUST THE PORTS AND DIMENSIONS TO MATCH YOUR ROBOT
robot = Robot(left_motor_port=Port.A, right_motor_port=Port.E, wheel_diameter=56, axle_track=112)

# Initialize the UI and Sound
ui = UI(robot.hub)
sound = Sound(robot.hub)

# Initialize the Ride Manager
ride_manager = RideManager(ui, robot)

# Define some mission functions
def mission_1(robot, ui):
    print("Running Mission 1")
    robot.straight(200)
    robot.turn(90)
    robot.straight(200)
    sound.play("funkytown")

def tool_change(robot, ui):
    print("Simulating tool change...")
    robot.hub.speaker.beep()
    wait(1000)
    print("Tool change finished.")

def mission_2(robot, ui):
    print("Running Mission 2")
    robot.set_drive_settings("fast")
    robot.circle(100, 360)
    robot.set_drive_settings("default")
    sound.play("megalovania")

# Define a ride using the @RideManager.ride decorator
@RideManager.ride
def my_first_ride(robot, ui):
    yield mission_1
    yield tool_change
    yield mission_2

@RideManager.ride
def another_ride(robot, ui):
    yield mission_2

# Add all registered rides to the UI
ride_manager.add_rides_to_ui()

# Start the UI
ui.screen.start()

# Main loop
while True:
    ui.screen.update()