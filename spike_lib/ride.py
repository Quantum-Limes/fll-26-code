from .ui import Page
from pybricks.tools import wait

class RideManager:
    rides = {}

    @staticmethod
    def ride(func):
        RideManager.rides[func.__name__] = func
        return func

    def __init__(self, ui, robot):
        self.ui = ui
        self.robot = robot

    def add_rides_to_ui(self):
        for ride_name in self.rides:
            # Create a lambda to capture the ride_name for the run_ride method
            run_func = lambda name=ride_name: self.run_ride(name)
            self.ui.screen.addPage(Page(run_func, icon=self.ui.smile))

    def run_ride(self, ride_name):
        if ride_name in self.rides:
            ride_generator = self.rides[ride_name](self.robot, self.ui)
            for step_func in ride_generator:
                if callable(step_func):
                    print(f"Executing step {step_func.__name__} of ride {ride_name}")
                    step_func(self.robot, self.ui)
                    print("Step finished. Waiting for user to continue.")
                    self.robot.hub.speaker.beep()
                    while not self.robot.hub.buttons.pressed():
                        wait(10)
                    # Wait for button release
                    while self.robot.hub.buttons.pressed():
                        wait(10)
