from .arm import Arm
from .mission import Mission

class Ride:
    def __init__(self, arm: Arm):
        self.arm = arm
        self.missions = []

    def add_mission(self, mission: Mission):
        self.missions.append(mission)

    def run(self, robot):
        print(f"Starting ride: {self.__class__.__name__}")
        for mission in self.missions:
            # The position update logic will be added later
            mission.execute(robot)
        print(f"Finished ride: {self.__class__.__name__}")
