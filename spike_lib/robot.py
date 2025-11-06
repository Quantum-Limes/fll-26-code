from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button, Color, Direction, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from spike_lib.maths import *
# from spike_lib.sound import Soundů


class Robot:
    #task management
    def __init__(self, hub: PrimeHub):
        self.hub = hub
        self.tasks = []

    def isTasksRunning(self, numOfTasks = 0):
        if len(self.tasks) > numOfTasks:
            return True
        return False
    
    def waitForTasks(self, numOfTasks = 0):
        while self.isTasksRunning(numOfTasks = numOfTasks):
            self.runTasks()
    
    def stopTasks(self):
        self.tasks = []
    
    def addTask(self, gen):
        self.tasks.append(gen)
    
    def runTasks(self):
        for task in self.tasks: #[:]
            try:
                next(task)
            except StopIteration:
                self.tasks.remove(task)

