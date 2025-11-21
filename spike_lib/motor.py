from spike_lib.robot import *
class BetterMotor(Motor):
    def __init__(self, port: Port, robot: Robot, positive_direction = Direction.CLOCKWISE, gears = None, reset_angle = True, profile = None):
        super().__init__(port, positive_direction , gears, reset_angle, profile)
        self.robot = robot

    def align(self, speed: int, angle0: float, wait: bool = True):
        """run till stalled (but better)"""
        if wait:
            for _ in self.alignGen(speed, angle0):
                self.robot.runTasks()
        else: 
            self.robot.addTask(self.alignGen(speed, angle0))
        
    def alignGen(self, speed: int, angle0: float):
        self.run(speed)
        while not self.stalled(): #may be replaced with something better
            yield
        self.hold()
        self.reset_angle(angle0)
