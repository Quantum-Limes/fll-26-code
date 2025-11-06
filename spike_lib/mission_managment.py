from spike_lib.arms import *
class Mission:
    def __init__(self, pos: vec2, rot: float, run: function):
        self.run = run
        self.pos = pos
        self.rot = rot
        self.completed = False

    def goToMission(self, drive: Drive, checkpoints: list = [vec2]):
        if len(checkpoints):
            for point in checkpoints:
                drive.moveToPos(point)
            if not self.completed:
                drive.moveToPos(self.pos)
                drive.rotate(self.rot)
                self.run(drive)
                self.completed = True
               
class Ride:
    def __init__(self, arm: Arm):
        self.arm = arm
        arm.align()
        self.color = arm.color
        self.missions = {}
    
    def run(self, drive: Drive):
        for mission in self.missions.values():
            mission.goToMission(drive)

    def appendMission(self, mission: Mission):
        self.missions.append(mission)

def rideManager(rides: set, La, Ra, colorSensor: ColorSensor, hub):
    def scanForArm():
        while 1:
            La.run_angle(400, 45, Stop.HOLD, False)
            Ra.run_angle(400, 45, Stop.HOLD, True)
            color = colorSensor.color() 
            if color != Color.NONE: break
            wait(150)
            La.run_angle(400, 45, Stop.HOLD, False)
            Ra.run_angle(400, 45, Stop.HOLD, True)
            if color != Color.NONE: break
            wait(150)
        for ride in rides:
            if ride.color == color:
                return ride
        return None
        
    while True:
        scan = scanForArm()
        if scan != None:
            confirmed = 0
            while not confirmed: confirmed = True if hub.buttons.pressed()[Button.CENTER] else False
            scan.run()
    

