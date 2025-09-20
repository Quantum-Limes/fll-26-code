from robotlib.robot import *

class Mission:
    def __init__(self, posx: int, posy: int, ang: int, ride: ride.Ride, order: int):
        self.posx = posx
        self.posy = posy
        self.ang = ang
        self.ride = ride
        self.arm = ride.arm