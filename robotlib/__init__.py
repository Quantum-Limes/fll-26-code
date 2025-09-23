'''
This is a prototype library for the fll26 robot.
'''

from robotlib.tools import *
from robotlib.robot.robot import Robot
import robotlib.robot.mission as _mission
Mission = _mission.Mission

def gandalf(robot: Robot):
    while True:
        robot.hub.speaker.play_notes(["A3/4", "R/4", "A3/8", "A3/16", "A3/16", "A3/4", "R/4", "A3/8", "A3/16", "A3/16", "A3/4", "R/8", "C4/4", "A3/8", "R/8", "G3/8", "G3/8", "F3/8", "R/8", "D3/8", "D3/8", "E3/8", "F3/8", "D3/8"], 130)