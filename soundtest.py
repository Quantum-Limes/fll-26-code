from spike_lib.robot import PrimeHub
from spike_lib.sound import Sound

hub = PrimeHub()

sound = Sound(hub)

sound.play("megalovania")