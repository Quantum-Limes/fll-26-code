from pybricks.tools import wait

class Sound:
    def __init__(self, hub):
        self.hub = hub
        self.songs = {
            "funkytown": [
                [440, 100], [440, 100], [587, 100], [440, 100], [415, 100], [415, 100], [554, 100], [415, 100],
                [370, 100], [370, 100], [493, 100], [370, 100], [440, 200], [370, 100], [440, 200]
            ],
            "megalovania": [
                [293, 100], [293, 100], [587, 100], [440, 100], [415, 100], [392, 100], [349, 100], [293, 100],
                [349, 100], [392, 100]
            ]
        }

    def play(self, song_name, mult=1):
        if song_name in self.songs:
            notes = self.songs[song_name]
            for i in notes:
                self.hub.speaker.beep(i[0], i[1] * mult)
                wait(i[1] * mult)
        else:
            print(f"Song '{song_name}' not found.")
