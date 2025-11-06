from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color
from pybricks.tools import wait
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import Matrix

# Image data from imgs.py
beari = Matrix([
    [0, 0, 0, 0, 0],
    [0, 100, 100, 100, 0],
    [0, 0, 100, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])

road = Matrix([
    [100, 0, 1000, 0, 100],
    [100, 0, 100, 0, 100],
    [100, 0, 0, 0, 100],
    [100, 0, 100, 0, 100],
    [0, 0, 0, 0, 0]
])

shortroadi = Matrix([
    [100, 0, 0, 0, 100],
    [100, 0, 0, 0, 100],
    [100, 0, 0, 0, 100],
    [100, 0, 0, 0, 100],
    [0, 0, 0, 0, 0]
])

ultraroadi = Matrix([
    [100, 0, 1000, 0, 100],
    [100, 0, 100, 0, 100],
    [0, 0, 0, 0, 0],
    [100, 0, 100, 0, 100],
    [0, 0, 0, 0, 0]
])

smile = Matrix([
    [0, 100, 0, 100, 0],
    [0, 0, 0, 0, 0],
    [100, 0, 0, 0, 100],
    [0, 100, 100, 100, 0],
    [0, 0, 0, 0, 0]
])

sad = Matrix([
    [0, 100, 0, 100, 0],
    [0, 0, 0, 0, 0],
    [0, 100, 100, 100, 0],
    [100, 0, 0, 0, 100],
    [0, 0, 0, 0, 0]
])

fish = Matrix([
    [100, 0, 100, 100, 0],
    [0, 100, 100, 100, 100],
    [100, 0, 100, 100, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])

fish1 = Matrix([
    [0, 100, 100, 0, 100],
    [100, 100, 100, 100, 0],
    [0, 100, 100, 0, 100],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])

a = 30
b = 100
c = 55

skull = Matrix([
    [a, a, a, a, a],
    [a, a, a, a, a],
    [a, 0, a, 0, a],
    [0, a, a, a, 0],
    [0, a, 0, a, 0]
])

skull2 = Matrix([
    [a, a, a, a, a],
    [a, a, a, a, a],
    [a, 0, a, b, a],
    [0, a, a, a, 0],
    [0, a, 0, a, 0]
])

d = 100
e = 0
f = 100
s = 100

a1 = Matrix([
    [0, 0, 0, 0, 0],
    [e, e, e, e, e],
    [d, d, d, d, d],
    [0, d, d, d, 0],
    [0, 0, f, 0, 0],
])

a2 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [e, e, e, e, e],
    [d, d, d, d, d],
    [0, f, f, f, 0],
])

a3 = Matrix([
    [0, 0, s, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [e, e, e, e, e],
    [f, f, f, f, f],
])

a4 = Matrix([
    [0, s, s, s, 0],
    [0, 0, s, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [e, e, e, e, e],
])

a5 = Matrix([
    [s, s, f, s, s],
    [0, s, s, s, 0],
    [0, 0, s, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
])

a6 = Matrix([
    [0, 0, 0, 0, 0],
    [s, s, f, s, s],
    [0, s, s, s, 0],
    [0, 0, s, 0, 0],
    [0, 0, 0, 0, 0],
])

arrow = [a1, a2, a3, a4, a5, a6]

# Loading animation from screen.py
l1 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [30, 60, 100, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l2 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 30, 60, 100, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l3 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 30, 60, 100],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l31 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 30, 100],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l4 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 100, 60],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l5 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0,0, 100, 60, 30],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l6 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 100, 60, 30, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l7 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [100, 60, 30,0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l71 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [100, 30, 0,0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
l8 = Matrix([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [60, 100, 0,0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])
loading = [l1, l2, l3, l31, l4, l5, l6, l7, l71, l8]

def maxV(a, b):
    return max(a, b)

def minV(a, b):
    return min(a, b)

class Page:
    def __init__(self, func, icon=None, image=None, delta=0):
        self.func = func
        self.icon = icon
        self.image = image
        self.hub = None # Will be set by the Screen class
        self.delta = delta

    def renderIcon(self):
        if self.icon is None:
            for i in range(5):
                for o in range(4):
                    self.hub.display.pixel(i, o, 0)
            return
        if hasattr(self.icon, 'shape'):
            for i in range(maxV(self.icon.shape[0], 4)):
                for o in range(self.icon.shape[1]):
                    self.hub.display.pixel(o, i, self.icon[i, o])
        else:
            self.hub.display.image(self.icon)

    def renderImage(self):
        if self.image is None:
            return
        if self.delta == 0:
            self.hub.display.image(self.image)
        else:
            self.hub.display.animate(self.image, self.delta)

class UI:
    def __init__(self, hub):
        self.hub = hub
        self.screen = self.Screen(hub)

    class Screen:
        def __init__(self, hub: PrimeHub, menuRow=4):
            self.menuRow = menuRow
            self.hub = hub
            self.width = 5
            self.height = 5
            self.currPage = 0
            self.pages = []
            self.hub.light.on(Color.CYAN)

        def __str__(self):
            return f"Screen(width={self.width}, height={self.height})"

        def __repr__(self):
            return self.__str__()

        def addPage(self, page):
            page.hub = self.hub
            self.pages.append(page)

        def start(self):
            self.renderPage()

        def update(self):
            if Button.LEFT in self.hub.buttons.pressed():
                self.currPage = (self.currPage - 1) % len(self.pages)
                self.renderPage()
                self.release(Button.LEFT)

            if Button.RIGHT in self.hub.buttons.pressed():
                self.currPage = (self.currPage + 1) % len(self.pages)
                self.renderPage()
                self.release(Button.RIGHT)

            if Button.BLUETOOTH in self.hub.buttons.pressed():
                self.battery()
                self.renderPage()

            if Button.CENTER in self.hub.buttons.pressed():
                self.hub.light.on(Color.RED)
                self.hub.display.clear()
                self.loading()
                while Button.CENTER in self.hub.buttons.pressed():
                    pass
                self.pages[self.currPage].renderImage()
                self.hub.light.on(Color.MAGENTA)
                self.pages[self.currPage].func()
                raise SystemExit("Exiting screen")

        def battery(self):
            max_voltage = 8400
            min_voltage = 6000
            voltage = self.hub.battery.voltage()
            per = int(100 * (voltage - min_voltage) / (max_voltage - min_voltage))
            print(per)
            a = maxV(minV(per - 80, 0) * 5, 100)
            b = maxV(minV(per - 60, 0) * 5, 100)
            c = maxV(minV(per - 40, 0) * 5, 100)
            d = maxV(minV(per - 20, 0) * 5, 100)
            e = maxV(minV(per, 0) * 5, 100)
            time = 0
            dur = 3000
            delta = 10
            t = [0, 0, 0, 0, 0]
            k = 0
            while Button.BLUETOOTH in self.hub.buttons.pressed():
                t[k] = (time) // (dur // 500) - k * 100
                if t[k] >= 100:
                    k += 1
                i = 15
                bat = Matrix([
                    [t[4], 0, 0, a, i],
                    [t[3], 0, 0, b, 0],
                    [t[2], 0, 0, c, 0],
                    [t[1], 0, 0, d, 0],
                    [t[0], 0, 0, e, i]
                ])
                self.hub.display.image(bat)
                time += delta
                if time > dur:
                    raise SystemExit("Exiting screen")
                wait(delta)

        def release(self, button: Button):
            a = 0
            while button in self.hub.buttons.pressed() and a < 200:
                wait(10)
                a += 10

        def renderPage(self):
            self.render()
            self.pages[self.currPage].renderIcon()

        def loading(self):
            self.hub.display.animate(loading, 80)

        def render(self):
            for i in range(self.width):
                if len(self.pages) == 0:
                    return
                bright = 0
                if i == self.currPage:
                    bright = 100
                elif i < len(self.pages):
                    bright = 30
                self.hub.display.pixel(i, self.menuRow, bright)