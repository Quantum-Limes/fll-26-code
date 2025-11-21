from test import *
from pybricks.iodevices import XboxController
from pybricks.tools import Matrix, StopWatch

controller = XboxController()
timer = StopWatch()

turnSpeed = 0.4
strSpeed = 1

cursor = [0, 0]
settings = [3, 0, 0]
speeds = [0.5, 1, 2, 4, 9.5]
arms = [La, Ra]
armSpeeds = [0.5, 1, 2, 4, 9.5]
settingsAtributes = [speeds, arms, armSpeeds]
lightIntensity = [[30, 75], [50, 100]]

timeout = 200
delay = [0, 0, 0, 0, 0]

hub.display.off()
while True:
    drive.locate()

    time = timer.time()
    pressed = controller.buttons.pressed()
    Rdir = controller.joystick_left()
    Adir = controller.joystick_right()
    armcontrol = controller.triggers()
    dpad = controller.dpad()

    if dpad == 1 and delay[0] < time:
        hub.speaker.beep(500, 50)
        cursor[0] = (cursor[0] - 1) % len(settingsAtributes)
        cursor[1] = settings[cursor[0]]
        delay[0] = time + timeout
    elif dpad == 3 and delay[1] < time:
        hub.speaker.beep(500, 50)
        cursor[1] = (cursor[1] + 1) % len(settingsAtributes[cursor[0]])
        delay[1] = time + timeout
    elif dpad == 5 and delay[2] < time:
        hub.speaker.beep(500, 50)
        cursor[0] = (cursor[0] + 1) % len(settingsAtributes)
        cursor[1] = settings[cursor[0]]
        delay[2] = time + timeout
    elif dpad == 7 and delay[3] < time:
        hub.speaker.beep(500, 50)
        cursor[1] = (cursor[1] - 1) % len(settingsAtributes[cursor[0]])
        delay[3] = time + timeout

    settings[cursor[0]] = cursor[1] % len(settingsAtributes[cursor[0]])
    speed = speeds[settings[0]]
    motor = arms[settings[1]]
    armsSpeed = armSpeeds[settings[2]]

    Lw.run(speed * strSpeed * Rdir[1] + speed * turnSpeed * Rdir[0])
    Rw.run(speed * strSpeed * Rdir[1] - speed * turnSpeed * Rdir[0])
    motor.run(armsSpeed * (armcontrol[1] - armcontrol[0]))

    pixel = [0, 0]
    for atribute in settingsAtributes:
        pixel[1] = 0
        if pixel[0] == cursor[0]:
            rowselected = 1
        else:
            rowselected = 0
        for value in atribute:
            if pixel[1] == settings[pixel[0]]:
                valueselected = 1
            else:
                valueselected = 0
            hub.display.pixel(pixel[0], pixel[1], lightIntensity[rowselected][valueselected])
            pixel[1] += 1
        pixel[0] += 1

    if Button.X in pressed and delay[4] < time:
        hub.speaker.beep(1000, 50)
        print(f"Status: pos: vec2({round(drive.pos.x)}, {round(drive.pos.y)}), Orientation: {round(degrees(drive.orientation))}, La: {round(La.angle())}, Ra: {round(Ra.angle())}, speed {round(speed)}")
        delay[4] = time + timeout