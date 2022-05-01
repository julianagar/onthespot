#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import debug as d
from debug import test_decorator

ev3 = EV3Brick()
ev3.speaker.beep()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
center_motor = Motor(Port.D)
touchSensor = TouchSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
robot.settings(250, 250, 360, 720)

def onButton(button):
    while (not button in ev3.buttons.pressed()):
        wait(10)


#WHile touch sensor not pressed, go forward. Once it is pressed, lower arm. After lowering arm, move back. 
@test_decorator
def main():

    if (Button.CENTER in ev3.buttons.pressed()):

        pass

    else: 
        
        d.run_until_stalled(1000)

    while touchSensor.pressed() is False:

        d.drive(-1000, 0)

    d.run_until_stalled(1000)

    d.drive(-1000, 0)


if __name__ == "__main__":

    main()
