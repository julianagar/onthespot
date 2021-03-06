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
import mission as m

ev3 = EV3Brick()
ev3.speaker.beep()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
center_motor = Motor(Port.D)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
robot.settings(250, 250, 360, 720)

# While touch sensor not pressed, go forward. Once it is pressed, lower arm. After lowering arm, move back. 
@test_decorator
def main():

    d.straight(1035)

    d.run_until_stalled(-1000)

    m.drag_motor(50, 1000, 12)

    d.straight(-1100)

if __name__ == "__main__":
        
    main()
