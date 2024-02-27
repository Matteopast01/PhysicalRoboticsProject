#! # !/usr/bin/env python3
from typing import Any, List
from my_robot.action.Motor import Motor
from my_robot.action.Buzzer import Buzzer
import time
MY_SIM_HOST = "host.docker.internal"  # from the container

class RobotBody:
    _PWM: Motor
    _Buzzer: Buzzer


    def __init__(self, name: str):
        self._my_name = name
        self._PWM = Motor()
        self._Buzzer = Buzzer()

    def move_wheels(self, front_right, front_left, back_right, back_left):
        self._PWM.setMotorModel(int(front_right), int(back_right) , int(front_left), int(back_left))

    def run_buzzer(self, sound_time):
        self._Buzzer.run('1')
        time.sleep(sound_time)
        self._Buzzer.run('0')

if __name__ == "__main__":
    robot = RobotBody("name")
    robot.run_buzzer(1)
