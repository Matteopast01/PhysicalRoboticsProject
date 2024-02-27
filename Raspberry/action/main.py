#!/usr/bin/env python3
from time import sleep
from typing import Any
from my_robot.ReadConfig import ReadConfig
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot.action.body import RobotBody




class Subscriber(Node):
    _robot: RobotBody
    _controllerSubscriber: Any
    _config: ReadConfig
    _acting : bool
    _speed: list
    _abs_speed: list
    _turn_speed : float

    def __init__(self, robot):
        super().__init__("action_node")
        self._robot = robot
        self._acting = False
        self._actionSubscriber = self.create_subscription(String, "action_topic", self.sub_callback, 10)
        self.get_logger().info("Hello from action_module")
        self._config = ReadConfig()
        self._turn_speed = self._config.read_data("TURN_SPEED")
        forword_speeds = self._config.read_data("SPEED")
        abs_speeds = self._config.read_data("ABS_SPEED")
        self._speed =[-speed for speed in forword_speeds]
        self._abs_speed = [-speed for speed in abs_speeds]

    def set_speed(self, right_value, left_value):
        self._robot.move_wheels(-right_value, -left_value, -right_value, -left_value)

    def set_all_speed(self, speeds_list):
        self._robot.move_wheels(speeds_list[2], speeds_list[0], speeds_list[3],speeds_list[1])

    def make_sound(self, action_time=1):
        self._robot.run_buzzer(action_time)


    def go_forward(self, action_time=0):
        self._acting = True
        self.set_all_speed(self._speed)
        if action_time != 0:
            sleep(action_time)
            self.set_all_speed(self._abs_speed)
            self.stop()
        self._acting = False

    def turn_left(self, action_time=0):
        self._acting = True
        self.set_speed(-self._turn_speed, self._turn_speed)
        if action_time != 0:
            sleep(action_time)
            self.stop()
        self._acting = False

    def find_qr(self, action_time=0):
        self._acting = True
        self.set_speed(-self._turn_speed*0.7, self._turn_speed*0.7)
        if action_time != 0:
            sleep(action_time)
            self.stop()
        self._acting = False


    def turn_right(self, action_time= 0):
        self._acting = True
        self.set_speed(self._turn_speed, -self._turn_speed)
        if action_time != 0:
            sleep(action_time)
            self.stop()
        self._acting = False


    def stop(self, action_time = 0):
        self.set_speed(0, 0)

    def sub_callback(self, msg: String):
        if self._acting == False:
            self.get_logger().info(str(msg.data))
            action_dict = json.loads(msg.data)
            action = action_dict["action"]
            action_time = action_dict["time"]
            try:
                print("action", "action_time")
                getattr(self, action)(action_time)
            except:
                raise Exception(f"Unknown command {action}")


def main(args=None):
    robot = RobotBody("action_module")
    #robot.start()
    rclpy.init(args=args)
    node = Subscriber(robot)
    rclpy.spin(node)
    rclpy.shtdown()
    robot.stop()


if __name__ == "__main__":
     robot = RobotBody("action_module")
     rclpy.init(args=None)
     node = Subscriber(robot)
     getattr(node, "make_sound")(1)
     rclpy.spin(node)
     rclpy.shtdown()
