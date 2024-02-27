#! # !/usr/bin/env python3
from typing import Any, List
import numpy as np
from PIL import Image
import cv2
import io
import requests
from my_robot.sense.Ultrasonic import Ultrasonic
from my_robot.sense.ArduinoSensors import ArduinoSensors

# MY_SIM_HOST = "localhost"  # in PyCharm


#MY_HOST = "host.docker.internal"  # from the container
MY_HOST = "localhost"
URL = f"http://{MY_HOST}:5008/"

class RobotBody:
    _sensors_values: dict
    _center_ultrasonic: Ultrasonic
    _arduino_connector: ArduinoSensors

    def __init__(self, name: str):
        self._my_name = name
        print("init robot")
        self._center_ultrasonic = Ultrasonic()
        print("ultasonic init")
        self._arduino_connector = ArduinoSensors()
        print("serial init")
        img = requests.get(URL + "sense/camera").json()["data"]
        print("api init")
        # zmqRemoteApi connection
        print("Connecting to simulator...")
        self._sensors_values = {"ultrasonic":
        {
            "left": self._arduino_connector.get_left_sensor(),
            "center": self._center_ultrasonic.get_distance(),
            "right": self._arduino_connector.get_right_sensor()
        },
        "camera": img,
        "compass": self._arduino_connector.get_orientation()
        }
        print("SIM objects referenced")

    def update_sensors(self):
        img = requests.get(URL + "sense/camera").json()["data"]
        self._sensors_values = {"ultrasonic":
        {
            "left": self._arduino_connector.get_left_sensor(),
            "center": self._center_ultrasonic.get_distance(),
            "right": self._arduino_connector.get_right_sensor()
        },
        "camera": img,
        "compass": self._arduino_connector.get_orientation()
        }

    def read_orientation(self, axis=2, convert_to_degree=False):
        if convert_to_degree:
            return self._sensors_values["compass"]
        else:
            angle_rad = np.deg2rad(self._sensors_values["compass"])
            angle_rad = (2*np.pi - angle_rad) % (2*np.pi)
            return angle_rad

    """ this method works but we can't retrieve the position because we are cheating
    def read_position(self):

        return self._sim.getObjectPosition(self._my_pioneer, self._sim.handle_world)
        """
    

    def read_camera(self):
        # require to api
        return self._sensors_values["camera"]

    def read_proximity_sensor(self, sensor_name=None):
        try:
            # i = 0 : front sensors
            # i = 1 : back sensors
            assert sensor_name in self._sensors_values.keys() or sensor_name == None
            if sensor_name == None:
                values = {}
                for sensor, value in self._sensors_values["ultrasonic"].items():
                    values[sensor] = value
                return values
            else:
                value = self._sensors_values[sensor_name]
                return value
        except Exception as e:
           print("sensor_name must be one of left, center or right")
