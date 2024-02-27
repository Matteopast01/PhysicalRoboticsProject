import json

from isrlab_project.ReadConfig import ReadConfig
from pyzbar.pyzbar import decode
from PIL import Image
from io import BytesIO
import cv2
import numpy as np


class Computation:
    _orientation: float
    _config: ReadConfig
    _angle_side = {"left": np.pi / 2, "right": -np.pi / 2, "center": 0}

    def __init__(self, start_orientation):
        self._config = ReadConfig()
        self._orientation = start_orientation

    def get_orientation(self):
        return self._orientation

    def set_orientation(self, orientation):
        self._orientation = orientation

    def compute_position(self, space, orientation):
        x = space * np.cos(orientation)
        y = space * np.sin(orientation)
        return x, y

    def is_side_free(self, proximity_val):
        threshold = self._config.read_data("FREE_SIDE_THRESHOLD")
        if proximity_val == 0 or proximity_val > threshold:
            return True
        return False

    def compute_position_node(self, free_side):
        space = self._config.read_data("SPACE")
        return self.compute_position(space, self._orientation + self._angle_side[free_side])

    def recognize_img(self, img):
        decode_qr = decode(Image.open(BytesIO(img)))
        if len(decode_qr) > 0:
            text = decode_qr[0].data.decode('ascii')
            text_splitted = text.split("#")
            dict_result = {"arrived": True, "pos": [text_splitted[0], text_splitted[1]], "text": text_splitted[2]}
            return json.dumps(dict_result)
        else:
            dict_result = {"arrived": False, "pos": [-1, -1], "text": ""}
            return json.dumps(dict_result)

