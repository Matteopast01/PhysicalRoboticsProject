from time import sleep
from time import time
from typing import Any

import numpy as np
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class SetCurrentPosition(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(SetCurrentPosition, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"SetCurrentPosition::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"SetCurrentPosition::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"SetCurrentPosition::update {self.name}")
        orientation = Knowledge().get_orientation()
        action = Knowledge().get_action()
        last_position = Knowledge().get_current_node()
        now = time()
        start_action_time = Knowledge().get_start_action_time()
        speed = Knowledge().read_config_var("SPEED")
        if action == "go_forward":
            end_action_time = Knowledge().get_end_action_time()
            if now < start_action_time + end_action_time:
                space= speed *(now-start_action_time)
                Knowledge().set_start_action_time(now)
                end_action_time = end_action_time -(now-start_action_time)
                Knowledge().set_end_action_time(end_action_time)
            else:
                space = end_action_time * speed
            #Knowledge().set_start_action_time(now)
            x_pos = last_position[0] + space * np.cos(orientation)
            y_pos = last_position[1] + space * np.sin(orientation)
            Knowledge().set_current_node((x_pos, y_pos))
        self._controller.print_log(f"SetCurrentPosition::update {Knowledge().get_current_node()}")
        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"SetCurrentPosition::terminate {self.name} to {new_status}")
