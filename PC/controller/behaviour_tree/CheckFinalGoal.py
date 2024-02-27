from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class CheckFinalGoal(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(CheckFinalGoal, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"CheckFinalGoal::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"CheckFinalGoal::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"CheckFinalGoal::initialise {self.name}")
        text_to_decode = Knowledge().get_arrived_data()["text"]
        decoded_text = Knowledge().decode_text_qr(text_to_decode)
        if decoded_text == Knowledge().read_config_var("STRING_TARGET_ARRIVE"):
            return Status.SUCCESS
        else:
            Knowledge().set_text_goal(decoded_text)
            return Status.FAILURE

    def terminate(self, new_status):
        self._controller.print_log(f"CheckFinalGoal::terminate {self.name} to {new_status}")
