from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class Error(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(Error, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"Error::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"Error::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"Error::update {self.name}")
        Knowledge().set_end_game(True)
        self._controller.perform_action("stop")
        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"Error::terminate {self.name} to {new_status}")
