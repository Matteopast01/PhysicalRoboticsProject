from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class IsThereAComputedPath(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(IsThereAComputedPath, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"IsThereAComputedPath::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"IsThereAComputedPath::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"IsThereAComputedPath::update {self.name}")
        if len(Knowledge().get_path()) > 0:
            return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self._controller.print_log(f"IsThereAComputedPath::terminate {self.name} to {new_status}")
