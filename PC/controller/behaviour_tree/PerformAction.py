from time import sleep
from typing import Any
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class PerformAction(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(PerformAction, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"PerformAction::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"PerformAction::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"PerformAction::update {self.name}")
        action = Knowledge().get_action()
        self._controller.perform_action(action)
        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"PerformAction::terminate {self.name} to {new_status}")
