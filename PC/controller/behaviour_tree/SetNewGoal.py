from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class SetNewGoal(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(SetNewGoal, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"SetNewGoal::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"SetNewGoal::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"SetNewGoal::update {self.name}")
        new_goal = tuple(Knowledge().get_arrived_data()["pos"])
        Knowledge().set_goal((float(new_goal[0]), float(new_goal[1])))
        self._controller.print_log(f"SetNewGoal::new_goal {(float(new_goal[0]), float(new_goal[1]))}")
        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"SetNewGoal::terminate {self.name} to {new_status}")
