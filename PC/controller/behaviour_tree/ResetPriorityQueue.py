from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class ResetPriorityQueue(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(ResetPriorityQueue, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"ResetPriorityQueue::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"ResetPriorityQueue::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"ResetPriorityQueue::update {self.name}")
        Knowledge().get_graph().reset_priority_queue()
        Knowledge().set_read_qr(False)
        Knowledge().reset_arrived_data_json()
        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"ResetPriorityQueue::terminate {self.name} to {new_status}")
