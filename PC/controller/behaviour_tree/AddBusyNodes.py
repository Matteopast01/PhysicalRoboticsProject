from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class AddBusyNodes(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(AddBusyNodes, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"AddBusyNodes::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"AddBusyNodes::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"AddBusyNodes::update {self.name}")
        delta_busy_nodes = Knowledge().get_delta_busy_nodes()
        current_node = Knowledge().get_current_node()
        for delta in delta_busy_nodes:
            new_node_x = current_node[0] + delta[0]
            new_node_y = current_node[1] + delta[1]
            new_node = (new_node_x, new_node_y)
            Knowledge().add_busy_node(new_node)

        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"AddBusyNodes::terminate {self.name} to {new_status}")
