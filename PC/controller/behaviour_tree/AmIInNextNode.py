from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class AmIInNextNode(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(AmIInNextNode, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"AmIInNextNode::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"AmIInNextNode::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"AmIInNextNode::update {self.name}")
        current_pos = Knowledge().get_current_node()
        next_node = Knowledge().get_next_node()
        graph = Knowledge().get_graph()
        if next_node is None or graph.is_nodes_position_equals(current_pos, next_node, True):
            self._controller.perform_action("stop")
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self._controller.print_log(f"AmIInNextNode::terminate {self.name} to {new_status}")
