from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class ComputeNodeAndPath(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(ComputeNodeAndPath, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"ComputeNodeAndPath::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"ComputeNodeAndPath::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"ComputeNodeAndPath::update {self.name}")
        graph = Knowledge().get_graph()
        current_position = graph.get_approximate_node(Knowledge().get_current_node())
        while not graph.is_priority_queue_empty():
            node = graph.get_next_node()
            path = graph.path_to_next_node(current_position, node)
            self._controller.print_log(f"ComputeNodeAndPath::update next_node {node}, path: {path}")
            if len(path) > 0:
                Knowledge().set_path(path)
                Knowledge().set_next_node(path.pop(0))
                return Status.SUCCESS
        return Status.FAILURE

    def terminate(self, new_status):
        self._controller.print_log(f"ComputeNodeAndPath::terminate {self.name} to {new_status}")
