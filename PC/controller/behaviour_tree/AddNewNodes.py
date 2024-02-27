from time import sleep
from typing import Any

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class AddNewNodes(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(AddNewNodes, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"AddNewNodes::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"AddNewNodes::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"AddNewNodes::update {self.name}")
        current_node = Knowledge().get_current_node()
        current_node = Knowledge().get_graph().get_approximate_node(current_node)
        action = Knowledge().get_action()
        for side in ["left", "right", "center"]:
            if Knowledge().is_side_free(side):
                delta = Knowledge().get_delta_pos_neighbors(side)
                new_node_x = current_node[0] + delta[0]
                new_node_y = current_node[1] + delta[1]
                new_node = (new_node_x, new_node_y)
                Knowledge().get_graph().add_node(current_node, new_node)
                if Knowledge().get_graph().is_just_visited(new_node):
                    new_node = Knowledge().get_graph().get_approximate_node(new_node)
                    self._controller.print_log(f"AddNewNodes::new_node {new_node}")
                Knowledge().add_neighbors(side, new_node)

        return Status.SUCCESS

    def terminate(self, new_status):
        self._controller.print_log(f"AddNewNodes::terminate {self.name} to {new_status}")
