from time import sleep
from typing import Any
import numpy as np
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.Knowledge import Knowledge


class ComputeAction(Behaviour):
    _controller: Any

    def __init__(self, name, controller):
        super(ComputeAction, self).__init__(name)
        self._controller = controller

    def setup(self):
        self._controller.print_log(f"ComputeAction::setup {self.name}")

    def initialise(self):
        self._controller.print_log(f"ComputeAction::initialise {self.name}")

    def update(self):
        self._controller.print_log(f"ComputeAction::update {self.name}")

        current_node = Knowledge().get_current_node()
        next_node = Knowledge().get_next_node()
        current_orientation = Knowledge().get_orientation()
        next_orientation = self.convert_angle(np.arctan2(next_node[1] - current_node[1], next_node[0] - current_node[0])) 
        angle_to_perform = self._delta_angle(next_orientation, current_orientation)
        self._controller.print_log(f"ComputeAction:: angle current {current_orientation} next {next_orientation}")
        self._controller.print_log(f"ComputeAction:: current node {current_node} next node {next_node}")
        self._controller.print_log(f"ComputeAction:: angle_to_perform {angle_to_perform}")

        self._compute_action(angle_to_perform)


        return Status.SUCCESS


    def _compute_action(self, angle_to_perform):
        slope = Knowledge().read_config_var("SLOPE")
        offset = Knowledge().read_config_var("OFFSET")
        if angle_to_perform > 0.4:
            Knowledge().set_action("turn_left")
            prova = "ti ho detto gira a sinistra"
            self._controller.print_log(f"ComputeAction:: action_time {prova}")
            action_time = offset + slope * (np.abs(angle_to_perform/2))
            Knowledge().set_end_action_time(action_time)

        elif angle_to_perform <=- 0.4:
            Knowledge().set_action("turn_right")
            prova = "ti ho detto gira a destra"
            self._controller.print_log(f"ComputeAction:: computeAction {prova}")
            action_time = offset + slope * (np.abs(angle_to_perform /2))
            Knowledge().set_end_action_time(action_time)

        else:
            if Knowledge().is_side_free("center"):
                speed = Knowledge().read_config_var("SPEED")
                space = Knowledge().read_config_var("SPACE")
                Knowledge().set_end_action_time(space/speed)
                Knowledge().set_action("go_forward")
            else:
                Knowledge().set_next_node(None)

    def _delta_angle(self, next, current):
       a = next - current
       if a> np.pi:
           a-=2*np.pi
       elif a<-np.pi:
           a+=2*np.pi
       return a

    def convert_angle(self, angle):
        if angle < 0:
            return 2 * np.pi + angle
        return angle

    def terminate(self, new_status):
        self._controller.print_log(f"ComputeAction::terminate {self.name} to {new_status}")
