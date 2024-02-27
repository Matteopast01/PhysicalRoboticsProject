import json
from time import sleep, time
from typing import Any
import random
import json
from isrlab_project.ReadConfig import ReadConfig
import base64
from isrlab_project.controller.behaviour_tree.BehaviourTree import BehaviourTree
from isrlab_project.controller.Knowledge import Knowledge
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool

MOTION_WAIT = 0.5


class Controller(Node):
    _print_enabled = []
    _print_arrived_data = True
    _new_node_sub: Any
    _free_side_sub: Any
    _arrived_sub: Any
    _action_pub: Any
    _behavior_tree: BehaviourTree
    _config: ReadConfig
    _orientation_sub: Any

    def __init__(self):
        super().__init__("controller_node")
        self._behavior_tree = BehaviourTree(self)
        self._config = ReadConfig()

        self._free_side_sub = self.create_subscription(String, "free_side", self.free_side_callback, 10)
        self._arrived_sub = self.create_subscription(String, "arrived", self.arrived_callback, 10)
        self._orientation_sub = self.create_subscription(Float32, "orientation", self.orientation_callback, 10)

        self._action_pub = self.create_publisher(String, "action_topic", 10)

        self.get_logger().info("Hello from controller_module")

    def free_side_callback(self, msg: String):
        if self._print_arrived_data:
            self.get_logger().info("free side: " + str(msg))
        dict_side = json.loads(msg.data)
        # reset nodes list
        Knowledge().reset_delta_neighbors()
        Knowledge().reset_neighbors()
        Knowledge().reset_delta_busy_nodes()
        # split nodes between neighbors and busy_nodes
        for side, dict_new_node in dict_side.items():
            new_delta_node = (dict_new_node["dx"], dict_new_node["dy"])
            if dict_new_node["free"]:
                Knowledge().add_delta_pos_neighbors(side, new_delta_node)
            else:
                Knowledge().add_delta_busy_nodes(new_delta_node)

        self._behavior_tree.tick()

    def arrived_callback(self, msg: String):
        if self._print_arrived_data and True:
            self.get_logger().info("arrived: " + str(msg))
        Knowledge().set_arrived_data_json(msg.data)

    def orientation_callback(self, msg: Float32):
        if self._print_arrived_data and False:
            self.get_logger().info("orientation: " + str(msg))
        Knowledge().set_orientation(msg.data)

    def perform_action(self, action):
        action_msg = String()
        Knowledge().set_action(action)
        Knowledge().set_start_action_time(time())
        action_time = Knowledge().get_end_action_time()
        action = {"action": action, "time": action_time}
        action = json.dumps(action)
        action_msg.data = action
        self._action_pub.publish(action_msg)

    def perform_action_with_time(self, action, action_time):
         action_msg = String()
         Knowledge().set_action(action)
         Knowledge().set_start_action_time(time())
         action = {"action": action, "time": action_time}
         action = json.dumps(action)
         action_msg.data = action
         self._action_pub.publish(action_msg)




    def print_log(self, text):
        splitted_text = text.split("::")
        node_name = splitted_text[0]
        if len(self._print_enabled)==0 or node_name in self._print_enabled:
            self.get_logger().info("python print : " + str(text))


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shtdown()
