from isrlab_project.ReadConfig import ReadConfig
from isrlab_project.controller.PriorityQueue import PriorityQueue
import numpy as np


class MapGraph:
    _graph: dict
    _visited: set
    _queue: PriorityQueue
    _node_distance_threshold: float
    _node_distance_threshold_small: float
    _position_goal: tuple
    _busy_nodes: list
    _k_goal: float

    def __init__(self, start_node, position_goal):
        self._node_distance_threshold = ReadConfig().read_data("NODE_DISTANCE_THRESHOLD")
        self._k_goal = ReadConfig().read_data("PRIORITY_WEIGHT")
        self._node_distance_threshold_small = ReadConfig().read_data("AM_I_IN_NEXT_NODE_THRESHOLD")
        self._graph = {start_node: []}
        self._visited = set()
        self._visited.add(start_node)
        self._position_goal = position_goal
        self._queue = PriorityQueue()
        self._busy_nodes = []

    def set_positional_goal(self, new_goal):
        self._position_goal = new_goal

    def add_node(self, node_from: tuple, new_node: tuple):
        if self.is_just_visited(new_node):
            new_node = self.get_approximate_node(new_node)
            if node_from not in self._graph[node_from]:
                self._graph[node_from].append(new_node)
            if new_node not in self._graph[new_node]:
                self._graph[new_node].append(node_from)
        else:
            self._graph[node_from].append(new_node)
            self._graph[new_node] = [node_from]
            self._queue.put((self._compute_priority(new_node, self._k_goal), new_node))

    def add_busy_node(self, busy_node):
        if self.is_node_new(busy_node, self._busy_nodes):
            self._busy_nodes.append(busy_node)

    def _compute_priority(self, node, k_goal=0.5):
        goal_ptp_dis = self._compute_ptp_distance(node)
        busy_nodes_ptp_dis = [self._compute_ptp_distance(node, busy_node) for busy_node in self._busy_nodes]
        if len(busy_nodes_ptp_dis) == 0:
            avg_busy_nodes_distance = 0
        else:
            avg_busy_nodes_distance = np.min(self._busy_nodes)
        return k_goal * goal_ptp_dis - (1 - k_goal) * avg_busy_nodes_distance

    def _compute_ptp_distance(self, node, node_from=None):
        if node_from is None:
            node_from = self._position_goal
        return np.sqrt((node_from[0] - node[0]) ** 2 + (node_from[1] - node[1]) ** 2)

    def is_nodes_position_equals(self, node1, node2, small = False):
        distance = np.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)
        if small:
            return distance < self._node_distance_threshold_small
        return distance < self._node_distance_threshold

    def is_node_new(self, node, nodes_list):
        for v in nodes_list:
            if self.is_nodes_position_equals(node, v):
                return False
        return True

    def is_just_visited(self, node):
        return not self.is_node_new(node, self._graph.keys())

    def get_approximate_node(self, node):
        min_distance = np.inf
        result_node = None
        for graph_node in self._graph.keys():
            distance = self._compute_ptp_distance(node, graph_node)
            if distance < min_distance:
                min_distance = distance
                result_node = graph_node

        if result_node is None:
            raise ValueError("get_approximate_node returns None")

        return result_node

    def reset_priority_queue(self):
        self._queue = PriorityQueue()
        for node in self._graph.keys():
            self._queue.put((self._compute_priority(node, self._k_goal), node))

    def remove_node(self, node):
        for n in self._graph[node]:
            self._graph[n].remove(node)
        del self._graph[node]

    def get_next_node(self):
        if not self._queue.empty():
            return self._queue.get()[1]
        return None

    def is_priority_queue_empty(self):
        return self._queue.empty()

    def get_priority_queue(self):
        return self._queue

    def path_to_next_node(self, current_node: tuple, arrive_node: tuple):
        visited = {current_node}
        queue = [current_node]
        tree = {}
        while len(queue) > 0:
            node_v = queue.pop(0)
            visited.add(node_v)
            for node_u in self._graph[node_v]:
                if node_u not in visited:
                    queue.append(node_u)
                    visited.add(node_u)
                    tree[node_u] = node_v
                    if node_u == arrive_node:
                        path = []
                        while node_u in tree:
                            path.insert(0, node_u)
                            node_u = tree[node_u]
                        return path
        return []
