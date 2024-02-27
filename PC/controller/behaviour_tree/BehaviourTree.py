from typing import Any
from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees import logging as log_tree
from isrlab_project.controller.behaviour_tree.AddNewNodes import AddNewNodes
from isrlab_project.controller.behaviour_tree.CheckPositionGoal import CheckPositionGoal
from isrlab_project.controller.behaviour_tree.CheckFinalGoal import CheckFinalGoal
from isrlab_project.controller.behaviour_tree.Error import Error
from isrlab_project.controller.behaviour_tree.SetNewGoal import SetNewGoal
from isrlab_project.controller.behaviour_tree.ComputeNodeAndPath import ComputeNodeAndPath
from isrlab_project.controller.behaviour_tree.FindQrCode import FindQrCode
from isrlab_project.controller.behaviour_tree.SetEndGame import SetEndGame
from isrlab_project.controller.behaviour_tree.CheckEndGame import CheckEndGame
from isrlab_project.controller.behaviour_tree.ResetPriorityQueue import ResetPriorityQueue
from isrlab_project.controller.behaviour_tree.SetCurrentPosition import SetCurrentPosition
from isrlab_project.controller.behaviour_tree.AmIInNextNode import AmIInNextNode
from isrlab_project.controller.behaviour_tree.ComputeAction import ComputeAction
from isrlab_project.controller.behaviour_tree.PerformAction import PerformAction
from isrlab_project.controller.behaviour_tree.IsThereAComputedPath import IsThereAComputedPath
from isrlab_project.controller.behaviour_tree.SetNextNode import SetNextNode
from isrlab_project.controller.behaviour_tree.AddBusyNodes import AddBusyNodes







class BehaviourTree:
    _root: Any

    def __init__(self, controller_handle):
        self._create_bt(controller_handle)

    def _create_bt(self, controller_handle):
        # leaves
        checkEndGame = CheckEndGame(name="CheckEndGame", controller=controller_handle)
        setCurrentPosition = SetCurrentPosition(name="SetCurrentPosition", controller=controller_handle)
        addNewNodes = AddNewNodes(name="AddNewNodes", controller=controller_handle)
        checkPositionalGoal = CheckPositionGoal(name="CheckPositionalGoal", controller=controller_handle)
        error2B = Error(name="Error2B", controller=controller_handle)
        findQrCode = FindQrCode(name="FindQrCode", controller=controller_handle)
        checkFinalGoal = CheckFinalGoal(name="checkFinalGoal", controller=controller_handle)
        setEndGame = SetEndGame(name="SetEndGame", controller=controller_handle)
        setNewGoal = SetNewGoal(name="SetNewGoal", controller=controller_handle)
        resetPriorityQueue = ResetPriorityQueue(name="ResetPriorityQueue", controller=controller_handle)
        computeNodeAndPath = ComputeNodeAndPath(name="ComputeNodeAndPath", controller=controller_handle)
        error3B = Error(name="error3B", controller=controller_handle)
        amIInNextNode = AmIInNextNode(name="AmIinNextNode", controller=controller_handle)
        computeAction = ComputeAction(name="computeAction", controller=controller_handle)
        performAction = PerformAction(name="performAction", controller=controller_handle)
        isThereAComputedPath = IsThereAComputedPath(name="isThereAComputedPath", controller=controller_handle)
        setNextNode = SetNextNode(name="setNextNode", controller=controller_handle)
        addBusyNodes = AddBusyNodes(name="addBusyNodes", controller=controller_handle)


        # level nodes
        sequence5L1B = Sequence(name="sequence5L1B", memory=True)
        sequence5L2B = Sequence(name="sequence5L1B", memory=True)
        selector4L2B = Selector(name="selector4L2B", memory=True)
        sequence3L1B = Sequence(name="sequence3L1B", memory=True)
        selector2L5B = Selector(name="selector2L5B", memory=True)
        sequence1L2B = Sequence(name="sequence1L2B", memory=True)
        selector1L3B = Selector(name="selector1L3B", memory=True)
        sequence2L6B = Sequence(name="sequence2L6B", memory=True)
        selector3L4B = Selector(name="selector3L4B", memory=True)
        sequence2L7B = Sequence(name="sequence2L7B", memory=True)
        sequence4L3B = Sequence(name="sequence4L3B", memory=True)

        root = Selector(name="root", memory=True)

        # add children
        sequence5L1B.add_children([checkFinalGoal, setEndGame])
        sequence5L2B.add_children([setNewGoal, resetPriorityQueue])
        selector4L2B.add_children([sequence5L1B, sequence5L2B])
        sequence3L1B.add_children([findQrCode, selector4L2B])
        selector2L5B.add_children([sequence3L1B, error2B])
        sequence1L2B.add_children([setCurrentPosition, addBusyNodes, addNewNodes, checkPositionalGoal, selector2L5B])
        sequence2L6B.add_children([amIInNextNode, selector3L4B])
        sequence2L7B.add_children([computeAction, performAction])
        sequence4L3B.add_children([isThereAComputedPath, setNextNode])
        selector3L4B.add_children([sequence4L3B, computeNodeAndPath, error3B])
        selector1L3B.add_children([sequence2L6B, sequence2L7B])
        root.add_children([checkEndGame, sequence1L2B, selector1L3B])

        self._root = root

    def tick(self):
        self._root.tick_once()
