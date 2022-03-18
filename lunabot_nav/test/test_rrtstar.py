#!/usr/bin/env python3
import unittest
from lunabot_nav.global_planner import RRTStarPlanner,Node
import logging

# create logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

# create formatter
formatter = logging.Formatter('%(name)s %(message)s')

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)

import numpy as np

G1 =[[0, 0, 0, 0, 0], 
     [0, 0, 1, 0, 0], 
     [0, 0, 1, 0, 0],
     [0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0]]

class RRTStarTest(unittest.TestCase):
    def setUp(self):
        self.planner = RRTStarPlanner(visualize=False,max_iter=50,goal_sample_rate=10,DISCRETIZATION_STEP=0.1)
        resolution = 0.1
        self.planner.set_grid_data(np.array(G1),resolution,5,5)
    # Testing steering and collision testing
    def test_steer_to(self):
        start = Node(np.array([0,0]))
        end = Node(np.array([0.1,0.1]))
        success, _ = self.planner.steer_to(end,start)
        self.assertTrue(success, "1 step diagonal")
        logger.debug("1 step diagonal")

        start = Node(np.array([0.1,0]))
        end = Node(np.array([0.1,0.5]))
        success, _ = self.planner.steer_to(end,start)
        self.assertFalse(success,"out of bounds")
        logger.debug("out of bounds")

        start = Node(np.array([0.1,0]))
        end = Node(np.array([0.1,0.4]))
        success, _ = self.planner.steer_to(end,start)
        self.assertFalse(success, "horizontal with obstacle")
        logger.debug("horizontal with obstacle")

        start = Node(np.array([0,0]))
        end = Node(np.array([0.4,0.4]))
        success, _ = self.planner.steer_to(end,start)
        self.assertFalse(success, "diagonal false")
        logger.debug("diagonal false")

        start = Node(np.array([0,0]))
        end = Node(np.array([0.1,0.4]))
        success, _ = self.planner.steer_to(end,start)
        self.assertTrue(success, "weird path fail")
        logger.debug("weird path success")

        start = Node(np.array([0,0]))
        end = Node(np.array([0.4,0.1]))
        success, _ = self.planner.steer_to(end,start)
        self.assertTrue(success, "weird path success 2")
        logger.debug("weird path success 2")

    def test_rrtstar_easy(self):
        start = np.array([0,0])
        end = np.array([0.1,0.1])
        plan = self.planner.plan(start,end)
        logger.debug("path: %s",plan)
        self.assertIsNotNone(plan, "1 step diagonal")

        start = np.array([0,0])
        end = np.array([0.4,0.4])
        plan = self.planner.plan(start,end)
        logger.debug("path: %s",plan)
        self.assertIsNotNone(plan, "1 step diagonal")


    def test_rrtstar_harder(self):
        self.planner.GAMMA = 20
        grid = np.zeros((200,200))
        resolution = 0.1
        lower,upper = [50,50],[70,70]
        grid[lower[0]:upper[0]+1,lower[1]:upper[1]+1] = 1

        lower,upper = [20,10],[50,70]
        grid[lower[0]:upper[0]+1,lower[1]:upper[1]+1] = 1
        self.planner.set_grid_data(grid,resolution,200,200)
        start = np.array([1,1])
        end = np.array([10,10])
        
        plan = self.planner.plan(start,end)
        logger.debug("path: %s",plan)
        self.assertIsNotNone(plan, "large obstacle")