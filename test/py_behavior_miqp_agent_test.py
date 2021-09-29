# Copyright (c) 2021 fortiss GmbH
# 
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import time
import math

from random import random
import logging

from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from common.viewer.custom_matplotlib_viewer import CustomMPViewer
from common.simulator.simulator import WorldForMiqpDebugging, TestAgent
from bark.runtime.commons.xodr_parser import XodrParser
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorIDMLaneTracking, BehaviorMiqpAgent
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel
from bark.core.world import World
from bark.core.world.goal_definition import GoalDefinitionPolygon
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface
from bark.core.geometry.standard_shapes import CarLimousine, CarRectangle, GenerateCarRectangle, GenerateGoalRectangle
from bark.core.geometry import Point2d, Polygon2d
from bark.core.world.opendrive import OpenDriveMap, MakeXodrMapOneRoadTwoLanes, MakeXodrMapCurved, XodrDrivingDirection
from bark.core.commons import SetVerboseLevel
from bark.core.models.dynamic import StateDefinition
from miqp import CplexWrapper, SolutionProperties, WarmstartType

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

SetVerboseLevel(5)
logging.getLogger().setLevel(logging.INFO)

max_step_number = 3


class BehaviorMiqpAgentTest(unittest.TestCase):
    """Closed loop test of BehaviorMiqpAgent on various BARK maps
    """

    def setUp(self):
        params = ParameterServer()

        params["Miqp"]["WriteDebugFiles"] = True
        params["Miqp"]["DebugFilePath"] = "/tmp"
        params["Miqp"]["DesiredVelocity"] = 4.0
        params["Miqp"]["CollisionRadius"] = 0.9
        params["Miqp"]["WheelBase"] = 2.7
        params["Miqp"]["MaxSolutionTime"] = 100
        params["Miqp"]["UseBoxAsEnv"] = False
        params["Miqp"]["AdditionalStepsForReferenceLongerHorizon"] = 2
        #params["Miqp"]["RefLineInterpInc"] = 0.2
        params["Miqp"]["Precision"] = 8
        params["Miqp"]["RelativeMiqpGapTolerance"] = 0.1
        params["Miqp"]["NrRegions"] = 16
        params["Miqp"]["NrNeighbouringPossibleRegions"] = 2

        params["Visualization"]["Agents"]["Alpha"]["Other"] = 0.4
        params["Visualization"]["Agents"]["Color"]["UseColormapForOtherAgents"] = True
        params["Visualization"]["Agents"]["DrawAgentId"] = False

        params["Visualization"]["Agents"]["DrawBehaviorPlanEvalAgent"] = True

        params["Simulation"]["StepTime"] = 0.25

        self.params = params

        # viewer follows agent
        # change x_length for wider view
        viewer = CustomMPViewer(params=params, follow_agent_id=True,
                                x_length=80)

        # viewer shows full map
        #viewer = CustomMPViewer(params=params, use_world_bounds=True, axis=axis00)

        self.viewer = viewer

    def tearDown(self):
        plt.close()

    #@unittest.skip
    def test_merging_ending_lane(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_merging_ending_lane_"

        xodr_file_path = "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"

        xodr_parser = XodrParser(xodr_file_path)
        world = WorldForMiqpDebugging(params, xodr_parser.map)

        vel_other = 4
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other

        # Model Definitions
        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 966, 1008.2, 3.34, vel_des])  # before merge
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 935.0, 1004.9, 3.10, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        for __ in range(0, 1):
            world.Step(self.viewer, behavior_model)
            self.assertTrue(behavior_model.last_planning_success)

    #@unittest.skip
    def test_merging_continuing_lane_before_merging_point(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_merging_continuing_lane_before_merging_point_"
        params["Miqp"]["NrRegions"] = 32

        xodr_file_path = "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"

        xodr_parser = XodrParser(xodr_file_path)
        world = WorldForMiqpDebugging(params, xodr_parser.map)

        vel_other = 4
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other

        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 966, 1005.5, 3.14, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 935.0, 1004.9, 3.10, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        for __ in range(0, 1):
            world.Step(self.viewer, behavior_model)
            self.assertTrue(behavior_model.last_planning_success)

    #@unittest.skip
    def test_merging_continuing_lane_after_merging_point(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_merging_continuing_lane_after_merging_point_"

        xodr_file_path = "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"

        xodr_parser = XodrParser(xodr_file_path)
        world = WorldForMiqpDebugging(params, xodr_parser.map)

        vel_other = 4
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other
        params["Miqp"]["WheelBase"] = 2.7
        params["Miqp"]["NrRegions"] = 32

        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 938.0, 1004.8, 3.10, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 966, 1005.2, 3.14, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        for __ in range(0, 1):
            world.Step(self.viewer, behavior_model)
            self.assertTrue(behavior_model.last_planning_success)

    #@unittest.skip
    def test_curved_road(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_curved_road_"
        params["Miqp"]["NrRegions"] = 32

        xodr_map = MakeXodrMapCurved(80, 0.07)
        world = WorldForMiqpDebugging(params, xodr_map)

        behavior_model = BehaviorMiqpAgent(params)

        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 2.6, -1.6, 0.17, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))

        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        for __ in range(0, 1):
            world.Step(self.viewer, behavior_model)
            self.assertTrue(behavior_model.last_planning_success)

    #@unittest.skip
    def test_straight_driving_one_road_two_lanes(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_straight_driving_one_road_two_lanes_"

        xodr_map = MakeXodrMapOneRoadTwoLanes()
        world = WorldForMiqpDebugging(params, xodr_map)

        vel_other = 4
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other

        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 3, -1.75, 0, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 12, -1.75, 0, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        for __ in range(0, 1):
            world.Step(self.viewer, behavior_model)
            self.assertTrue(behavior_model.last_planning_success)

    #@unittest.skip
    def test_straight_driving_one_road_two_lanes_without_rule_no_right_passing(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_straight_driving_one_road_two_lanes_"
        params["Miqp"]["RuleNoRightPassing"] = False

        xodr_map = MakeXodrMapOneRoadTwoLanes()
        world = WorldForMiqpDebugging(params, xodr_map)

        vel_other = 2
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other

        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 3, -1.75-3.5, 0, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 12, -1.75, 0, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        world.Step(self.viewer, behavior_model)
        self.assertTrue(behavior_model.last_planning_success)
        for idx in range(1, len(behavior_model.last_trajectory)):
            vel_i = behavior_model.last_trajectory[idx, int(StateDefinition.VEL_POSITION)]
            self.assertAlmostEqual(vel_i, vel_des, delta=1e-6)

    #@unittest.skip
    def test_straight_driving_one_road_two_lanes_with_rule_no_right_passing(self):
        params = self.params
        params["Miqp"]["DebugFilePrefix"] = "behavior_miqp_agent_test_straight_driving_one_road_two_lanes_"
        params["Miqp"]["RuleNoRightPassing"] = True

        xodr_map = MakeXodrMapOneRoadTwoLanes()
        world = WorldForMiqpDebugging(params, xodr_map)

        vel_other = 2
        params["BehaviorIDMClassic"]["DesiredVelocity"] = vel_other

        behavior_model = BehaviorMiqpAgent(params)
        vel_des = params["Miqp"]["DesiredVelocity"]
        init_state = np.array([0, 3, -1.75-3.5, 0, vel_des])
        goal_poly = GenerateGoalRectangle(2, 2)
        goal_poly = goal_poly.Translate(Point2d(50, -2))
        agent = TestAgent(init_state, behavior_model,
                          goal_poly, world.map, params)
        world.AddAgent(agent)

        behavior_model2 = BehaviorIDMClassic(params)
        init_state2 = np.array([0, 12, -1.75, 0, vel_other])
        agent2 = TestAgent(init_state2, behavior_model2,
                           goal_poly, world.map, params)
        world.AddAgent(agent2)

        world.Step(self.viewer, behavior_model)
        self.assertTrue(behavior_model.last_planning_success)
        # after some time, we expect a slowdown
        for idx in range(5, len(behavior_model.last_trajectory)):
            vel_i = behavior_model.last_trajectory[idx, int(StateDefinition.VEL_POSITION)]
            self.assertLess(vel_i, vel_des)

if __name__ == '__main__':
    unittest.main()
