# Copyright (c) 2021 fortiss GmbH
#
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import logging
import matplotlib.pyplot as plt

from common.simulator.simulator import SimulateScenario, RunSimulation
from bark.core.commons import SetVerboseLevel


SetVerboseLevel(0)
logging.getLogger().setLevel(logging.INFO)

speed_4 = 4.0

merging = "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"
start_merging_ending_lane = [966, 1008.2, 3.34]
start_merging_bottom_lane_front = [935.0, 1004.9, 3.10]
goal_merging = [900, 1004-7, -1.65]


class MiqpSimulationTest(unittest.TestCase):

    # def setUp(self):

    def tearDown(self):
        plt.close()

    def test_merging_ending_lane(self):
        scenario = [{"agent_type": "miqp", "start_pose": start_merging_ending_lane,
                     "start_vel": speed_4, "goal_poly_pose": goal_merging, "target_vel": speed_4},
                    {"agent_type": "idm", "start_pose": start_merging_bottom_lane_front,
                     "start_vel": speed_4, "goal_poly_pose": goal_merging, "target_vel": speed_4}]
        parameters = {"Miqp::WarmstartType": 2,
                      "Miqp::NrRegions": 32,
                      "Miqp::UseSpecialOrderedSet": False,
                      "Miqp::MaxSolutionTime": 5,
                      "Miqp::mipdisplay": 4,
                      "Miqp::relobjdif": 0.9,
                      "Miqp::mipemphasis": 1,
                      "Miqp::cutpass": -1,
                      "Miqp::probe": 0,
                      "Miqp::repairtries": 10,
                      "Miqp::rinsheur": 10,
                      "Miqp::varsel": 0,
                      "Miqp::mircuts": 0,
                      "Miqp::UseBranchingPriorities": True,
                      "Miqp::MaxVelocityFitting": 10.0,
                      "Miqp::AdditionalStepsForReferenceLongerHorizon": 2,
                      "Miqp::ParallelMode": -1,
                      "Miqp::DoNotChangeLaneCorridor": False,
                      "Miqp::PredictionErrorTimePercentage": 0.0
                      }
        self.assertTrue(RunSimulation(merging, scenario, parameters))


if __name__ == '__main__':
    unittest.main()
