# Copyright (c) 2021 fortiss GmbH
# 
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import math
import inspect

import logging

from bark.runtime.commons import ParameterServer
from common.viewer.custom_matplotlib_viewer import CustomMPViewer
from bark.runtime.commons.xodr_parser import XodrParser
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorMiqpAgent
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel
from bark.core.world import World
from bark.core.world.goal_definition import GoalDefinitionPolygon
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface
from bark.core.geometry.standard_shapes import GenerateCarRectangle, GenerateGoalRectangle
from miqp import CplexWrapper, SolutionProperties, WarmstartType

import matplotlib.pyplot as plt


def drawVelocityOverTime(last_trajectory, ref_trajectory, file_path_out):
    fig = plt.figure("miqp_vel", figsize=(8, 8), dpi=150)
    fig.clf()

    plt.plot(last_trajectory[:, 0], last_trajectory[:, 4])
    plt.plot(ref_trajectory[:, 0], ref_trajectory[:, 4], 'k--')

    plt.title('Velocities t = ' + str(last_trajectory[0, 0]))
    plt.legend(('planned', 'reference'))

    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.ylim([math.floor(min(last_trajectory[:, 4])),
              math.ceil(max(last_trajectory[:, 4]))])
    plt.savefig(file_path_out)


def RunSimulation(map_file_path, agent_dict, params=None):
    caller_function = inspect.stack()[1].function
    print("---- Simulating Testcase: " + caller_function + " ----")
    additional_parameters = {
        "Miqp::DebugFilePrefix": "py_simulator_miqp_"+caller_function+"_"}
    if params is not None:
        additional_parameters = {**additional_parameters, **params}
    ss = SimulateScenario(map_file_path, agent_dict, additional_parameters)
    ret = ss.Simulate()
    plot_file_path_out = ss.world._params["Miqp"]["DebugFilePath"] + "/" + \
        ss.world._params["Miqp"]["DebugFilePrefix"] + \
        "solution_status.png"
    data_file_path_out = ss.world._params["Miqp"]["DebugFilePath"] + "/" + \
        ss.world._params["Miqp"]["DebugFilePrefix"] + \
        "solution_status.csv"
    ss.PrintStoreSimulationStatus(plot_file_path_out, data_file_path_out)
    return ret


class SimulateScenario():

    def __init__(self, map_file_path, agent_dict, parameter_dict=None):
        # Parameters
        self.SetDefaultParameters()
        if parameter_dict != None:
            for param, val in parameter_dict.items():
                self.params[param] = val
        self.max_simulation_steps = 200

        # viewer follows agent
        # change x_length for wider view
        viewer = CustomMPViewer(params=self.params, follow_agent_id=True,
                                x_length=80, draw_env_polygon=True)
        # viewer shows full map
        # viewer = CustomMPViewer(params=self.params, use_world_bounds=True)
        self.viewer = viewer

        # Map
        xodr_parser = XodrParser(map_file_path)
        self.world = WorldForMiqpDebugging(self.params, xodr_parser.map)

        # Extract agent dics
        self.behavior_model = []
        if isinstance(agent_dict, dict):  # convenience in scenario definition
            agent_dict = [agent_dict]
        for agent in agent_dict:
            start_pose = agent.get("start_pose")
            start_vel = agent.get("start_vel")
            goal_poly_pose = agent.get("goal_poly_pose")
            agent_type = agent.get("agent_type")

            # Goal
            goal_poly = GenerateGoalRectangle(20, 5)
            goal_poly = goal_poly.ScalingTransform(
                1, np.array([goal_poly_pose[0], goal_poly_pose[1], goal_poly_pose[2]]))

            # Agent
            if agent_type == "miqp":
                self.behavior_model.append(BehaviorMiqpAgent(self.params))
                init_state = np.array(
                    [0, start_pose[0], start_pose[1], start_pose[2], start_vel])
                self.agent = TestAgent(
                    init_state, self.behavior_model[-1], goal_poly, self.world.map, self.params)
                self.world.AddAgent(self.agent)

            elif agent_type == "idm":
                # if no desired velocity specified, set start velocity as desired.
                if self.params["BehaviorIDMClassic"]["DesiredVelocity"] is not None:
                    self.params["BehaviorIDMClassic"]["DesiredVelocity"] = start_vel
                other_behavior_model = BehaviorIDMClassic(self.params)
                init_state = np.array(
                    [0, start_pose[0], start_pose[1], start_pose[2], start_vel])
                agent = TestAgent(init_state, other_behavior_model,
                                  goal_poly, self.world.map, self.params)
                self.world.AddAgent(agent)

            else:
                raise TypeError("Unknown agent type: " + agent_type)

        # store solutions
        self.solution_properties = []

    def SetDefaultParameters(self):
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
        params["Miqp"]["WarmstartType"] = 0  # no warmstart
        params["Miqp"]["NrSteps"] = 10
        params["Miqp"]["Ts"] = 0.5

        params["Visualization"]["Agents"]["Alpha"]["Other"] = 0.4
        params["Visualization"]["Agents"]["Color"]["UseColormapForOtherAgents"] = True
        params["Visualization"]["Agents"]["DrawAgentId"] = False

        params["Visualization"]["Agents"]["DrawBehaviorPlanEvalAgent"] = True

        params["Simulation"]["StepTime"] = params["Miqp"]["Ts"]

        self.params = params

    def Simulate(self):
        for __ in range(0, self.max_simulation_steps):
            # here we assume that the first miqp agent is the ego agent
            self.world.Step(self.viewer, self.behavior_model[0])
            self.solution_properties.append(
                self.behavior_model[0].GetLastSolutionProperties())
            if self.behavior_model[0].last_planning_success != True:
                return False
            if (self.agent.goalReached()):
                return True
        return False

    def PrintStoreSimulationStatus(self, savepath_fig, savepath_data):
        t = []
        s = []
        g = []
        o = []
        for sp in self.solution_properties:
            t.append(sp.time)
            s.append(sp.status)
            g.append(sp.gap)
            o.append(sp.objective)
        # print("Miqp Solver Time: ", t)
        fig, axs = plt.subplots(4)
        fig.suptitle('miqp_solution_properties')
        # fig = plt.figure("miqp_solution_properties", figsize=(8, 8), dpi=150)
        # fig.clf()
        axs[0].plot(np.array(t), 'rx-')
        axs[0].legend('solution time [s]')
        axs[0].set(xlabel='step', ylabel='time [s]')
        axs[1].plot(np.array(s), 'rx-')
        axs[1].legend('solution status [-]')
        axs[1].set(xlabel='step', ylabel='status [-]')
        axs[2].plot(np.array(g), 'rx-')
        axs[2].legend('aptimallity gap [-]')
        axs[2].set(xlabel='step', ylabel='gap [-]')
        axs[3].plot(np.array(o), 'rx-')
        axs[3].legend('obective value [-]')
        axs[3].set(xlabel='step', ylabel='objective [-]')
        plt.savefig(savepath_fig)
        data = np.array([t, s, g, o]).transpose()
        # print(data)
        np.savetxt(savepath_data, data, delimiter=",")


class WorldForMiqpDebugging(World):
    """Derived World Class to plot miqp plan after planning before execution
    """

    def __init__(self, params, xodr_map):
        World.__init__(self, params)
        self.LoadMapInterface(xodr_map)
        self._params = params

    def Step(self, viewer, behavior_model_miqp):
        # instead of world.Step(), we call Plan() and Execute() seperately, so that we can plot
        # world and planning at the current timestep, and not after all agents have moved (execute)
        delta_time = self._params["Simulation"]["StepTime"]
        self.PlanAgents(delta_time)

        if (behavior_model_miqp.last_planning_success):
            idx = self.time / delta_time

            viewer.clear()
            viewer.drawPlanningMiqp(behavior_model_miqp)
            # viewer.show(block=True)
            # we need to pass ego agent id for viewer feature to follow agent
            filename = self._params["Miqp"]["DebugFilePath"] + "/" + \
                self._params["Miqp"]["DebugFilePrefix"] + \
                "img" + str(idx) + ".png"
            agent_ids = list(self.agents.keys())
            viewer.drawWorld(self, eval_agent_ids=[agent_ids[0]], filename=filename)
            # viewer.show(block=True)

            try:
                vel_file_path_out = self._params["Miqp"]["DebugFilePath"] + "/" + \
                    self._params["Miqp"]["DebugFilePrefix"] + \
                    "velocity" + str(idx) + ".png"
                drawVelocityOverTime(behavior_model_miqp.last_trajectory,
                                     behavior_model_miqp.ref_trajectories[0], vel_file_path_out)
            except:
                logging.info("Cannot draw velocity over time")

            self.Execute(delta_time)

    def LoadMapInterface(self, xodr_map):
        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_map)
        self.SetMap(map_interface)


class TestAgent(Agent):
    """Derived World Class to plot miqp plan after planning before execution
    """

    def __init__(self, init_state, behavior_model, goal_polygon, map_interface, params):

        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        l = params["Miqp"]["WheelBase"]
        r = params["Miqp"]["CollisionRadius"]
        agent_2d_shape = GenerateCarRectangle(l, r)

        agent_params = params.AddChild("agent")
        super(TestAgent, self).__init__(init_state, behavior_model, dynamic_model, execution_model,
                                        agent_2d_shape, agent_params, GoalDefinitionPolygon(goal_polygon), map_interface)
        self._params = params

    def goalReached(self):
        return super(TestAgent, self).AtGoal()
