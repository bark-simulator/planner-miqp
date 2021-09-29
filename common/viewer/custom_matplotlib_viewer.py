# Copyright (c) 2021 fortiss GmbH
# 
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import logging
import os
import math
from sortedcontainers import SortedDict

from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.core.geometry import Point2d, Polygon2d
try:
    from bark.core.models.behavior import BehaviorMiqpAgent, OccupancyType
except:
    logging.warning("Cannot import BehaviorMiqpAgent")

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.transforms as mtransforms
import matplotlib.gridspec as gridspec

from bark.core.models.dynamic import *

def drawFrontAxleApproximation(viewer, behavior_model_miqp):
    frontPts = behavior_model_miqp.GetFrontLbUb()
    xlb = frontPts[0, :]
    ylb = frontPts[1, :]
    xub = frontPts[2, :]
    yub = frontPts[3, :]
    for idx in range(0, len(xlb)):
        poly = Polygon2d()
        poly.AddPoint([xlb[idx], ylb[idx]])
        poly.AddPoint([xub[idx], ylb[idx]])
        poly.AddPoint([xub[idx], yub[idx]])
        poly.AddPoint([xlb[idx], yub[idx]])
        poly.AddPoint([xlb[idx], ylb[idx]])
        viewer.drawPolygon2d(poly, 'r', 0.5, (1.0, 1.0, 1.0))


def drawShrinkedConvexEnv(viewer, behavior_model_miqp):
    for conv_env_p in behavior_model_miqp.convex_shrinked_env_polygons_all_cars.values():
        color_rgb = (0, 0.3961, 0.7412)  # TUM blue
        viewer.drawPolygon2d(conv_env_p, 'k', 0.2, color_rgb)


def drawCollisionCircles(viewer, behavior_model_miqp):
    """
    Draws collision circles at state 0
    """
    circles = behavior_model_miqp.GetCollisionCircleCenters()
    r = behavior_model_miqp.GetCollisionRadius()
    viewer.drawCircles(circles, r)


class CustomMPViewer(MPViewer):

    def __init__(self,
                 params=None,
                 **kwargs):
        
        self._draw_reference = kwargs.pop("draw_reference", False)
        self._draw_env_polygon = kwargs.pop("draw_env_polygon", False)
        self._draw_shrinked_convex_env = kwargs.pop("draw_shrinked_convex_env", False)
        self._draw_front_axle = kwargs.pop("draw_front_axle", False)
        self._draw_collision_circles = kwargs.pop("draw_collision_circles", False)
        self._drawDynamicOccupancies = kwargs.pop("drawDynamicOccupancies", True)
        self._drawPrediction = kwargs.pop("drawPrediction", True)
        self._occupancyConstantSize = kwargs.pop("occupancyConstantSize", False)

        # setup viewer
        fig_plt = plt.figure(figsize=(8, 8), dpi=150)
        gs = gridspec.GridSpec(1, 1, width_ratios=[1], height_ratios=[
                               1], left=0, right=1, bottom=0, top=1)
        axis00 = plt.subplot(gs[0])
        kwargs.setdefault('axis', axis00)

        super(CustomMPViewer, self).__init__(params, **kwargs)

    def drawBehaviorPlan(self, agent):
        behavior_model = agent.behavior_model
        if isinstance(behavior_model, BehaviorMiqpAgent):
            logging.info("Plotting Information for BehaviorMiqpAgent")
            try:
                self.drawPlanningMiqp(behavior_model)
            except:
                logging.warning(
                    "Plotting Information for BehaviorMiqpAgent not possible, maybe planning failed")
        else:
            logging.info("Plotting Information not defined for type {}".format(
                type(behavior_model)))
            self.drawTrajectory(behavior_model.last_trajectory,
                                color='black', linewidth=1.0)

    def drawPlanningMiqp(self, behavior_model):
        for i in range(len(behavior_model.last_trajectories_all_cars)):
            self.drawTrajectory(behavior_model.last_trajectories_all_cars[i],
                                color='black', linewidth=1.0, marker='o', markersize=7)
            if self._draw_reference:
                self.drawTrajectory(
                    behavior_model.ref_trajectories[i], color='red', linewidth=0.9)

        if self._draw_env_polygon:
            color_rgb = (0.8902, 0.4471, 0.1333)  # TUM orange (Pantone 158)
            self.drawPolygon2d(
                behavior_model.env_poly, 'k', 0.2, color_rgb)

        if self._draw_shrinked_convex_env:
            drawShrinkedConvexEnv(self, behavior_model)

        if self._draw_front_axle:
            drawFrontAxleApproximation(self, behavior_model)

        if self._draw_collision_circles:
            drawCollisionCircles(self, behavior_model)

        if self._drawDynamicOccupancies:
            for dyno in behavior_model.last_dynamic_occupancies:
                #print(dyno.id, dyno.prediction, dyno.type)
                if not dyno.id in self.agent_color_map:
                    color_idx = len(self.agent_color_map) % self.getSizeOfColormap()
                    self.agent_color_map[dyno.id] = self.getColorFromMap(color_idx)
                color_face = self.agent_color_map[dyno.id]
                
                ax = plt.gca()
                x = dyno.prediction[:, 1]
                y = dyno.prediction[:, 2]
                if self._occupancyConstantSize is False:
                    s = (dyno.prediction[:, 0] - dyno.prediction[0, 0])*20
                else:
                    s = 200
                colors = np.repeat(np.atleast_2d(
                    color_face), x.shape[0], axis=0)
                if dyno.type == OccupancyType.PREDICTION and self._drawPrediction:
                    ax.scatter(x, y, s=s, c=colors, edgecolors='k', alpha=0.5)
                elif dyno.type == OccupancyType.RULE_NO_RIGHT_PASSING:
                    ax.scatter(x, y, s=s, c=colors, edgecolors='k', marker='X', alpha=0.5)
                elif dyno.type == OccupancyType.RULE_SAFE_DISTANCE:
                    ax.scatter(x, y, s=s, c=colors, edgecolors='k', marker='P', alpha=0.5)
                
        # viewer.drawTrajectory(
        #     behavior_model.ref_trajectories_longer_horizon[0], color='green', linewidth=0.8)
