# Copyright (c) 2021 fortiss GmbH
# 
# Authors: Klemens Esterle and Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import pickle
from random import random

from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.commons.xodr_parser import XodrParser

from bark.core.geometry import Polygon2d, Line2d, Point2d, SmoothLine, GetLineFromSInterval
from bark.core.world.map import MapInterface
from bark.core.world import World
from bark.core.world.opendrive import XodrDrivingDirection, MakeXodrMapCurved

from miqp import ConvexifiedMap


def plot_polygon_vector(polygons, viewer):
    for idx, poly in enumerate(polygons.values()):
        viewer.drawPolygon2d(
            poly, 'k', 0.25, (random(), random(), random()), 2)
        points = poly.ToArray()
        # viewer.axes.text(np.mean(points[:, 0]),
        #                  np.mean(points[:, 1]), str(idx))


def CreateReferenceTrajectory(lc, relative_length):
    s_start = 0
    s_end = relative_length*lc.center_line.Length()
    ref_line = GetLineFromSInterval(lc.center_line, s_start, s_end).ToArray()
    ref_traj = np.zeros((len(ref_line), 5))
    ref_traj[:, 1] = ref_line[:, 0]
    ref_traj[:, 2] = ref_line[:, 1]
    return ref_traj


def TrajToBarkLine(traj):
    bark_line_obj = Line2d()
    for row in traj:
        bark_line_obj.AddPoint(Point2d(row[1], row[2]))
    return bark_line_obj


class ConvexifiedMapTest(unittest.TestCase):
    # @unittest.skip
    def test_convexified_map(self):
        poly_in = Polygon2d()
        poly_in.AddPoint([0, 0])
        poly_in.AddPoint([0, 2])
        poly_in.AddPoint([1, 3])
        poly_in.AddPoint([4, 2])
        poly_in.AddPoint([4, 0])
        poly_in.AddPoint([3, 1])
        poly_in.AddPoint([0, 0])

        viewer = MPViewer()
        viewer.drawPolygon2d(poly_in, 'r', 1, (1, 1, 1))

        params = ParameterServer()
        conv_map = ConvexifiedMap(params, poly_in, 0.1, 1e-6, 1.0, 0.1)
        succ = conv_map.Convert()
        self.assertTrue(succ)

        # print(conv_map.map_convex_polygons)
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)
        viewer.show(block=False)

    # @unittest.skip
    def test_convexified_map2(self):
        poly_in = Polygon2d()
        poly_in.AddPoint([-100, -50])
        poly_in.AddPoint([-100, 50])
        poly_in.AddPoint([0, 20])
        poly_in.AddPoint([50, 50])
        poly_in.AddPoint([50, -50])
        poly_in.AddPoint([-100, -50])

        viewer = MPViewer()
        viewer.drawPolygon2d(poly_in, 'r', 1, (1, 1, 1))

        params = ParameterServer()
        conv_map = ConvexifiedMap(params, poly_in, 1.0, 0.2, 1.0, 0.1)
        succ = conv_map.Convert()
        self.assertTrue(succ)
        # print(conv_map.map_convex_polygons)

        plot_polygon_vector(conv_map.map_convex_polygons, viewer)
        viewer.show(block=False)

    # @unittest.skip
    def test_xodr_map_deu_merging_road_corridor(self):
        xodr_parser = XodrParser(
            "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        # creating road corridor
        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        conv_map = ConvexifiedMap(
            params, road_corr.polygon, 1.0, 0.2, 1.0, 0.1)
        self.assertTrue(conv_map.Convert())
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)

        viewer.show(block=False)

    # @unittest.skip
    def test_xodr_map_deu_merging_seperate_lane_corridors(self):
        xodr_parser = XodrParser(
            "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        # creating road corridor
        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        # lane corridors are convexified seperately

        conv_map = ConvexifiedMap(
            params, road_corr.lane_corridors[0].polygon, 1.0, 0.2, 1.0, 0.1)
        self.assertTrue(conv_map.Convert())
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)

        conv_map = ConvexifiedMap(
            params, road_corr.lane_corridors[1].polygon, 1.0, 0.2, 1.0, 0.1)
        self.assertTrue(conv_map.Convert())
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)

        viewer.show(block=False)

    # @unittest.skip
    def test_xodr_map_deu_merging_combined_lane_corridors(self):
        xodr_parser = XodrParser(
            "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        # creating road corridor
        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        # concatenating lane corridor polygons
        poly_road = road_corr.lane_corridors[0].polygon
        poly_road.ConcatenatePolygons(road_corr.lane_corridors[1].polygon)

        conv_map = ConvexifiedMap(params, poly_road, 1.0, 0.2, 1.0, 0.1)
        suc = conv_map.Convert()
        self.assertTrue(suc)
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)
        # print(conv_map.map_convex_polygons)

        viewer.show(block=False)

    # @unittest.skip
    def test_xodr_map_deu_merging_combined_lane_corridors_reference_intersecting(self):
        xodr_parser = XodrParser(
            "common/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        center_line = road_corr.lane_corridors[0].center_line.ToArray()
        ref_line = np.delete(center_line, [3, 4], 0)  # remove last two rows
        ref_traj = np.zeros((len(ref_line), 5))
        ref_traj[:, 1] = ref_line[:, 0]
        ref_traj[:, 2] = ref_line[:, 1]

        bark_line_obj = Line2d()
        for row in ref_traj:
            bark_line_obj.AddPoint(Point2d(row[1], row[2]))

        poly_road = road_corr.lane_corridors[0].polygon
        poly_road.ConcatenatePolygons(road_corr.lane_corridors[1].polygon)

        conv_map = ConvexifiedMap(params, poly_road, 1.0, 0.2, 1.0, 0.1)

        self.assertTrue(conv_map.Convert())
        convex_polygons = conv_map.GetIntersectingConvexPolygons(ref_traj)

        plot_polygon_vector(convex_polygons, viewer)
        viewer.drawLine2d(bark_line_obj)

        viewer.show(block=False)

    # @unittest.skip
    def test_xodr_map_curved(self):
        xodr_map = MakeXodrMapCurved(80, 0.07)

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        roads = [100]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        poly_road = road_corr.lane_corridors[0].polygon

        conv_map = ConvexifiedMap(params, poly_road, 1.0, 0.2, 1.0, 0.1)

        self.assertTrue(conv_map.Convert())
        plot_polygon_vector(conv_map.map_convex_polygons, viewer)
        # print(conv_map.map_convex_polygons)

        viewer.show(block=False)

    # @unittest.skip
    def test_guerickestrasse(self):
        poly_in = Polygon2d()
        poly_in.AddPoint([926.315, 73.977])
        poly_in.AddPoint([927.29, 73.4847])
        poly_in.AddPoint([928.178, 73.0103])
        poly_in.AddPoint([929.05, 72.5422])
        poly_in.AddPoint([929.955, 72.0585])
        poly_in.AddPoint([930.811, 71.5785])
        poly_in.AddPoint([931.687, 71.1045])
        poly_in.AddPoint([932.582, 70.624])
        poly_in.AddPoint([933.503, 70.1086])
        poly_in.AddPoint([934.322, 69.65])
        poly_in.AddPoint([935.208, 69.1662])
        poly_in.AddPoint([936.207, 68.6089])
        poly_in.AddPoint([936.93, 68.1848])
        poly_in.AddPoint([937.819, 67.6949])
        poly_in.AddPoint([938.681, 67.2168])
        poly_in.AddPoint([939.596, 66.7177])
        poly_in.AddPoint([940.436, 66.2428])
        poly_in.AddPoint([941.305, 65.7653])
        poly_in.AddPoint([942.189, 65.278])
        poly_in.AddPoint([943.054, 64.8003])
        poly_in.AddPoint([943.939, 64.3168])
        poly_in.AddPoint([944.806, 63.8407])
        poly_in.AddPoint([945.753, 63.3226])
        poly_in.AddPoint([946.551, 62.8702])
        poly_in.AddPoint([947.428, 62.3911])
        poly_in.AddPoint([948.304, 61.9124])
        poly_in.AddPoint([949.245, 61.3971])
        poly_in.AddPoint([950.048, 60.943])
        poly_in.AddPoint([950.921, 60.4634])
        poly_in.AddPoint([951.795, 59.9839])
        poly_in.AddPoint([952.671, 59.4949])
        poly_in.AddPoint([953.548, 59.0132])
        poly_in.AddPoint([954.422, 58.5339])
        poly_in.AddPoint([955.299, 58.0518])
        poly_in.AddPoint([956.18, 57.5831])
        poly_in.AddPoint([957.063, 57.0983])
        poly_in.AddPoint([957.937, 56.6147])
        poly_in.AddPoint([958.752, 56.163])
        poly_in.AddPoint([959.703, 55.6533])
        poly_in.AddPoint([960.561, 55.1733])
        poly_in.AddPoint([961.441, 54.6906])
        poly_in.AddPoint([962.327, 54.1943])
        poly_in.AddPoint([963.172, 53.7455])
        poly_in.AddPoint([964.079, 53.2496])
        poly_in.AddPoint([964.94, 52.765])
        poly_in.AddPoint([965.81, 52.284])
        poly_in.AddPoint([966.609, 51.8379])
        poly_in.AddPoint([967.557, 51.3338])
        poly_in.AddPoint([968.452, 50.8409])
        poly_in.AddPoint([969.313, 50.3585])
        poly_in.AddPoint([970.107, 49.918])
        poly_in.AddPoint([971.076, 49.4041])
        poly_in.AddPoint([971.938, 48.9249])
        poly_in.AddPoint([972.817, 48.439])
        poly_in.AddPoint([973.691, 47.9555])
        poly_in.AddPoint([974.567, 47.4623])
        poly_in.AddPoint([975.431, 46.9822])
        poly_in.AddPoint([976.313, 46.4977])
        poly_in.AddPoint([977.185, 46.0142])
        poly_in.AddPoint([978.059, 45.546])
        poly_in.AddPoint([978.938, 45.0637])
        poly_in.AddPoint([979.813, 44.5832])
        poly_in.AddPoint([980.689, 44.1019])
        poly_in.AddPoint([981.575, 43.6165])
        poly_in.AddPoint([982.444, 43.1381])
        poly_in.AddPoint([983.314, 42.6612])
        poly_in.AddPoint([984.189, 42.1838])
        poly_in.AddPoint([985.07, 41.7035])
        poly_in.AddPoint([985.887, 41.2573])
        poly_in.AddPoint([986.824, 40.7625])
        poly_in.AddPoint([987.698, 40.288])
        poly_in.AddPoint([988.584, 39.8087])
        poly_in.AddPoint([989.459, 39.3324])
        poly_in.AddPoint([990.338, 38.8581])
        poly_in.AddPoint([991.221, 38.3785])
        poly_in.AddPoint([992.106, 37.8958])
        poly_in.AddPoint([992.979, 37.4177])
        poly_in.AddPoint([993.861, 36.9363])
        poly_in.AddPoint([994.739, 36.4576])
        poly_in.AddPoint([995.609, 35.9814])
        poly_in.AddPoint([996.488, 35.5026])
        poly_in.AddPoint([997.373, 35.0191])
        poly_in.AddPoint([998.247, 34.5555])
        poly_in.AddPoint([999.128, 34.0757])
        poly_in.AddPoint([1000, 33.5959])
        poly_in.AddPoint([1000.88, 33.1159])
        poly_in.AddPoint([1001.77, 32.6324])
        poly_in.AddPoint([1002.56, 32.1926])
        poly_in.AddPoint([1003.52, 31.6913])
        poly_in.AddPoint([1004.41, 31.2016])
        poly_in.AddPoint([1005.28, 30.7202])
        poly_in.AddPoint([1006.16, 30.2336])
        poly_in.AddPoint([1007.03, 29.7445])
        poly_in.AddPoint([1007.9, 29.2648])
        poly_in.AddPoint([1008.78, 28.7789])
        poly_in.AddPoint([1009.66, 28.2893])
        poly_in.AddPoint([1010.52, 27.8057])
        poly_in.AddPoint([1011.41, 27.3284])
        poly_in.AddPoint([1012.28, 26.8379])
        poly_in.AddPoint([1013.15, 26.3519])
        poly_in.AddPoint([1013.15, 26.3519])
        poly_in.AddPoint([1014.03, 25.862])
        poly_in.AddPoint([1014.91, 25.3661])
        poly_in.AddPoint([1015.72, 24.9055])
        poly_in.AddPoint([1016.65, 24.3996])
        poly_in.AddPoint([1017.52, 23.9048])
        poly_in.AddPoint([1018.39, 23.4154])
        poly_in.AddPoint([1019.26, 22.9199])
        poly_in.AddPoint([1020.13, 22.4412])
        poly_in.AddPoint([1021, 21.9488])
        poly_in.AddPoint([1021.88, 21.4543])
        poly_in.AddPoint([1022.74, 20.9706])
        poly_in.AddPoint([1023.62, 20.4756])
        poly_in.AddPoint([1024.48, 20.0085])
        poly_in.AddPoint([1025.35, 19.5223])
        poly_in.AddPoint([1026.23, 19.029])
        poly_in.AddPoint([1027.08, 18.5481])
        poly_in.AddPoint([1027.98, 18.049])
        poly_in.AddPoint([1028.82, 17.5689])
        poly_in.AddPoint([1029.7, 17.0845])
        poly_in.AddPoint([1030.59, 16.5877])
        poly_in.AddPoint([1031.42, 16.1182])
        poly_in.AddPoint([1032.33, 15.626])
        poly_in.AddPoint([1033.14, 15.1775])
        poly_in.AddPoint([1034.03, 14.7177])
        poly_in.AddPoint([1034.88, 14.2673])
        poly_in.AddPoint([1035.7, 13.8637])
        poly_in.AddPoint([1036.63, 13.4228])
        poly_in.AddPoint([1037.5, 13.0125])
        poly_in.AddPoint([1038.37, 12.6171])
        poly_in.AddPoint([1039.23, 12.256])
        poly_in.AddPoint([1040.09, 11.9145])
        poly_in.AddPoint([1040.94, 11.6047])
        poly_in.AddPoint([1041.79, 11.3323])
        poly_in.AddPoint([1042.64, 11.1093])
        poly_in.AddPoint([1043.49, 10.9238])
        poly_in.AddPoint([1044.33, 10.7861])
        poly_in.AddPoint([1045.14, 10.6974])
        poly_in.AddPoint([1046.04, 10.6622])
        poly_in.AddPoint([1046.89, 10.6723])
        poly_in.AddPoint([1047.66, 10.7213])
        poly_in.AddPoint([1048.56, 10.8554])
        poly_in.AddPoint([1049.41, 11.0202])
        poly_in.AddPoint([1050.24, 11.2283])
        poly_in.AddPoint([1051.05, 11.5043])
        poly_in.AddPoint([1051.85, 11.8055])
        poly_in.AddPoint([1052.66, 12.161])
        poly_in.AddPoint([1053.42, 12.5774])
        poly_in.AddPoint([1054.17, 12.9967])
        poly_in.AddPoint([1054.91, 13.4615])
        poly_in.AddPoint([1055.62, 13.9898])
        poly_in.AddPoint([1056.31, 14.5221])
        poly_in.AddPoint([1057.01, 15.1134])
        poly_in.AddPoint([1057.56, 15.6072])
        poly_in.AddPoint([1058.29, 16.3813])
        poly_in.AddPoint([1058.92, 17.047])
        poly_in.AddPoint([1059.45, 17.7325])
        poly_in.AddPoint([1060.02, 18.4465])
        poly_in.AddPoint([1060.55, 19.1557])
        poly_in.AddPoint([1061.08, 19.941])
        poly_in.AddPoint([1061.61, 20.7366])
        poly_in.AddPoint([1062.13, 21.5667])
        poly_in.AddPoint([1062.67, 22.4284])
        poly_in.AddPoint([1063.2, 23.2601])
        poly_in.AddPoint([1063.73, 24.1153])
        poly_in.AddPoint([1064.25, 24.9359])
        poly_in.AddPoint([1064.79, 25.802])
        poly_in.AddPoint([1065.31, 26.6387])
        poly_in.AddPoint([1065.83, 27.4921])
        poly_in.AddPoint([1066.36, 28.3572])
        poly_in.AddPoint([1066.88, 29.179])
        poly_in.AddPoint([1067.4, 30.0456])
        poly_in.AddPoint([1067.93, 30.8905])
        poly_in.AddPoint([1068.44, 31.7377])
        poly_in.AddPoint([1068.97, 32.6001])
        poly_in.AddPoint([1069.5, 33.4623])
        poly_in.AddPoint([1070, 34.2635])
        poly_in.AddPoint([1070.54, 35.1597])
        poly_in.AddPoint([1071.08, 36.0225])
        poly_in.AddPoint([1071.61, 36.8722])
        poly_in.AddPoint([1072.13, 37.6869])
        poly_in.AddPoint([1072.68, 38.57])
        poly_in.AddPoint([1073.22, 39.4215])
        poly_in.AddPoint([1073.76, 40.2717])
        poly_in.AddPoint([1074.29, 41.1079])
        poly_in.AddPoint([1074.81, 41.9082])
        poly_in.AddPoint([1075.37, 42.8025])
        poly_in.AddPoint([1075.91, 43.6388])
        poly_in.AddPoint([1076.45, 44.4871])
        poly_in.AddPoint([1076.99, 45.3196])
        poly_in.AddPoint([1077.54, 46.1688])
        poly_in.AddPoint([1078.07, 47.0062])
        poly_in.AddPoint([1078.62, 47.8561])
        poly_in.AddPoint([1079.17, 48.6934])
        poly_in.AddPoint([1079.72, 49.5349])
        poly_in.AddPoint([1080.26, 50.3779])
        poly_in.AddPoint([1080.81, 51.2201])
        poly_in.AddPoint([1081.37, 52.055])
        poly_in.AddPoint([1081.94, 52.8991])
        poly_in.AddPoint([1082.51, 53.7394])
        poly_in.AddPoint([1083.04, 54.5166])
        poly_in.AddPoint([1087.23, 51.7791])
        poly_in.AddPoint([1086.63, 50.9087])
        poly_in.AddPoint([1086.08, 50.0953])
        poly_in.AddPoint([1085.53, 49.2778])
        poly_in.AddPoint([1084.98, 48.4483])
        poly_in.AddPoint([1084.43, 47.6226])
        poly_in.AddPoint([1083.9, 46.7906])
        poly_in.AddPoint([1083.35, 45.9581])
        poly_in.AddPoint([1082.81, 45.1217])
        poly_in.AddPoint([1082.27, 44.2921])
        poly_in.AddPoint([1081.73, 43.4474])
        poly_in.AddPoint([1081.2, 42.6156])
        poly_in.AddPoint([1080.65, 41.7682])
        poly_in.AddPoint([1080.11, 40.9353])
        poly_in.AddPoint([1079.57, 40.0901])
        poly_in.AddPoint([1079.07, 39.3019])
        poly_in.AddPoint([1078.51, 38.417])
        poly_in.AddPoint([1077.97, 37.5699])
        poly_in.AddPoint([1077.43, 36.7342])
        poly_in.AddPoint([1076.9, 35.8964])
        poly_in.AddPoint([1076.4, 35.0876])
        poly_in.AddPoint([1075.85, 34.2098])
        poly_in.AddPoint([1075.32, 33.3655])
        poly_in.AddPoint([1074.8, 32.5302])
        poly_in.AddPoint([1074.31, 31.7179])
        poly_in.AddPoint([1073.75, 30.8199])
        poly_in.AddPoint([1073.23, 29.9809])
        poly_in.AddPoint([1072.71, 29.1372])
        poly_in.AddPoint([1072.19, 28.2807])
        poly_in.AddPoint([1071.66, 27.4253])
        poly_in.AddPoint([1071.15, 26.5831])
        poly_in.AddPoint([1070.6, 25.7074])
        poly_in.AddPoint([1070.09, 24.8673])
        poly_in.AddPoint([1069.57, 24.017])
        poly_in.AddPoint([1069.03, 23.1557])
        poly_in.AddPoint([1068.51, 22.3208])
        poly_in.AddPoint([1067.96, 21.4499])
        poly_in.AddPoint([1067.44, 20.611])
        poly_in.AddPoint([1066.9, 19.7524])
        poly_in.AddPoint([1066.37, 18.9231])
        poly_in.AddPoint([1065.83, 18.057])
        poly_in.AddPoint([1065.25, 17.1837])
        poly_in.AddPoint([1064.67, 16.3147])
        poly_in.AddPoint([1064, 15.4193])
        poly_in.AddPoint([1063.32, 14.5715])
        poly_in.AddPoint([1062.65, 13.7155])
        poly_in.AddPoint([1061.9, 12.9244])
        poly_in.AddPoint([1061.27, 12.257])
        poly_in.AddPoint([1060.35, 11.3985])
        poly_in.AddPoint([1059.52, 10.6925])
        poly_in.AddPoint([1058.63, 9.99678])
        poly_in.AddPoint([1057.75, 9.34293])
        poly_in.AddPoint([1056.8, 8.73989])
        poly_in.AddPoint([1055.8, 8.18175])
        poly_in.AddPoint([1054.83, 7.65577])
        poly_in.AddPoint([1053.81, 7.20525])
        poly_in.AddPoint([1052.74, 6.80156])
        poly_in.AddPoint([1051.68, 6.44035])
        poly_in.AddPoint([1050.57, 6.15697])
        poly_in.AddPoint([1049.46, 5.9359])
        poly_in.AddPoint([1048.41, 5.7777])
        poly_in.AddPoint([1047.18, 5.68094])
        poly_in.AddPoint([1046.03, 5.6622])
        poly_in.AddPoint([1044.93, 5.70188])
        poly_in.AddPoint([1043.75, 5.82027])
        poly_in.AddPoint([1042.63, 5.99812])
        poly_in.AddPoint([1041.51, 6.23856])
        poly_in.AddPoint([1040.44, 6.51993])
        poly_in.AddPoint([1039.38, 6.85645])
        poly_in.AddPoint([1038.35, 7.22732])
        poly_in.AddPoint([1037.35, 7.62395])
        poly_in.AddPoint([1036.37, 8.03703])
        poly_in.AddPoint([1035.42, 8.46688])
        poly_in.AddPoint([1034.48, 8.90769])
        poly_in.AddPoint([1033.61, 9.32354])
        poly_in.AddPoint([1032.63, 9.80443])
        poly_in.AddPoint([1031.71, 10.2845])
        poly_in.AddPoint([1030.83, 10.7469])
        poly_in.AddPoint([1029.89, 11.2624])
        poly_in.AddPoint([1029.03, 11.7262])
        poly_in.AddPoint([1028.13, 12.2342])
        poly_in.AddPoint([1027.27, 12.7126])
        poly_in.AddPoint([1026.4, 13.1984])
        poly_in.AddPoint([1025.51, 13.7009])
        poly_in.AddPoint([1024.66, 14.1746])
        poly_in.AddPoint([1023.77, 14.6729])
        poly_in.AddPoint([1022.91, 15.159])
        poly_in.AddPoint([1022.03, 15.6492])
        poly_in.AddPoint([1021.16, 16.1232])
        poly_in.AddPoint([1020.3, 16.6052])
        poly_in.AddPoint([1019.41, 17.1042])
        poly_in.AddPoint([1018.55, 17.5926])
        poly_in.AddPoint([1017.68, 18.0804])
        poly_in.AddPoint([1016.8, 18.5716])
        poly_in.AddPoint([1015.93, 19.061])
        poly_in.AddPoint([1015.06, 19.5538])
        poly_in.AddPoint([1014.19, 20.0445])
        poly_in.AddPoint([1013.37, 20.492])
        poly_in.AddPoint([1012.44, 21.0167])
        poly_in.AddPoint([1011.58, 21.5009])
        poly_in.AddPoint([1010.71, 21.9854])
        poly_in.AddPoint([1010.71, 21.9854])
        poly_in.AddPoint([1009.83, 22.4775])
        poly_in.AddPoint([1008.97, 22.9646])
        poly_in.AddPoint([1008.1, 23.4322])
        poly_in.AddPoint([1007.22, 23.9248])
        poly_in.AddPoint([1006.35, 24.4095])
        poly_in.AddPoint([1005.48, 24.8929])
        poly_in.AddPoint([1004.6, 25.3782])
        poly_in.AddPoint([1003.73, 25.8631])
        poly_in.AddPoint([1002.86, 26.3427])
        poly_in.AddPoint([1001.98, 26.8311])
        poly_in.AddPoint([1001.12, 27.3054])
        poly_in.AddPoint([1000.31, 27.7284])
        poly_in.AddPoint([999.355, 28.2522])
        poly_in.AddPoint([998.484, 28.7295])
        poly_in.AddPoint([997.607, 29.2081])
        poly_in.AddPoint([996.73, 29.6881])
        poly_in.AddPoint([995.854, 30.1652])
        poly_in.AddPoint([994.965, 30.6368])
        poly_in.AddPoint([994.095, 31.1122])
        poly_in.AddPoint([993.218, 31.59])
        poly_in.AddPoint([992.333, 32.0745])
        poly_in.AddPoint([991.456, 32.5525])
        poly_in.AddPoint([990.582, 33.0295])
        poly_in.AddPoint([989.701, 33.5123])
        poly_in.AddPoint([988.83, 33.9873])
        poly_in.AddPoint([987.956, 34.4621])
        poly_in.AddPoint([987.075, 34.9376])
        poly_in.AddPoint([986.193, 35.4173])
        poly_in.AddPoint([985.32, 35.89])
        poly_in.AddPoint([984.436, 36.3698])
        poly_in.AddPoint([983.613, 36.8042])
        poly_in.AddPoint([982.674, 37.3151])
        poly_in.AddPoint([981.799, 37.792])
        poly_in.AddPoint([980.918, 38.2723])
        poly_in.AddPoint([980.034, 38.7572])
        poly_in.AddPoint([979.153, 39.2423])
        poly_in.AddPoint([978.284, 39.7184])
        poly_in.AddPoint([977.408, 40.2])
        poly_in.AddPoint([976.53, 40.6821])
        poly_in.AddPoint([975.655, 41.1617])
        poly_in.AddPoint([974.769, 41.6368])
        poly_in.AddPoint([973.892, 42.1227])
        poly_in.AddPoint([973.021, 42.6017])
        poly_in.AddPoint([972.137, 43.0928])
        poly_in.AddPoint([971.273, 43.5793])
        poly_in.AddPoint([970.397, 44.0638])
        poly_in.AddPoint([969.526, 44.5455])
        poly_in.AddPoint([968.64, 45.0378])
        poly_in.AddPoint([967.848, 45.4572])
        poly_in.AddPoint([966.891, 45.984])
        poly_in.AddPoint([966.008, 46.4793])
        poly_in.AddPoint([965.151, 46.9509])
        poly_in.AddPoint([964.34, 47.3821])
        poly_in.AddPoint([963.391, 47.9083])
        poly_in.AddPoint([962.511, 48.3945])
        poly_in.AddPoint([961.629, 48.8906])
        poly_in.AddPoint([960.781, 49.3546])
        poly_in.AddPoint([959.877, 49.8356])
        poly_in.AddPoint([959.018, 50.3168])
        poly_in.AddPoint([958.144, 50.7964])
        poly_in.AddPoint([957.257, 51.2922])
        poly_in.AddPoint([956.455, 51.722])
        poly_in.AddPoint([955.519, 52.2385])
        poly_in.AddPoint([954.643, 52.7226])
        poly_in.AddPoint([953.773, 53.2003])
        poly_in.AddPoint([952.895, 53.6677])
        poly_in.AddPoint([952.02, 54.1486])
        poly_in.AddPoint([951.14, 54.6316])
        poly_in.AddPoint([950.264, 55.1125])
        poly_in.AddPoint([949.397, 55.5964])
        poly_in.AddPoint([948.518, 56.0791])
        poly_in.AddPoint([947.638, 56.5626])
        poly_in.AddPoint([946.723, 57.0796])
        poly_in.AddPoint([945.909, 57.5236])
        poly_in.AddPoint([945.03, 58.0042])
        poly_in.AddPoint([944.152, 58.4837])
        poly_in.AddPoint([943.221, 59.011])
        poly_in.AddPoint([942.413, 59.4507])
        poly_in.AddPoint([941.527, 59.9369])
        poly_in.AddPoint([940.656, 60.4127])
        poly_in.AddPoint([939.771, 60.9015])
        poly_in.AddPoint([938.903, 61.3796])
        poly_in.AddPoint([938.019, 61.8658])
        poly_in.AddPoint([937.119, 62.3741])
        poly_in.AddPoint([936.278, 62.832])
        poly_in.AddPoint([935.391, 63.3241])
        poly_in.AddPoint([934.528, 63.7999])
        poly_in.AddPoint([933.544, 64.3775])
        poly_in.AddPoint([932.793, 64.7884])
        poly_in.AddPoint([931.924, 65.2625])
        poly_in.AddPoint([931.007, 65.7765])
        poly_in.AddPoint([930.189, 66.2339])
        poly_in.AddPoint([929.322, 66.6989])
        poly_in.AddPoint([928.439, 67.1768])
        poly_in.AddPoint([927.565, 67.6663])
        poly_in.AddPoint([926.706, 68.1253])
        poly_in.AddPoint([925.816, 68.6033])
        poly_in.AddPoint([924.941, 69.071])
        poly_in.AddPoint([924.151, 69.4695])
        poly_in.AddPoint([926.315, 73.977])

        viewer = MPViewer()
        viewer.drawPolygon2d(poly_in, 'k', 1, (1, 1, 1), linewidth=2)
        poly_buf = poly_in.BufferPolygon(-1.0)
        viewer.drawPolygon2d(poly_buf, 'r', 1, (1, 1, 1))

        params = ParameterServer()
        simplify_dist = 0.1  # 0.1 / 0.5 / 0.2 / 0.1
        buffer_for_merging_tolerance = 0.1  # 0.5 / 0.1 / 0.4 / 0.1
        conv_map = ConvexifiedMap(
            params, poly_in, 1.0, simplify_dist, 1.0, buffer_for_merging_tolerance)
        succ = conv_map.Convert()

        # print(conv_map.map_convex_polygons)

        plot_polygon_vector(conv_map.map_convex_polygons, viewer)
        viewer.axes.set_xlim([1000, 1080])
        viewer.axes.set_ylim([0, 50])
        viewer.axes.set_aspect('equal', adjustable='box')
        viewer.axes.set_xlabel('x [m]')
        viewer.axes.set_ylabel('y [m]')
        viewer.show(block=False)
        fn = "/tmp/" + "convexification_guerickestr_"+str(simplify_dist) + "_"+str(buffer_for_merging_tolerance)
        viewer.axes.get_figure().savefig(fn+".pgf", bbox_inches='tight')
        viewer.axes.get_figure().savefig(fn+".png", bbox_inches='tight')
        self.assertTrue(succ)


if __name__ == '__main__':
    unittest.main()
