// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
extern "C" {
#include "src/miqp_planner_c_api.h"
}
#include "src/miqp_planner_data.hpp"

TEST(c_api, construction) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  ASSERT_NE(nullptr, &planner);
  DelCMiqpPlanner(planner);
}

TEST(c_api, add_car) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  const int ref_size = 3;
  double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
  double initial_state[6] = {0, 0, 0, 1, 0.01, 0};
  double vDes = 5;
  double deltaSDes = 1;
  double timestep = 0.0;
  bool track_reference_positions = true;
  int idx = AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes,
                               deltaSDes, timestep, track_reference_positions);
  EXPECT_EQ(idx, 0);
  DelCMiqpPlanner(planner);
}

TEST(c_api, update_car) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  const int ref_size = 3;
  double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
  double initial_state[6] = {0, 0, 0, 1, 0.01, 0};
  double vDes = 5;
  double deltaSDes = 1;
  double timestep = 0.0;
  bool track_reference_positions = true;
  int idx = AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes,
                               deltaSDes, timestep, track_reference_positions);

  double initial_state2[6] = {0, 0, 0, -2, 0.01, 0};
  UpdateCarCMiqpPlanner(planner, idx, initial_state2, ref, ref_size, timestep,
                        track_reference_positions);
  DelCMiqpPlanner(planner);
}

TEST(c_api, add_and_remove_obstacle) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  int time_steps = 2;
  double p1_x[time_steps] = {0, 0};
  double p1_y[time_steps] = {0, 1};
  double p2_x[time_steps] = {2, 2};
  double p2_y[time_steps] = {0, 1};
  double p3_x[time_steps] = {2, 2};
  double p3_y[time_steps] = {4, 5};
  double p4_x[time_steps] = {0, 0};
  double p4_y[time_steps] = {4, 5};

  int id = AddObstacleCMiqpPlanner(planner, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                                   p4_x, p4_y, time_steps, false, false);
  EXPECT_EQ(id, 0);
  RemoveAllObstaclesCMiqpPlanner(planner);

  DelCMiqpPlanner(planner);
}

TEST(c_api, update_obstacle) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  int time_steps = 1;
  double p1_x[time_steps] = {0};
  double p1_y[time_steps] = {0};
  double p2_x[time_steps] = {2};
  double p2_y[time_steps] = {0};
  double p3_x[time_steps] = {2};
  double p3_y[time_steps] = {4};
  double p4_x[time_steps] = {0};
  double p4_y[time_steps] = {4};

  int id = AddObstacleCMiqpPlanner(planner, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                                   p4_x, p4_y, time_steps, false, false);

  p1_x[time_steps] = {0};
  p1_y[time_steps] = {1};
  p2_x[time_steps] = {2};
  p2_y[time_steps] = {1};
  p3_x[time_steps] = {2};
  p3_y[time_steps] = {5};
  p4_x[time_steps] = {0};
  p4_y[time_steps] = {5};
  UpdateObstacleCMiqpPlanner(planner, id, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                             p4_x, p4_y, time_steps, false);

  DelCMiqpPlanner(planner);
}

TEST(c_api, plan1) {
  CMiqpPlanner planner = NewCMiqpPlanner();

  const int ref_size = 3;
  double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
  double initial_state[6] = {0, 0, 0, 1, 0.01, 0};
  double vDes = 5;
  double deltaSDes = 1;
  double timestep = 0.0;
  bool track_reference_positions = true;
  AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes, deltaSDes,
                     timestep, track_reference_positions);

  const int N = GetNCMiqpPlanner(planner);

  double p1_x[N] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                    10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
  double p1_y[N] = {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
                    -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5};
  double p2_x[N] = {11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
                    11, 11, 11, 11, 11, 11, 11, 11, 11, 11};
  double p2_y[N] = {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5,
                    -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5};
  double p3_x[N] = {11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
                    11, 11, 11, 11, 11, 11, 11, 11, 11, 11};
  double p3_y[N] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  double p4_x[N] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                    10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
  double p4_y[N] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

  bool is_static = false;
  bool is_soft = false;
  int id = AddObstacleCMiqpPlanner(planner, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y,
                                   p4_x, p4_y, N, is_static, is_soft);

  bool succ = PlanCMiqpPlanner(planner, timestep);
  EXPECT_TRUE(succ);

  RemoveAllObstaclesCMiqpPlanner(planner);
  bool succ2 = PlanCMiqpPlanner(planner, timestep);
  EXPECT_TRUE(succ2);

  DelCMiqpPlanner(planner);
}

TEST(c_api, set_debug_paths) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  char path[] = "/tmp";
  char name[] = "test_c_api_";
  ActivateDebugFileWriteCMiqpPlanner(planner, path, name);
  DelCMiqpPlanner(planner);
}

TEST(c_api, get_n) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  int n = GetNCMiqpPlanner(planner);
  EXPECT_EQ(n, miqp::planner::ApolloDefaultSettings().nr_steps);
  DelCMiqpPlanner(planner);
}

TEST(c_api, get_raw_traj) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  const int ref_size = 3;
  double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
  double initial_state[6] = {0, 0, 0, 1, 0.01, 0};
  double vDes = 5;
  double deltaSDes = 1;
  double timestep = 0.0;
  bool track_reference_positions = true;
  int idx = AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes,
                               deltaSDes, timestep, track_reference_positions);
  PlanCMiqpPlanner(planner, timestep);

  const int N = GetNCMiqpPlanner(planner);

  double traj[TRAJECTORY_SIZE * N];
  int size;
  GetRawCMiqpTrajectoryCMiqpPlanner(planner, idx, timestep, traj, size);
  EXPECT_EQ(size, N);
  int r = size;
  int c = TRAJECTORY_SIZE;
  for (int i = 0; i < r; ++i) {
    for (int j = 0; j < c; ++j) {
      std::cout << traj[i * c + j] << "\t";
    }
    std::cout << std::endl;
  }

  EXPECT_EQ(traj[TRAJECTORY_SIZE * 0 + TRAJECTORY_TIME_IDX], 0);
  EXPECT_EQ(traj[TRAJECTORY_SIZE * 0 + TRAJECTORY_X_IDX], 0);
  EXPECT_EQ(traj[TRAJECTORY_SIZE * 1 + TRAJECTORY_TIME_IDX], 0.25);

  EXPECT_NEAR(traj[TRAJECTORY_SIZE * 1 + TRAJECTORY_X_IDX], 0.005, 1e-3);
  DelCMiqpPlanner(planner);
}

TEST(c_api, get_raw_reference) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  const int ref_size = 3;
  double ref[ref_size * 2] = {0, 0, 5, 0, 30, 0};
  double initial_state[6] = {0, 0, 0, 1, 0.01, 0};
  double vDes = 5;
  double deltaSDes = 1;
  double timestep = 0.0;
  bool track_reference_positions = true;
  int idx = AddCarCMiqpPlanner(planner, initial_state, ref, ref_size, vDes,
                               deltaSDes, timestep, track_reference_positions);
  const int N = GetNCMiqpPlanner(planner);

  double traj[TRAJECTORY_SIZE * N];
  int size;
  GetRawCLastReferenceTrajectoryCMiqpPlaner(planner, idx, timestep, traj, size);
  EXPECT_EQ(size, N);
  int r = size;
  int c = TRAJECTORY_SIZE;
  for (int i = 0; i < r; ++i) {
    for (int j = 0; j < c; ++j) {
      std::cout << traj[i * c + j] << "\t";
    }
    std::cout << std::endl;
  }

  EXPECT_EQ(traj[TRAJECTORY_SIZE * 0 + TRAJECTORY_TIME_IDX], 0);
  EXPECT_EQ(traj[TRAJECTORY_SIZE * 1 + TRAJECTORY_TIME_IDX], 0.25);

  int idx_row = N - 1;
  float vx = traj[TRAJECTORY_SIZE * idx_row + TRAJECTORY_VX_IDX];
  float vy = traj[TRAJECTORY_SIZE * idx_row + TRAJECTORY_VY_IDX];
  float vel = sqrt(std::pow(vx, 2) + std::pow(vy, 2));
  EXPECT_EQ(vel, vDes);
  DelCMiqpPlanner(planner);
}

TEST(c_api, update_map) {
  CMiqpPlanner planner = NewCMiqpPlanner();
  const int poly_size = 5;
  double poly_pts[poly_size * 2] = {0, 0, 0, 8, 4, 8, 4, 0, 0, 0};

  bool succ = UpdateConvexifiedMapCMiqpPlaner(planner, poly_pts, poly_size);
  EXPECT_TRUE(succ);
}

TEST(c_api, construction_settings) {
  MiqpPlannerSettings s = miqp::planner::ApolloDefaultSettings();
  s.nr_steps = 44;
  CMiqpPlanner planner = NewCMiqpPlannerSettings(s);
  ASSERT_NE(nullptr, &planner);
  int nr_steps = GetNCMiqpPlanner(planner);
  ASSERT_EQ(nr_steps, s.nr_steps);
  DelCMiqpPlanner(planner);
}