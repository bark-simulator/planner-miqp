// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/miqp_planner_c_api.h"
#include "bark/commons/timer/timer.hpp"
#include "src/miqp_planner.hpp"
#include "src/miqp_planner_data.hpp"

using bark::models::dynamic::StateDefinition;
using bark::models::dynamic::Trajectory;
using miqp::planner::ApolloDefaultSettings;
using miqp::planner::DefaultSettings;
using miqp::planner::MiqpPlanner;
using miqp::planner::RawResults;

CMiqpPlanner NewCMiqpPlanner() {
#if PLANNER_MIQP_CAPI_NO_APOLLO
  MiqpPlannerSettings s = DefaultSettings();
#else
  MiqpPlannerSettings s = ApolloDefaultSettings();
#endif
  return reinterpret_cast<void*>(new MiqpPlanner(s));
}

CMiqpPlanner NewCMiqpPlannerSettings(MiqpPlannerSettings settings) {
  return reinterpret_cast<void*>(new MiqpPlanner(settings));
}

void DelCMiqpPlanner(CMiqpPlanner c_miqp_planner) {
  delete reinterpret_cast<MiqpPlanner*>(c_miqp_planner);
}

int AddCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, double initial_state_in[],
                       double ref_in[], const int ref_size, double vDes,
                       double deltaSDes, const double timestep,
                       const bool track_reference_positions) {
  bark::geometry::Line simp_ref;
  const double simpDistRef = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
                                 ->GetSettings()
                                 .simplificationDistanceReferenceLine;
  miqp::common::geometry::ConvertToBarkLine(ref_in, ref_size, simpDistRef,
                                            simp_ref);
  Eigen::MatrixXd initial_state =
      Eigen::Map<Eigen::Matrix<double, 1, 6> >(initial_state_in);
  return reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
      ->AddCar(initial_state, simp_ref, vDes, deltaSDes, timestep,
               track_reference_positions);
}

bool PlanCMiqpPlanner(CMiqpPlanner c_miqp_planner, const double timestep) {
  return reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->Plan(timestep);
}

void UpdateCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                           double initial_state_in[], double ref_in[],
                           const int ref_size, const double timestep,
                           bool track_reference_positions) {
  auto timer = bark::commons::timer::Timer();
  timer.Start();
  bark::geometry::Line simp_ref;
  const double simpDistRef = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
                                 ->GetSettings()
                                 .simplificationDistanceReferenceLine;
  miqp::common::geometry::ConvertToBarkLine(ref_in, ref_size, simpDistRef,
                                            simp_ref);

  double duration = timer.DurationInSeconds();
  LOG(INFO) << "parse reference in UpdateCarCMiqpPlanner took Time [s] ="
            << duration;
  Eigen::MatrixXd initial_state =
      Eigen::Map<Eigen::Matrix<double, 1, 6> >(initial_state_in);
  reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
      ->UpdateCar(idx, initial_state, simp_ref, timestep,
                  track_reference_positions);
}

void ActivateDebugFileWriteCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                        char path[], char name[]) {
  std::string path_str(path);
  std::string name_str(name);
  reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
      ->ActivateDebugFileWrite(path_str, name_str);
}

int GetNCMiqpPlanner(CMiqpPlanner c_miqp_planner) {
  return reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetN();
}

float GetTsCMiqpPlanner(CMiqpPlanner c_miqp_planner) {
  return reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetTs();
}

float GetCollisionRadius(CMiqpPlanner c_miqp_planner) {
  return reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetCollisionRadius();
}

void GetRawCMiqpTrajectoryCMiqpPlanner(CMiqpPlanner c_miqp_planner, int carIdx,
                                       double start_time, double* trajectory,
                                       int& size) {
  const std::shared_ptr<RawResults> rawResults =
      reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetSolution();
  assert(carIdx < rawResults->NrCars);
  const int N = rawResults->N;
  const float dt =
      reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetParameters()->ts;

  size = N;

  double time = start_time;
  for (int i = 0; i < N; ++i) {
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_TIME_IDX] = time;
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_X_IDX] =
        rawResults->pos_x(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_Y_IDX] =
        rawResults->pos_y(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_VX_IDX] =
        rawResults->vel_x(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_VY_IDX] =
        rawResults->vel_y(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_AX_IDX] =
        rawResults->acc_x(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_AY_IDX] =
        rawResults->acc_y(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_UX_IDX] =
        rawResults->u_x(carIdx, i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_UY_IDX] =
        rawResults->u_y(carIdx, i);
    time += dt;
  }
}

void GetRawCLastReferenceTrajectoryCMiqpPlaner(CMiqpPlanner c_miqp_planner,
                                               int carIdx, double start_time,
                                               double* trajectory, int& size) {
  bark::models::dynamic::Trajectory ref_traj =
      reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetLastReference(carIdx);
  const float dt = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetTs();
  const int N = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->GetN();

  size = N;

  double time = start_time;
  for (int i = 0; i < N; ++i) {
    double theta_i = ref_traj(i, StateDefinition::THETA_POSITION);
    double vel_i = ref_traj(i, StateDefinition::VEL_POSITION);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_TIME_IDX] = time;
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_X_IDX] =
        ref_traj(i, StateDefinition::X_POSITION);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_Y_IDX] =
        ref_traj(i, StateDefinition::Y_POSITION);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_VX_IDX] = vel_i * cos(theta_i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_VY_IDX] = vel_i * sin(theta_i);
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_AX_IDX] = 0;  // DUMMY VALUE
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_AY_IDX] = 0;  // DUMMY VALUE
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_UX_IDX] = 0;  // DUMMY VALUE
    trajectory[i * TRAJECTORY_SIZE + TRAJECTORY_UY_IDX] = 0;  // DUMMY VALUE
    time += dt;
  }
}

bool UpdateConvexifiedMapCMiqpPlaner(CMiqpPlanner c_miqp_planner,
                                     double poly_pts[], const int poly_size) {
  bark::geometry::Line simplf_poly_line;
  const double simpDistMap = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
                                 ->GetSettings()
                                 .simplificationDistanceMap;
  miqp::common::geometry::ConvertToBarkLine(poly_pts, poly_size, simpDistMap,
                                            simplf_poly_line);

  bark::geometry::Polygon mapPolygon;
  for (auto const& p : simplf_poly_line) {
    mapPolygon.AddPoint(p);
  }
  mapPolygon.AddPoint(
      *(simplf_poly_line.begin()));  // close polygon with first point

  boost::geometry::correct(mapPolygon.obj_);

  bool succ = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
                  ->UpdateConvexifiedMap(mapPolygon);
  return succ;
}

void UpdateDesiredVelocityCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                       const int carIdx, const double vDes,
                                       const double deltaSDes) {
  reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
      ->UpdateDesiredVelocity(carIdx, vDes, deltaSDes);
}

int AddObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, double p1_x[],
                            double p1_y[], double p2_x[], double p2_y[],
                            double p3_x[], double p3_y[], double p4_x[],
                            double p4_y[], const int size, bool is_static,
                            bool is_soft) {
  if (is_static) {
    // try to merge it with an existing obstacle ... merge them if (buffered
    // polygons) are intersecting
  }

  std::vector<Eigen::MatrixXd> dynamic_obstacle;

  for (int i = 0; i < size; ++i) {
    Eigen::MatrixXd occupancy(4, 2);  // fill clockwise

    occupancy(0, 0) = p1_x[i];
    occupancy(0, 1) = p1_y[i];

    occupancy(1, 0) = p2_x[i];
    occupancy(1, 1) = p2_y[i];

    occupancy(2, 0) = p3_x[i];
    occupancy(2, 1) = p3_y[i];

    occupancy(3, 0) = p4_x[i];
    occupancy(3, 1) = p4_y[i];

    dynamic_obstacle.push_back(occupancy);
  }

  int id = reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
               ->AddObstacle(dynamic_obstacle, is_soft, is_static);
  return id;
}

void UpdateObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, int id,
                                double p1_x[], double p1_y[], double p2_x[],
                                double p2_y[], double p3_x[], double p3_y[],
                                double p4_x[], double p4_y[], const int size,
                                bool is_static) {
  std::vector<Eigen::MatrixXd> dynamic_obstacle;

  for (int i = 0; i < size; ++i) {
    Eigen::MatrixXd occupancy(4, 2);  // fill anti-clockwise

    occupancy(0, 0) = p1_x[i];
    occupancy(0, 1) = p1_y[i];

    occupancy(1, 0) = p2_x[i];
    occupancy(1, 1) = p2_y[i];

    occupancy(2, 0) = p3_x[i];
    occupancy(2, 1) = p3_y[i];

    occupancy(3, 0) = p4_x[i];
    occupancy(3, 1) = p4_y[i];

    dynamic_obstacle.push_back(occupancy);
  }

  reinterpret_cast<MiqpPlanner*>(c_miqp_planner)
      ->UpdateObstacle(id, dynamic_obstacle);
}

void RemoveAllObstaclesCMiqpPlanner(CMiqpPlanner c_miqp_planner) {
  reinterpret_cast<MiqpPlanner*>(c_miqp_planner)->RemoveAllObstacles();
}