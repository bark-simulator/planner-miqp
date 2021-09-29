// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "miqp_planner_settings.h"

#define TRAJECTORY_TIME_IDX 0
#define TRAJECTORY_X_IDX 1
#define TRAJECTORY_Y_IDX 2
#define TRAJECTORY_VX_IDX 3
#define TRAJECTORY_VY_IDX 4
#define TRAJECTORY_AX_IDX 5
#define TRAJECTORY_AY_IDX 6
#define TRAJECTORY_UX_IDX 7
#define TRAJECTORY_UY_IDX 8
#define TRAJECTORY_SIZE 9

typedef void* CMiqpPlanner;

extern "C" {

/**
 * @brief creates new instance of MiqpPlanner on heap
 *
 * @return CMiqpPlanner
 */
CMiqpPlanner NewCMiqpPlanner();

/**
 * @brief creates new instance of MiqpPlanner on heap
 *
 * @param settings
 * @return CMiqpPlanner
 */
CMiqpPlanner NewCMiqpPlannerSettings(MiqpPlannerSettings settings);

/**
 * @brief deletes miqp planner
 *
 * @param c_miqp_planner
 */
void DelCMiqpPlanner(CMiqpPlanner c_miqp_planner);

/**
 * @brief adds car to plan for to MiqpPlanner
 *
 * @param c_miqp_planner
 * @param initial_state_in: 1x6 array of the initial states
 * @param ref_in: 2xref_size array of reference line points, in the order x0,
 * y0, x1, y1
 * @param ref_size
 * @param vDes
 * @param deltaSDes
 * @param timestep
 * @param track_reference_positions
 * @return int
 */
int AddCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, double initial_state_in[],
                       double ref_in[], const int ref_size, double vDes,
                       double deltaSDes, const double timestep,
                       const bool track_reference_positions);

/**
 * @brief let's MiqpPlanner plan a trajectory for each controlled agent
 *
 * @param c_miqp_planner
 * @param timestep
 * @return true
 * @return false
 */
bool PlanCMiqpPlanner(CMiqpPlanner c_miqp_planner, const double timestep);

/**
 * @brief updates initial state and reference of controlled agent
 *
 * @param c_miqp_planner
 * @param idx specifies controlled agent
 * @param initial_state_in
 * @param ref_in reference as 1d array
 * @param ref_size size of reference
 * @param timestep
 * @param track_reference_positions
 */
void UpdateCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                           double initial_state_in[], double ref_in[],
                           const int ref_size, const double timestep,
                           bool track_reference_positions);

/**
 * @brief activates writing of optimization debug files to file system
 *
 * @param c_miqp_planner
 * @param path path where files will be written to
 * @param name name of file
 */
void ActivateDebugFileWriteCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                        char path[], char name[]);

/**
 * @brief returns number of optimization support points
 *
 * @param c_miqp_planner
 * @return int
 */
int GetNCMiqpPlanner(CMiqpPlanner c_miqp_planner);

/**
 * @brief returns time increment of optimization
 *
 * @param c_miqp_planner
 * @return float
 */
float GetTsCMiqpPlanner(CMiqpPlanner c_miqp_planner);

/**
 * @brief returns collision radius
 *
 * @param c_miqp_planner
 * @return float
 */
float GetCollisionRadius(CMiqpPlanner c_miqp_planner);

/**
 * @brief returns the trajectory obtained from optimization as an array
 *
 * @param c_miqp_planner
 * @param carIdx specifies controlled agent
 * @param start_time
 * @param trajectory
 * @param size
 */
void GetRawCMiqpTrajectoryCMiqpPlanner(CMiqpPlanner c_miqp_planner, int carIdx,
                                       double start_time, double* trajectory,
                                       int& size);

/**
 * @brief Get the last reference trajectory as an array
 *
 * @param c_miqp_planner
 * @param carIdx specifies controlled agent
 * @param start_time
 * @param trajectory array containing the trajectory
 * @param size size of array
 */
void GetRawCLastReferenceTrajectoryCMiqpPlaner(CMiqpPlanner c_miqp_planner,
                                               int carIdx, double start_time,
                                               double* trajectory, int& size);

/**
 * @brief updates convexified map based on current position of the ego vehicle
 *
 * @param c_miqp_planner
 * @param poly_pts array containing the polygon points
 * @param poly_size size of array
 * @return true if update was successfull
 */
bool UpdateConvexifiedMapCMiqpPlaner(CMiqpPlanner c_miqp_planner,
                                     double poly_pts[], const int poly_size);

/**
 * @brief updates desired velocity of MiqpPlanner
 *
 * @param c_miqp_planner
 * @param carIdx
 * @param vDes
 * @param deltaSDes
 */
void UpdateDesiredVelocityCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                       const int carIdx, const double vDes,
                                       const double deltaSDes);

/**
 * @brief adds obstacle to MiqpPlanner
 *
 * @param c_miqp_planner
 * @param p1_x
 * @param p1_y
 * @param p2_x
 * @param p2_y
 * @param p3_x
 * @param p3_y
 * @param p4_x
 * @param p4_y
 * @param size
 * @param is_static: flag wether obstacle will be treated as soft constraint
 * @param is_soft: is obstacle static or moving over time
 * @return int index of obstacle
 */
int AddObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, double p1_x[],
                            double p1_y[], double p2_x[], double p2_y[],
                            double p3_x[], double p3_y[], double p4_x[],
                            double p4_y[], const int size, bool is_static,
                            bool is_soft);

/**
 * @brief updates occupied polygon of obstacle
 *
 * @param c_miqp_planner
 * @param id
 * @param p1_x
 * @param p1_y
 * @param p2_x
 * @param p2_y
 * @param p3_x
 * @param p3_y
 * @param p4_x
 * @param p4_y
 * @param size
 * @param is_static
 */
void UpdateObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, int id,
                                double p1_x[], double p1_y[], double p2_x[],
                                double p2_y[], double p3_x[], double p3_y[],
                                double p4_x[], double p4_y[], const int size,
                                bool is_static);

/**
 * @brief removes all obstacles within MiqpPlanner
 *
 * @param c_miqp_planner
 */
void RemoveAllObstaclesCMiqpPlanner(CMiqpPlanner c_miqp_planner);
}
