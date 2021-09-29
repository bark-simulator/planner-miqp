// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_PLANNER_SETTINGS_HEADER
#define MIQP_PLANNER_SETTINGS_HEADER

enum MiqpPlannerWarmstartType {
    NO_WARMSTART = 0,
    RECEDING_HORIZON_WARMSTART = 1,
    LAST_SOLUTION_WARMSTART = 2,
    BOTH_WARMSTART_STRATEGIES = 3
  };


enum MiqpPlannerParallelMode {
  // https://www.ibm.com/support/knowledgecenter/SSSA5P_20.1.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/ParallelMode.html
  DETERMINISTIC = 0, // CPX_PARALLEL_DETERMINISTIC,
  AUTO = 1, // CPX_PARALLEL_AUTO,
  OPPORTUNISTIC = 2 // CPX_PARALLEL_OPPORTUNISTIC
};

struct MiqpPlannerSettings {
  int nr_regions;
  int nr_steps;
  int nr_neighbouring_possible_regions;
  float ts;
  int precision;
  float constant_agent_safety_distance_slack;
  float minimum_region_change_speed;
  float lambda;
  float wheelBase;
  float collisionRadius;
  float slackWeight;
  float slackWeightObstacle;
  float jerkWeight;
  float positionWeight;
  float velocityWeight;
  float acclerationWeight;
  float accLonMaxLimit;
  float accLonMinLimit;
  float jerkLonMaxLimit;
  float accLatMinMaxLimit;
  float jerkLatMinMaxLimit;
  float simplificationDistanceMap;
  float simplificationDistanceReferenceLine;
  float bufferReference;
  float buffer_for_merging_tolerance;
  float refLineInterpInc;
  int additionalStepsForReferenceLongerHorizon;
  float max_solution_time;
  float relative_mip_gap_tolerance;
  int mipdisplay;
  int mipemphasis;
  float relobjdif;
  int cutpass;
  int probe;
  int repairtries;
  int rinsheur;
  int varsel;
  int mircuts;
  const char* cplexModelpath;
  bool useSos;
  bool useBranchingPriorities;
  MiqpPlannerWarmstartType warmstartType;
  MiqpPlannerParallelMode parallelMode;
  float max_velocity_fitting;
  bool buffer_cplex_outputs;
  bool obstacle_roi_filter;
  float obstacle_roi_behind_distance;
  float obstacle_roi_front_distance;
  float obstacle_roi_side_distance;
};

#endif  // MIQP_PLANNER_SETTINGS_HEADER
