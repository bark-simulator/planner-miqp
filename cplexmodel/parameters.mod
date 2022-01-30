// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

//=========================================
// Solver settings
float max_solution_time = ...;
float relative_mip_gap_tolerance = ...;
int mipdisplay = ...;
int mipemphasis = ...;
float relobjdif = ...;
int cutpass = ...;
int probe = ...;
int repairtries = ...;
int rinsheur = ...;
int varsel = ...;
int mircuts = ...;
int parallelmode = ...;


//=========================================
// Global Constant Parameters, independent of car, time
int BigM_jerk = 10;
int BigM_vel_fractionparam = 1000; //100 should be ok, has to be greater than vx and vy, but only 1000 is stable
int BigM_pos_poly_vel = 100; //only relative positions
int BigM_acc = 10;
int BigM_acc_poly_kappa = 1000;
int BigM_vel = 100; //100 should be ok, has to be greater than vx and vy
int BigM_pos_env = 10000; //1000 is ok, if one environment segment can be 100m long
int BigM_pos_obs = 10000; //1000 is ok, if one obstacle can be 10m long and 100m away from the agent(s)
int BigM_agents = 1000;

int NumSteps = ...; // number of optimization steps
range idxRange = 1..NumSteps; // time range
float ts = ...; // step-step size

int nr_regions = ...;
range RegionRange = 1..nr_regions;

int NumCars = ...; // number of vehicle agents (not obstacles)
range CarRange = 1..NumCars;

// Size and range for agent2agent collision matrix set after initialization

float min_vel_x_y = ...; 
float max_vel_x_y = ...;

float total_min_acc = ...;
float total_max_acc = ...;
float total_min_jerk = ...;
float total_max_jerk = ...;


//=========================================
// Time-dependent Parameters
float agent_safety_distance[idxRange] = ...;
float agent_safety_distance_slack[idxRange] = ...;
float maximum_slack = ...;


//=========================================
// Car-dependent Parameters
float WEIGHTS_POS_X[CarRange] = ...;// = 1;
float WEIGHTS_VEL_X[CarRange] = ...;// = 0;
float WEIGHTS_ACC_X[CarRange] = ...;// = 0;
float WEIGHTS_POS_Y[CarRange] = ...;// = 1;
float WEIGHTS_VEL_Y[CarRange] = ...;// = 0;
float WEIGHTS_ACC_Y[CarRange] = ...;// = 0;

float WEIGHTS_JERK_X[CarRange] = ...;// = 1;
float WEIGHTS_JERK_Y[CarRange] = ...;// = 1; // see [1] Eq. 24 psi = 10 (weight for input signal lateral acceleration)

float WEIGHTS_SLACK = ...;
float WEIGHTS_SLACK_OBSTACLE = ...;

float WheelBase[CarRange] = ...;
float CollisionRadius[CarRange] = ...;

float IntitialState[CarRange][1..6] = ...;

// REFERENCE
float x_ref[CarRange][idxRange] = ...;
float vx_ref[CarRange][idxRange] = ...; // restrict vx_ref to sqrt(a_lat_max,*r_curve(s)), see [2], Eq. (5) ... requires radius of curve
float y_ref[CarRange][idxRange] = ...;
float vy_ref[CarRange][idxRange] = ...;

float min_acc_x[CarRange][RegionRange] = ...; 
float max_acc_x[CarRange][RegionRange] = ...; 
float min_acc_y[CarRange][RegionRange] = ...; 
float max_acc_y[CarRange][RegionRange] = ...; 
float min_jerk_x[CarRange][RegionRange] = ...; 
float max_jerk_x[CarRange][RegionRange] = ...; 
float min_jerk_y[CarRange][RegionRange] = ...; 
float max_jerk_y[CarRange][RegionRange] = ...; 

int initial_region[CarRange] = ...;

int possible_region[CarRange][RegionRange] = ...;


//=========================================
// Obstacle and Environment-dependent parameters
tuple line {
	key int idx;
	float x1;
	float y1;
	float x2;
	float y2; 
}
// N lines form the convex polygon at each timestep in idxRange
int nr_obstacles = ...;
range ObstaclesRange = 1..nr_obstacles;
{line} ObstacleConvexPolygon[ObstaclesRange][idxRange]= ...;

int max_lines_obstacles = ...;
range obstacle_lines_range = 1..max_lines_obstacles;
int obstacle_is_soft[ObstaclesRange] = ...; // 0: hard, 1: soft

int nr_environments = ...;
range EnvironmentsRange = 1..nr_environments;
{line} MultiEnvironmentConvexPolygon[EnvironmentsRange] = ...;


//=========================================
// Region-dependent parameters
float fraction_parameters[RegionRange][1..4] = ...; // lines representation, thus length 4
float minimum_region_change_speed = ...; 

float POLY_SINT_UB[RegionRange][1..3] = ...;
float POLY_SINT_LB[RegionRange][1..3] = ...;
float POLY_COSS_UB[RegionRange][1..3] = ...;
float POLY_COSS_LB[RegionRange][1..3] = ...;

float POLY_KAPPA_AX_MAX[RegionRange][1..3] = ...;
float POLY_KAPPA_AX_MIN[RegionRange][1..3] = ...;

