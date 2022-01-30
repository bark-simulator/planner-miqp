// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


// Decision Variables

// INPUT VARIABLES - no ranges any more as these now relate to the areas
dvar float u_x[CarRange][idxRange];// in LimitsJerkLong.min_..LimitsJerkLong.max_;
dvar float u_y[CarRange][idxRange];// in LimitsJerkLat.min_..LimitsJerkLat.max_;

// STATE VARIABLES
// !!! Remark: would be nice to be able to write something like this
// dexpr float pos_x[i in 2..N] = pos_x[i-1] + u_x[i-1];
dvar float pos_x[CarRange][idxRange];
dvar float vel_x[CarRange][idxRange];// in absolut_velocity_range; // bound velocites seems to be slower than using normal constraints!
dvar float acc_x[CarRange][idxRange];
dvar float pos_y[CarRange][idxRange];
dvar float vel_y[CarRange][idxRange];// in absolut_velocity_range; // bound velocites seems to be slower than using normal constraints!
dvar float acc_y[CarRange][idxRange];

dvar float pos_x_front_UB[CarRange][idxRange];
dvar float pos_x_front_LB[CarRange][idxRange];
dvar float pos_y_front_UB[CarRange][idxRange];
dvar float pos_y_front_LB[CarRange][idxRange];

dvar boolean notWithinEnvironmentRear[CarRange][EnvironmentsRange][idxRange];
dvar boolean notWithinEnvironmentFrontUbUb[CarRange][EnvironmentsRange][idxRange];
dvar boolean notWithinEnvironmentFrontLbUb[CarRange][EnvironmentsRange][idxRange];
dvar boolean notWithinEnvironmentFrontUbLb[CarRange][EnvironmentsRange][idxRange];
dvar boolean notWithinEnvironmentFrontLbLb[CarRange][EnvironmentsRange][idxRange];
dvar boolean active_region[CarRange][idxRange][RegionRange];
//range float absolut_velocity_range = min_vel_x_y..max_vel_x_y;
dvar boolean region_change_not_allowed_x_positive[CarRange][idxRange];
dvar boolean region_change_not_allowed_y_positive[CarRange][idxRange];
dvar boolean region_change_not_allowed_x_negative[CarRange][idxRange];
dvar boolean region_change_not_allowed_y_negative[CarRange][idxRange];
dvar boolean region_change_not_allowed_combined[CarRange][idxRange];

// integer variables for obstacle avoidance (polygon)
dvar boolean deltacc[CarRange][ObstaclesRange][idxRange][obstacle_lines_range]; 
dvar boolean deltacc_front[CarRange][ObstaclesRange][idxRange][obstacle_lines_range][1..4]; // 4 is not a magic number here but all combinations of UBUB, UBLB, LBUB, UBUB

// slack variables for obstacle avoidance
dvar float slackvarsObstacle[CarRange][ObstaclesRange][idxRange] in 0..1; 
dvar float slackvarsObstacle_front[CarRange][ObstaclesRange][idxRange][1..4] in 0..1; // 4 is not a magic number here but all combinations of UBUB, UBLB, LBUB, UBUB

// car to car collisions
dvar boolean car2car_collision[car2carCollisionRange][car2carCollisionRange][idxRange][1..16]; // Idxs: Rear/rear = 1..4, Rear/front = 5..8, Front/rear = 9..12, Front/front = 13..16

// slack variables for agent to agent collision
dvar float slackvars[car2carCollisionRange][car2carCollisionRange][idxRange][1..4] in 0..maximum_slack; //Idxs: rear x = 1, rear y = 2, front x = 3, front y = 4
