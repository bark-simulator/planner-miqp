// Parameter declarations to warmstart the decision variables

float u_x_init[CarRange][idxRange] = ...;
float u_y_init[CarRange][idxRange] = ...;

float pos_x_init[CarRange][idxRange] = ...;
float vel_x_init[CarRange][idxRange] = ...;
float acc_x_init[CarRange][idxRange] = ...;
float pos_y_init[CarRange][idxRange] = ...;
float vel_y_init[CarRange][idxRange] = ...;
float acc_y_init[CarRange][idxRange] = ...;

float pos_x_front_UB_init[CarRange][idxRange] = ...;
float pos_x_front_LB_init[CarRange][idxRange] = ...;
float pos_y_front_UB_init[CarRange][idxRange] = ...;
float pos_y_front_LB_init[CarRange][idxRange] = ...;

int notWithinEnvironmentRear_init[CarRange][EnvironmentsRange][idxRange] = ...;
int notWithinEnvironmentFrontUbUb_init[CarRange][EnvironmentsRange][idxRange] = ...;
int notWithinEnvironmentFrontLbUb_init[CarRange][EnvironmentsRange][idxRange] = ...;
int notWithinEnvironmentFrontUbLb_init[CarRange][EnvironmentsRange][idxRange] = ...;
int notWithinEnvironmentFrontLbLb_init[CarRange][EnvironmentsRange][idxRange] = ...;
int active_region_init[CarRange][idxRange][RegionRange] = ...;
int region_change_not_allowed_x_positive_init[CarRange][idxRange] = ...;
int region_change_not_allowed_y_positive_init[CarRange][idxRange] = ...;
int region_change_not_allowed_x_negative_init[CarRange][idxRange] = ...;
int region_change_not_allowed_y_negative_init[CarRange][idxRange] = ...;
int region_change_not_allowed_combined_init[CarRange][idxRange] = ...;

int deltacc_init[CarRange][ObstaclesRange][idxRange][obstacle_lines_range] = ...; 
int deltacc_front_init[CarRange][ObstaclesRange][idxRange][obstacle_lines_range][1..4] = ...; 

int car2car_collision_init[car2carCollisionRange][idxRange][1..16] = ...; 

float slackvars_init[car2carCollisionRange][idxRange][1..4] = ...;
