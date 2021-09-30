// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Objective Function
dexpr float costs = sum(i in idxRange, c in CarRange)   
						( WEIGHTS_POS_X[c]  * (pos_x[c,i]-x_ref[c,i]) * (pos_x[c,i]-x_ref[c,i])
						+ WEIGHTS_VEL_X[c]  * (vel_x[c,i]-vx_ref[c,i]) * (vel_x[c,i]-vx_ref[c,i])
						+ WEIGHTS_ACC_X[c]  * acc_x[c,i] * acc_x[c,i]
						+ WEIGHTS_POS_Y[c]  * (pos_y[c,i]-y_ref[c,i])   * (pos_y[c,i]-y_ref[c,i])
						+ WEIGHTS_VEL_Y[c]  * (vel_y[c,i]-vy_ref[c,i])   * (vel_y[c,i]-vy_ref[c,i])
						+ WEIGHTS_ACC_Y[c]  * acc_y[c,i] * acc_y[c,i]
						+ WEIGHTS_JERK_X[c] * u_x[c,i]*u_x[c,i] 
						+ WEIGHTS_JERK_Y[c] * u_y[c,i]*u_y[c,i])
						+ sum(i in idxRange, c in CarRange, o in ObstaclesRange) (WEIGHTS_SLACK_OBSTACLE*slackvarsObstacle[c,o,i]*slackvarsObstacle[c,o,i])
						+ sum(i in idxRange, c in CarRange, o in ObstaclesRange, s in 1..4) (WEIGHTS_SLACK_OBSTACLE*slackvarsObstacle_front[c,o,i,s]*slackvarsObstacle_front[c,o,i,s])
						+ sum(i in idxRange, c in car2carCollisionRange, s in 1..4) (WEIGHTS_SLACK*slackvars[c,i,s]*slackvars[c,i,s]);
minimize costs;

// Does not work out of the box: Multi-Objective formulation:
//dexpr float costs_one = sum(i in idxRange)   
//						( WEIGHTS_POS_X[1]  * (pos_x[i,1]-x_ref[1,i]) * (pos_x[i,1]-x_ref[1,i])
//						+ WEIGHTS_VEL_X[1]  * (vel_x[i,1]-vx_ref[1,i]) * (vel_x[i,1]-vx_ref[1,i])
//						+ WEIGHTS_ACC_X[1]  * acc_x[i,1] * acc_x[1,1]
//						+ WEIGHTS_POS_Y[1]  * (pos_y[i,1]-y_ref[1,i])   * (pos_y[i,1]-y_ref[1,i])
//						+ WEIGHTS_VEL_Y[1]  * (vel_y[i,1]-vy_ref[1,i])   * (vel_y[i,1]-vy_ref[1,i])
//						+ WEIGHTS_ACC_Y[1]  * acc_y[i,1] * acc_y[i,1]
//						+ WEIGHTS_JERK_X[1] * u_x[i,1]*u_x[i,1] 
//						+ WEIGHTS_JERK_Y[1] * u_y[i,1]*u_y[i,1]);					
//dexpr float costs_two = sum(i in idxRange)   
//						( WEIGHTS_POS_X[2]  * (pos_x[i,2]-x_ref[2,i]) * (pos_x[i,2]-x_ref[2,i])
//						+ WEIGHTS_VEL_X[2]  * (vel_x[i,2]-vx_ref[2,i]) * (vel_x[i,2]-vx_ref[2,i])
//						+ WEIGHTS_ACC_X[2]  * acc_x[i,2] * acc_x[2,2]
//						+ WEIGHTS_POS_Y[2]  * (pos_y[i,2]-y_ref[2,i])   * (pos_y[i,2]-y_ref[2,i])
//						+ WEIGHTS_VEL_Y[2]  * (vel_y[i,2]-vy_ref[2,i])   * (vel_y[i,2]-vy_ref[2,i])
//						+ WEIGHTS_ACC_Y[2]  * acc_y[i,2] * acc_y[i,2]
//						+ WEIGHTS_JERK_X[2] * u_x[i,2]*u_x[i,2] 
//						+ WEIGHTS_JERK_Y[2] * u_y[i,2]*u_y[i,2]);
//minimize staticLex(costs_one, costs_two); 