// Collision avoidance constraints
subject to 
{

	// environment constraints: stay in convex environment (Formulation according to frese using bigM)
	if (nr_environments > 0) 
	{
		forall (i in 1..NumSteps, c in CarRange)
		{ 
			forall (j in EnvironmentsRange) 
			{
				// Generic Formulation: check if point is to the left of all lanes  
				forall (line in MultiEnvironmentConvexPolygon[j]) 
				{  	
					(line.x2 - line.x1) * (pos_y[c,i] - line.y1) - 
						(pos_x[c,i] - line.x1) * (line.y2 - line.y1) >= -BigM_pos_env * notWithinEnvironmentRear[c,j,i];
	
					(line.x2 - line.x1) * (pos_y_front_UB[c,i] - line.y1) - 
						(pos_x_front_UB[c,i] - line.x1) * (line.y2 - line.y1) >= -BigM_pos_env * notWithinEnvironmentFrontUbUb[c,j,i];
	
					(line.x2 - line.x1) * (pos_y_front_UB[c,i] - line.y1) - 
						(pos_x_front_LB[c,i] - line.x1) * (line.y2 - line.y1) >= -BigM_pos_env * notWithinEnvironmentFrontLbUb[c,j,i];
	
					(line.x2 - line.x1) * (pos_y_front_LB[c,i] - line.y1) - 
						(pos_x_front_UB[c,i] - line.x1) * (line.y2 - line.y1) >= -BigM_pos_env * notWithinEnvironmentFrontUbLb[c,j,i];
	
					(line.x2 - line.x1) * (pos_y_front_LB[c,i] - line.y1) - 
						(pos_x_front_LB[c,i] - line.x1) * (line.y2 - line.y1) >= -BigM_pos_env * notWithinEnvironmentFrontLbLb[c,j,i];
				}
			}
			sum(j in EnvironmentsRange) notWithinEnvironmentRear[c,j,i] <= nr_environments-1;
			sum(j in EnvironmentsRange) notWithinEnvironmentFrontUbUb[c,j,i] <= nr_environments-1; 	
			sum(j in EnvironmentsRange) notWithinEnvironmentFrontLbUb[c,j,i] <= nr_environments-1;
			sum(j in EnvironmentsRange) notWithinEnvironmentFrontUbLb[c,j,i] <= nr_environments-1; 	
			sum(j in EnvironmentsRange) notWithinEnvironmentFrontLbLb[c,j,i] <= nr_environments-1; 	
		}			
	} else { 
		// explicitly setting the values even if not needed avoids warnings and brings no overhead time
		forall (i in idxRange, j in EnvironmentsRange, c in CarRange) 
		{
			 notWithinEnvironmentRear[c,j,i] == 0;
			 notWithinEnvironmentFrontUbUb[c,j,i] == 0;
			 notWithinEnvironmentFrontLbUb[c,j,i] == 0;
			 notWithinEnvironmentFrontUbLb[c,j,i] == 0;
			 notWithinEnvironmentFrontLbLb[c,j,i] == 0;
		}  	
	} 


 	// obstacle avoidance constraints ... avoid generic convex obstacle
 	// This works smooth, it would be possible to check for card(ObstacleConvexRegion) == 1 => static obstacle and implement the above without the [i]s
 	if (nr_obstacles > 0)
 	{
	    forall (i in 1..NumSteps, c in CarRange) 
	    {
	    	forall (j in ObstaclesRange)
	    	{
				forall (line in ObstacleConvexPolygon[j][i]) 
				{  	
					// Formulation using BigM
					(line.x2 - line.x1) * (pos_y[c,i] - line.y1) - (pos_x[c,i] - line.x1) * (line.y2 - line.y1) <= 0 + deltacc[c,j,i,line.idx] * BigM_pos_obs;
					(line.x2 - line.x1) * (pos_y_front_LB[c,i] - line.y1) - (pos_x_front_LB[c,i] - line.x1) * (line.y2 - line.y1) <= 0 + (deltacc_front[c,j,i,line.idx,1]) * BigM_pos_obs;
					(line.x2 - line.x1) * (pos_y_front_LB[c,i] - line.y1) - (pos_x_front_UB[c,i] - line.x1) * (line.y2 - line.y1) <= 0 + (deltacc_front[c,j,i,line.idx,2]) * BigM_pos_obs;
					(line.x2 - line.x1) * (pos_y_front_UB[c,i] - line.y1) - (pos_x_front_LB[c,i] - line.x1) * (line.y2 - line.y1) <= 0 + (deltacc_front[c,j,i,line.idx,3]) * BigM_pos_obs;
					(line.x2 - line.x1) * (pos_y_front_UB[c,i] - line.y1) - (pos_x_front_UB[c,i] - line.x1) * (line.y2 - line.y1) <= 0 + (deltacc_front[c,j,i,line.idx,4]) * BigM_pos_obs;
				}
				
//				This construct does not work (cplex cannot extract), the forall loop breaks but I dont know why. ATM not a problem as all obstacles can have the
//				same number of edges. maybe precompute the number of edges per obstacle? dont use card() here?
//				// if one of the obstacles has less lines than the other obstacles explicity set the collision variables to zero for the non-existing lines
//				if(card(ObstacleConvexPolygon[j][i]) < max_lines_obstacles)
//				{
//					forall(k in card(ObstacleConvexPolygon[j][i])..max_lines_obstacles)
//					{					
//					  	deltacc[c,j,i,k] == 0;	
//						deltacc_front[c,j,i,k,1] == 0;
//						deltacc_front[c,j,i,k,2] == 0;
//						deltacc_front[c,j,i,k,3] == 0;
//						deltacc_front[c,j,i,k,4] == 0;	 
//					}	
//				}	
				if (obstacle_is_soft[j] == 1) 
				{
					sum(line in ObstacleConvexPolygon[j][i]) deltacc[c,j,i,line.idx] - slackvarsObstacle[c,j,i] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,1] - slackvarsObstacle_front[c,j,i,1] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,2] - slackvarsObstacle_front[c,j,i,2] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,3] - slackvarsObstacle_front[c,j,i,3] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,4] - slackvarsObstacle_front[c,j,i,4] <= card(ObstacleConvexPolygon[j][i]) - 1;
  				}
  				else {
  				  	sum(line in ObstacleConvexPolygon[j][i]) deltacc[c,j,i,line.idx] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,1] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,2] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,3] <= card(ObstacleConvexPolygon[j][i]) - 1;
					sum(line in ObstacleConvexPolygon[j][i]) deltacc_front[c,j,i,line.idx,4] <= card(ObstacleConvexPolygon[j][i]) - 1;
  				}				 
			}	
 		}				
	} else {
		// explicitly setting the values even if not needed avoids warnings and brings no overhead time	
		forall (i in 1..NumSteps, c in CarRange, j in ObstaclesRange, k in obstacle_lines_range) 
	    {
			deltacc[c,j,i,k] == 0;	
			deltacc_front[c,j,i,k,1] == 0;
			deltacc_front[c,j,i,k,2] == 0;
			deltacc_front[c,j,i,k,3] == 0;
			deltacc_front[c,j,i,k,4] == 0;							
  		}    							
	}	//end obstacle avoidance
  
}