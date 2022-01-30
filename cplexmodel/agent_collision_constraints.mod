// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// car2car collision constaints using bigM:
subject to
{
	
    if(NumCars == 1) 
    {
    	forall (i in 1..NumSteps, c1 in car2carCollisionRange, c2 in car2carCollisionRange) 
    	{
        	forall (s in 1..4)
        	{             
            	slackvars[c1,c2,i,s] == 0;
           	}
           	forall (s in 1..16)
        	{         	
           		car2car_collision[c1,c2,i,s] == 0;
            }           		
        }
    } else {
      
        // Zero out upper triagles of the collision and slack matrices
      	forall (i in 1..NumSteps, c1 in 2..NumCar2CarCollisions, c2 in 1..c1-1)
	    {
	        forall (s in 1..4)
        	{             
            	slackvars[c1,c2,i,s] == 0;
           	}
           	forall (s in 1..16)
        	{         	
           		car2car_collision[c1,c2,i,s] == 0;
            }  
	    }

		forall (i in 1..NumSteps, c1 in 1..NumCars-1, c2 in c1+1..NumCars)
	    {			
			// rear c1 - rear c2 collision check
			pos_x[c1,i] <= pos_x[c2,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,1]) + BigM_agents * car2car_collision[c1,c2-1,i,1];
			pos_x[c1,i] >= pos_x[c2,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,1]) - BigM_agents * car2car_collision[c1,c2-1,i,2]; 
			pos_y[c1,i] <= pos_y[c2,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,2]) + BigM_agents * car2car_collision[c1,c2-1,i,3]; 
			pos_y[c1,i] >= pos_y[c2,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,2]) - BigM_agents * car2car_collision[c1,c2-1,i,4]; 
			3 >= sum(j in 1..4) car2car_collision[c1,c2-1,i,j];
			slackvars[c1,c2-1,i,1] <= agent_safety_distance_slack[i];
			slackvars[c1,c2-1,i,2] <= agent_safety_distance_slack[i];
			
			// front c1 - rear c2 collision check
			pos_x[c1,i] <= pos_x_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[c1,c2-1,i,5];
			pos_x[c1,i] >= pos_x_front_UB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[c1,c2-1,i,6]; 
			pos_y[c1,i] <= pos_y_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[c1,c2-1,i,7]; 
			pos_y[c1,i] >= pos_y_front_UB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[c1,c2-1,i,8]; 
			3 >= sum(j in 5..8) car2car_collision[c1,c2-1,i,j];
			
			// front c2 - rear c1 collision check
			pos_x[c2,i] <= pos_x_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[c1,c2-1,i,9];
			pos_x[c2,i] >= pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[c1,c2-1,i,10]; 
			pos_y[c2,i] <= pos_y_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[c1,c2-1,i,11]; 
			pos_y[c2,i] >= pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[c1,c2-1,i,12]; 
			3 >= sum(j in 9..12) car2car_collision[c1,c2-1,i,j];
			
			
			// worst-case error front/front collision check
			0 <= pos_x_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,3]) - pos_x_front_UB[c2,i] + BigM_agents * car2car_collision[c1,c2-1,i,13];
			0 >= pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,3]) - pos_x_front_LB[c2,i] - BigM_agents * car2car_collision[c1,c2-1,i,14];
			0 <= pos_y_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,4]) - pos_y_front_UB[c2,i] + BigM_agents * car2car_collision[c1,c2-1,i,15];
			0 >= pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[c1,c2-1,i,4]) - pos_y_front_LB[c2,i] - BigM_agents * car2car_collision[c1,c2-1,i,16];
			3 >= sum(j in 13..16) car2car_collision[c1,c2-1,i,j];
			slackvars[c1,c2-1,i,3] <= agent_safety_distance_slack[i];
			slackvars[c1,c2-1,i,4] <= agent_safety_distance_slack[i];
			
	    }
	}	
}