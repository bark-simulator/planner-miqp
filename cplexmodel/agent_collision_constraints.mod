// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// car2car collision constaints using bigM:
subject to
{
	
    if(NumCars == 1) 
    {
    	forall (i in 1..NumSteps, c in car2carCollisionRange) 
    	{
        	forall (s in 1..4)
        	{             
            	slackvars[c,i,s] == 0;
           	}
           	forall (s in 1..16)
        	{         	
           		car2car_collision[c,i,s] == 0;
            }           		
        }
    } else {

		// WARNING!!! The indexation only works for up to three agents!!!!!
		forall (i in 1..NumSteps, c1 in 1..NumCars-1, c2 in c1+1..NumCars)
	    {
	    	//car2car_collision_idx = (NumCars-1)*(c1-1) + (c2-c1); <-- seems to be a limitation of cplex, I cannot define such a variable.
			
			// rear c1 - rear c2 collision check
			pos_x[c1,i] <= pos_x[c2,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,1]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,1];
			pos_x[c1,i] >= pos_x[c2,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,1]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,2]; 
			pos_y[c1,i] <= pos_y[c2,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,2]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,3]; 
			pos_y[c1,i] >= pos_y[c2,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,2]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,4]; 
			3 >= sum(j in 1..4) car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,j];
			slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,1] <= agent_safety_distance_slack[i];
			slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,2] <= agent_safety_distance_slack[i];
			
			// front c1 - rear c2 collision check
			pos_x[c1,i] <= pos_x_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,5];
			pos_x[c1,i] >= pos_x_front_UB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,6]; 
			pos_y[c1,i] <= pos_y_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,7]; 
			pos_y[c1,i] >= pos_y_front_UB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,8]; 
			3 >= sum(j in 5..8) car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,j];
			
			// front c2 - rear c1 collision check
			pos_x[c2,i] <= pos_x_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,9];
			pos_x[c2,i] >= pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,10]; 
			pos_y[c2,i] <= pos_y_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]) + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,11]; 
			pos_y[c2,i] >= pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]) - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,12]; 
			3 >= sum(j in 9..12) car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,j];
			
			
			// worst-case error front/front collision check
			0 <= pos_x_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,3]) - pos_x_front_UB[c2,i] + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,13];
			0 >= pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,3]) - pos_x_front_LB[c2,i] - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,14];
			0 <= pos_y_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,4]) - pos_y_front_UB[c2,i] + BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,15];
			0 >= pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i] + agent_safety_distance_slack[i] - slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,4]) - pos_y_front_LB[c2,i] - BigM_agents * car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,16];
			3 >= sum(j in 13..16) car2car_collision[(NumCars-1)*(c1-1)+(c2-c1),i,j];
			slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,3] <= agent_safety_distance_slack[i];
			slackvars[(NumCars-1)*(c1-1)+(c2-c1),i,4] <= agent_safety_distance_slack[i];
			
			/*
			This implementation works but is ~2-4 times slower + slack is harder to include
			// front c1 - front c2 collision check
			BigM_agents * (car2car_collision[i][c1][c2][13])     >= -pos_x_front_LB[c2,i] + pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //LB UB X violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][13]) >=  pos_x_front_LB[c2,i] - pos_x_front_UB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //LB UB X violated
			BigM_agents * (car2car_collision[i][c1][c2][14])     >= -pos_x_front_LB[c1,i] + pos_x_front_LB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]); //LB LB X violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][14]) >=  pos_x_front_LB[c1,i] - pos_x_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]); //LB LB X violated
			car2car_collision[i][c1][c2][13] >= car2car_collision[i][c1][c2][21]; // LB UB X violated AND LB LB X violated
			car2car_collision[i][c1][c2][14] >= car2car_collision[i][c1][c2][21]; // LB UB X violated AND LB LB X violated
			car2car_collision[i][c1][c2][13] + car2car_collision[i][c1][c2][14] - car2car_collision[i][c1][c2][21] <= 1; // LB UB X violated AND LB LB X violated
			
			BigM_agents * (car2car_collision[i][c1][c2][15])     >=  pos_x_front_UB[c2,i] - pos_x_front_LB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //UB LB X violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][15]) >= -pos_x_front_UB[c2,i] + pos_x_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //UB LB X violated
			BigM_agents * (car2car_collision[i][c1][c2][16])     >= -pos_x_front_UB[c2,i] + pos_x_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //UB UB X violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][16]) >=  pos_x_front_UB[c2,i] - pos_x_front_UB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //UB UB X violated
			car2car_collision[i][c1][c2][15] >= car2car_collision[i][c1][c2][22]; // UB LB X violated AND UB UB X violated
			car2car_collision[i][c1][c2][16] >= car2car_collision[i][c1][c2][22]; // UB LB X violated AND UB UB X violated
			car2car_collision[i][c1][c2][15] + car2car_collision[i][c1][c2][26] - car2car_collision[i][c1][c2][22] <= 1; // UB LB X violated AND UB UB X violated
			
			car2car_collision[i][c1][c2][21] <= car2car_collision[i][c1][c2][25]; //Collision X: OR
			car2car_collision[i][c1][c2][22] <= car2car_collision[i][c1][c2][25]; //Collision X: OR
			car2car_collision[i][c1][c2][21] + car2car_collision[i][c1][c2][22] >= car2car_collision[i][c1][c2][25]; //Collision X: OR
			
			BigM_agents * (car2car_collision[i][c1][c2][17])     >= -pos_y_front_LB[c2,i] + pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //LB UB Y violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][17]) >=  pos_y_front_LB[c2,i] - pos_y_front_UB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //LB UB Y violated
			BigM_agents * (car2car_collision[i][c1][c2][18])     >= -pos_y_front_LB[c1,i] + pos_y_front_LB[c2,i] + (RR[c1,c2] + agent_safety_distance[i]); //LB LB Y violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][18]) >=  pos_y_front_LB[c1,i] - pos_y_front_LB[c2,i] - (RR[c1,c2] + agent_safety_distance[i]); //LB LB Y violated
			car2car_collision[i][c1][c2][17] >= car2car_collision[i][c1][c2][23]; // LB UB Y violated AND LB LB Y violated
			car2car_collision[i][c1][c2][18] >= car2car_collision[i][c1][c2][23]; // LB UB Y violated AND LB LB Y violated
			car2car_collision[i][c1][c2][17] + car2car_collision[i][c1][c2][18] - car2car_collision[i][c1][c2][23] <= 1; // LB UB Y violated AND LB LB Y violated
			
			BigM_agents * (car2car_collision[i][c1][c2][19])     >=  pos_y_front_UB[c2,i] - pos_y_front_LB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //UB LB Y violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][19]) >= -pos_y_front_UB[c2,i] + pos_y_front_LB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //UB LB Y violated
			BigM_agents * (car2car_collision[i][c1][c2][20])     >= -pos_y_front_UB[c2,i] + pos_y_front_UB[c1,i] + (RR[c1,c2] + agent_safety_distance[i]); //UB UB Y violated
			BigM_agents * (1 - car2car_collision[i][c1][c2][20]) >=  pos_y_front_UB[c2,i] - pos_y_front_UB[c1,i] - (RR[c1,c2] + agent_safety_distance[i]); //UB UB Y violated
			car2car_collision[i][c1][c2][19] >= car2car_collision[i][c1][c2][24]; // UB LB Y violated AND UB UB Y violated
			car2car_collision[i][c1][c2][20] >= car2car_collision[i][c1][c2][24]; // UB LB Y violated AND UB UB Y violated
			car2car_collision[i][c1][c2][19] + car2car_collision[i][c1][c2][20] - car2car_collision[i][c1][c2][24] <= 1; // UB LB Y violated AND UB UB Y violated
			
			car2car_collision[i][c1][c2][23] <= car2car_collision[i][c1][c2][26]; //Collision Y: OR
			car2car_collision[i][c1][c2][24] <= car2car_collision[i][c1][c2][26]; //Collision Y: OR
			car2car_collision[i][c1][c2][23] + car2car_collision[i][c1][c2][24] >= car2car_collision[i][c1][c2][26]; //Collision Y: OR
			
			car2car_collision[i][c1][c2][25] + car2car_collision[i][c1][c2][26] <= 1; // No collision
			*/
			
	    }  
	}	
}