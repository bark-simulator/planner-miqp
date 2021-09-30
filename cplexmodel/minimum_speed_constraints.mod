// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


subject to {
	// at very low speeds, avoid to change the region
	forall (i in 2..NumSteps, c in CarRange) // note that we go from index two!
	{ 
		forall (j in RegionRange) 
		{
			// Formulation using bigM
			
			vel_x[c,i] - minimum_region_change_speed >= -(region_change_not_allowed_x_positive[c,i])*BigM_vel; 
			vel_x[c,i] - minimum_region_change_speed <= (1-region_change_not_allowed_x_positive[c,i])*BigM_vel;
			
			-vel_x[c,i] - minimum_region_change_speed <= (1-region_change_not_allowed_x_negative[c,i])*BigM_vel; 
			-vel_x[c,i] - minimum_region_change_speed >= -(region_change_not_allowed_x_negative[c,i])*BigM_vel;
			
			vel_y[c,i] - minimum_region_change_speed >= -(region_change_not_allowed_y_positive[c,i])*BigM_vel; 
			vel_y[c,i] - minimum_region_change_speed <= (1-region_change_not_allowed_y_positive[c,i])*BigM_vel;
			
			-vel_y[c,i] - minimum_region_change_speed <= (1-region_change_not_allowed_y_negative[c,i])*BigM_vel; 
			-vel_y[c,i] - minimum_region_change_speed >= -(region_change_not_allowed_y_negative[c,i])*BigM_vel;
			
			
			//-minimum_region_change_speed <= vel_x[i] + region_change_not_allowed_long[i]*BigM_vel;
			//vel_x[i] >= minimum_region_change_speed - region_change_not_allowed_long[i]*BigM_vel;
			//-minimum_region_change_speed <= vel_y[i] + region_change_not_allowed_lat[i]*BigM_vel;
			//vel_y[i] >= minimum_region_change_speed - region_change_not_allowed_lat[i]*BigM_vel;
			
			//active_region[i][j] - active_region[i-1][j] <= (4-region_change_not_allowed_x_negative[i]-region_change_not_allowed_x_positive[i]-region_change_not_allowed_y_negative[i]-region_change_not_allowed_y_positive[i]); 
			//active_region[i][j] - active_region[i-1][j] >= -(4-region_change_not_allowed_x_negative[i]-region_change_not_allowed_x_positive[i]-region_change_not_allowed_y_negative[i]-region_change_not_allowed_y_positive[i]); 
			
			active_region[c,i,j] - active_region[c,i-1,j] <= (1-region_change_not_allowed_combined[c,i]);
			active_region[c,i,j] - active_region[c,i-1,j] >= -(1-region_change_not_allowed_combined[c,i]);
			
			
			region_change_not_allowed_combined[c,i] <= region_change_not_allowed_x_positive[c,i];
			region_change_not_allowed_combined[c,i] <= region_change_not_allowed_y_positive[c,i];
			region_change_not_allowed_combined[c,i] <= region_change_not_allowed_x_negative[c,i];
			region_change_not_allowed_combined[c,i] <= region_change_not_allowed_y_negative[c,i];
			region_change_not_allowed_combined[c,i] >= region_change_not_allowed_x_positive[c,i] + region_change_not_allowed_y_positive[c,i] + region_change_not_allowed_x_negative[c,i] + region_change_not_allowed_y_negative[c,i] - 3; // case: both var are true

			// Formulation using implies =>	
			//			(-minimum_region_change_speed <= vel_x[i] && vel_x[i] <= minimum_region_change_speed && -minimum_region_change_speed <= vel_y[i] && vel_y[i] <= minimum_region_change_speed) =>
			//			(active_region[i][j] == active_region[i-1][j]);
		}				
	}	

}