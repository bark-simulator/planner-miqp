// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// Model an Region Constraints
subject to 
{  

  // Kinematic model constraints
  forall (i in 2..NumSteps, c in CarRange) 
  {
    pos_x[c,i] == pos_x[c,i-1] + ts * vel_x[c,i-1] + (1/2) * ts^2 * acc_x[c,i-1] + (1/6) * ts^3 * u_x[c,i-1];
    vel_x[c,i] == vel_x[c,i-1] + ts * acc_x[c,i-1] + (1/2) * ts^2 * u_x[c,i-1];
    acc_x[c,i] == acc_x[c,i-1] + ts * u_x[c,i-1];
    pos_y[c,i] == pos_y[c,i-1] + ts * vel_y[c,i-1] + (1/2) * ts^2 * acc_y[c,i-1] + (1/6) * ts^3 * u_y[c,i-1];
    vel_y[c,i] == vel_y[c,i-1] + ts * acc_y[c,i-1] + (1/2) * ts^2 * u_y[c,i-1];
    acc_y[c,i] == acc_y[c,i-1] + ts * u_y[c,i-1];   
  }
    
  	// General velocity limits (faster than bound constaints)
  	forall (i in 1..NumSteps, c in CarRange) 
  	{
  	  min_vel_x_y <= vel_x[c,i];
	  min_vel_x_y <= vel_y[c,i];
	  max_vel_x_y >= vel_x[c,i];
	  max_vel_x_y >= vel_x[c,i];
	
	  acc_x[c,i] <= total_max_acc;
	  acc_x[c,i] >= total_min_acc;
	  acc_y[c,i] <= total_max_acc;
	  acc_y[c,i] >= total_min_acc;
	  
	  u_x[c,i] <= total_max_jerk;
	  u_x[c,i] >= total_min_jerk;
	  u_y[c,i] <= total_max_jerk;
	  u_y[c,i] >= total_min_jerk;
	  
	}
  	
  	// Set front pos lb ub long lat and acc, jerk limits for all regions
  	// The regions have to be disjoint! (Which they are not atm due to the >= inequalities, > is not possible) --> we solve this with the sum of active regions below
	forall (i in 2..NumSteps, c in CarRange) 
	{
		forall (j in RegionRange) 
		{
			// velocity constraints only active if the region statically defined as reachable
			// for non-possible regions hard set the active region to inactive
			if (possible_region[c, j] == true) 
			{		
				// ensure we are in the correct region 
				// disable correct region selection in case of low speed as the inequalities can be violated here: initial conditions might force the model to be in another region, which is not allowed.
				fraction_parameters[j,1] * vel_y[c,i] >= fraction_parameters[j,2] * vel_x[c,i] - BigM_vel_fractionparam*(1-active_region[c,i,j]) - BigM_vel_fractionparam*region_change_not_allowed_combined[c,i];
				fraction_parameters[j,3] * vel_y[c,i] <= fraction_parameters[j,4] * vel_x[c,i] + BigM_vel_fractionparam*(1-active_region[c,i,j]) + BigM_vel_fractionparam*region_change_not_allowed_combined[c,i];

				// set pos_x_front_UB 
		  		- BigM_pos_poly_vel*(1-active_region[c,i,j]) <= pos_x_front_UB[c,i] -  pos_x[c,i] -  WheelBase[c] * (POLY_COSS_UB[j,1] +  POLY_COSS_UB[j,2]*vel_x[c,i] + POLY_COSS_UB[j,3]*vel_y[c,i]);	  		
		  		BigM_pos_poly_vel*(1-active_region[c,i,j]) >= pos_x_front_UB[c,i] -  pos_x[c,i] -  WheelBase[c] * (POLY_COSS_UB[j,1] +  POLY_COSS_UB[j,2]*vel_x[c,i] + POLY_COSS_UB[j,3]*vel_y[c,i]);
		  		
		  		// set pos_x_front_LB
		  		- BigM_pos_poly_vel*(1-active_region[c,i,j]) <= pos_x_front_LB[c,i] -  pos_x[c,i] -  WheelBase[c] * (POLY_COSS_LB[j,1] +  POLY_COSS_LB[j,2]*vel_x[c,i] + POLY_COSS_LB[j,3]*vel_y[c,i]);	  		
		  		BigM_pos_poly_vel*(1-active_region[c,i,j]) >= pos_x_front_LB[c,i] -  pos_x[c,i] -  WheelBase[c] * (POLY_COSS_LB[j,1] +  POLY_COSS_LB[j,2]*vel_x[c,i] + POLY_COSS_LB[j,3]*vel_y[c,i]);
		  		
		  		// set pos_y_front_UB 
		  		- BigM_pos_poly_vel*(1-active_region[c,i,j]) <= pos_y_front_UB[c,i] -  pos_y[c,i] -  WheelBase[c] * (POLY_SINT_UB[j,1] +  POLY_SINT_UB[j,2]*vel_x[c,i] + POLY_SINT_UB[j,3]*vel_y[c,i]);	  		
		  		BigM_pos_poly_vel*(1-active_region[c,i,j]) >= pos_y_front_UB[c,i] -  pos_y[c,i] -  WheelBase[c] * (POLY_SINT_UB[j,1] +  POLY_SINT_UB[j,2]*vel_x[c,i] + POLY_SINT_UB[j,3]*vel_y[c,i]);
		  		
		  		// set pos_y_front_LB 
		  		- BigM_pos_poly_vel*(1-active_region[c,i,j]) <= pos_y_front_LB[c,i] -  pos_y[c,i] -  WheelBase[c] * (POLY_SINT_LB[j,1] +  POLY_SINT_LB[j,2]*vel_x[c,i] + POLY_SINT_LB[j,3]*vel_y[c,i]);	  		
		  		BigM_pos_poly_vel*(1-active_region[c,i,j]) >= pos_y_front_LB[c,i] -  pos_y[c,i] -  WheelBase[c] * (POLY_SINT_LB[j,1] +  POLY_SINT_LB[j,2]*vel_x[c,i] + POLY_SINT_LB[j,3]*vel_y[c,i]);
	
				// upper limit jerk long
				u_x[c,i] <= max_jerk_x[c,j] + BigM_jerk*(1-active_region[c,i,j]); 
				
				// lower limit jerk long
				u_x[c,i] >= min_jerk_x[c,j] - BigM_jerk*(1-active_region[c,i,j]);
				
				// upper limit jerk lat
				u_y[c,i] <= max_jerk_y[c,j] + BigM_jerk*(1-active_region[c,i,j]); 
				
				// lower limit jerk lat
				u_y[c,i] >= min_jerk_y[c,j]- BigM_jerk*(1-active_region[c,i,j]); 
				
				// absolut upper limit acc long
				acc_x[c,i] <= max_acc_x[c,j]  + BigM_acc*(1-active_region[c,i,j]); 
		  		
		  		// absolut lower limit acc long
		  		acc_x[c,i] >= min_acc_x[c,j] - BigM_acc*(1-active_region[c,i,j]); 
		  		
		  		// absolut upper limit acc lat
				acc_y[c,i] <= max_acc_y[c,j] + BigM_acc*(1-active_region[c,i,j]); 
				
				// absolut lower limit acc lat
				acc_y[c,i] >= min_acc_y[c,j] - BigM_acc*(1-active_region[c,i,j]); 
				
				// maximal curvature upper limit acc long
				acc_y[c,i] <= (POLY_KAPPA_AX_MAX[j,1] +  POLY_KAPPA_AX_MAX[j,2]*vel_x[c,i] + POLY_KAPPA_AX_MAX[j,3]*vel_y[c,i]) +
						((fraction_parameters[j,2] + fraction_parameters[j,4]) / (fraction_parameters[j,1] + fraction_parameters[j,3])) * acc_x[c,i]
						 + BigM_acc_poly_kappa*(1-active_region[c,i,j]) + BigM_acc_poly_kappa*region_change_not_allowed_combined[c,i];
				
		  		// minimal curvature upper limit acc long
				acc_y[c,i] >= (POLY_KAPPA_AX_MIN[j,1] +  POLY_KAPPA_AX_MIN[j,2]*vel_x[c,i] + POLY_KAPPA_AX_MIN[j,3]*vel_y[c,i]) +
						((fraction_parameters[j,2] + fraction_parameters[j,4]) / (fraction_parameters[j,1] + fraction_parameters[j,3])) * acc_x[c,i]
						 - BigM_acc_poly_kappa*(1-active_region[c,i,j]) - BigM_acc_poly_kappa*region_change_not_allowed_combined[c,i]; 
	  		
	  		} else {
	  			// a region cannot be selected if it is not within the list of possible regions
	  			active_region[c,i,j] == false;
	  		}	 
		}
		
		//make sure we are in exactly one region
		sum(j in RegionRange) active_region[c,i,j] == 1; 
		
		// only possible aka. reachable regions can be active --> this should be redundant to active_region[i,j] == false if possible_regions[j] == 0
		//forall (j in RegionRange) { active_region[i,j] <= possible_region[j]; }
	}
}