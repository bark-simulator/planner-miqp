// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


// Initial conditions
subject to
{
  	// initial conditions model
  	forall (c in CarRange)
	{
		pos_x[c,1] == IntitialState[c,1]; //.pos_x;
		vel_x[c,1] == IntitialState[c,2]; //.vel_x;
		acc_x[c,1] == IntitialState[c,3]; //.acc_x;
		pos_y[c,1] == IntitialState[c,4]; //.pos_y;
		vel_y[c,1] == IntitialState[c,5]; //.vel_y;
		acc_y[c,1] == IntitialState[c,6]; //.acc_y;
		
		pos_x_front_UB[c,1] == IntitialState[c,1] + costheta[c] * WheelBase[c];
		pos_x_front_LB[c,1] == IntitialState[c,1] + costheta[c] * WheelBase[c];
		pos_y_front_UB[c,1] == IntitialState[c,4] + sintheta[c] * WheelBase[c];
		pos_y_front_LB[c,1] == IntitialState[c,4] + sintheta[c] * WheelBase[c];
		
		u_x[c,NumSteps] == 0; // the last control input is never used by the model -> set it to zero (would be zero anyway, as it is in the objective)
		u_y[c,NumSteps] == 0;
	}

	forall (j in RegionRange, c in CarRange) 
	{ 
	    // initial condition region selection
		if(j == initial_region[c]) 
		{
		  	active_region[c,1,j] == true;
		} else {
			active_region[c,1,j] == false;  	
		}
		
		// upper limit jerk long
		u_x[c,1] <= max_jerk_x[c,j] + BigM_jerk*(1-active_region[c,1,j]); 
		
		// lower limit jerk long
		u_x[c,1] >= min_jerk_x[c,j] - BigM_jerk*(1-active_region[c,1,j]);
		
		// upper limit jerk lat
		u_y[c,1] <= max_jerk_y[c,j] + BigM_jerk*(1-active_region[c,1,j]); 
		
		// lower limit jerk lat
		u_y[c,1] >= min_jerk_y[c,j] - BigM_jerk*(1-active_region[c,1,j]); 
	  }
  
	forall (c in CarRange)
	{
		//region is fixed for first step anyway
		region_change_not_allowed_x_positive[c,1] == false; 
		region_change_not_allowed_y_positive[c,1] == false;
		region_change_not_allowed_x_negative[c,1] == false;
		region_change_not_allowed_y_negative[c,1] == false;
		region_change_not_allowed_combined[c,1] == false;
	}	  
}