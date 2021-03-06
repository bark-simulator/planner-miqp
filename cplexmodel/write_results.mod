execute 
{
	var ofile = new IloOplOutputFile("modelRun.txt");
	ofile.writeln("u_x=",u_x,";");
	ofile.writeln("u_y=",u_y,";");
	ofile.writeln("pos_x=",pos_x,";");
	ofile.writeln("vel_x=",vel_x,";");
	ofile.writeln("acc_x=",acc_x,";");
	ofile.writeln("pos_y=",pos_y,";");
	ofile.writeln("vel_y=",vel_y,";");
	ofile.writeln("acc_y=",acc_y,";");
	ofile.writeln("pos_x_front_LB=",pos_x_front_LB,";");
	ofile.writeln("pos_y_front_LB=",pos_y_front_LB,";");
	ofile.writeln("pos_x_front_UB=",pos_x_front_UB,";");
	ofile.writeln("pos_y_front_UB=",pos_y_front_UB,";");
	ofile.writeln("region_change_not_allowed_x_positive=",region_change_not_allowed_x_positive,";");
	ofile.writeln("region_change_not_allowed_y_positive=",region_change_not_allowed_y_positive,";");
	ofile.writeln("region_change_not_allowed_x_negative=",region_change_not_allowed_x_negative,";");
	ofile.writeln("region_change_not_allowed_y_negative=",region_change_not_allowed_y_negative,";");
	ofile.writeln("region_change_not_allowed_combined=",region_change_not_allowed_combined,";");
	ofile.writeln("notWithinEnvironmentRear=",notWithinEnvironmentRear,";");
	ofile.writeln("notWithinEnvironmentFrontUbUb=",notWithinEnvironmentFrontUbUb,";");
	ofile.writeln("notWithinEnvironmentFrontLbUb=",notWithinEnvironmentFrontLbUb,";");
	ofile.writeln("notWithinEnvironmentFrontUbLb=",notWithinEnvironmentFrontUbLb,";");
	ofile.writeln("notWithinEnvironmentFrontLbLb=",notWithinEnvironmentFrontLbLb,";");
	ofile.writeln("active_region=",active_region,";");
	ofile.writeln("deltacc=",deltacc,";");
	ofile.writeln("deltacc_front=",deltacc_front,";");
	ofile.writeln("car2car_collision=",car2car_collision,";");
	ofile.writeln("slackvars=",slackvars,";");  
	ofile.close();
}