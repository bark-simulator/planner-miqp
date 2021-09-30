// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

include "parameters.mod";

execute PARAMS {
  cplex.tilim = max_solution_time; 
  cplex.epgap = relative_mip_gap_tolerance;
  cplex.mipdisplay = mipdisplay;
  cplex.mipemphasis = mipemphasis;
  cplex.relobjdif = relobjdif; 
  cplex.cutpass = cutpass;
  cplex.probe = probe;
  cplex.repairtries = repairtries;
  cplex.rinsheur = rinsheur; 
  cplex.varsel = varsel;
  cplex.mircuts = mircuts;
  cplex.parallelmode = parallelmode;
  //cplex.intsollim = 2; //stop after the nth integer solution

  // Parameters to experiment with:

  // cplex.mipkappastats = 2; //report singularities in the solutions
  // cplex.numericalemphasis = 0; // set to 1 to debug precision problems etc.
  // cplex.eprhs = 1e-6; //feasibility tolerance
  // cplex.epopt = 1e-6; //optimality tolerance
  // cplex.epint = 1e-6; //Specifies the amount by which an integer variable can be different from an integer and still be considered feasible. 
  // cplex.eprhs = 1e-6; //the degree to which values of the basic variables calculated may violate their bounds.

  // Specify types of cuts: 
  // https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/cuts/41_params.html?view=kc#User_manual.uss_solveMIP.672903__User_manual.uss_solveMIP.679513
  //cplex.mircuts = -1;
  //cplex.fraccuts = 1;
  //cplex.implbd = 1;
  //cplex.cliques = 1;
  // cplex.localimplied = -1; //does not work?????
  
  // Too different for different problems: fix solution algorithms
  // https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/troubleshoot/63_unsat_subprob.html#descriptiveTopic1197481289949__User_manual.uss_solveMIP.647582
  //cplex.rootalg = 3; //4: no benefits, 3:benefits in some cases
  //cplex.nodealg = 0; //other than 0: always slow

  // Did not help, speed up the presolve
  //cplex.heurfreq = -1; //proposed by the tuning tool //https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/troubleshoot/55_time_node_zero.html
  //cplex.varsel = 4; //https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/troubleshoot/55_time_node_zero.html

  //cplex.polishafterintsol = 99999; //solution polishing from optimized solution -> tends to find bad final results.
  //cplex.polishaftertime = 0; //solution polishing of an initial solution -> tends to find bad final results.
  //cplex.advind = 1;
}

include "decision_variables.mod";

// for multi-objective
//using CP;

// Measure Time
float temp_time;
execute
{
	var before = new Date();
	temp_time = before.getTime();
}

include "initialization.mod";
include "objective_function.mod";
include "initial_conditions.mod";
include "model_region_constraints.mod";
include "minimum_speed_constraints.mod";
include "obstacle_environment_constraints.mod";
include "agent_collision_constraints.mod";

//measure time
execute
{
	var after = new Date();
	writeln("solving time [ms] ~= ",after.getTime()-temp_time);
}

include "write_results.mod";


