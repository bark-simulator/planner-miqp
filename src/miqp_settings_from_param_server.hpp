// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_SETTINGS_FROM_PARAMETER_SERVER_HPP_
#define MIQP_SETTINGS_FROM_PARAMETER_SERVER_HPP_

#include "bark/commons/params/params.hpp"
#include "src/miqp_planner_data.hpp"

namespace bark{
namespace models {
namespace behavior {

miqp::planner::Settings MiqpSettingsFromParamServer(const bark::commons::ParamsPtr& params) {
    miqp::planner::Settings s = miqp::planner::Settings();
    
    s.nr_regions = params->GetInt("Miqp::NrRegions", "Number of Regions", 16);
    s.nr_steps = params->GetInt("Miqp::NrSteps", "Number of Steps", 20);
    s.nr_neighbouring_possible_regions = params->GetInt("Miqp::NrNeighbouringPossibleRegions", "Number of neibouring possible regions", 2);
    s.ts = params->GetReal("Miqp::Ts", "Time Step Increment", 0.25);
    s.max_solution_time = params->GetReal("Miqp::MaxSolutionTime", "Max Time after Solver Terminates", 10);
    s.relative_mip_gap_tolerance = params->GetReal("Miqp::RelativeMiqpGapTolerance", "Relative Gap Tolerance", 0.1);
    s.mipdisplay = params->GetInt("Miqp::mipdisplay", "Cplex Display Verbosity", 2);
    s.mipemphasis = params->GetInt("Miqp::mipemphasis", "Balance optimal vs. fast solutions", 0);
    s.relobjdif = params->GetReal("Miqp::relobjdif", "Igone integer solutions not better than x", 0.0);
    s.cutpass = params->GetInt("Miqp::cutpass", "Nr cutting planes", 0);
    s.probe = params->GetInt("Miqp::probe", "Cplex probing", 0);
    s.repairtries = params->GetInt("Miqp::repairtries", "repair warmstart solutions", 0);
    s.rinsheur = params->GetInt("Miqp::rinsheur", "RINS heuristic", 0);
    s.varsel = params->GetInt("Miqp::varsel", "Select branching variable", 0);
    s.mircuts = params->GetInt("Miqp::mircuts", "Mixed integer rounding cuts", 0);
    s.precision = params->GetInt("Miqp::Precision", "Precision used to create cplex problem", 12);
    s.constant_agent_safety_distance_slack = params->GetReal("Miqp::ConstantAgentSafetyDistanceSlack", "constant agent safety distance slack", 3.0);
    s.minimum_region_change_speed = params->GetReal("Miqp::MinimumRegionChangeSpeed", "minimum region change speed", 2.0);
    s.lambda = params->GetReal("Miqp::Lambda", "cooperation factor", 0.5);
    s.wheelBase = params->GetReal("Miqp::WheelBase", "Wheel base", 2.8);
    s.collisionRadius = params->GetReal("Miqp::CollisionRadius", "Collision Radius", 1.0);
    s.slackWeight = params->GetReal("Miqp::SlackWeight", "Weight of slack", 30.0);
    s.slackWeightObstacle = params->GetReal("Miqp::SlackWeightObstacle", "Weight of slack for obstacles", 2000.0);
    s.jerkWeight = params->GetReal("Miqp::JerkWeight", "Weight of jerk costs", 0.5);
    s.positionWeight = params->GetReal("Miqp::PositionWeight", "Weight of position costs", 2.0);
    s.velocityWeight = params->GetReal("Miqp::VelocityWeight", "Weight of velocity costs", 0.0);
    s.acclerationWeight = params->GetReal("Miqp::AccelerationWeight", "Weight of acceleration costs", 0.0);
    s.accLonMaxLimit = params->GetReal("Miqp::AccLonMaxLimit", "Limit on AccLonMax (positive)", 2.0);
    s.accLonMinLimit = params->GetReal("Miqp::AccLonMinLimit", "Limit on AccLonMin (negative)", -4.0);
    s.jerkLonMaxLimit = params->GetReal("Miqp::JerkLonMaxLimit", "Limit on JerkLonMax", 3.0);
    s.accLatMinMaxLimit = params->GetReal("Miqp::AccLatMinMaxLimit", "Limit on AccLatMinMax", 1.6);
    s.jerkLatMinMaxLimit = params->GetReal("Miqp::JerkLatMinMaxLimit", "Limit on JerkLatMinMax", 1.4);
    s.simplificationDistanceMap = params->GetReal("Miqp::SimplificationDistanceMap", "Map simplification distance", 0.2);
    s.simplificationDistanceReferenceLine = params->GetReal("Miqp::SimplificationDistanceReferenceLine", "Simplification Distance Reference line from apollo", 0.05);
    s.bufferReference = params->GetReal("Miqp::BufferReference", "Buffer distance Reference Intersection with Environment", 2.0);
    s.buffer_for_merging_tolerance = params->GetReal("Miqp::BufferForMergingTolerance", "Buffer serving as merging tolerance", 0.1);
    s.refLineInterpInc = params->GetReal("Miqp::RefLineInterpInc", "Reference line interpolation increment", 0.2);
    s.additionalStepsForReferenceLongerHorizon = params->GetInt("Miqp::AdditionalStepsForReferenceLongerHorizon", "additional steps for reference to intersect environment", 4);
    s.cplexModelpath = "../cplex_models/"; // cmp.c_str();
    s.useSos = params->GetBool("Miqp::UseSpecialOrderedSet", "", false);
    s.useBranchingPriorities = params->GetBool("Miqp::UseBranchingPriorities", "", false);
    int param_warmstart = params->GetInt("Miqp::WarmstartType", "", static_cast<int>(MiqpPlannerWarmstartType::NO_WARMSTART));
    s.warmstartType = static_cast<MiqpPlannerWarmstartType>(param_warmstart);
    int param_parallel_mode = params->GetInt("Miqp::ParallelMode", "", static_cast<int>(MiqpPlannerParallelMode::AUTO));
    s.parallelMode = static_cast<MiqpPlannerParallelMode>(param_parallel_mode);
    s.max_velocity_fitting = params->GetReal("Miqp::MaxVelocityFitting", "Velocity the fit was performed", 20.0);
    s.buffer_cplex_outputs = params->GetBool("Miqp::BufferCplexOutputs", "", false);
    s.obstacle_roi_filter = params->GetBool("Miqp::ObstacleRoiFilter", "", false);;
    s.obstacle_roi_behind_distance = params->GetReal("Miqp::ObstacleRoiBehindDistance", "", 5.0);
    s.obstacle_roi_front_distance = params->GetReal("Miqp::ObstacleRoiFrontDistance", "", 30.0);
    s.obstacle_roi_side_distance = params->GetReal("Miqp::ObstacleRoiSideDistance", "", 15.0);
    return s;
}


} // namespace behavior
} // namespace models
} // namespace bark

#endif // MIQP_SETTINGS_FROM_PARAMETER_SERVER_HPP_