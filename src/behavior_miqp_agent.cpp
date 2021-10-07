// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_miqp_agent.hpp"
#include "common/dynamic/dynamic_conversion.hpp"
#include "src/miqp_settings_from_param_server.hpp"

#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::geometry::SignedAngleDiff;
using bark::models::dynamic::StateDefinition;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::map::LaneCorridorPtr;
using bark::world::objects::AgentId;
using miqp::planner::InitialStateIndices;
using miqp::planner::MiqpPlanner;
using miqp::planner::cplex::CplexWrapper;
namespace bg = boost::geometry;

BehaviorMiqpAgent::BehaviorMiqpAgent(const commons::ParamsPtr& params)
    : BehaviorModel(params),
      settings_(MiqpSettingsFromParamServer(params)),
      planner_(MiqpPlanner(settings_)),
      deltaSDesiredVelocity_(params->GetReal(
          "Miqp::DeltaSDesiredVelocity",
          "meters in which desired velocity will be used", 5.0)),
      currentSolution_(),
      ref_trajectories_(),
      ref_trajectories_longer_horizon_(),
      convex_shrinked_env_polygons_all_cars_(),
      last_cplex_wrapper_(),
      last_axy_all_cars_(),
      use_box_as_env_(params->GetBool("Miqp::UseBoxAsEnv",
                                      "Use Box as Env Polygon", false)),
      write_debug_files_(params->GetBool("Miqp::WriteDebugFiles",
                                         "Activate Debug File Output", false)),
      multi_agent_planning_(params->GetBool(
          "Miqp::MultiAgentPlanning", "Activate multi agent planning", false)),
      rule_no_right_passing_(params->GetBool(
          "Miqp::RuleNoRightPassing", "Activate rule no right passing", false)),
      rule_no_right_passing_is_soft_(
          params->GetBool("Miqp::RuleNoRightPassingIsSoft",
                          "Treat rule no right passing as soft", false)),
      rule_safe_distance_(params->GetBool(
          "Miqp::RuleSafeDistance", "Activate rule safe distance", false)),
      rule_safe_distance_is_soft_(
          params->GetBool("Miqp::RuleSafeDistanceIsSoft",
                          "Treat rule safe distance as soft", false)),
      choose_ego_model_joint_prediction_(
          params->GetInt("Miqp::ChooseEgoModelJointPrediction",
                         "Choose Model for Ego: IDM (0), ConstVel (1)", 0)),
      last_planning_success_(false),
      debug_file_path_(params->GetString("Miqp::DebugFilePath", "", "")),
      debug_file_prefix_(params->GetString("Miqp::DebugFilePrefix", "", "")),
      envPoly_(),
      pose_out_of_map_(100, 0, 0),
      firstrun_(true),
      idx_ego_(-1),  // invalid
      car_idxs_(),
      obstacle_ids_(),
      last_dyn_occupancies_(),
      warmstart_type_(settings_.warmstartType),
      last_lane_corridor_(nullptr),
      do_no_change_lane_corridor_(params->GetBool(
          "Miqp::DoNotChangeLaneCorridor",
          "Stay in the lane corridor selected in the first step", true)),
      prediction_error_time_percentage_(
          params->GetReal("Miqp::PredictionErrorTimePercentage", "", 0.0)),
      obstacles_soft_(params->GetBool(
          "Miqp::ObstaclesSoft",
          "true: soft obstacle constraints, false: hard obstacle constraints",
          true)) {
  Input input = Input(2);
  input << 0.0, 0.0;
  SetLastAction(input);

  if (write_debug_files_) {
    planner_.ActivateDebugFileWrite(debug_file_path_, debug_file_prefix_);
  }
  // SetWarmstartType(warmstart_type_);  // not needed, as already set in the
  // constructor of the MiqpPlanner class
}

BehaviorMiqpAgent::BehaviorMiqpAgent(const BehaviorMiqpAgent& bm)
    : BehaviorModel(bm),
      settings_(bm.settings_),
      planner_(bm.planner_),
      deltaSDesiredVelocity_(bm.deltaSDesiredVelocity_),
      currentSolution_(bm.currentSolution_),
      ref_trajectories_(bm.ref_trajectories_),
      ref_trajectories_longer_horizon_(bm.ref_trajectories_longer_horizon_),
      convex_shrinked_env_polygons_all_cars_(
          bm.convex_shrinked_env_polygons_all_cars_),
      last_cplex_wrapper_(bm.last_cplex_wrapper_),
      last_axy_all_cars_(bm.last_axy_all_cars_),
      use_box_as_env_(bm.use_box_as_env_),
      write_debug_files_(bm.write_debug_files_),
      multi_agent_planning_(bm.multi_agent_planning_),
      rule_no_right_passing_(bm.rule_no_right_passing_),
      rule_no_right_passing_is_soft_(bm.rule_no_right_passing_is_soft_),
      rule_safe_distance_(bm.rule_safe_distance_),
      last_planning_success_(bm.last_planning_success_),
      rule_safe_distance_is_soft_(bm.rule_safe_distance_is_soft_),
      choose_ego_model_joint_prediction_(bm.choose_ego_model_joint_prediction_),
      debug_file_path_(bm.debug_file_path_),
      debug_file_prefix_(bm.debug_file_prefix_),
      envPoly_(bm.envPoly_),
      pose_out_of_map_(bm.pose_out_of_map_),
      firstrun_(bm.firstrun_),
      idx_ego_(bm.idx_ego_),
      car_idxs_(bm.car_idxs_),
      obstacle_ids_(bm.obstacle_ids_),
      last_dyn_occupancies_(bm.last_dyn_occupancies_),
      warmstart_type_(bm.warmstart_type_),
      last_lane_corridor_(bm.last_lane_corridor_),
      do_no_change_lane_corridor_(bm.do_no_change_lane_corridor_),
      obstacles_soft_(bm.obstacles_soft_) {}

dynamic::Trajectory BehaviorMiqpAgent::Plan(
    double delta_time, const world::ObservedWorld& observed_world) {
  if (std::fmod(delta_time, settings_.ts) != 0) {
    LOG(ERROR) << "Use different step size!!! (" << delta_time << "/ "
               << settings_.ts << ")";
  }
  const double current_time = observed_world.GetWorldTime();

  // Check if overall environment polygon changed and update planner
  if (CalculateEnvironmentPolygon(observed_world)) {
    planner_.UpdateConvexifiedMap(envPoly_);
  }

  // Warmstart
  if (warmstart_type_ != planner_.GetDoWarmstart()) {
    planner_.SetDoWarmstart(warmstart_type_);
  }

  CheckPoseOutOfMap();

  const double desiredVelocity = GetParams()->GetReal(
      "Miqp::DesiredVelocity", "Desired Velocity Ego", 10.0);

  Eigen::MatrixXd initialState(
      1, static_cast<int>(InitialStateIndices::MIQP_INITIAL_STATE_SIZE));
  bool track_reference_positions = true;  // TODO: move?
  Line ref_line;
  double tmp_desiredVelocity = desiredVelocity;
  ProcessBarkAgent(observed_world, initialState, ref_line, tmp_desiredVelocity,
                   true);

  if (firstrun_) {
    idx_ego_ = planner_.AddCar(initialState, ref_line, desiredVelocity,
                               deltaSDesiredVelocity_, current_time,
                               track_reference_positions);
    LOG(INFO) << "Setting desired velocity for MIQP ego agent to "
            << desiredVelocity;
    firstrun_ = false;
    car_idxs_.insert(std::make_pair(observed_world.GetEgoAgentId(), idx_ego_));
  } else {
    planner_.UpdateCar(idx_ego_, initialState, ref_line, current_time,
                       track_reference_positions);
  }
  reference_lines_.clear();
  reference_lines_.push_back(ref_line);

  if (multi_agent_planning_) {
    // Remove cars from previous step, reverse order as deleting intermediate
    // cars is not implemented in the miqp planner class
    for (auto it_cars = car_idxs_.end(); it_cars != car_idxs_.begin();
         --it_cars) {
      if (it_cars->second != idx_ego_) {
        planner_.RemoveCar(it_cars->second);
      }
    }
    // Add new cars
    for (const auto& agent_j : observed_world.GetOtherAgents()) {
      ObservedWorldPtr observed_world_j =
          observed_world.ObserveForOtherAgent(agent_j.first);  // clones world

      Eigen::MatrixXd initial_state_j(
          1, static_cast<int>(InitialStateIndices::MIQP_INITIAL_STATE_SIZE));
      Line ref_line_j;
      double desiredVelocity;
      ProcessBarkAgent(*observed_world_j, initial_state_j, ref_line_j,
                       desiredVelocity, false);
      if (fabs(prediction_error_time_percentage_) > 0.01) {
        desiredVelocity *= prediction_error_time_percentage_;
      }
      int idx_other = planner_.AddCar(initial_state_j, ref_line_j,
                                      desiredVelocity, current_time);
      car_idxs_.insert(std::make_pair(agent_j.first, idx_other));
      reference_lines_.push_back(ref_line_j);
    }
    // last_dyn_occupancies_ can be used to plot future simulated behavior
    // last_dyn_occupancies_ = CollectDynOccupancies(observed_world);
  } else {
    // Remove Obstacles that are not present any more
    std::vector<DynamicOccupancyPtr> dyn_occ =
        CollectDynOccupancies(observed_world);
    last_dyn_occupancies_ = dyn_occ;
    for (auto it_obs = obstacle_ids_.begin(); it_obs != obstacle_ids_.end();
         ++it_obs) {
      bool present = false;
      for (auto it_ag = dyn_occ.begin(); it_ag != dyn_occ.end(); it_ag++) {
        auto dyno_hash =
            DynamicOccupancy::GetHash((*it_ag)->id, (*it_ag)->type);
        if (it_obs->first == dyno_hash) {
          present = true;
        }
      }
      if (!present) {
        planner_.RemoveObstacle(it_obs->second);
      }
    }

    // Add or update obstacles for dynamic_predictions
    for (auto it = dyn_occ.begin(); it != dyn_occ.end(); it++) {
      // get hash
      std::size_t dyno_hash = DynamicOccupancy::GetHash((*it)->id, (*it)->type);
      Trajectory prediction = (*it)->prediction;
      if (fabs(prediction_error_time_percentage_) > 0.01) {
        // LOG(INFO) << "True Other Prediction\n" << prediction;
        IntroducePredictionError(prediction);
        // LOG(INFO) << "Prediction with error\n" << prediction;
        (*it)->prediction = prediction;  // for visualization
      }
      if (obstacle_ids_.count(dyno_hash) == 0) {
        // obstacle needs to be added
        int added_miqp_id = planner_.AddObstacle(prediction, (*it)->shape,
                                                 (*it)->is_soft, false);
        obstacle_ids_[dyno_hash] = added_miqp_id;
      } else {
        // obstacle is present -> update traj
        planner_.UpdateObstacle(obstacle_ids_[dyno_hash], prediction,
                                (*it)->shape);
      }
    }
  }

  // Planning Loop
  last_planning_success_ = planner_.Plan(current_time);
  if (!last_planning_success_) {
    LOG(ERROR) << "Plan Function of MiqpPlanner did not succeed at "
               << current_time;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    SetLastSolutionTime(nan(""));
    return GetLastTrajectory();
  }

  std::get<0>(currentSolution_) = current_time;
  std::get<1>(currentSolution_) = idx_ego_;
  std::get<2>(currentSolution_) = planner_.GetSolution();
  std::get<3>(currentSolution_) = planner_.GetSolutionProperties();

  dynamic::Trajectory traj =
      planner_.GetBarkTrajectory(idx_ego_, observed_world.GetWorldTime());

  // Calculating inputs to bicycle model
  Eigen::MatrixXd planned_state =
      planner_.Get2ndOrderStateFromSolution(1, idx_ego_);
  double planned_vel =
      sqrt(pow(planned_state(1), 2) + pow(planned_state(4), 2));
  double current_vel =
      observed_world.CurrentEgoState()(StateDefinition::VEL_POSITION);
  double acc = (planned_vel - current_vel) / delta_time;
  double theta_dot = SignedAngleDiff(traj(0, StateDefinition::THETA_POSITION),
                                     traj(1, StateDefinition::THETA_POSITION)) /
                     delta_time;
  double delta = atan2(theta_dot * settings_.wheelBase, current_vel);
  Input input(2);
  input << acc, delta;  // saving also steering angle

  // saving references
  ref_trajectories_ = planner_.CalculateReferenceTrajectories();
  ref_trajectories_longer_horizon_ =
      planner_.CalculateReferenceTrajectoriesLongerHorizon();

  // saving convexified environmental polygon
  convex_shrinked_env_polygons_all_cars_ =
      planner_.GetConvexShrinkedEnvPolygonsAllCars();

  // save last cplexwrapper
  last_cplex_wrapper_ = planner_.GetCplexWrapper();

  // save solutions of all cars
  last_trajectories_all_cars_.clear();
  last_axy_all_cars_.clear();
  for (auto it_cars = car_idxs_.begin(); it_cars != car_idxs_.end();
       ++it_cars) {
    dynamic::Trajectory t = planner_.GetBarkTrajectory(
        it_cars->second, observed_world.GetWorldTime());
    last_trajectories_all_cars_.push_back(t);

    Eigen::MatrixXd planned_state =
        planner_.Get2ndOrderStateFromSolution(1, it_cars->second);
    double ax = planned_state(2);
    double ay = planned_state(5);
    last_axy_all_cars_.insert(
        std::make_pair(it_cars->first, std::make_pair(ax, ay)));
  }

  SetLastSolutionTime(planner_.GetSolutionProperties().time);
  LOG(INFO) << "Solution time: " << GetLastSolutionTime();

  SetLastTrajectory(traj);
  SetLastAction(input);
  return traj;
}

LaneCorridorPtr BehaviorMiqpAgent::ChooseLaneCorridor(
    const ObservedWorld& observed_world, const double prediction_speed) const {
  // Ego Car as Agent
  auto ego_pos = observed_world.CurrentEgoPosition();
  auto road_corr = observed_world.GetRoadCorridor();
  // get adjacent corridors
  LaneCorridorPtr corr_left, corr_right;
  std::tie(corr_left, corr_right) =
      road_corr->GetLeftRightLaneCorridor(ego_pos);
  LaneCorridorPtr corr_curr = observed_world.GetLaneCorridor();

  // default: we stay in current lane corridor
  LaneCorridorPtr target_corr = corr_curr;

  // coose rightmost lane
  // TODO bug: somehow left and right is confused on some maps when parsing in
  // bark if (corr_right) {
  //   LOG(INFO) << "Right corridor available";
  //   if (target_corr != corr_right) {
  //     target_corr = corr_right;
  //     LOG(INFO) << "A more right lane corridor is available, choosing this
  //     one by default";
  //   }
  // }
  // TODO: delete this piece of code once the above bug is fixed
  // if (corr_left) {
  //   LOG(INFO) << "Left corridor available";
  //   if (target_corr != corr_left) {
  //     target_corr = corr_left;
  //     LOG(INFO) << "A more right lane corridor is available, choosing this
  //     one by default";
  //   }
  // }

  // Lane ends
  float estimated_travel_dist =
      prediction_speed * settings_.ts * settings_.nr_steps;
  float curr_length_until_end = corr_curr->LengthUntilEnd(ego_pos);
  const float dist_tolerance = 2.0;
  if (curr_length_until_end < estimated_travel_dist) {
    LOG(INFO) << "there is not much left, checking switching to another lane";
    float right_length_until_end = 0.0, left_length_until_end = 0.0;
    if (corr_right) {
      right_length_until_end = corr_right->LengthUntilEnd(ego_pos);
      if (right_length_until_end > curr_length_until_end + dist_tolerance) {
        // use corr_right
        target_corr = corr_right;
        LOG(INFO) << "Choosing right corridor as reference";
      }
    }
    if (corr_left) {
      left_length_until_end = corr_left->LengthUntilEnd(ego_pos);
      if (left_length_until_end > curr_length_until_end + dist_tolerance &&
          left_length_until_end > right_length_until_end + dist_tolerance) {
        // use corr_left
        target_corr = corr_left;
        LOG(INFO) << "Choosing left corridor as reference";
      }
    }
  } else {
    LOG(INFO) << "no reason to change lanes yet due to estimated_travel_dist "
              << estimated_travel_dist << " curr_length_until_end "
              << curr_length_until_end;
  }

  return target_corr;
}

void BehaviorMiqpAgent::ApplyEgoModelForJointPrediction(
    ObservedWorldPtr& observed_world) const {
  auto params =
      std::make_shared<commons::SetterParams>();  // we could use param server
                                                  // from constructor
  // overwriting ego behavior for joint prediction
  BehaviorModelPtr pred_ego_behavior;
  if (choose_ego_model_joint_prediction_ == 0) {
    LOG(INFO) << "Ego Model for prediction: BehaviorIDMClassic";
    const double desiredVelocity = GetParams()->GetReal(
        "Miqp::DesiredVelocity", "Desired Velocity Ego", 10.0);
    params->SetReal("BehaviorIDMClassic::DesiredVelocity", desiredVelocity);
    pred_ego_behavior = std::make_shared<BehaviorIDMClassic>(params);
  } else if (choose_ego_model_joint_prediction_ == 1) {
    LOG(INFO) << "Ego Model for prediction: BehaviorConstantAcceleration";
    pred_ego_behavior = std::make_shared<BehaviorConstantAcceleration>(params);
  } else {
    LOG(ERROR) << "Ego Model for prediction is ill-selected";
  }
  observed_world->SetBehaviorModel(observed_world->GetEgoAgentId(),
                                   pred_ego_behavior);
}

std::map<AgentId, DynamicOccupancyPtr>
BehaviorMiqpAgent::PredictAgentsAsDynamicObstacles(
    ObservedWorldPtr& observed_world) const {
  ApplyEgoModelForJointPrediction(observed_world);

  std::map<AgentId, DynamicOccupancyPtr> dyno_map;
  for (const auto& agent_j : observed_world->GetOtherAgents()) {
    DynamicOccupancy dyn_pred(settings_.nr_steps, agent_j.first,
                              agent_j.second->GetShape(),
                              OccupancyType::PREDICTION, obstacles_soft_);
    dyno_map.insert(std::pair<AgentId, DynamicOccupancyPtr>(
        agent_j.first, std::make_shared<DynamicOccupancy>(dyn_pred)));
  }

  for (int i = 0; i < settings_.nr_steps; ++i) {
    for (const auto& agent_j : observed_world->GetOtherAgents()) {
      if (agent_j.second->GetBehaviorStatus() != BehaviorStatus::VALID ||
          agent_j.second->IsValidAtTime(observed_world->GetWorldTime()) ==
              false) {
        dyno_map.at(agent_j.first)->prediction.row(i)
            << observed_world->GetWorldTime(),
            pose_out_of_map_(0), pose_out_of_map_(1), pose_out_of_map_(2), 0;
      } else {
        dyno_map.at(agent_j.first)->prediction.row(i) =
            agent_j.second->GetCurrentState();
      }
    }
    // step world after adding states, so that we include state at t=0 into
    // prediction
    observed_world->Step(settings_.ts);
  }
  return dyno_map;
}

std::map<AgentId, DynamicOccupancyPtr>
BehaviorMiqpAgent::PredictAgentsForNoRightDriving(
    ObservedWorldPtr& observed_world) const {
  ApplyEgoModelForJointPrediction(observed_world);

  std::map<AgentId, DynamicOccupancyPtr> dyno_map;
  for (const auto& agent_j : observed_world->GetOtherAgents()) {
    DynamicOccupancy dyn_pred(
        settings_.nr_steps, agent_j.first, agent_j.second->GetShape(),
        OccupancyType::RULE_NO_RIGHT_PASSING, rule_no_right_passing_is_soft_);
    dyno_map.insert(std::pair<AgentId, DynamicOccupancyPtr>(
        agent_j.first, std::make_shared<DynamicOccupancy>(dyn_pred)));
  }

  for (int i = 0; i < settings_.nr_steps; ++i) {
    for (const auto& agent_j : observed_world->GetOtherAgents()) {
      if (agent_j.second->GetBehaviorStatus() != BehaviorStatus::VALID ||
          agent_j.second->IsValidAtTime(observed_world->GetWorldTime()) ==
              false) {
        dyno_map.at(agent_j.first)->prediction.row(i)
            << observed_world->GetWorldTime(),
            pose_out_of_map_(0), pose_out_of_map_(1), pose_out_of_map_(2), 0;
      } else {
        Point2d ego_pos = agent_j.second->GetCurrentPosition();
        LaneCorridorPtr left_lc, right_lc;
        std::tie(left_lc, right_lc) =
            agent_j.second->GetRoadCorridor()->GetLeftRightLaneCorridor(
                ego_pos);
        if (right_lc) {
          LOG(INFO) << "Right area of obstacle from agent id " << agent_j.first
                    << " will be forbidden";
          // calculate projected values on s-coordinate
          double s = GetNearestS(right_lc->GetCenterLine(), ego_pos);
          Point2d point = GetPointAtS(right_lc->GetCenterLine(), s);
          double angle = GetTangentAngleAtS(right_lc->GetCenterLine(), s);
          dyno_map.at(agent_j.first)->prediction.row(i)
              << observed_world->GetWorldTime(),
              bg::get<0>(point), bg::get<1>(point), angle, 0.0;
        } else {
          dyno_map.at(agent_j.first)->prediction.row(i) =
              agent_j.second->GetCurrentState();
        }
      }
    }
    // step world after adding states, so that we include state at t=0 into
    // prediction
    observed_world->Step(settings_.ts);
  }
  return dyno_map;
}

std::map<AgentId, DynamicOccupancyPtr>
BehaviorMiqpAgent::PredictAgentsForSafeDistance(
    ObservedWorldPtr& observed_world) const {
  ApplyEgoModelForJointPrediction(observed_world);

  std::map<AgentId, DynamicOccupancyPtr> dyno_map;
  for (const auto& agent_j : observed_world->GetOtherAgents()) {
    DynamicOccupancy dyn_pred(
        settings_.nr_steps, agent_j.first, agent_j.second->GetShape(),
        OccupancyType::RULE_SAFE_DISTANCE, rule_safe_distance_is_soft_);
    dyno_map.insert(std::pair<AgentId, DynamicOccupancyPtr>(
        agent_j.first, std::make_shared<DynamicOccupancy>(dyn_pred)));
  }

  const auto CalcSafeDistance = [](double v_r, double v_f, double a_r,
                                   double a_f, double delta) {
    return v_r * delta - pow(v_r, 2) / (2.0 * a_r) + pow(v_f, 2) / (2.0 * a_f);
  };

  for (int i = 0; i < settings_.nr_steps; ++i) {
    for (const auto& agent_j : observed_world->GetOtherAgents()) {
      if (agent_j.second->GetBehaviorStatus() != BehaviorStatus::VALID ||
          agent_j.second->IsValidAtTime(observed_world->GetWorldTime()) ==
              false) {
        dyno_map.at(agent_j.first)->prediction.row(i)
            << observed_world->GetWorldTime(),
            pose_out_of_map_(0), pose_out_of_map_(1), pose_out_of_map_(2), 0;
      } else {
        auto agent_in_front = observed_world->GetAgentInFront();
        if (agent_in_front.first &&
            agent_j.first == agent_in_front.first->GetAgentId()) {
          VLOG(5) << "Preceding agent " << agent_j.first;
          double v_r, v_f;
          v_r =
              observed_world->CurrentEgoState()[StateDefinition::VEL_POSITION];
          v_f =
              agent_j.second->GetCurrentState()[StateDefinition::VEL_POSITION];
          double sd = CalcSafeDistance(v_r, v_f, -5.0, -5.0, 1.0);
          // implement rectangle with center point at front point of vehicle
          Line center_line = observed_world->GetLaneCorridor()->GetCenterLine();
          double s_o =
              GetNearestS(center_line, agent_j.second->GetCurrentPosition());
          double s = s_o - sd;
          Point2d point = GetPointAtS(center_line, s);
          double angle = GetTangentAngleAtS(center_line, s);
          dyno_map.at(agent_j.first)->prediction.row(i)
              << observed_world->GetWorldTime(),
              bg::get<0>(point), bg::get<1>(point), angle, 0.0;
          VLOG(5) << "at i: "
                  << "v_r=" << v_r << ", v_f=" << v_f << ", sd=" << sd
                  << ", s=" << s << " adding as sd: "
                  << dyno_map.at(agent_j.first)->prediction.row(i);
        } else {
          VLOG(5) << "agent " << agent_j.first << " filling with default";
          dyno_map.at(agent_j.first)->prediction.row(i)
              << observed_world->GetWorldTime(),
              pose_out_of_map_(0), pose_out_of_map_(1), pose_out_of_map_(2), 0;
        }
      }
    }
    // step world after adding states, so that we include state at t=0 into
    // prediction
    observed_world->Step(settings_.ts);
  }
  return dyno_map;
}

std::vector<DynamicOccupancyPtr> BehaviorMiqpAgent::CollectDynOccupancies(
    const ObservedWorld& observed_world) const {
  // Other Cars as Obstacles
  ObservedWorldPtr predicted_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  auto occ_predictions = PredictAgentsAsDynamicObstacles(predicted_world);
  std::vector<DynamicOccupancyPtr> dyn_obst_all;
  for (auto it = occ_predictions.begin(); it != occ_predictions.end(); ++it) {
    dyn_obst_all.push_back(it->second);
  }

  if (rule_no_right_passing_) {
    ObservedWorldPtr predicted_world2 =
        std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
    auto occ_no_right_dr = PredictAgentsForNoRightDriving(predicted_world2);
    for (auto it = occ_no_right_dr.begin(); it != occ_no_right_dr.end(); ++it) {
      dyn_obst_all.push_back(it->second);
    }
  }
  if (rule_safe_distance_) {
    ObservedWorldPtr predicted_world2 =
        std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
    auto occ_sd = PredictAgentsForSafeDistance(predicted_world2);
    for (auto it = occ_sd.begin(); it != occ_sd.end(); ++it) {
      dyn_obst_all.push_back(it->second);
    }
  }

  return dyn_obst_all;
}

Eigen::MatrixXd BehaviorMiqpAgent::GetCollisionCircleCenters() const {
  Eigen::MatrixXd cc(5, 2);
  const int idx = std::get<1>(currentSolution_);
  const std::shared_ptr<miqp::planner::RawResults> rr =
      std::get<2>(currentSolution_);
  cc(0, 0) = rr->pos_x(idx, 0);
  cc(0, 1) = rr->pos_y(idx, 0);
  cc(1, 0) = rr->pos_x_front_LB(idx, 0);
  cc(1, 1) = rr->pos_y_front_LB(idx, 0);
  cc(2, 0) = rr->pos_x_front_LB(idx, 0);
  cc(2, 1) = rr->pos_y_front_UB(idx, 0);
  cc(3, 0) = rr->pos_x_front_UB(idx, 0);
  cc(3, 1) = rr->pos_y_front_LB(idx, 0);
  cc(4, 0) = rr->pos_x_front_UB(idx, 0);
  cc(4, 1) = rr->pos_y_front_UB(idx, 0);
  return cc;
}

Eigen::MatrixXd BehaviorMiqpAgent::GetFrontLbUb() const {
  const int idx = std::get<1>(currentSolution_);
  const std::shared_ptr<miqp::planner::RawResults> rr =
      std::get<2>(currentSolution_);
  const int N = rr->N;
  Eigen::MatrixXd cc(4, N);
  for (int i = 0; i < N; ++i) {  // nicer: use Tensor slices
    cc(0, i) = rr->pos_x_front_LB(idx, i);
    cc(1, i) = rr->pos_y_front_LB(idx, i);
    cc(2, i) = rr->pos_x_front_UB(idx, i);
    cc(3, i) = rr->pos_y_front_UB(idx, i);
  }
  return cc;
}

bool BehaviorMiqpAgent::CalculateEnvironmentPolygon(
    const world::ObservedWorld& observed_world) {
  namespace bg = boost::geometry;
  Polygon p;
  if (use_box_as_env_) {
    double box_offset_ = 20;
    bg::model::box<Point2d> box;
    Polygon input_polygon = observed_world.GetRoadCorridor()->GetPolygon();
    bg::envelope(input_polygon.obj_, box);

    double min_x = bg::get<bg::min_corner, 0>(box);
    double min_y = bg::get<bg::min_corner, 1>(box);
    double max_x = bg::get<bg::max_corner, 0>(box);
    double max_y = bg::get<bg::max_corner, 1>(box);

    p.AddPoint(Point2d(min_x - box_offset_, min_y - box_offset_));
    p.AddPoint(Point2d(max_x + box_offset_, min_y - box_offset_));
    p.AddPoint(Point2d(max_x + box_offset_, max_y + box_offset_));
    p.AddPoint(Point2d(min_x - box_offset_, max_y + box_offset_));
    p.AddPoint(Point2d(min_x - box_offset_, min_y - box_offset_));

    bg::correct(p.obj_);
  } else {
    // for now, we only use the road corridor of the ego vehicle
    p = observed_world.GetRoadCorridor()->GetPolygon();
  }
  if (Equals(p, envPoly_)) {
    return false;
  } else {
    envPoly_ = p;
    return true;
  }
}

void BehaviorMiqpAgent::CheckPoseOutOfMap() const {
  Polygon poly_out_of_map(
      Pose(0, 0, 0),
      std::vector<Point2d>{Point2d(-5, -5), Point2d(-5, 5), Point2d(5, 5),
                           Point2d(5, -5), Point2d(-5, -5)});
  poly_out_of_map.Transform(pose_out_of_map_);
  if (bark::geometry::Within(poly_out_of_map, envPoly_)) {
    LOG(FATAL) << "pose_out_of_map_ is colliding with road, this might yield "
                  "invalid occupancies";
  }
}

void BehaviorMiqpAgent::ProcessBarkAgent(const ObservedWorld observed_world,
                                         Eigen::MatrixXd& initialState,
                                         Line& refLine, double& desiredVelocity,
                                         bool isEgo) {
  // make sure ego vehicle is initially within map
  const auto state = observed_world.CurrentEgoState();
  Polygon occupancyEgo = observed_world.GetEgoAgent()->GetPolygonFromState(
      observed_world.CurrentEgoState());
  if (!bark::geometry::Within(observed_world.CurrentEgoPosition(), envPoly_)) {
    LOG(FATAL) << "Ego position is initially not within road";
  } else if (!bark::geometry::Within(occupancyEgo, envPoly_)) {
    LOG(INFO) << "Ego occupancy is initially not within road";
    LOG(INFO) << "envpoly = [" << envPoly_.ToArray() << "]";
    LOG(INFO) << "egopoly = [" << occupancyEgo.ToArray() << "]";
  }

  if (!isEgo) {
    const Trajectory last_traj =
        observed_world.GetEgoAgent()->GetBehaviorTrajectory();
    if (last_traj.rows() > 0) {
      const double last_traj_vel_end =
          last_traj(last_traj.rows() - 1, StateDefinition::VEL_POSITION);
      desiredVelocity = last_traj_vel_end;
      LOG(INFO) << "Setting agent target speed to last trajectory end speed = "
                << desiredVelocity << " for agent id "
                << observed_world.GetEgoAgent()->GetAgentId();
    } else {
      desiredVelocity = state(StateDefinition::VEL_POSITION);
      LOG(INFO) << "Setting agent target speed to current speed = "
                << desiredVelocity << " for agent id "
                << observed_world.GetEgoAgent()->GetAgentId();
    }
  }

  double speed_for_lc_prediction =
      std::max(desiredVelocity, state(StateDefinition::VEL_POSITION));
  if (isEgo && do_no_change_lane_corridor_) {
    if (!last_lane_corridor_) {
      last_lane_corridor_ =
          ChooseLaneCorridor(observed_world, speed_for_lc_prediction);
    }
    refLine = last_lane_corridor_->GetFineCenterLine();
  } else {
    LaneCorridorPtr target_lc =
        ChooseLaneCorridor(observed_world, speed_for_lc_prediction);
    refLine = target_lc->GetFineCenterLine();
  }

  std::pair<double, double> axy;
  AgentId id = observed_world.GetEgoAgentId();
  if (last_axy_all_cars_.find(id) != last_axy_all_cars_.end()) {
    axy = last_axy_all_cars_.at(id);
  } else {
    axy = std::make_pair(0.0, 0.0);
  }
  initialState = miqp::common::dynamic::ConvertBarkStateTo2ndOrder(
      observed_world.CurrentEgoState(), axy.first, axy.second);
}

void BehaviorMiqpAgent::SetWarmstartType(
    miqp::planner::cplex::CplexWrapper::WarmstartType type) {
  warmstart_type_ = type;
  LOG(INFO) << "Warmstart type is now " << type;
}

void BehaviorMiqpAgent::IntroducePredictionError(Trajectory& prediction) {
  assert(prediction.rows() > 1);
  Trajectory out;
  const double new_dt = (prediction(1, StateDefinition::TIME_POSITION) -
                         prediction(0, StateDefinition::TIME_POSITION)) *
                        prediction_error_time_percentage_;
  double t = prediction(0, StateDefinition::TIME_POSITION);
  out = prediction;  // for first row
  // do not modify start point
  for (size_t idx = 1; idx < prediction.rows(); ++idx) {
    t += new_dt;
    // Find interpolation/extrapolation idx
    size_t idx_interp;
    for (idx_interp = 1; idx_interp < prediction.rows(); ++idx_interp) {
      if (prediction(idx_interp, StateDefinition::TIME_POSITION) >= t) {
        break;
      }
    }
    if (idx_interp <= prediction.rows() - 1) {
      // Interpolate
      const State p0 = prediction.row(idx_interp - 1);
      const State p1 = prediction.row(idx_interp);
      out.row(idx) = Interpolate(p0, p1, t);
    } else {
      // Extrapolate
      const State p0 = prediction.row(idx_interp - 1);
      out.row(idx) = Extrapolate(p0, t);
    }
  }
  prediction = out;
}

State BehaviorMiqpAgent::Interpolate(const State& p0, const State& p1,
                                     const double& time) const {
  const double start_time = p0(StateDefinition::TIME_POSITION);
  const double end_time = p1(StateDefinition::TIME_POSITION);
  const double lambda = fabs((time - start_time) / (end_time - start_time));
  return (1 - lambda) * p0 + (lambda)*p1;
}

State BehaviorMiqpAgent::Extrapolate(const State& p0,
                                     const double& time) const {
  State out(StateDefinition::MIN_STATE_SIZE);
  out(StateDefinition::TIME_POSITION) = time;
  const double dt = time - p0(StateDefinition::TIME_POSITION);
  out(StateDefinition::X_POSITION) = p0(StateDefinition::X_POSITION) +
                                     cos(p0(StateDefinition::THETA_POSITION)) *
                                         p0(StateDefinition::VEL_POSITION) * dt;
  out(StateDefinition::Y_POSITION) = p0(StateDefinition::Y_POSITION) +
                                     sin(p0(StateDefinition::THETA_POSITION)) *
                                         p0(StateDefinition::VEL_POSITION) * dt;
  out(StateDefinition::VEL_POSITION) = p0(StateDefinition::VEL_POSITION);
  out(StateDefinition::THETA_POSITION) = p0(StateDefinition::THETA_POSITION);
  return out;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
