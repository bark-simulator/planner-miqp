// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_miqp_agent.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "gtest/gtest.h"

using namespace std;
using namespace miqp::planner;
using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::models::behavior;
using namespace bark::geometry::standard_shapes;
using namespace bark::world::map;
using bark::commons::ParamsPtr;
using bark::commons::SetterParams;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::models::behavior::DynamicOccupancyPtr;
using bark::models::behavior::OccupancyType;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::objects::Agent;
using bark::world::objects::AgentId;
using bark::world::objects::AgentPtr;

AgentPtr CreateAgent(const ParamsPtr& params, const BehaviorModelPtr& beh_model,
                     const State& init_state, const GoalDefinitionPtr& goal_def,
                     const MapInterfacePtr& map) {
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  Polygon car_polygon = CarRectangle();
  AgentPtr agent(new Agent(init_state, beh_model, dyn_model, exec_model,
                           car_polygon, params, goal_def, map,
                           bark::geometry::Model3D()));
  return agent;
}

AgentPtr CreateAgentWithConstantAcceleration(const ParamsPtr& params,
                                             State init_state,
                                             const GoalDefinitionPtr& goal_def,
                                             const MapInterfacePtr& map) {
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  AgentPtr agent = CreateAgent(params, beh_model, init_state, goal_def, map);
  return agent;
}

// IDEA: test for Highway and Osterwaldstr.
ObservedWorld make_observed_straight_world_miqp(ParamsPtr params,
                                                bool with_obstacles,
                                                bool ego_on_left_lane,
                                                double vel_ego = 10.0,
                                                double vel_other = 10.0) {
  // Setting Up Map
  auto open_drive_map = bark::world::tests::MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  BehaviorModelPtr beh_model(new BehaviorMiqpAgent(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  if (ego_on_left_lane) {
    init_state1 << 0.0, 53.0, -1.75, 0.0, vel_ego;
  } else {
    init_state1 << 0.0, 53.0, -1.75 - 3.5, 0.0, vel_ego;
  }
  AgentPtr agent1 = CreateAgent(params, beh_model, init_state1,
                                goal_definition_ptr, map_interface);

  // Preceding Agent
  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 70.0, -1.75, 0.0, vel_other;
  AgentPtr agent2 = CreateAgentWithConstantAcceleration(
      params, init_state2, goal_definition_ptr, map_interface);

  // Agent coming from behind on the right
  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 43.0, -1.75 - 3.5, 0.0, vel_other;
  AgentPtr agent3 = CreateAgentWithConstantAcceleration(
      params, init_state3, goal_definition_ptr, map_interface);

  // Following Agent
  State init_state4(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state4 << 0.0, 20.0, -1.75, 0.0, vel_other;
  AgentPtr agent4 = CreateAgentWithConstantAcceleration(
      params, init_state4, goal_definition_ptr, map_interface);

  // Agent on the right in front
  State init_state5(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state5 << 0.0, 100.0, -1.75 - 3.5, 0.0, vel_other;
  AgentPtr agent5 = CreateAgentWithConstantAcceleration(
      params, init_state5, goal_definition_ptr, map_interface);

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  if (with_obstacles) {
    world->AddAgent(agent2);
    world->AddAgent(agent3);
    world->AddAgent(agent4);
    world->AddAgent(agent5);
  }
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  return observed_world;
}

ObservedWorld make_observed_curved_world_miqp(ParamsPtr params, double length,
                                              double curvature) {
  // Setting Up Map
  auto open_drive_map =
      bark::world::tests::MakeXodrMapCurved(length, curvature);
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon car_polygon = CarRectangle();

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(-15, 15))));  // goal polygon is not really important

  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMiqpAgent(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 2.6, -1.6, 0.17, 4.0;  // 10deg orientation
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  return observed_world;
}

void CompareTwoStates(State state1, State state2) {
  ASSERT_EQ(state1(StateDefinition::TIME_POSITION),
            state2(StateDefinition::TIME_POSITION))
      << state1 << "\n"
      << state2;
  ASSERT_EQ(state1(StateDefinition::X_POSITION),
            state2(StateDefinition::X_POSITION));
  ASSERT_EQ(state1(StateDefinition::Y_POSITION),
            state2(StateDefinition::Y_POSITION));
  ASSERT_EQ(state1(StateDefinition::THETA_POSITION),
            state2(StateDefinition::THETA_POSITION));
  ASSERT_EQ(state1(StateDefinition::VEL_POSITION),
            state2(StateDefinition::VEL_POSITION));
}

TEST(miqp_bark_agent, construction) {
  auto params = std::make_shared<SetterParams>();
  BehaviorModelPtr beh_model(new BehaviorMiqpAgent(params));
  ASSERT_NE(beh_model, nullptr);
}

TEST(miqp_bark_agent, prediction) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, true);

  ObservedWorldPtr predicted_world_ptr =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());

  BehaviorMiqpAgent beh_model = BehaviorMiqpAgent(params);
  std::map<AgentId, DynamicOccupancyPtr> dynpred =
      beh_model.PredictAgentsAsDynamicObstacles(predicted_world_ptr);

  ASSERT_EQ(dynpred.size(), observed_world.GetOtherAgents().size());

  miqp::planner::Settings settings = beh_model.GetSettings();
  for (const auto& agent_j : observed_world.GetOtherAgents()) {
    ASSERT_EQ(agent_j.first, dynpred.at(agent_j.first)->id);

    auto timevec = Eigen::VectorXd::LinSpaced(
        settings.nr_steps, 0, (settings.nr_steps - 1) * settings.ts);
    for (int k = 0; k < settings.nr_steps; ++k) {
      EXPECT_NEAR(dynpred.at(agent_j.first)->prediction(k, 0), timevec(k),
                  1e-8);
    }
  }

  for (const auto& agent_j : observed_world.GetOtherAgents()) {
    State state_world =
        observed_world.GetAgent(agent_j.first)->GetCurrentState();
    State state_pred = dynpred.at(agent_j.first)->prediction.row(0);
    CompareTwoStates(state_world, state_pred);
  }

  // Removing Ego Agent, so that stepping the world will not always trigger an
  // optimization run
  observed_world.RemoveAgentById(observed_world.GetEgoAgentId());
  for (int i = 0; i < settings.nr_steps - 1; i++) {
    observed_world.Step(settings.ts);
  }

  for (const auto& agent_j : observed_world.GetOtherAgents()) {
    State state_world =
        observed_world.GetAgent(agent_j.first)->GetCurrentState();
    int last_row_idx = dynpred.at(agent_j.first)->prediction.rows() - 1;
    State state_pred = dynpred.at(agent_j.first)->prediction.row(last_row_idx);
    CompareTwoStates(state_world, state_pred);
  }
}
TEST(miqp_bark_agent, observed_world) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, true);

  // assert environment (we get N polygons!!!)

  // assert reference line (we have a valid reference line)
}

TEST(miqp_bark_agent, step_1_no_obstacles) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, false, true);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(behavior_model->GetLastPlanningSuccess());
  ASSERT_EQ(traj.row(0)(StateDefinition::TIME_POSITION), 0.0);
  ASSERT_EQ(traj.row(1)(StateDefinition::TIME_POSITION), delta_time);
}

TEST(miqp_bark_agent, step_1_with_obstacles) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, true);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(behavior_model->GetLastPlanningSuccess());
  ASSERT_EQ(traj.row(0)(StateDefinition::TIME_POSITION), 0.0);
  ASSERT_EQ(traj.row(1)(StateDefinition::TIME_POSITION), delta_time);
}

TEST(miqp_bark_agent, step_1_multi_agent) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, true);
  params->SetBool("Miqp::MultiAgentPlanning", true);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(behavior_model->GetLastPlanningSuccess());
  ASSERT_EQ(traj.row(0)(StateDefinition::TIME_POSITION), 0.0);
  ASSERT_EQ(traj.row(1)(StateDefinition::TIME_POSITION), delta_time);
}

TEST(miqp_bark_agent, step_1_with_obstacles_not_valid_yet) {
  // test where agent (treated as obstacle) becomes only valid after some time.
  // if the planner does not treat this correctly, optimization will fail, as
  // initial states are the same (thus colliding, but other agent should start
  // 1s later)
  auto params = std::make_shared<SetterParams>();

  double vel_ego = 10.0;
  auto open_drive_map = bark::world::tests::MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  std::shared_ptr<BehaviorMiqpAgent> beh_model(new BehaviorMiqpAgent(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 53.0, -1.75, 0.0, 10.0;
  AgentPtr agent1 = CreateAgent(params, beh_model, init_state1,
                                goal_definition_ptr, map_interface);

  // Following Agent
  State init_state4(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state4 << 0.0, 53.0, -1.75, 0.0, 10.0;
  AgentPtr agent4 = CreateAgentWithConstantAcceleration(
      params, init_state4, goal_definition_ptr, map_interface);
  agent4->SetFirstValidTimestamp(1.25);

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent4);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  float delta_time = 0.25;
  Trajectory traj = beh_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(beh_model->GetLastPlanningSuccess());
  // velocity does not really change
  EXPECT_NEAR(traj.row(1)(StateDefinition::VEL_POSITION), 10.0, 1e-8);
  EXPECT_NEAR(traj.row(10)(StateDefinition::VEL_POSITION), 10.0, 1e-8);
}

TEST(miqp_bark_agent, step_n) {
  auto params = std::make_shared<SetterParams>();
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, true);

  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());

  float delta_time = 0.25;
  for (int i = 0; i < 2; ++i) {
    observed_world.Step(delta_time);
    ASSERT_TRUE(behavior_model->GetLastPlanningSuccess());
  }
  // Lets plan for ego one more time
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
}

TEST(miqp_bark_agent, step_1_without_rule_no_right_passing_faster) {
  // vehicle is on right lane and driving faster than vehicle on left lane.
  // without modelling the rule, it will pass the vehicle on the right
  auto params = std::make_shared<SetterParams>();
  params->SetReal("Miqp::DesiredVelocity", 15.0);
  params->SetBool("Miqp::RuleNoRightPassing", false);
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, false, 15.0);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  EXPECT_TRUE(behavior_model->GetLastPlanningSuccess());

  // velocity does not really change
  EXPECT_NEAR(traj.row(1)(StateDefinition::VEL_POSITION), 15.0, 1e-8) << traj;
  EXPECT_NEAR(traj.row(10)(StateDefinition::VEL_POSITION), 15.0, 1e-8);
}

TEST(miqp_bark_agent, step_1_with_rule_no_right_passing_faster) {
  // vehicle is on right lane and driving faster than vehicle on left lane.
  // when modelling the rule, it will not pass the vehicle on the right
  auto params = std::make_shared<SetterParams>();
  params->SetReal("Miqp::DesiredVelocity", 15.0);
  params->SetBool("Miqp::RuleNoRightPassing", true);
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, false, 15.0);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  EXPECT_TRUE(behavior_model->GetLastPlanningSuccess());

  // there is a significant slowdown
  EXPECT_LT(traj.row(10)(StateDefinition::VEL_POSITION), 13.0);
  EXPECT_LT(traj.row(19)(StateDefinition::VEL_POSITION), 13.0);
}

TEST(miqp_bark_agent, step_1_with_rule_no_right_passing_not_faster) {
  // vehicle is on right lane and driving faster than vehicle on left lane.
  // no slowdown, if rule implementation works correctly
  auto params = std::make_shared<SetterParams>();
  params->SetReal("Miqp::DesiredVelocity", 10.0);
  params->SetBool("Miqp::RuleNoRightPassing", true);
  ObservedWorld observed_world =
      make_observed_straight_world_miqp(params, true, false, 10.0);
  auto behavior_model = std::dynamic_pointer_cast<BehaviorMiqpAgent>(
      observed_world.GetEgoAgent()->GetBehaviorModel());
  float delta_time = 0.25;
  Trajectory traj = behavior_model->Plan(delta_time, observed_world);
  EXPECT_TRUE(behavior_model->GetLastPlanningSuccess());

  // velocity does not really change
  EXPECT_NEAR(traj.row(1)(StateDefinition::VEL_POSITION), 10.0, 1e-8);
  EXPECT_NEAR(traj.row(10)(StateDefinition::VEL_POSITION), 10.0, 1e-8);
}

TEST(miqp_bark_agent, step_1_with_rule_safe_distance) {
  // test with preceding agent. safety distance is calculated. test should
  // succeed.
  auto params = std::make_shared<SetterParams>();

  double vel_ego = 10.0;
  params->SetReal("Miqp::DesiredVelocity", vel_ego);
  params->SetBool("Miqp::RuleSafeDistance", true);
  auto open_drive_map = bark::world::tests::MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  std::shared_ptr<BehaviorMiqpAgent> beh_model(new BehaviorMiqpAgent(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 53.0, -1.75, 0.0, 10.0;
  AgentPtr agent1 = CreateAgent(params, beh_model, init_state1,
                                goal_definition_ptr, map_interface);

  // Preceding Agent
  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 70.0, -1.75, 0.0, 10.0;
  AgentPtr agent2 = CreateAgentWithConstantAcceleration(
      params, init_state2, goal_definition_ptr, map_interface);

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  float delta_time = 0.25;
  Trajectory traj = beh_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(beh_model->GetLastPlanningSuccess());

  const auto CalcSafeDistance = [](double v_r, double v_f, double a_r,
                                   double a_f, double delta) {
    return v_r * delta - pow(v_r, 2) / (2.0 * a_r) + pow(v_f, 2) / (2.0 * a_f);
  };
  double sd = CalcSafeDistance(10, 10, -5, -5, 1);

  auto occupancies = beh_model->GetLastDynamicOccupancies();
  for (const auto& occ : occupancies) {
    if (occ->type == OccupancyType::RULE_SAFE_DISTANCE) {
      EXPECT_EQ(occ->prediction.row(0)[StateDefinition::X_POSITION],
                init_state2(StateDefinition::X_POSITION) - sd);
    }
  }
}

TEST(miqp_bark_agent, step_1_with_rule_safe_distance_soft) {
  // test with preceding agent. safety distance is calculated. optimization
  // should only be possible if safe distacne is treated as soft
  auto params = std::make_shared<SetterParams>();

  double vel_ego = 10.0;
  params->SetReal("Miqp::DesiredVelocity", vel_ego);
  params->SetBool("Miqp::RuleSafeDistance", true);
  params->SetBool("Miqp::RuleSafeDistanceIsSoft", true);
  auto open_drive_map = bark::world::tests::MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  std::shared_ptr<BehaviorMiqpAgent> beh_model(new BehaviorMiqpAgent(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 53.0, -1.75, 0.0, 10.0;
  AgentPtr agent1 = CreateAgent(params, beh_model, init_state1,
                                goal_definition_ptr, map_interface);

  // Preceding Agent
  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 63.0, -1.75, 0.0, 10.0;
  AgentPtr agent2 = CreateAgentWithConstantAcceleration(
      params, init_state2, goal_definition_ptr, map_interface);

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  float delta_time = 0.25;
  Trajectory traj = beh_model->Plan(delta_time, observed_world);
  ASSERT_TRUE(beh_model->GetLastPlanningSuccess());
}