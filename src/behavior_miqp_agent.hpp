// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BEHAVIOR_MIQP_AGENT_HPP_
#define BEHAVIOR_MIQP_AGENT_HPP_

#include <memory>

#include "bark/models/behavior/behavior_model.hpp"
#include "dynamic_occupancy.hpp"
#include "src/miqp_planner.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
namespace map {
class LaneCorridor;
}
}  // namespace world
namespace models {
namespace behavior {

/**
 * @brief miqp-based behavior model for BARK
 *
 */
class BehaviorMiqpAgent : public BehaviorModel {
 public:
  explicit BehaviorMiqpAgent(const commons::ParamsPtr& params);

  BehaviorMiqpAgent(const BehaviorMiqpAgent& bm);

  virtual ~BehaviorMiqpAgent() {}

  /**
   * @brief Plan function returning a trajectory
   *
   * @param delta_time
   * @param observed_world
   * @return Trajectory
   */
  virtual Trajectory Plan(double delta_time,
                          const world::ObservedWorld& observed_world);

  /**
   * @brief choosing a lane corridor
   *
   * @param observed_world
   * @return std::shared_ptr<bark::world::map::LaneCorridor>
   */
  std::shared_ptr<bark::world::map::LaneCorridor> ChooseLaneCorridor(
      const world::ObservedWorld& observed_world) const;

  /**
   * @brief set's behavior model for ego vehicle selected through parameter
   * server
   *
   * @param observed_world
   */
  void ApplyEgoModelForJointPrediction(
      world::ObservedWorldPtr& observed_world) const;

  /**
   * @brief predicts world throughout planning horizon of ego vehicle and
   * returns predictions as DynamicOccupancy objects
   *
   * @param observed_world
   * @return std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
   */
  std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
  PredictAgentsAsDynamicObstacles(
      world::ObservedWorldPtr& observed_world) const;

  /**
   * @brief predicts world throughout planning horizon of ego vehicle and
   * returns forbidden occupancy due to NoRightDriving rule
   *
   * @param observed_world
   * @return std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
   */
  std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
  PredictAgentsForNoRightDriving(world::ObservedWorldPtr& observed_world) const;

  /**
   * @brief predicts world throughout planning horizon of ego vehicle and
   * returns forbidden occupancy due to SafeDistance rule
   *
   * @param observed_world
   * @return std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
   */
  std::map<bark::world::objects::AgentId, DynamicOccupancyPtr>
  PredictAgentsForSafeDistance(world::ObservedWorldPtr& observed_world) const;

  /**
   * @brief collects all occupancies by calling respective prediction functions
   *
   * @param observed_world
   * @return std::vector<DynamicOccupancyPtr>
   */
  std::vector<DynamicOccupancyPtr> CollectDynOccupancies(
      const world::ObservedWorld& observed_world) const;

  /**
   * @brief Get the Settings object
   *
   * @return const miqp::planner::Settings&
   */
  const miqp::planner::Settings& GetSettings() const { return settings_; }

  /**
   * @brief clones the behavior model
   *
   * @return std::shared_ptr<BehaviorModel>
   */
  virtual std::shared_ptr<BehaviorModel> Clone() const;

  /**
   * @brief retrieves the center positions of the collision circles from the
   * optimization's result
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd GetCollisionCircleCenters() const;

  /**
   * @brief retrieves the center positions of the front collision circles from
   * the optimization's result
   *
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd GetFrontLbUb() const;

  /**
   * @brief Get the Last Trajectories of all controlled cars
   *
   * @return std::vector<Trajectory>
   */
  std::vector<Trajectory> GetLastTrajectoriesAllCars() const {
    return last_trajectories_all_cars_;
  }

  /**
   * @brief Get the Reference Trajectories of all controlled cars
   *
   * @return std::vector<Trajectory>
   */
  std::vector<Trajectory> GetRefTrajectories() const {
    return ref_trajectories_;
  }

  /**
   * @brief Get the Collision Radius
   *
   * @return double
   */
  double GetCollisionRadius() const { return settings_.collisionRadius; }

  /**
   * @brief Get the Ref Trajectories that have a Longer Horizon
   *
   * @return std::vector<Trajectory>
   */
  std::vector<Trajectory> GetRefTrajectoriesLongerHorizon() const {
    return ref_trajectories_longer_horizon_;
  }

  /**
   * @brief Get the Convex Shrinked Environment Polygon for All Cars
   *
   * @return miqp::common::geometry::PolygonMap
   */
  miqp::common::geometry::PolygonMap GetConvexShrinkedEnvPolygonsAllCars()
      const {
    return convex_shrinked_env_polygons_all_cars_;
  }

  /**
   * @brief Get the Last Cplex Wrapper object
   *
   * @return miqp::planner::cplex::CplexWrapper
   */
  miqp::planner::cplex::CplexWrapper GetLastCplexWrapper() const {
    return last_cplex_wrapper_;
  }

  /**
   * @brief Get the Last Solution's Properties object
   *
   * @return miqp::planner::cplex::SolutionProperties
   */
  miqp::planner::cplex::SolutionProperties GetLastSolutionProperties() const {
    return std::get<3>(currentSolution_);
  }

  /**
   * @brief Get the Last Planning Success flag
   *
   */
  bool GetLastPlanningSuccess() const { return last_planning_success_; }

  /**
   * @brief Get the Last Dynamic Occupancies
   *
   * @return std::vector<DynamicOccupancyPtr>
   */
  std::vector<DynamicOccupancyPtr> GetLastDynamicOccupancies() const {
    return last_dyn_occupancies_;
  }

  /**
   * @brief Get the Environment Polygon
   *
   * @return bark::geometry::Polygon
   */
  bark::geometry::Polygon GetEnvironmentPolygon() const { return envPoly_; }

  /**
   * @brief Set the Warmstart Type
   *
   * @param type
   */
  void SetWarmstartType(miqp::planner::cplex::CplexWrapper::WarmstartType type);

 private:
  /**
   * @brief calculates the environment polygon
   *
   * @param observed_world
   * @return true if successfull
   * @return false if polygon could not be created
   */
  bool CalculateEnvironmentPolygon(const world::ObservedWorld& observed_world);

  /**
   * @brief check if PoseOutOfMap is colliding with road
   *
   */
  void CheckPoseOutOfMap() const;

  /**
   * @brief converts bark agent to input representation of miqp planner
   *
   * @param observed_world
   * @param initialState
   * @param refLine
   */
  void ProcessBarkAgent(const world::ObservedWorld observed_world,
                        Eigen::MatrixXd& initialState,
                        bark::geometry::Line& refLine);

  MiqpPlannerSettings settings_;
  miqp::planner::MiqpPlanner planner_;
  double desiredVelocity_;
  double deltaSDesiredVelocity_;
  // timestep, egoid, result, solution properties
  std::tuple<double, int, std::shared_ptr<miqp::planner::RawResults>,
             miqp::planner::cplex::SolutionProperties>
      currentSolution_;
  std::vector<Trajectory> last_trajectories_all_cars_;
  std::vector<Trajectory> ref_trajectories_;
  std::vector<Trajectory> ref_trajectories_longer_horizon_;
  miqp::common::geometry::PolygonMap convex_shrinked_env_polygons_all_cars_;
  std::map<int, std::pair<double, double>> last_axy_all_cars_;
  miqp::planner::cplex::CplexWrapper last_cplex_wrapper_;
  bool use_box_as_env_;
  bool write_debug_files_;
  bool multi_agent_planning_;
  bool rule_no_right_passing_;
  bool rule_no_right_passing_is_soft_;
  bool rule_safe_distance_;
  bool rule_safe_distance_is_soft_;
  int choose_ego_model_joint_prediction_;
  bool last_planning_success_;
  std::string debug_file_path_;
  std::string debug_file_prefix_;
  bark::geometry::Polygon envPoly_;
  bark::geometry::Pose pose_out_of_map_;
  bool firstrun_;
  int idx_ego_;
  std::map<int, int> car_idxs_;
  std::map<std::size_t, int> obstacle_ids_;  // occupancy hash, miqp obstacle id
  std::vector<DynamicOccupancyPtr> last_dyn_occupancies_;
  miqp::planner::cplex::CplexWrapper::WarmstartType warmstart_type_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMiqpAgent::Clone() const {
  std::shared_ptr<BehaviorMiqpAgent> model_ptr =
      std::make_shared<BehaviorMiqpAgent>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MIQP_AGENT_HPP_
