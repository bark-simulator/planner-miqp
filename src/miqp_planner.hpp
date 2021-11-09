// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_PLANNER_HEADER
#define MIQP_PLANNER_HEADER

#include "common/map/convexified_map.hpp"
#include "common/parameter/parameter_preparer.hpp"
#include "common/reference/reference_trajectory_generator.hpp"
#include "cplex_wrapper.hpp"

#include "bark/commons/util/util.hpp"
#include "bark/geometry/line.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace miqp {
namespace planner {

/**
 * @brief class that creates and calls optimization problem
 *
 */
class MiqpPlanner {
 public:
  MiqpPlanner(const Settings& settings,
              const bark::geometry::Polygon& mapPolygon);

  MiqpPlanner(const Settings& settings);

  MiqpPlanner();

  MiqpPlanner(const MiqpPlanner& p);

  virtual ~MiqpPlanner();

  /**
   * @brief adds car to plan for to MiqpPlanner
   *
   * @param intitialState
   * @param referencePath
   * @param desiredVelocity
   * @param deltaSForDesiredVel
   * @param timestep
   * @param track_reference_positions
   * @return int
   */
  int AddCar(Eigen::MatrixXd intitialState,
             const bark::geometry::Line& referencePath,
             const double& desiredVelocity, const double& deltaSForDesiredVel,
             const double timestep = 0.0,
             const bool track_reference_positions = true);

  /**
   * @brief adds obstacle to MiqpPlanner based on Eigen representation
   * for obstacle
   *
   * @param dynamic_obstacle
   * @param is_soft: flag wether obstacle will be treated as soft constraint
   * @param is_static: is obstacle static or moving over time
   * @return int: index of obstacle
   */
  int AddObstacle(std::vector<Eigen::MatrixXd>& dynamic_obstacle, bool is_soft,
                  bool is_static);

  /**
   * @brief adds obstacle to MiqpPlanner based on predicted BARK Trajectory
   * representation for obstacle
   *
   * @param predicted_traj
   * @param shape
   * @param is_soft
   * @param is_static
   * @return int
   */
  int AddObstacle(bark::models::dynamic::Trajectory& predicted_traj,
                  bark::geometry::Polygon& shape, bool is_soft, bool is_static);

  /**
   * @brief updates occupied polygon of obstacle based on Eigen representation
   * for obstacle
   *
   * @param id
   * @param dynamic_obstacle
   */
  void UpdateObstacle(int id, std::vector<Eigen::MatrixXd>& dynamic_obstacle);

  /**
   * @brief updates occupied polygon of obstacle based on predicted BARK
   * Trajectory representation for obstacle
   *
   * @param id
   * @param predicted_traj
   * @param shape
   */
  void UpdateObstacle(int id, bark::models::dynamic::Trajectory& predicted_traj,
                      bark::geometry::Polygon& shape);

  /**
   * @brief adds static obstacle to optimization
   *
   * @param obstacleShape
   * @return int
   */
  int AddStaticObstacle(bark::geometry::Polygon& obstacleShape);

  /**
   * @brief returns number of agents
   *
   * @return int
   */
  int GetNrCars() { return parameters_->NumCars; };

  /**
   * @brief removes car from list of controlled agents
   *
   * @param thisIdx
   */
  void RemoveCar(int thisIdx);

  /**
   * @brief removes obstacle from list of obstacles to consider
   *
   * @param id
   */
  void RemoveObstacle(int id);

  /**
   * @brief removes all obstacles within MiqpPlanner
   *
   */
  void RemoveAllObstacles();

  /**
   * @brief updates initial state and reference of controlled agent
   *
   * @param thisIdx
   * @param intitialState
   * @param referencePath
   * @param timestep
   * @param track_reference_positions
   */
  void UpdateCar(int thisIdx, Eigen::MatrixXd intitialState,
                 const bark::geometry::Line& referencePath,
                 const double timestep = 0.0,
                 bool track_reference_positions = true);

  /**
   * @brief let's MiqpPlanner plan a trajectory for each controlled agent
   *
   * @param timestemp
   * @return true
   * @return false
   */
  bool Plan(const double timestemp = 0.0);

  /**
   * @brief calculates and returns reference trajectories for all controlled
   * agents
   *
   * @return std::vector<bark::models::dynamic::Trajectory>
   */
  std::vector<bark::models::dynamic::Trajectory>
  CalculateReferenceTrajectories() const;

  /**
   * @brief calculates and returns reference trajectories (long version) for all
   * controlled agents
   *
   * @return std::vector<bark::models::dynamic::Trajectory>
   */
  std::vector<bark::models::dynamic::Trajectory>
  CalculateReferenceTrajectoriesLongerHorizon() const;

  /**
   * @brief returns the Model Parameters object
   *
   * @return const std::shared_ptr<ModelParameters>
   */
  const std::shared_ptr<ModelParameters> GetParameters();

  /**
   * @brief returns state of optimization result at a given time index
   *
   * @param timeIdx
   * @param carIdx
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd Get2ndOrderStateFromSolution(const int timeIdx,
                                               const int carIdx) const;

  /**
   * @brief convert's optimization result to BARK's trajectory format
   *
   * @param carIdx
   * @param start_time
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GetBarkTrajectory(const int carIdx,
                                                      const float start_time);

  /**
   * @brief creates state definition suitable for optimization
   *
   * TODO@klemens: move this to common/dynamic?
   * @param x
   * @param y
   * @param theta
   * @param v
   * @param a
   * @return Eigen::MatrixXd
   */
  static Eigen::MatrixXd CarStateToMiqpState(float x, float y, float theta,
                                             float v, float a);

  /**
   * @brief sets warmstart type
   *
   * @param in
   */
  void SetDoWarmstart(miqp::planner::cplex::CplexWrapper::WarmstartType in);

  /**
   * @brief returns warmstart type
   *
   * @return miqp::planner::cplex::CplexWrapper::WarmstartType
   */
  miqp::planner::cplex::CplexWrapper::WarmstartType GetDoWarmstart() {
    return doWarmstart_;
  }

  /**
   * @brief returns solution properties of optimization
   *
   * @return miqp::planner::cplex::SolutionProperties
   */
  miqp::planner::cplex::SolutionProperties GetSolutionProperties() {
    return cplexWrapper_.getSolutionProperties();
  }

  /**
   * @brief activates writing of optimization debug files to file system
   *
   * @param path path where files will be written to
   * @param name name of file
   */
  void ActivateDebugFileWrite(std::string path, std::string name);

  std::shared_ptr<miqp::planner::RawResults> GetSolution() {
    return cplexWrapper_.getRawResults();
  }

  /**
   * @brief Get the Convex Shrinked Environment Polygon
   *
   * @return miqp::common::geometry::PolygonMap
   */
  miqp::common::geometry::PolygonMap GetConvexShrinkedEnvPolygonsAllCars()
      const {
    return convex_shrinked_env_polygons_all_cars_;
  }

  /**
   * @brief Get the Cplex Wrapper object
   *
   * @return const miqp::planner::cplex::CplexWrapper&
   */
  const miqp::planner::cplex::CplexWrapper& GetCplexWrapper() const {
    return cplexWrapper_;
  }

  /**
   * @brief Get the warmstart instance
   *
   * @return const std::shared_ptr<RawResults>
   */
  const std::shared_ptr<RawResults> GetWarmstart() { return warmstart_; }

  /**
   * @brief updates convexified map based on current position of the ego vehicle
   *
   * @param mapPolygon
   * @return true
   * @return false
   */
  bool UpdateConvexifiedMap(const bark::geometry::Polygon& mapPolygon);

  /**
   * @brief returns number of optimization support points
   *
   * @return int
   */
  int GetN() { return settings_.nr_steps; }

  /**
   * @brief returns time increment of optimization
   *
   * @return float
   */
  float GetTs() { return settings_.ts; }

  /**
   * @brief returns collision radius
   *
   * @return float
   */
  float GetCollisionRadius() { return settings_.collisionRadius; }

  /**
   * @brief Get the Settings object
   *
   * @return const Settings&
   */
  const Settings& GetSettings() { return settings_; }

  /**
   * @brief updates desired velocity of MiqpPlanner
   *
   * @param carIdx
   * @param vDes
   * @param sDes
   */
  void UpdateDesiredVelocity(const int carIdx, const double vDes,
                             const double sDes);

  /**
   * @brief Get the last reference trajectory of the specified controlled agent
   *
   * @param carIdx
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GetLastReference(int carIdx) const;

  /**
   * @brief returns the raw result state at the current idx
   *
   * @param idx time index
   * @param caridx car idx
   */
  Eigen::MatrixXd GetThirdOrderStateAtResultIdx(const int idx,
                                                const int caridx);

 private:
  /**
   * @brief calculates warmstart
   *
   */
  void CalculateWarmstart();

  /**
   * @brief calculates new environment polygon based on given reference
   * trajectories for controlled agents
   *
   * @param referenceTrajectories
   */
  void ResetEnvironment(
      std::vector<bark::models::dynamic::Trajectory>& referenceTrajectories);

  /**
   * @brief warmstart optimization variables related to being within environment
   *
   */
  void EnvironmentWarmstart();

  /**
   * @brief Create an Eigen-based representation of an obstacle based on BARK's
   * trajectory
   *
   * @param predicted_traj
   * @param shape
   * @return std::vector<Eigen::MatrixXd>
   */
  std::vector<Eigen::MatrixXd> CreateMiqpObstacle(
      bark::models::dynamic::Trajectory& predicted_traj,
      bark::geometry::Polygon& shape);

  /**
   * @brief checks if obstacle intersects with the environment (map)
   *
   * @param obstacle_vector
   * @param is_static
   * @return true if they intersect
   */
  bool ObstacleIntersectsEnvironment(
      std::vector<Eigen::MatrixXd>& obstacle_vector, bool is_static);

  /**
   * @brief calculates region of interest where obstacles must be within to be
   * considered in optimization
   *
   * @param x
   * @param y
   * @param theta
   */
  void UpdateObstaclesROI(const double x, const double y, const double theta);

  /**
   * @brief check if either vx or vy is below the low speed threshold
   * minimum_valid_speed_vx_vy_
   *
   * @param vx
   * @param vy
   */
  bool IsVxVyValid(const double& vx, const double& vy);

 private:
  std::shared_ptr<ModelParameters> parameters_;
  std::shared_ptr<RawResults> warmstart_;
  miqp::planner::cplex::CplexWrapper::WarmstartType doWarmstart_;
  bool validWarmstart_;
  std::vector<miqp::common::map::PolygonId> environmentIdsWarmstart_;
  const float eps_ = 0.000001;
  const Settings settings_;
  miqp::common::parameter::ParameterPreparer parameterPreparer_;
  const int egoCarIdx_;
  std::vector<miqp::common::reference::ReferenceTrajectoryGenerator>
      referenceGenerator_;
  std::vector<miqp::common::reference::ReferenceTrajectoryGenerator>
      referenceGeneratorLongerHorizon_;
  miqp::common::map::ConvexifiedMap convexifiedMap_;
  miqp::common::geometry::PolygonMap convex_shrinked_env_polygons_all_cars_;
  miqp::common::geometry::PolygonMap
      convex_shrinked_env_polygons_all_cars_union_;
  const std::string cplexModelname_;
  const std::string cplexModelpath_;
  miqp::planner::cplex::CplexWrapper cplexWrapper_;
  bark::geometry::Polygon obstacles_roi_;
  double minimum_valid_speed_vx_vy_;
};

}  // namespace planner
}  // namespace miqp

#endif  // MIQP_PLANNER_HEADER