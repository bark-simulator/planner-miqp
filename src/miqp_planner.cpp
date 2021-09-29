// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "miqp_planner.hpp"
#include "bark/commons/timer/timer.hpp"
#include "common/geometry/geometry.hpp"
#include "common/parameter/parameter_preparer.hpp"
#include "common/parameter/regions.hpp"
#include "common/parameter/vehicle_parameters.hpp"

using namespace miqp::common::parameter;
using bark::geometry::Line;
using bark::geometry::Pose;
using bark::models::dynamic::Trajectory;
using namespace miqp::common::reference;
using namespace bark::models::dynamic;
using namespace miqp::planner::cplex;
using namespace miqp::common::map;

namespace miqp {
namespace planner {

MiqpPlanner::MiqpPlanner(const Settings& settings,
                         const bark::geometry::Polygon& mapPolygon)
    : parameters_(std::make_shared<ModelParameters>()),
      warmstart_(std::make_shared<RawResults>()),
      doWarmstart_(settings.warmstartType),
      validWarmstart_(false),
      environmentIdsWarmstart_(),
      settings_(settings),
      parameterPreparer_(
          nullptr, settings.nr_regions, settings.max_velocity_fitting,
          settings.minimum_region_change_speed, settings.accLonMaxLimit,
          settings.accLonMinLimit, settings.jerkLonMaxLimit,
          settings.accLatMinMaxLimit, settings.jerkLatMinMaxLimit),
      egoCarIdx_(0),
      referenceGenerator_(),
      referenceGeneratorLongerHorizon_(),
      convexifiedMap_(ConvexifiedMap(
          nullptr, mapPolygon, settings.collisionRadius,
          settings.simplificationDistanceMap, settings.bufferReference,
          settings.buffer_for_merging_tolerance)),
      convex_shrinked_env_polygons_all_cars_(),
      cplexModelname_("cplexmodel.mod"),
      cplexModelpath_(settings.cplexModelpath),
      cplexWrapper_(CplexWrapper(cplexModelpath_, cplexModelname_,
                                 CplexWrapper::CPPINPUTS, settings.precision)),
      obstacles_roi_(bark::geometry::Polygon()) {
  // std::cout << "Constructing MiqpPlanner" << std::endl;
  // Bind the parameters_ ptr to the cplex wrapper
  cplexWrapper_.resetParameters(parameters_);
  cplexWrapper_.deleteLastSolutionWarmstartFile();

  // Sizes
  parameters_->nr_regions = settings.nr_regions;
  parameters_->NumSteps = settings.nr_steps;
  parameters_->NumCars = 0;
  parameters_->NumCar2CarCollisions = 0;
  parameters_->nr_obstacles = 0;
  parameters_->nr_environments = 0;

  // Get Fitting Polynomials
  parameters_->poly_orientation_params.POLY_SINT_UB =
      parameterPreparer_.GetFittingPolynomialParameters().GetPOLY_SINT_UB();
  parameters_->poly_orientation_params.POLY_SINT_LB =
      parameterPreparer_.GetFittingPolynomialParameters().GetPOLY_SINT_LB();
  parameters_->poly_orientation_params.POLY_COSS_UB =
      parameterPreparer_.GetFittingPolynomialParameters().GetPOLY_COSS_UB();
  parameters_->poly_orientation_params.POLY_COSS_LB =
      parameterPreparer_.GetFittingPolynomialParameters().GetPOLY_COSS_LB();
  parameters_->poly_curvature_params.POLY_KAPPA_AX_MAX =
      parameterPreparer_.GetFittingPolynomialParameters()
          .GetPOLY_KAPPA_AX_MAX();
  parameters_->poly_curvature_params.POLY_KAPPA_AX_MIN =
      parameterPreparer_.GetFittingPolynomialParameters()
          .GetPOLY_KAPPA_AX_MIN();

  // CPLEX Settings, Other Global Settings
  parameters_->max_solution_time = settings.max_solution_time;
  parameters_->relative_mip_gap_tolerance = settings.relative_mip_gap_tolerance;
  parameters_->mipdisplay = settings.mipdisplay;
  parameters_->mipemphasis = settings.mipemphasis;
  parameters_->relobjdif = settings.relobjdif;
  parameters_->cutpass = settings.cutpass;
  parameters_->probe = settings.probe;
  parameters_->repairtries = settings.repairtries;
  parameters_->rinsheur = settings.rinsheur;
  parameters_->varsel = settings.varsel;
  parameters_->mircuts = settings.mircuts;
  parameters_->parallelmode = static_cast<int>(settings.parallelMode);
  parameters_->ts = settings.ts;
  parameters_->minimum_region_change_speed =
      settings.minimum_region_change_speed;

  // Slack-dependent Variables
  parameters_->agent_safety_distance.resize(parameters_->NumSteps);
  parameters_->agent_safety_distance.setZero();
  parameters_->agent_safety_distance_slack.resize(parameters_->NumSteps);
  parameters_->agent_safety_distance_slack.setConstant(
      settings.constant_agent_safety_distance_slack);
  parameters_->maximum_slack = settings.constant_agent_safety_distance_slack;
  parameters_->WEIGHTS_SLACK = settings.slackWeight;
  parameters_->WEIGHTS_SLACK_OBSTACLE = settings.slackWeightObstacle;

  // Global Limits
  parameters_->min_vel_x_y = -settings.max_velocity_fitting - eps_;
  parameters_->max_vel_x_y = +settings.max_velocity_fitting + eps_;

  // Fraction Parameters
  parameters_->fraction_parameters = parameterPreparer_.GetFractionParameters();

  // Convexify Map
  convexifiedMap_.Convert();

  // Prepare warmstart
  warmstart_->N = settings_.nr_steps;
  warmstart_->NrEnvironments = 0;
  warmstart_->NrRegions = settings_.nr_regions;
  warmstart_->NrObstacles = 0;
  warmstart_->MaxLinesObstacles = 0;
  warmstart_->NrCarToCarCollisions = 0;
  warmstart_->NrCars = 0;

  // Warmstart and SpecialOrderedSet and Branching Priorities
  cplexWrapper_.setSpecialOrderedSets(settings.useSos);
  cplexWrapper_.setUseBranchingPriorities(settings.useBranchingPriorities);
  cplexWrapper_.setBranchingPriorityValueExtent(1, settings_.nr_steps - 1);
  LOG(INFO) << "SOS usage set to: " << settings.useSos
            << ", branching priorities usage set to: "
            << settings.useBranchingPriorities;

  // For apollo: buffer cplex outputs to a stream and print to logger afterwards
  if (settings_.buffer_cplex_outputs) {
    cplexWrapper_.setBufferCplexOutputsToStream(settings_.buffer_cplex_outputs);
  }
}

MiqpPlanner::MiqpPlanner(const Settings& settings)
    : MiqpPlanner(settings, bark::geometry::Polygon()) {}

MiqpPlanner::MiqpPlanner()
    : MiqpPlanner(miqp::planner::DefaultSettings(), bark::geometry::Polygon()) {
}

// TODO like this it is a hack: the cplex wrapper is not copyable, therefore I
// create a new one here
MiqpPlanner::MiqpPlanner(const MiqpPlanner& p)
    : parameters_(p.parameters_),
      warmstart_(p.warmstart_),
      doWarmstart_(p.doWarmstart_),
      validWarmstart_(p.validWarmstart_),
      environmentIdsWarmstart_(p.environmentIdsWarmstart_),
      eps_(p.eps_),
      settings_(p.settings_),
      parameterPreparer_(p.parameterPreparer_),
      egoCarIdx_(p.egoCarIdx_),
      referenceGenerator_(p.referenceGenerator_),
      referenceGeneratorLongerHorizon_(p.referenceGeneratorLongerHorizon_),
      convexifiedMap_(p.convexifiedMap_),
      convex_shrinked_env_polygons_all_cars_(
          p.convex_shrinked_env_polygons_all_cars_),
      cplexModelname_(p.cplexModelname_),
      cplexWrapper_(p.cplexWrapper_) {
  // cplexWrapper_(CplexWrapper(p.cplexModelname_, CplexWrapper::CPPINPUTS,
  //                            p.settings_.precision)) {
  cplexWrapper_.resetParameters(parameters_);
  // Do not delete old warmstart file here, as it could be used from the copied
  // instance. Also if the bark world is copied this constructor is called!
}

MiqpPlanner::~MiqpPlanner() {}

// Called once for each car
int MiqpPlanner::AddCar(Eigen::MatrixXd initialState, const Line& referencePath,
                        const double& desiredVelocity,
                        const double& deltaSForDesiredVel,
                        const double timestep,
                        const bool track_reference_positions) {
  const int thisIdx = parameters_->NumCars;
  const int nrCars = parameters_->NumCars + 1;
  const int nrStates = initialState.cols();
  const int nrSteps = settings_.nr_steps;
  parameters_->NumCars = nrCars;
  parameters_->NumCar2CarCollisions = nrCars * (nrCars - 1) / 2;

  parameters_->CollisionRadius.conservativeResize(nrCars);
  parameters_->CollisionRadius(thisIdx) = settings_.collisionRadius;
  parameters_->WheelBase.conservativeResize(nrCars);
  parameters_->WheelBase(thisIdx) = settings_.wheelBase;

  // Initial State
  parameters_->IntitialState.conservativeResize(nrCars, nrStates);

  // Acc and Jerk Limits
  LimitPerRegionParameters acc_limits =
      parameterPreparer_.CalculateAccLimitsPerCar();
  LimitPerRegionParameters jerk_limits =
      parameterPreparer_.CalculateJerkLimitsPerCar();

  const int nrRegions = parameters_->nr_regions;
  parameters_->acc_limit_params.min_x.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.max_x.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.min_y.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.max_y.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.min_x.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.max_x.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.min_y.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.max_y.conservativeResize(nrCars, nrRegions);

  parameters_->acc_limit_params.min_x.row(thisIdx) = acc_limits.min_x;
  parameters_->acc_limit_params.min_y.row(thisIdx) = acc_limits.min_y;
  parameters_->acc_limit_params.max_x.row(thisIdx) = acc_limits.max_x;
  parameters_->acc_limit_params.max_y.row(thisIdx) = acc_limits.max_y;
  parameters_->jerk_limit_params.min_x.row(thisIdx) = jerk_limits.min_x;
  parameters_->jerk_limit_params.min_y.row(thisIdx) = jerk_limits.min_y;
  parameters_->jerk_limit_params.max_x.row(thisIdx) = jerk_limits.max_x;
  parameters_->jerk_limit_params.max_y.row(thisIdx) = jerk_limits.max_y;

  // Global Acc and Jerk limits, +- epsilon
  parameters_->total_max_acc =
      std::max(parameters_->acc_limit_params.max_x.maxCoeff(),
               parameters_->acc_limit_params.max_y.maxCoeff()) +
      eps_;
  parameters_->total_min_acc =
      std::min(parameters_->acc_limit_params.min_x.minCoeff(),
               parameters_->acc_limit_params.min_y.minCoeff()) -
      eps_;
  parameters_->total_max_jerk =
      std::max(parameters_->jerk_limit_params.max_x.maxCoeff(),
               parameters_->jerk_limit_params.max_y.maxCoeff()) +
      eps_;
  parameters_->total_min_jerk =
      std::min(parameters_->jerk_limit_params.min_x.minCoeff(),
               parameters_->jerk_limit_params.min_y.minCoeff()) -
      eps_;

  // Reference
  parameters_->x_ref.conservativeResize(nrCars, nrSteps);
  parameters_->y_ref.conservativeResize(nrCars, nrSteps);
  parameters_->vx_ref.conservativeResize(nrCars, nrSteps);
  parameters_->vy_ref.conservativeResize(nrCars, nrSteps);

  double acc_lat_max = parameterPreparer_.GetVehicleParameters()
                           .straight_acc_limits.straight_lat_max.y;
  referenceGenerator_.push_back(ReferenceTrajectoryGenerator(
      nullptr, settings_.ts, settings_.nr_steps, settings_.refLineInterpInc,
      desiredVelocity, deltaSForDesiredVel, acc_lat_max, false));

  referenceGeneratorLongerHorizon_.push_back(ReferenceTrajectoryGenerator(
      nullptr, settings_.ts,
      settings_.nr_steps + settings_.additionalStepsForReferenceLongerHorizon,
      settings_.refLineInterpInc, desiredVelocity, deltaSForDesiredVel,
      acc_lat_max, false));

  // Possible + Initial Regions
  parameters_->possible_region.conservativeResize(nrCars, nrRegions);
  parameters_->initial_region.conservativeResize(nrCars);

  // Weights
  parameters_->WEIGHTS_POS_X.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_VEL_X.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_ACC_X.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_POS_Y.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_VEL_Y.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_ACC_Y.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_JERK_X.conservativeResize(parameters_->NumCars);
  parameters_->WEIGHTS_JERK_Y.conservativeResize(parameters_->NumCars);

  UpdateCar(thisIdx, initialState, referencePath, timestep,
            track_reference_positions);

  // Previous warmstart cannot be used as the sizes have changed
  validWarmstart_ = false;

  return thisIdx;
}

// Called periodically each timestep
void MiqpPlanner::UpdateCar(int thisIdx, Eigen::MatrixXd initialState,
                            const Line& referencePath, const double timestep,
                            bool track_reference_positions) {
  auto timer = bark::commons::timer::Timer();
  timer.Start();
  // Set Initial State
  parameters_->IntitialState.row(thisIdx) = initialState;

  // Calculate Reference
  State initialStateReference(
      static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  initialStateReference(TIME_POSITION) = timestep;
  initialStateReference(X_POSITION) = initialState(MIQP_STATE_X);
  initialStateReference(Y_POSITION) = initialState(MIQP_STATE_Y);
  initialStateReference(THETA_POSITION) =
      atan2(initialState(MIQP_STATE_VY), initialState(MIQP_STATE_VX));
  initialStateReference(VEL_POSITION) =
      sqrt(pow(initialState(MIQP_STATE_VX), 2) +
           pow(initialState(MIQP_STATE_VY), 2));

  Trajectory referenceTrajectory =
      referenceGenerator_.at(thisIdx).GenerateTrajectory(initialStateReference,
                                                         referencePath);

  parameters_->x_ref.row(thisIdx) =
      referenceTrajectory.col(StateDefinition::X_POSITION).cast<double>();
  parameters_->y_ref.row(thisIdx) =
      referenceTrajectory.col(StateDefinition::Y_POSITION).cast<double>();
  parameters_->vx_ref.row(thisIdx) =
      (referenceTrajectory.col(StateDefinition::VEL_POSITION).array() *
       referenceTrajectory.col(StateDefinition::THETA_POSITION).array().cos())
          .cast<double>();
  parameters_->vy_ref.row(thisIdx) =
      (referenceTrajectory.col(StateDefinition::VEL_POSITION).array() *
       referenceTrajectory.col(StateDefinition::THETA_POSITION).array().sin())
          .cast<double>();

  // Possible Regions
  Trajectory referenceTrajectoryPossibleRegions =
      referenceGeneratorLongerHorizon_.at(thisIdx).GenerateTrajectory(
          initialStateReference, referencePath);

  Eigen::VectorXd thetaRefPossibleRegions =
      referenceTrajectoryPossibleRegions.col(StateDefinition::THETA_POSITION)
          .array();
  std::set<RegionIdx> possible_regions = CalculatePossibleRegions(
      parameters_->fraction_parameters, thetaRefPossibleRegions);

  parameters_->possible_region.row(thisIdx) =
      RegionNumberToBinaryArray(possible_regions, settings_.nr_regions);
  bool suc = ReserveNeighborRegions(
      parameters_->possible_region, thisIdx,
      settings_.nr_neighbouring_possible_regions);  // 1: expand to direct
                                                    // neighbor regions
  LOG(INFO) << "Possible regions for car idx " << thisIdx << ": "
            << parameters_->possible_region.row(thisIdx);

  if (!suc) {
    LOG(ERROR) << "Region expansion failed at car idx = " << thisIdx
               << ", regions are " << parameters_->possible_region << std::endl;
  }

  // Calculate Weights

  double objectiveScale;
  if (thisIdx == egoCarIdx_) {
    objectiveScale = settings_.lambda;
  } else {
    objectiveScale = (1 - settings_.lambda) / parameters_->NumCars;
  }

  if (track_reference_positions) {
    parameters_->WEIGHTS_POS_X(thisIdx) =
        objectiveScale * settings_.positionWeight;
    parameters_->WEIGHTS_VEL_X(thisIdx) =
        objectiveScale * settings_.velocityWeight;

    parameters_->WEIGHTS_POS_Y(thisIdx) =
        objectiveScale * settings_.positionWeight;
    parameters_->WEIGHTS_VEL_Y(thisIdx) =
        objectiveScale * settings_.velocityWeight;
  } else {
    parameters_->WEIGHTS_POS_X(thisIdx) = 0;
    parameters_->WEIGHTS_VEL_X(thisIdx) =
        2;  // TODO move parameter to settings?

    parameters_->WEIGHTS_POS_Y(thisIdx) = 0;
    parameters_->WEIGHTS_VEL_Y(thisIdx) = 2;
  }
  parameters_->WEIGHTS_ACC_X(thisIdx) =
      objectiveScale * settings_.acclerationWeight;
  parameters_->WEIGHTS_ACC_Y(thisIdx) =
      objectiveScale * settings_.acclerationWeight;
  parameters_->WEIGHTS_JERK_X(thisIdx) = objectiveScale * settings_.jerkWeight;
  parameters_->WEIGHTS_JERK_Y(thisIdx) = objectiveScale * settings_.jerkWeight;

  // to filter out far away obstacles if parameter is set
  // TODO this could also be extended to all agents
  if (settings_.obstacle_roi_filter && thisIdx == egoCarIdx_) {
    const double theta =
        atan2(initialState(MIQP_STATE_VY), initialState(MIQP_STATE_VX));
    UpdateObstaclesROI(initialState(MIQP_STATE_X), initialState(MIQP_STATE_Y),
                       theta);
  }
  double duration = timer.DurationInSeconds();
  LOG(INFO) << "UpdateCar took Time [s] =" << duration;
}

int MiqpPlanner::AddStaticObstacle(bark::geometry::Polygon& obstacleShape) {
  Trajectory predictedTrajObstacle(
      settings_.nr_steps, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  State initialStateObstacle(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  initialStateObstacle << 0, obstacleShape.center_(0), obstacleShape.center_(1),
      0, 0;
  for (int i = 0; i < predictedTrajObstacle.rows(); ++i) {
    predictedTrajObstacle.row(i) = initialStateObstacle;
  }
  bool is_soft = false;
  return AddObstacle(predictedTrajObstacle, obstacleShape, is_soft, true);
}

int MiqpPlanner::AddObstacle(std::vector<Eigen::MatrixXd>& dynamic_obstacle,
                             bool is_soft, bool is_static) {
  // only add obstacle if it intersects at least one environment polygon
  if (ObstacleIntersectsEnvironment(dynamic_obstacle, is_static)) {
    parameters_->ObstacleConvexPolygon.push_back(dynamic_obstacle);
    parameters_->obstacle_is_soft.push_back((int)is_soft);
    parameters_->nr_obstacles = parameters_->ObstacleConvexPolygon.size();
    parameters_->max_lines_obstacles = 4;  // rectangle

    // Previous warmstart cannot be used as the sizes have changed
    validWarmstart_ = false;

    return parameters_->nr_obstacles - 1;  // ID of the obstacle
  } else {
    return -1;  // invalid id
  }
}

int MiqpPlanner::AddObstacle(Trajectory& predicted_traj,
                             bark::geometry::Polygon& shape, bool is_soft,
                             bool is_static) {
  std::vector<Eigen::MatrixXd> dynamic_obstacle =
      CreateMiqpObstacle(predicted_traj, shape);
  int id = AddObstacle(dynamic_obstacle, is_soft, is_static);
  return id;
}

void MiqpPlanner::UpdateObstacle(
    int id, std::vector<Eigen::MatrixXd>& dynamic_obstacle) {
  parameters_->ObstacleConvexPolygon.at(id) = dynamic_obstacle;
}

void MiqpPlanner::UpdateObstacle(int id, Trajectory& predicted_traj,
                                 bark::geometry::Polygon& shape) {
  std::vector<Eigen::MatrixXd> dynamic_obstacle =
      CreateMiqpObstacle(predicted_traj, shape);
  UpdateObstacle(id, dynamic_obstacle);
}

std::vector<Eigen::MatrixXd> MiqpPlanner::CreateMiqpObstacle(
    bark::models::dynamic::Trajectory& predicted_traj,
    bark::geometry::Polygon& shape) {
  assert(predicted_traj.rows() == parameters_->NumSteps);
  bark::geometry::Polygon shapebb;
  if (shape.obj_.outer().size() != 5) {
    shapebb = bark::geometry::CalculateBoundingBoxPolygon(shape);
    assert(shapebb.obj_.outer().size() == 5);
  } else {
    shapebb = shape;
  }

  std::vector<Eigen::MatrixXd> dynamic_obstacle;

  for (int i = 0; i < predicted_traj.rows(); ++i) {
    double x = predicted_traj(i, StateDefinition::X_POSITION);
    double y = predicted_traj(i, StateDefinition::Y_POSITION);
    double theta = predicted_traj(i, StateDefinition::THETA_POSITION);
    Pose pose = Pose(x, y, theta);

    std::shared_ptr<bark::geometry::Polygon> occupancy_polygon_ptr =
        std::dynamic_pointer_cast<bark::geometry::Polygon>(
            shapebb.Transform(pose));

    bark::geometry::Polygon occupancy_polygon = *occupancy_polygon_ptr;

    // inflate obstacle
    bark::geometry::Polygon buffered_occupancy_polygon;
    bark::geometry::BufferPolygon(occupancy_polygon, settings_.collisionRadius,
                                  &buffered_occupancy_polygon);
    bark::geometry::Polygon buffered_simplified_occupancy_polygon;
    // make sure the poly has only 4 edges
    boost::geometry::simplify(buffered_occupancy_polygon.obj_,
                              buffered_simplified_occupancy_polygon.obj_,
                              settings_.simplificationDistanceMap);

    Eigen::MatrixXd occupancy =
        miqp::common::geometry::Polygon2MiqpPolygonDefinition(
            buffered_simplified_occupancy_polygon);

    dynamic_obstacle.push_back(occupancy);
  }

  return dynamic_obstacle;
}

void MiqpPlanner::ResetEnvironment(
    std::vector<bark::models::dynamic::Trajectory>& referenceTrajectories) {
  miqp::common::geometry::PolygonMap convex_polygons_all_cars;
  // loop over every trajectory of every car
  for (bark::models::dynamic::Trajectory const& refTraj :
       referenceTrajectories) {
    miqp::common::geometry::PolygonMap polygon_map =
        convexifiedMap_.GetIntersectingConvexPolygons(refTraj);
    for (auto it = polygon_map.begin(); it != polygon_map.end(); ++it) {
      convex_polygons_all_cars.insert(*it);
    }
  }

  // save for debugging and easy access when intersecting obstacles
  convex_shrinked_env_polygons_all_cars_ = convex_polygons_all_cars;

  // store union of all convex polygons
  miqp::common::geometry::BoostMultiPolygon environment_union =
      miqp::common::geometry::BoostMultiPolygon();
  for (auto& p : convex_polygons_all_cars) {
    boost::geometry::union_(environment_union, p.second.obj_,
                            environment_union);
  }
  auto tmp = miqp::common::geometry::BoostPolygonToBarkPolygon(
      environment_union.at(0));
  if (environment_union.size() > 1) {
    LOG(WARNING) << "Merged convex environment polygons do not intersect!";
  }
  convex_shrinked_env_polygons_all_cars_union_.insert(std::pair(0, tmp));

  parameters_->MultiEnvironmentConvexPolygon.clear();
  parameters_->environmentPolygonIds.clear();
  parameters_->MultiEnvironmentConvexPolygon.reserve(
      convex_polygons_all_cars.size());
  for (auto it = convex_polygons_all_cars.begin();
       it != convex_polygons_all_cars.end(); ++it) {
    parameters_->MultiEnvironmentConvexPolygon.push_back(
        miqp::common::geometry::Polygon2MiqpPolygonDefinition(it->second));
    parameters_->environmentPolygonIds.push_back(it->first);
  }
  parameters_->nr_environments =
      parameters_->MultiEnvironmentConvexPolygon.size();

  // Alter Environment Warmstart variables
  if (validWarmstart_) {
    EnvironmentWarmstart();
  }
}

void MiqpPlanner::RemoveCar(int thisIdx) {
  if (thisIdx == egoCarIdx_) {
    LOG(ERROR) << "I cannot remove the ego vehicle with idx = " << thisIdx
               << "!" << std::endl;
    return;
  } else if (thisIdx >= parameters_->NumCars) {
    LOG(ERROR) << "No vehicle with idx = " << thisIdx << " here to delete!"
               << std::endl;
    return;
  } else if (thisIdx == parameters_->NumCars - 1) {
    // we delete the last index, nothing to shift in the matrices!
  } else {
    // we erase a middle row in the matrices; shift the content of the matrices
    // up

    LOG(ERROR) << "Deleting intermediate cars is not yet implmented!"
               << std::endl;
    throw NotImplementedException();
    return;
  }

  const int nrCars = parameters_->NumCars - 1;
  const int nrStates = parameters_->IntitialState.cols();
  const int nrSteps = settings_.nr_steps;
  const int nrRegions = settings_.nr_regions;
  parameters_->NumCars = nrCars;
  parameters_->NumCar2CarCollisions = nrCars * (nrCars - 1) / 2;

  // TODO TOBIAS  this is a code duplication with AddCar -> move to service
  // method.
  parameters_->CollisionRadius.conservativeResize(nrCars);
  parameters_->WheelBase.conservativeResize(nrCars);
  parameters_->IntitialState.conservativeResize(nrCars, nrStates);
  parameters_->acc_limit_params.min_x.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.max_x.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.min_y.conservativeResize(nrCars, nrRegions);
  parameters_->acc_limit_params.max_y.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.min_x.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.max_x.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.min_y.conservativeResize(nrCars, nrRegions);
  parameters_->jerk_limit_params.max_y.conservativeResize(nrCars, nrRegions);
  parameters_->x_ref.conservativeResize(nrCars, nrSteps);
  parameters_->y_ref.conservativeResize(nrCars, nrSteps);
  parameters_->vx_ref.conservativeResize(nrCars, nrSteps);
  parameters_->vy_ref.conservativeResize(nrCars, nrSteps);
  parameters_->possible_region.conservativeResize(nrCars, nrRegions);
  parameters_->initial_region.conservativeResize(nrCars);
  parameters_->WEIGHTS_POS_X.conservativeResize(nrCars);
  parameters_->WEIGHTS_VEL_X.conservativeResize(nrCars);
  parameters_->WEIGHTS_ACC_X.conservativeResize(nrCars);
  parameters_->WEIGHTS_POS_Y.conservativeResize(nrCars);
  parameters_->WEIGHTS_VEL_Y.conservativeResize(nrCars);
  parameters_->WEIGHTS_ACC_Y.conservativeResize(nrCars);
  parameters_->WEIGHTS_JERK_X.conservativeResize(nrCars);
  parameters_->WEIGHTS_JERK_Y.conservativeResize(nrCars);

  referenceGenerator_.erase(referenceGenerator_.begin() + thisIdx);
  referenceGeneratorLongerHorizon_.erase(
      referenceGeneratorLongerHorizon_.begin() + thisIdx);

  // Global Acc and Jerk limits, +- epsilon
  parameters_->total_max_acc =
      std::max(parameters_->acc_limit_params.max_x.maxCoeff(),
               parameters_->acc_limit_params.max_y.maxCoeff()) +
      eps_;
  parameters_->total_min_acc =
      std::min(parameters_->acc_limit_params.min_x.minCoeff(),
               parameters_->acc_limit_params.min_y.minCoeff()) -
      eps_;
  parameters_->total_max_jerk =
      std::max(parameters_->jerk_limit_params.max_x.maxCoeff(),
               parameters_->jerk_limit_params.max_y.maxCoeff()) +
      eps_;
  parameters_->total_min_jerk =
      std::min(parameters_->jerk_limit_params.min_x.minCoeff(),
               parameters_->jerk_limit_params.min_y.minCoeff()) -
      eps_;
}

void MiqpPlanner::RemoveObstacle(int id) { throw NotImplementedException(); }

void MiqpPlanner::RemoveAllObstacles() {
  parameters_->ObstacleConvexPolygon.clear();
  parameters_->nr_obstacles = parameters_->ObstacleConvexPolygon.size();
  parameters_->obstacle_is_soft.clear();
  parameters_->max_lines_obstacles = 0;

  // Previous warmstart cannot be used as the sizes have changed
  validWarmstart_ = false;
}

const std::shared_ptr<ModelParameters> MiqpPlanner::GetParameters() {
  return parameters_;
}

bool MiqpPlanner::Plan(const double timestemp) {
  // Calculate initial regions
  std::vector<Regions> regionIndices(parameters_->NumCars);
  for (int carIdx = 0; carIdx < parameters_->NumCars; ++carIdx) {
    regionIndices.at(carIdx) =
        CalculateRegionIdx(parameterPreparer_.GetFractionParameters(),
                           parameters_->IntitialState(carIdx, MIQP_STATE_VX),
                           parameters_->IntitialState(carIdx, MIQP_STATE_VY));
  }

  // Calculate all region combinations
  std::vector<std::vector<RegionIdx>> regionsCombinations;
  std::vector<RegionIdx> accumulated;
  CalculateRegionCombinations(regionIndices, accumulated, regionsCombinations);

  if (convexifiedMap_.HasValidPolygon()) {
    // Update Environment
    std::vector<Trajectory> referenceTrajectoriesLongerHorizon =
        CalculateReferenceTrajectoriesLongerHorizon();
    ResetEnvironment(referenceTrajectoriesLongerHorizon);

    // Check if initial car positions are within one of the environment polys
    for (int caridx = 0; caridx < parameters_->NumCars; ++caridx) {
      bool foundrear = false;
      bool foundfront = false;
      const bark::geometry::Point2d pt(parameters_->IntitialState(caridx, 0),
                                       parameters_->IntitialState(caridx, 3));
      const double theta = atan2(parameters_->IntitialState(caridx, 4),
                                 parameters_->IntitialState(caridx, 1));
      const bark::geometry::Point2d frontpt(
          parameters_->IntitialState(caridx, 0) +
              cos(theta) * parameters_->WheelBase(caridx),
          parameters_->IntitialState(caridx, 3) +
              sin(theta) * parameters_->WheelBase(caridx));
      for (auto& poly : convex_shrinked_env_polygons_all_cars_) {
        if (bark::geometry::Within(pt, poly.second)) {
          foundrear = true;
        }
        if (bark::geometry::Within(frontpt, poly.second)) {
          foundfront = true;
        }
      }
      if (!foundrear) {
        LOG(ERROR) << "Initial pose rear collides for car idx = " << caridx;
        return false;
      }
      if (!foundfront) {
        LOG(ERROR) << "Initial pose front collides for car idx = " << caridx;
        LOG(ERROR) << "Front pt = " << boost::geometry::get<0>(frontpt) << " "
                   << boost::geometry::get<1>(frontpt) << " theta = " << theta;
        return false;
      }
    }
  } else {
    LOG(INFO) << "No valid environment polygon, probably an empty one. This is "
                 "not a bug, I am using an empty environment polygon!";
  }

  // Loop over region-combinations and call optimization
  OptimizationStatus status;
  std::vector<bool> rollBackPossibleRegion(parameters_->NumCars);
  for (auto& thisRegionCombination : regionsCombinations) {
    // Region combination selection (+ 1 as cplex is 1-based, c++ 0-based)
    parameters_->initial_region =
        Eigen::Map<Eigen::VectorXi>(thisRegionCombination.data(),
                                    parameters_->NumCars, 1)
            .array() +
        1;

    // Make sure the start region is a possible region.
    for (int caridx = 0; caridx < parameters_->NumCars; ++caridx) {
      const int regidx = parameters_->initial_region(caridx) - 1;
      if (parameters_->possible_region(caridx, regidx) == 0) {
        parameters_->possible_region(caridx, regidx) = 1;
        rollBackPossibleRegion.at(caridx) = true;
        // TODO here we could fill gaps if this is a problem
      } else {
        rollBackPossibleRegion.at(caridx) = false;
      }
    }

    // Add warmstart to optimization problem
    LOG(INFO) << "warmstart status: do = " << doWarmstart_
              << " valid = " << validWarmstart_ << std::endl;
    if ((doWarmstart_ ==
             CplexWrapper::WarmstartType::RECEDING_HORIZON_WARMSTART ||
         doWarmstart_ ==
             CplexWrapper::WarmstartType::BOTH_WARMSTART_STRATEGIES) &&
        validWarmstart_) {
      cplexWrapper_.addRecedingHorizonWarmstart(warmstart_, doWarmstart_);
    }
    if (doWarmstart_ == CplexWrapper::WarmstartType::LAST_SOLUTION_WARMSTART ||
        doWarmstart_ ==
            CplexWrapper::WarmstartType::BOTH_WARMSTART_STRATEGIES) {
      cplexWrapper_.setLastSolutionWarmstart(doWarmstart_);
    }

    // Call optimization
    status = cplexWrapper_.callCplex(timestemp);
    if (status == OptimizationStatus::SUCCESS) {
      break;
    } else if (status == OptimizationStatus::FAILED_SEG_FAULT ||
               status == OptimizationStatus::FAILED_TIMEOUT) {
      LOG(ERROR) << "Optimization failed! Reason: " << status << std::endl;
      return false;
    }

    LOG(INFO) << "Resetting all variables";
    // Reset possible regions to original
    for (int caridx = 0; caridx < parameters_->NumCars; ++caridx) {
      if (rollBackPossibleRegion.at(caridx)) {
        const int regidx = parameters_->initial_region(caridx) - 1;
        parameters_->possible_region(caridx, regidx) = 0;
      }
    }
    LOG(INFO) << "Resetted all variables: done";
  }
  validWarmstart_ = false;
  if (status == OptimizationStatus::SUCCESS) {
    // successful optimization
    // Compute warmstart values from last results
    if (doWarmstart_ ==
            CplexWrapper::WarmstartType::RECEDING_HORIZON_WARMSTART ||
        doWarmstart_ ==
            CplexWrapper::WarmstartType::BOTH_WARMSTART_STRATEGIES) {
      CalculateWarmstart();
    }
    return true;
  } else {
    // no region combination produced a successful result -> fail
    LOG(ERROR) << "Optimization failed!" << std::endl;
    return false;
  }
}

std::vector<Trajectory> MiqpPlanner::CalculateReferenceTrajectories() const {
  std::vector<Trajectory> referenceTrajectories;
  referenceTrajectories.reserve(parameters_->NumCars);
  for (auto& refGen : referenceGenerator_) {
    referenceTrajectories.push_back(refGen.GetLastTrajectory());
  }
  return referenceTrajectories;
}

std::vector<Trajectory>
MiqpPlanner::CalculateReferenceTrajectoriesLongerHorizon() const {
  std::vector<Trajectory> referenceTrajectories;
  referenceTrajectories.reserve(parameters_->NumCars);
  for (auto& refGen : referenceGeneratorLongerHorizon_) {
    referenceTrajectories.push_back(refGen.GetLastTrajectory());
  }
  return referenceTrajectories;
}

void MiqpPlanner::CalculateWarmstart() {
  std::shared_ptr<RawResults> rr = cplexWrapper_.getRawResults();
  const int N = rr->N;
  const int NrCars = rr->NrCars;
  const int NrEnvironments = rr->NrEnvironments;
  const int NrRegions = rr->NrRegions;
  const int NrCarToCarCollisions = rr->NrCarToCarCollisions;
  const int MaxLinesObstacles = rr->MaxLinesObstacles;
  const int NrObstacles = rr->NrObstacles;

  if (rr->NrEnvironments != warmstart_->NrEnvironments) {
    warmstart_->NrEnvironments = rr->NrEnvironments;
    warmstart_->notWithinEnvironmentRear.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontLbLb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontLbUb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontUbLb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontUbUb.resize(NrCars, NrEnvironments, N);
  }
  if (rr->NrObstacles != warmstart_->NrObstacles) {
    warmstart_->NrObstacles = rr->NrObstacles;
    warmstart_->deltacc.resize(NrCars, NrObstacles, N, MaxLinesObstacles);
    warmstart_->deltacc_front.resize(NrCars, NrObstacles, N, MaxLinesObstacles,
                                     4);
    warmstart_->MaxLinesObstacles = MaxLinesObstacles;
  }
  if (rr->NrCars != warmstart_->NrCars) {
    warmstart_->NrCars = rr->NrCars;
    warmstart_->NrCarToCarCollisions = rr->NrCarToCarCollisions;
    warmstart_->u_x.resize(NrCars, N);
    warmstart_->u_y.resize(NrCars, N);
    warmstart_->pos_x.resize(NrCars, N);
    warmstart_->vel_x.resize(NrCars, N);
    warmstart_->acc_x.resize(NrCars, N);
    warmstart_->pos_y.resize(NrCars, N);
    warmstart_->vel_y.resize(NrCars, N);
    warmstart_->acc_y.resize(NrCars, N);
    warmstart_->pos_x_front_UB.resize(NrCars, N);
    warmstart_->pos_x_front_LB.resize(NrCars, N);
    warmstart_->pos_y_front_UB.resize(NrCars, N);
    warmstart_->pos_y_front_LB.resize(NrCars, N);
    warmstart_->notWithinEnvironmentRear.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontLbLb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontLbUb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontUbLb.resize(NrCars, NrEnvironments, N);
    warmstart_->notWithinEnvironmentFrontUbUb.resize(NrCars, NrEnvironments, N);
    warmstart_->active_region.resize(NrCars, N, NrRegions);
    warmstart_->region_change_not_allowed_combined.resize(NrCars, N);
    warmstart_->region_change_not_allowed_x_negative.resize(NrCars, N);
    warmstart_->region_change_not_allowed_x_positive.resize(NrCars, N);
    warmstart_->region_change_not_allowed_y_negative.resize(NrCars, N);
    warmstart_->region_change_not_allowed_y_positive.resize(NrCars, N);
    warmstart_->car2car_collision.resize(NrCarToCarCollisions, N, 16);
    warmstart_->slackvars.resize(NrCarToCarCollisions, N, 4);
  }

  // Warmstart model parameters
  {
    Eigen::array<long, 2> offset = {0, 0};
    Eigen::array<long, 2> extent = {NrCars, N - 1};
    Eigen::array<long, 2> offset_shift = {0, 1};
    warmstart_->u_x.slice(offset, extent) = rr->u_x.slice(offset_shift, extent);
    warmstart_->u_y.slice(offset, extent) = rr->u_y.slice(offset_shift, extent);
    warmstart_->pos_x.slice(offset, extent) =
        rr->pos_x.slice(offset_shift, extent);
    warmstart_->vel_x.slice(offset, extent) =
        rr->vel_x.slice(offset_shift, extent);
    warmstart_->acc_x.slice(offset, extent) =
        rr->acc_x.slice(offset_shift, extent);
    warmstart_->pos_y.slice(offset, extent) =
        rr->pos_y.slice(offset_shift, extent);
    warmstart_->vel_y.slice(offset, extent) =
        rr->vel_y.slice(offset_shift, extent);
    warmstart_->acc_y.slice(offset, extent) =
        rr->acc_y.slice(offset_shift, extent);
    warmstart_->pos_x_front_UB.slice(offset, extent) =
        rr->pos_x_front_UB.slice(offset_shift, extent);
    warmstart_->pos_x_front_LB.slice(offset, extent) =
        rr->pos_x_front_LB.slice(offset_shift, extent);
    warmstart_->pos_y_front_UB.slice(offset, extent) =
        rr->pos_y_front_UB.slice(offset_shift, extent);
    warmstart_->pos_y_front_LB.slice(offset, extent) =
        rr->pos_y_front_LB.slice(offset_shift, extent);
    warmstart_->region_change_not_allowed_combined.slice(offset, extent) =
        rr->region_change_not_allowed_combined.slice(offset_shift, extent);
    warmstart_->region_change_not_allowed_x_negative.slice(offset, extent) =
        rr->region_change_not_allowed_x_negative.slice(offset_shift, extent);
    warmstart_->region_change_not_allowed_x_positive.slice(offset, extent) =
        rr->region_change_not_allowed_x_positive.slice(offset_shift, extent);
    warmstart_->region_change_not_allowed_y_negative.slice(offset, extent) =
        rr->region_change_not_allowed_y_negative.slice(offset_shift, extent);
    warmstart_->region_change_not_allowed_y_positive.slice(offset, extent) =
        rr->region_change_not_allowed_y_positive.slice(offset_shift, extent);

    offset = {0, N - 1};
    extent = {NrCars, 1};
    double ts = parameters_->ts;
    warmstart_->u_x.slice(offset, extent).setZero();
    warmstart_->u_y.slice(offset, extent).setZero();
    for (int carIdx = 0; carIdx < NrCars; ++carIdx) {
      warmstart_->pos_x(carIdx, N - 1) = warmstart_->pos_x(carIdx, N - 2) +
                                         ts * warmstart_->vel_x(carIdx, N - 2);
      warmstart_->pos_y(carIdx, N - 1) = warmstart_->pos_y(carIdx, N - 2) +
                                         ts * warmstart_->vel_y(carIdx, N - 2);
      warmstart_->vel_x(carIdx, N - 1) = warmstart_->vel_x(carIdx, N - 2) +
                                         ts * warmstart_->acc_x(carIdx, N - 2);
      warmstart_->vel_y(carIdx, N - 1) = warmstart_->vel_y(carIdx, N - 2) +
                                         ts * warmstart_->acc_y(carIdx, N - 2);
      warmstart_->acc_x(carIdx, N - 1) = warmstart_->acc_x(carIdx, N - 2) +
                                         ts * warmstart_->u_x(carIdx, N - 2);
      warmstart_->acc_y(carIdx, N - 1) = warmstart_->acc_y(carIdx, N - 2) +
                                         ts * warmstart_->u_y(carIdx, N - 2);

      warmstart_->pos_x_front_UB(carIdx, N - 1) =
          warmstart_->pos_x_front_UB(carIdx, N - 2) +
          ts * warmstart_->vel_x(carIdx, N - 2);
      warmstart_->pos_x_front_LB(carIdx, N - 1) =
          warmstart_->pos_x_front_LB(carIdx, N - 2) +
          ts * warmstart_->vel_x(carIdx, N - 2);
      warmstart_->pos_y_front_UB(carIdx, N - 1) =
          warmstart_->pos_y_front_UB(carIdx, N - 2) +
          ts * warmstart_->vel_y(carIdx, N - 2);
      warmstart_->pos_y_front_LB(carIdx, N - 1) =
          warmstart_->pos_y_front_LB(carIdx, N - 2) +
          ts * warmstart_->vel_y(carIdx, N - 2);

      warmstart_->region_change_not_allowed_x_positive(carIdx, N - 1) =
          static_cast<int>(warmstart_->vel_x(carIdx, N - 1) <=
                           parameters_->minimum_region_change_speed);
      warmstart_->region_change_not_allowed_y_positive(carIdx, N - 1) =
          static_cast<int>(warmstart_->vel_y(carIdx, N - 1) <=
                           parameters_->minimum_region_change_speed);
      warmstart_->region_change_not_allowed_x_negative(carIdx, N - 1) =
          static_cast<int>(warmstart_->vel_x(carIdx, N - 1) >=
                           -parameters_->minimum_region_change_speed);
      warmstart_->region_change_not_allowed_y_negative(carIdx, N - 1) =
          static_cast<int>(warmstart_->vel_y(carIdx, N - 1) >=
                           -parameters_->minimum_region_change_speed);
      warmstart_->region_change_not_allowed_combined(carIdx, N - 1) =
          static_cast<int>(
              (warmstart_->region_change_not_allowed_x_positive(carIdx, N - 1) +
               warmstart_->region_change_not_allowed_y_positive(carIdx, N - 1) +
               warmstart_->region_change_not_allowed_x_negative(carIdx, N - 1) +
               warmstart_->region_change_not_allowed_y_negative(carIdx,
                                                                N - 1)) > 3);
    }
  }

  if (NrEnvironments > 0) {
    Eigen::array<long, 3> offset = {0, 0, 0};
    Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N - 1};
    Eigen::array<long, 3> offset_shift = {0, 0, 1};
    warmstart_->notWithinEnvironmentRear.slice(offset, extent) =
        rr->notWithinEnvironmentRear.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontUbUb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontUbUb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontUbLb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontUbLb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontLbUb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontLbUb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontLbLb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontLbLb.slice(offset_shift, extent);

    offset = {0, 0, N - 1};
    extent = {NrCars, NrEnvironments, 1};
    offset_shift = {0, 0, N - 1};
    warmstart_->notWithinEnvironmentRear.slice(offset, extent) =
        rr->notWithinEnvironmentRear.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontUbUb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontUbUb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontUbLb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontUbLb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontLbUb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontLbUb.slice(offset_shift, extent);
    warmstart_->notWithinEnvironmentFrontLbLb.slice(offset, extent) =
        rr->notWithinEnvironmentFrontLbLb.slice(offset_shift, extent);
  }

  // Environment warmstart is only done when the new environment polygons are
  // added, must not be done here, as it also shifts by one!!

  // Warmstart active region
  {
    warmstart_->active_region.setZero();
    Eigen::array<long, 3> offset = {0, 0, 0};
    Eigen::array<long, 3> extent = {NrCars, N - 1, NrRegions};
    Eigen::array<long, 3> offset_shift = {0, 1, 0};
    warmstart_->active_region.slice(offset, extent) =
        rr->active_region.slice(offset_shift, extent);

    for (int carIdx = 0; carIdx < NrCars; ++carIdx) {
      Regions region_indices = CalculateRegionIdx(
          parameterPreparer_.GetFractionParameters(), rr->vel_x(carIdx, N - 1),
          rr->vel_y(carIdx, N - 1));
      warmstart_->active_region(carIdx, N - 1, region_indices.begin()->second);
    }
  }

  // Warmstart car2car collisions
  // TODO Tobias: Implementations untested!
  if (NrCarToCarCollisions > 0) {
    {
      Eigen::array<long, 3> offset = {0, 0, 0};
      Eigen::array<long, 3> extent = {NrCarToCarCollisions, N - 1, 16};
      Eigen::array<long, 3> offset_shift = {0, 1, 0};
      warmstart_->car2car_collision.slice(offset, extent) =
          rr->car2car_collision.slice(offset_shift, extent);

      offset = {0, N - 1, 0};
      extent = {NrCarToCarCollisions, 1, 16};
      offset_shift = {0, N - 1, 0};
      warmstart_->car2car_collision.slice(offset, extent) =
          rr->car2car_collision.slice(offset_shift, extent);
    }
    {
      Eigen::array<long, 3> offset = {0, 0, 0};
      Eigen::array<long, 3> extent = {NrCarToCarCollisions, N - 1, 4};
      Eigen::array<long, 3> offset_shift = {0, 1, 0};
      warmstart_->slackvars.slice(offset, extent) =
          rr->slackvars.slice(offset_shift, extent);

      offset = {0, N - 1, 0};
      extent = {NrCarToCarCollisions, 1, 4};
      offset_shift = {0, N - 1, 0};
      warmstart_->slackvars.slice(offset, extent) =
          rr->slackvars.slice(offset_shift, extent);
    }
  }

  // Warmstart obstacle collisions
  if (NrObstacles > 0) {
    {
      Eigen::array<long, 4> offset = {0, 0, 0, 0};
      Eigen::array<long, 4> extent = {NrCars, NrObstacles, N - 1,
                                      MaxLinesObstacles};
      Eigen::array<long, 4> offset_shift = {0, 0, 1, 0};
      warmstart_->deltacc.slice(offset, extent) = rr->deltacc.slice(
          offset_shift,
          extent);  //(NrCars, NrObstacles,  N,  MaxLinesObstacles);
      offset = {0, 0, N - 1, 0};
      extent = {NrCars, NrObstacles, 1, MaxLinesObstacles};
      offset_shift = {0, 0, N - 1, 0};
      warmstart_->deltacc.slice(offset, extent) =
          rr->deltacc.slice(offset_shift, extent);
    }
    {
      Eigen::array<long, 5> offset = {0, 0, 0, 0, 0};
      Eigen::array<long, 5> extent = {NrCars, NrObstacles, N - 1,
                                      MaxLinesObstacles, 4};
      Eigen::array<long, 5> offset_shift = {0, 0, 1, 0, 0};
      warmstart_->deltacc_front.slice(offset, extent) =
          rr->deltacc_front.slice(offset_shift, extent);
      offset = {0, 0, N - 1, 0, 0};
      extent = {NrCars, NrObstacles, 1, MaxLinesObstacles, 4};
      offset_shift = {0, 0, N - 1, 0, 0};
      warmstart_->deltacc_front.slice(offset, extent) =
          rr->deltacc_front.slice(offset_shift, extent);
    }
  }

  validWarmstart_ = true;
}

void MiqpPlanner::EnvironmentWarmstart() {
  const int NumSteps = parameters_->NumSteps;
  const int NumCars = parameters_->NumCars;
  const int nr_environments = parameters_->nr_environments;
  if (parameters_->environmentPolygonIds != environmentIdsWarmstart_) {
    Eigen::Tensor<int, 3> notWithinEnvironmentRear_lastrun =
        warmstart_->notWithinEnvironmentRear;  // store last result in temp var
    warmstart_->notWithinEnvironmentRear.resize(NumCars, nr_environments,
                                                NumSteps);
    warmstart_->notWithinEnvironmentRear.setConstant(
        1);  // initialize with ones, aka. not in this env poly
    Eigen::Tensor<int, 3> notWithinEnvironmentFrontUbUb_lastrun =
        warmstart_->notWithinEnvironmentFrontUbUb;
    warmstart_->notWithinEnvironmentFrontUbUb.resize(NumCars, nr_environments,
                                                     NumSteps);
    warmstart_->notWithinEnvironmentFrontUbUb.setConstant(1);
    Eigen::Tensor<int, 3> notWithinEnvironmentFrontUbLb_lastrun =
        warmstart_->notWithinEnvironmentFrontUbLb;
    warmstart_->notWithinEnvironmentFrontUbLb.resize(NumCars, nr_environments,
                                                     NumSteps);
    warmstart_->notWithinEnvironmentFrontUbLb.setConstant(1);
    Eigen::Tensor<int, 3> notWithinEnvironmentFrontLbUb_lastrun =
        warmstart_->notWithinEnvironmentFrontLbUb;
    warmstart_->notWithinEnvironmentFrontLbUb.resize(NumCars, nr_environments,
                                                     NumSteps);
    warmstart_->notWithinEnvironmentFrontLbUb.setConstant(1);
    Eigen::Tensor<int, 3> notWithinEnvironmentFrontLbLb_lastrun =
        warmstart_->notWithinEnvironmentFrontLbLb;
    warmstart_->notWithinEnvironmentFrontLbLb.resize(NumCars, nr_environments,
                                                     NumSteps);
    warmstart_->notWithinEnvironmentFrontLbLb.setConstant(1);

    for (int idxCar = 0; idxCar < NumCars; ++idxCar) {
      for (int idxEnv = 0; idxEnv < nr_environments; ++idxEnv) {
        auto idxEnvWarmstart_it = std::find(
            environmentIdsWarmstart_.begin(), environmentIdsWarmstart_.end(),
            parameters_->environmentPolygonIds.at(idxEnv));
        if (idxEnvWarmstart_it != environmentIdsWarmstart_.end()) {
          int envIdxToCopy = std::distance(environmentIdsWarmstart_.begin(),
                                           idxEnvWarmstart_it);
          Eigen::array<long, 3> offset = {idxCar, idxEnv, 0};
          Eigen::array<long, 3> extent = {1, 1, NumSteps - 1};
          Eigen::array<long, 3> offset_lastrun = {idxCar, envIdxToCopy, 0};
          warmstart_->notWithinEnvironmentRear.slice(offset, extent) =
              notWithinEnvironmentRear_lastrun.slice(offset_lastrun, extent);
          warmstart_->notWithinEnvironmentFrontUbUb.slice(offset, extent) =
              notWithinEnvironmentFrontUbUb_lastrun.slice(offset_lastrun,
                                                          extent);
          warmstart_->notWithinEnvironmentFrontLbUb.slice(offset, extent) =
              notWithinEnvironmentFrontLbUb_lastrun.slice(offset_lastrun,
                                                          extent);
          warmstart_->notWithinEnvironmentFrontUbLb.slice(offset, extent) =
              notWithinEnvironmentFrontUbLb_lastrun.slice(offset_lastrun,
                                                          extent);
          warmstart_->notWithinEnvironmentFrontLbLb.slice(offset, extent) =
              notWithinEnvironmentFrontLbLb_lastrun.slice(offset_lastrun,
                                                          extent);
        }
      }
    }
    environmentIdsWarmstart_ = parameters_->environmentPolygonIds;
  }
}

Eigen::MatrixXd MiqpPlanner::Get2ndOrderStateFromSolution(
    const int timeIdx, const int carIdx) const {
  const std::shared_ptr<RawResults> rawResults = cplexWrapper_.getRawResults();
  assert(carIdx < rawResults->NrCars);

  Eigen::MatrixXd state(1, 6);
  state(0) = rawResults->pos_x(carIdx, timeIdx);
  state(1) = rawResults->vel_x(carIdx, timeIdx);
  state(2) = rawResults->acc_x(carIdx, timeIdx);
  state(3) = rawResults->pos_y(carIdx, timeIdx);
  state(4) = rawResults->vel_y(carIdx, timeIdx);
  state(5) = rawResults->acc_y(carIdx, timeIdx);
  return state;
}

bark::models::dynamic::Trajectory MiqpPlanner::GetBarkTrajectory(
    const int carIdx, const float start_time) const {
  const std::shared_ptr<RawResults> rawResults = cplexWrapper_.getRawResults();
  assert(carIdx < rawResults->NrCars);
  const int N = rawResults->N;
  const float dt = parameters_->ts;

  Eigen::VectorXd time(N), x(N), y(N), theta(N), vel(N);
  for (int i = 0; i < N; ++i) {
    time(i) = start_time + i * dt;
    x(i) = rawResults->pos_x(carIdx, i);
    y(i) = rawResults->pos_y(carIdx, i);
    theta(i) =
        atan2(rawResults->vel_y(carIdx, i), rawResults->vel_x(carIdx, i));
    vel(i) = sqrt(pow(rawResults->vel_x(carIdx, i), 2) +
                  pow(rawResults->vel_y(carIdx, i), 2));
  }

  Trajectory trajBark(N, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  trajBark.col(StateDefinition::TIME_POSITION) = std::move(time);
  trajBark.col(StateDefinition::X_POSITION) = std::move(x);
  trajBark.col(StateDefinition::Y_POSITION) = std::move(y);
  trajBark.col(StateDefinition::THETA_POSITION) = std::move(theta);
  trajBark.col(StateDefinition::VEL_POSITION) = std::move(vel);
  return trajBark;
}

Eigen::MatrixXd MiqpPlanner::CarStateToMiqpState(float x, float y, float theta,
                                                 float v, float a) {
  Eigen::MatrixXd state(1, 6);
  state(0, MIQP_STATE_X) = static_cast<double>(x);
  state(0, MIQP_STATE_Y) = static_cast<double>(y);
  state(0, MIQP_STATE_VX) = static_cast<double>(cos(theta) * v);
  state(0, MIQP_STATE_VY) = static_cast<double>(sin(theta) * v);
  state(0, MIQP_STATE_AX) = static_cast<double>(cos(theta) * a);
  state(0, MIQP_STATE_AY) = static_cast<double>(sin(theta) * a);
  return state;
}

void MiqpPlanner::ActivateDebugFileWrite(std::string path, std::string name) {
  cplexWrapper_.setDebugOutputFilePath(path);
  cplexWrapper_.setDebugOutputFilePrefix(name);
  cplexWrapper_.setDebugOutputPrint(true);
}

bool MiqpPlanner::UpdateConvexifiedMap(
    const bark::geometry::Polygon& mapPolygon) {
  size_t hash_new = miqp::common::geometry::GetHash(mapPolygon);
  size_t hash_old =
      miqp::common::geometry::GetHash(convexifiedMap_.GetMapNonConvexPolygon());
  if (hash_old != hash_new) {
    convexifiedMap_.SetMapPolygon(mapPolygon);
    bool succ = convexifiedMap_.Convert();
    LOG(INFO) << "Map Polygon did change, converted new one";
    return succ;
  } else {
    LOG(INFO) << "Map Polygon did not change, reusing previous one";
    return true;
  }
}

void MiqpPlanner::SetDoWarmstart(
    miqp::planner::cplex::CplexWrapper::WarmstartType in) {
  doWarmstart_ = in;
  LOG(INFO) << "Warmstart switched to: " << in;
}

void MiqpPlanner::UpdateDesiredVelocity(const int carIdx, const double vDes,
                                        const double deltaSDes) {
  assert(static_cast<int>(referenceGenerator_.size()) > carIdx);
  referenceGenerator_.at(carIdx).ResetDesiredVelocity(vDes, deltaSDes);
  assert(static_cast<int>(referenceGeneratorLongerHorizon_.size()) > carIdx);
  referenceGeneratorLongerHorizon_.at(carIdx).ResetDesiredVelocity(vDes,
                                                                   deltaSDes);
  LOG(INFO) << "Setting desired velocity for car idx = " << carIdx
            << " to v = " << vDes << " at deltaSDes=" << deltaSDes;
}

bark::models::dynamic::Trajectory MiqpPlanner::GetLastReference(
    int carIdx) const {
  assert(static_cast<int>(referenceGenerator_.size()) > carIdx);
  bark::models::dynamic::Trajectory traj =
      referenceGenerator_.at(carIdx).GetLastTrajectory();
  return traj;
}

bool MiqpPlanner::ObstacleIntersectsEnvironment(
    std::vector<Eigen::MatrixXd>& obstacle_vector, bool is_static) {
  // in case of an empty environment, add all obstacles
  if (convexifiedMap_.GetMapConvexPolygons().empty()) {
    LOG(INFO) << "poly is empty!";
    return true;
  }

  // for non-empty environments, if in the last planning step an subset of the
  // environment polygon was active use this subset to check for obstacle
  // intersections
  // TODO avoid copy here?
  miqp::common::geometry::PolygonMap environment_polygons;
  if (!convex_shrinked_env_polygons_all_cars_.empty()) {
    // usage without the union_ suffix is also possible!
    environment_polygons = convex_shrinked_env_polygons_all_cars_union_;
  } else {
    environment_polygons = convexifiedMap_.GetMapConvexPolygons();
  }

  // loop over all timesteps in the obstacle trajectory and all environments
  for (auto& obstacle : obstacle_vector) {
    bark::geometry::Polygon obstacle_poly;
    // TODO can we fill more elegantly from an eigen matrix?
    obstacle_poly.AddPoint({obstacle(0, 0), obstacle(0, 1)});
    obstacle_poly.AddPoint({obstacle(1, 0), obstacle(1, 1)});
    obstacle_poly.AddPoint({obstacle(2, 0), obstacle(2, 1)});
    obstacle_poly.AddPoint({obstacle(3, 0), obstacle(3, 1)});
    obstacle_poly.AddPoint({obstacle(0, 0), obstacle(0, 1)});

    if (!boost::geometry::is_empty(obstacles_roi_.obj_) &&
        !boost::geometry::intersects(obstacles_roi_.obj_, obstacle_poly.obj_)) {
      if (is_static) {
        // static obstacle outside ROI -> no relevant obstacle
        return false;
      } else {
        // non-static: next timestep could be relevant -> continue
        continue;
      }
    }

    for (auto& environment : environment_polygons) {
      if (boost::geometry::intersects(environment.second.obj_,
                                      obstacle_poly.obj_)) {
        return true;  // on the first intersection, we can safely return true
      }
    }

    // In case the obstacle is static we only need to check at time index 0. If
    // we found something here, we already returned true, otherwise we can
    // safely return false now.
    if (is_static) {
      return false;
    }
  }

  // no intersection found at all
  return false;
}

void MiqpPlanner::UpdateObstaclesROI(const double x, const double y,
                                     const double theta) {
  double behind_dist = settings_.obstacle_roi_behind_distance;
  double front_dist = settings_.obstacle_roi_front_distance;
  double width = settings_.obstacle_roi_side_distance;

  // a rectangle around the car with car orientation, "infinite" to the sides
  // and in front but limited to the bark
  double frontx = x + cos(theta) * front_dist;
  double fronty = y + sin(theta) * front_dist;
  double frontxu = frontx + sin(theta) * width;
  double frontyu = fronty + cos(theta) * width;
  double frontxl = frontx - sin(theta) * width;
  double frontyl = fronty - cos(theta) * width;
  double rearx = x + cos(theta + M_PI) * behind_dist;
  double reary = y + sin(theta + M_PI) * behind_dist;
  double rearxu = rearx + sin(theta) * width;
  double rearyu = reary + cos(theta) * width;
  double rearxl = rearx - sin(theta) * width;
  double rearyl = reary - cos(theta) * width;

  bark::geometry::Polygon poly;
  poly.AddPoint({frontxu, frontyu});
  poly.AddPoint({frontxl, frontyl});
  poly.AddPoint({rearxl, rearyl});
  poly.AddPoint({rearxu, rearyu});
  poly.AddPoint({frontxu, frontyu});
  obstacles_roi_ = poly;
}

}  // namespace planner
}  // namespace miqp