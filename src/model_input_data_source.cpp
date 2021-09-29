// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <iostream>

#include "model_input_data_source.hpp"

namespace miqp {
namespace planner {

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const float& item) const {
  handler.startElement(name);
  double value = RoundWithPrecision(item, precision_);
  handler.addNumItem(value);
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name, const int& item) const {
  handler.startElement(name);
  handler.addIntItem(item);
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const Eigen::VectorXd& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.rows(); ++i) {
    double value = RoundWithPrecision(item(i), precision_);
    handler.addNumItem(value);
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const Eigen::MatrixXd& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.rows(); ++i) {
    handler.startArray();
    for (int j = 0; j < item.cols(); ++j) {
      double value = RoundWithPrecision(item(i, j), precision_);
      handler.addNumItem(value);
    }
    handler.endArray();
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const Eigen::VectorXi& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.rows(); ++i) {
    handler.addIntItem(item(i));
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const std::vector<int>& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.size(); ++i) {
    handler.addIntItem(item[i]);
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addElement(IloOplDataHandler& handler,
                                      const char* name,
                                      const Eigen::MatrixXi& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.rows(); ++i) {
    handler.startArray();
    for (int j = 0; j < item.cols(); ++j) {
      handler.addIntItem(item(i, j));
    }
    handler.endArray();
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addElement(
    IloOplDataHandler& handler, const char* name,
    const Eigen::Tensor<double, 3>& item) const {
  handler.startElement(name);
  handler.startArray();
  for (int i = 0; i < item.dimension(0); ++i) {
    handler.startArray();
    for (int j = 0; j < item.dimension(1); ++j) {
      handler.startArray();
      for (int k = 0; k < item.dimension(2); ++k) {
        double value = RoundWithPrecision(item(i, j, k), precision_);
        handler.addNumItem(value);
      }
      handler.endArray();
    }
    handler.endArray();
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addObstacleElement(
    IloOplDataHandler& handler, const char* name,
    const std::vector<std::vector<Eigen::MatrixXd> >& obstacles) const {
  handler.startElement(name);
  handler.startArray();
  for (auto& t : obstacles) {  // time range
    handler.startArray();
    for (auto& o : t) {  // obstacle range
      addLineSet(handler, o);
    }
    handler.endArray();
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addEnvironmentElement(
    IloOplDataHandler& handler, const char* name,
    const std::vector<Eigen::MatrixXd>& environments) const {
  handler.startElement(name);
  handler.startArray();
  for (auto& e : environments) {
    addLineSet(handler, e);
  }
  handler.endArray();
  handler.endElement();
}

void ModelInputDataSource::addLineTuple(IloOplDataHandler& handler,
                                        const Eigen::MatrixXd& item,
                                        const int& key, const int& idx1,
                                        const int& idx2) const {
  handler.startTuple();
  handler.addIntItem(key);
  double value1 = RoundWithPrecision(item(idx1, 0), precision_);
  double value2 = RoundWithPrecision(item(idx1, 1), precision_);
  double value3 = RoundWithPrecision(item(idx2, 0), precision_);
  double value4 = RoundWithPrecision(item(idx2, 1), precision_);
  handler.addNumItem(value1);  // x1
  handler.addNumItem(value2);  // y1
  handler.addNumItem(value3);  // x1
  handler.addNumItem(value4);  // y1
  handler.endTuple();
}

void ModelInputDataSource::addLineSet(IloOplDataHandler& handler,
                                      const Eigen::MatrixXd& item) const {
  handler.startSet();
  int r = item.rows();
  int key = 1;
  for (int i = 0; i < r - 1; ++i) {  // line tuple
    addLineTuple(handler, item, key, i, i + 1);
    key++;
  }
  addLineTuple(handler, item, key, r - 1, 0);
  handler.endSet();
}

void ModelInputDataSource::read() const {
  IloOplDataHandler handler = getDataHandler();

  // sizes
  addElement(handler, "NumSteps", parameters_->NumSteps);
  addElement(handler, "nr_environments", parameters_->nr_environments);
  addElement(handler, "nr_regions", parameters_->nr_regions);
  addElement(handler, "nr_obstacles", parameters_->nr_obstacles);
  addElement(handler, "max_lines_obstacles", parameters_->max_lines_obstacles);
  addElement(handler, "NumCar2CarCollisions",
             parameters_->NumCar2CarCollisions);
  addElement(handler, "NumCars", parameters_->NumCars);

  // cplex solver settings
  addElement(handler, "max_solution_time", parameters_->max_solution_time);
  addElement(handler, "relative_mip_gap_tolerance",
             parameters_->relative_mip_gap_tolerance);
  addElement(handler, "mipdisplay", parameters_->mipdisplay);
  addElement(handler, "mipemphasis", parameters_->mipemphasis);
  addElement(handler, "relobjdif", parameters_->relobjdif);
  addElement(handler, "cutpass", parameters_->cutpass);
  addElement(handler, "probe", parameters_->probe);
  addElement(handler, "repairtries", parameters_->repairtries);
  addElement(handler, "rinsheur", parameters_->rinsheur);
  addElement(handler, "varsel", parameters_->varsel);
  addElement(handler, "mircuts", parameters_->mircuts);
  addElement(handler, "parallelmode", parameters_->parallelmode);

  // globals
  addElement(handler, "ts", parameters_->ts);
  addElement(handler, "min_vel_x_y", parameters_->min_vel_x_y);
  addElement(handler, "max_vel_x_y", parameters_->max_vel_x_y);
  addElement(handler, "total_min_acc", parameters_->total_min_acc);
  addElement(handler, "total_max_acc", parameters_->total_max_acc);
  addElement(handler, "total_min_jerk", parameters_->total_min_jerk);
  addElement(handler, "total_max_jerk", parameters_->total_max_jerk);

  // time-dependent
  addElement(handler, "agent_safety_distance",
             parameters_->agent_safety_distance);
  addElement(handler, "agent_safety_distance_slack",
             parameters_->agent_safety_distance_slack);
  addElement(handler, "maximum_slack", parameters_->maximum_slack);

  // car-dependent
  addElement(handler, "WEIGHTS_POS_X", parameters_->WEIGHTS_POS_X);
  addElement(handler, "WEIGHTS_VEL_X", parameters_->WEIGHTS_VEL_X);
  addElement(handler, "WEIGHTS_ACC_X", parameters_->WEIGHTS_ACC_X);
  addElement(handler, "WEIGHTS_POS_Y", parameters_->WEIGHTS_POS_Y);
  addElement(handler, "WEIGHTS_VEL_Y", parameters_->WEIGHTS_VEL_Y);
  addElement(handler, "WEIGHTS_ACC_Y", parameters_->WEIGHTS_ACC_Y);
  addElement(handler, "WEIGHTS_JERK_X", parameters_->WEIGHTS_JERK_X);
  addElement(handler, "WEIGHTS_JERK_Y", parameters_->WEIGHTS_JERK_Y);
  addElement(handler, "WEIGHTS_SLACK", parameters_->WEIGHTS_SLACK);
  addElement(handler, "WEIGHTS_SLACK_OBSTACLE",
             parameters_->WEIGHTS_SLACK_OBSTACLE);
  addElement(handler, "WheelBase", parameters_->WheelBase);
  addElement(handler, "CollisionRadius", parameters_->CollisionRadius);
  addElement(handler, "IntitialState", parameters_->IntitialState);
  addElement(handler, "x_ref", parameters_->x_ref);
  addElement(handler, "vx_ref", parameters_->vx_ref);
  addElement(handler, "y_ref", parameters_->y_ref);
  addElement(handler, "vy_ref", parameters_->vy_ref);
  addElement(handler, "min_acc_x", parameters_->acc_limit_params.min_x);
  addElement(handler, "max_acc_x", parameters_->acc_limit_params.max_x);
  addElement(handler, "min_acc_y", parameters_->acc_limit_params.min_y);
  addElement(handler, "max_acc_y", parameters_->acc_limit_params.max_y);
  addElement(handler, "min_jerk_x", parameters_->jerk_limit_params.min_x);
  addElement(handler, "max_jerk_x", parameters_->jerk_limit_params.max_x);
  addElement(handler, "min_jerk_y", parameters_->jerk_limit_params.min_y);
  addElement(handler, "max_jerk_y", parameters_->jerk_limit_params.max_y);
  addElement(handler, "initial_region", parameters_->initial_region);
  addElement(handler, "possible_region", parameters_->possible_region);

  // obstacle-dependent
  addObstacleElement(handler, "ObstacleConvexPolygon",
                     parameters_->ObstacleConvexPolygon);
  addEnvironmentElement(handler, "MultiEnvironmentConvexPolygon",
                        parameters_->MultiEnvironmentConvexPolygon);
  addElement(handler, "obstacle_is_soft", parameters_->obstacle_is_soft);

  // region-dependent
  addElement(handler, "fraction_parameters", parameters_->fraction_parameters);
  addElement(handler, "minimum_region_change_speed",
             parameters_->minimum_region_change_speed);
  addElement(handler, "POLY_SINT_UB",
             parameters_->poly_orientation_params.POLY_SINT_UB);
  addElement(handler, "POLY_SINT_LB",
             parameters_->poly_orientation_params.POLY_SINT_LB);
  addElement(handler, "POLY_COSS_UB",
             parameters_->poly_orientation_params.POLY_COSS_UB);
  addElement(handler, "POLY_COSS_LB",
             parameters_->poly_orientation_params.POLY_COSS_LB);
  addElement(handler, "POLY_KAPPA_AX_MAX",
             parameters_->poly_curvature_params.POLY_KAPPA_AX_MAX);
  addElement(handler, "POLY_KAPPA_AX_MIN",
             parameters_->poly_curvature_params.POLY_KAPPA_AX_MIN);
}

void ModelInputDataSource::resetParameters(
    std::shared_ptr<ModelParameters> parameters) {
  parameters_ = parameters;
}

void ModelInputDataSource::resetCplexSolverSettingsOnly(
    std::shared_ptr<ModelParameters> parameters) {
  parameters_->max_solution_time = parameters->max_solution_time;
  parameters_->relative_mip_gap_tolerance =
      parameters->relative_mip_gap_tolerance;
  parameters_->mipdisplay = parameters->mipdisplay;
  parameters_->mipemphasis = parameters->mipemphasis;
  parameters_->relobjdif = parameters->relobjdif;
  parameters_->cutpass = parameters->cutpass;
  parameters_->probe = parameters->probe;
  parameters_->repairtries = parameters->repairtries;
  parameters_->rinsheur = parameters->rinsheur;
  parameters_->varsel = parameters->varsel;
  parameters_->mircuts = parameters->mircuts;
  parameters_->parallelmode = parameters->parallelmode;
}

}  // namespace planner
}  // namespace miqp
