// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODEL_INPUT_DATA_SOURCE_HEADER
#define MODEL_INPUT_DATA_SOURCE_HEADER

#include "miqp_planner_data.hpp"

#include <sstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "bark/commons/util/util.hpp"

#include "ilopl/iloopl.h"

namespace miqp {
namespace planner {

// adapted from the customdatasource.cpp example
class ModelInputDataSource : public IloOplDataSourceBaseI {
 public:
  ModelInputDataSource(IloEnv& env, int precision)
      : IloOplDataSourceBaseI(env), precision_(precision){};

  ~ModelInputDataSource() {}

  void read() const;

  void resetParameters(std::shared_ptr<ModelParameters> parameters);

  void resetCplexSolverSettingsOnly(
      std::shared_ptr<ModelParameters> parameters);

  int getParallelMode() { return parameters_->parallelmode; }

 private:
  std::shared_ptr<ModelParameters> parameters_;
  int precision_;

  //! one function for each type as the cplex api calls different methods! (hard
  //! to model with templates)
  void addElement(IloOplDataHandler& handler, const char* name,
                  const int& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const float& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const Eigen::VectorXd& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const Eigen::MatrixXd& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const Eigen::VectorXi& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const std::vector<int>& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const Eigen::MatrixXi& item) const;
  void addElement(IloOplDataHandler& handler, const char* name,
                  const Eigen::Tensor<double, 3>& item) const;
  void addEnvironmentElement(
      IloOplDataHandler& handler, const char* name,
      const std::vector<Eigen::MatrixXd>& environments) const;
  void addObstacleElement(
      IloOplDataHandler& handler, const char* name,
      const std::vector<std::vector<Eigen::MatrixXd> >& obstacles) const;
  void addLineTuple(IloOplDataHandler& handler, const Eigen::MatrixXd& item,
                    const int& key, const int& idx1, const int& idx2) const;
  void addLineSet(IloOplDataHandler& handler,
                  const Eigen::MatrixXd& item) const;
  inline double RoundWithPrecision(const double val,
                                   const int precision) const {
    const double scale = pow(10, precision);
    const double r_val = round(val * scale) / scale;
    return r_val;
  }
};

}  // namespace planner
}  // namespace miqp

#endif  // MODEL_INPUT_DATA_SOURCE_HEADER