// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "cplex_wrapper.hpp"

#include "model_input_data_source.hpp"

#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <unsupported/Eigen/CXX11/Tensor>

#include "bark/commons/util/util.hpp"

#include "ilopl/iloopl.h"

namespace miqp {
namespace planner {
namespace cplex {

void CplexWrapper::setParameterDatFileRelative(const char* datfile) {
  datfile_ = std::string("cplexmodel/") + std::string(datfile);
}

void CplexWrapper::setParameterDatFileAbsolute(const char* datfile) {
  datfile_ = std::string(datfile);
}

void CplexWrapper::setParametersFromDatfile(IloOplModel& opl) {
  IloOplDataSource dataSource_dat(env_, datfile_.c_str());
  opl.addDataSource(dataSource_dat);
}

void CplexWrapper::setParametersFromModelInputDataSource(IloOplModel& opl) {
  IloOplDataSource dataSource_input(&ds_);
  opl.addDataSource(dataSource_input);
}

void CplexWrapper::setParallelMode(IloCplex& cplex) {
  // https://www.ibm.com/support/knowledgecenter/SSSA5P_20.1.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/ParallelMode.html
  ParallelMode pm = static_cast<ParallelMode>(ds_.getParallelMode());
  if (pm == ParallelMode::DETERMINISTIC) {
    cplex.setParam(IloCplex::Param::Parallel, CPX_PARALLEL_DETERMINISTIC);
  } else if (pm == ParallelMode::OPPORTUNISTIC) {
    cplex.setParam(IloCplex::Param::Parallel, CPX_PARALLEL_OPPORTUNISTIC);
  } else {
    cplex.setParam(IloCplex::Param::Parallel, CPX_PARALLEL_AUTO);
  }
}

void CplexSegfaultHandler(int sig) {
  LOG(ERROR) << "CplexSegfaultHandler: Signal = " << sig << std::endl;
  throw std::logic_error("Segfault in Cplex call!");
}

OptimizationStatus CplexWrapper::callCplex(const double timestemp) {
  // adapted from
  // https://www.ibm.com/support/pages/run-iis-analysis-opl-model-using-c-api
  using namespace std;

  OptimizationStatus status;

  // the cplex object could be generated once in the constructor
  IloCplex cplex(env_);
  IloOplModel opl(def_, cplex);
  cplexHandlerOut_.clear();
  std::stringstream cplexout;
  if (bufferCplexOutputsToStream_) {
    cplex.setOut(cplexout);
    cplex.setWarning(cplexout);
    cplex.setError(cplexout);
  }

  // cplex.setParam(IloCplex::PreInd, 0); // cplex parameters could also
  // be set like this
  if (parameterSource_ == CPPINPUTS) {
    setParametersFromModelInputDataSource(opl);
  } else if (parameterSource_ == DATFILE) {
    setParametersFromDatfile(opl);
  } else if (parameterSource_ == MIXED) {
    setParametersFromModelInputDataSource(opl);
    setParametersFromDatfile(opl);
  } else {
    LOG(ERROR) << "Input Data not set!";
  }

  std::signal(SIGSEGV, CplexSegfaultHandler);
  try {
    opl.generate();
  } catch (IloException& e) {
    LOG(ERROR) << "### CPLEX EXCEPTION GENERATE: ";
    e.print(LOG(ERROR));
    return OptimizationStatus::FAILED_SEG_FAULT;
  } catch (std::exception& e) {
    LOG(ERROR) << "### UNEXPECTED STD ERROR GENERATE: " << e.what() << endl;
    return OptimizationStatus::FAILED_SEG_FAULT;
  } catch (...) {
    LOG(ERROR) << "### UNEXPECTED CPLEX ERROR GENERATE ..." << endl;
    return OptimizationStatus::FAILED_SEG_FAULT;
  }

  // SOS = Special Ordered Set Constraints for active_region
  if (useSpecialOrderedSets_) {
    addSOSConstraints(opl);
  }

  // Set branching priorieties
  if (useBranchingPriorities_) {
    setBranchingPriorities(opl, cplex);
  }

  // TODO specialized presolve?

  // Initialize warmstart
  if (doWarmstart_ == RECEDING_HORIZON_WARMSTART ||
      doWarmstart_ == BOTH_WARMSTART_STRATEGIES) {
    initializeWarmstart(opl, cplex);
  }
  if (doWarmstart_ == LAST_SOLUTION_WARMSTART ||
      doWarmstart_ == BOTH_WARMSTART_STRATEGIES) {
    std::ifstream infile(tmpWarmstartFile_.c_str());
    if (infile.good()) {
      cplex.readMIPStarts(tmpWarmstartFile_.c_str());
      LOG(INFO) << "Reading warmstarts from file";
    } else {
      LOG(ERROR) << "Could not read warmstart file "
                 << tmpWarmstartFile_.c_str();
    }
  }

  // Debugging file write
  if (print_debug_outputs_) {
    stringstream filename;
    filename << std::setprecision(15) << debugOutputFilePath_ << "/"
             << debugOutputFilePrefix_ << "parameters_" << timestemp << ".txt";

    std::ofstream ofs(filename.str().c_str(), std::ofstream::out);
    opl.printExternalData(ofs);
    ofs.close();
    debugOutputParameterFilePath_ = filename.str();

    stringstream lpfile;
    lpfile << std::setprecision(15) << debugOutputFilePath_ << "/"
           << debugOutputFilePrefix_ << "lpexport_" << timestemp << ".lp";
    cplex.exportModel(lpfile.str().c_str());
  }

  // Call solver
  struct timespec start, finish;
  double elapsed;
  clock_gettime(CLOCK_MONOTONIC, &start);
  bool cplex_call_status;
  try {
    cplex_call_status = cplex.solve();
    if (bufferCplexOutputsToStream_) {
      LOG(INFO) << cplexout.str().c_str();
    }
    if (cplexHandlerOut_.rdbuf()->in_avail() != 0) {  // is not empty
      LOG(INFO) << "Cplex handler out: " << cplexHandlerOut_.str().c_str();
    }
  } catch (IloException& e) {
    LOG(ERROR) << "### CPLEX EXCEPTION SOLVE: ";
    e.print(LOG(ERROR));
    return OptimizationStatus::FAILED_SEG_FAULT;
  } catch (std::exception& e) {
    LOG(ERROR) << "### UNEXPECTED STD ERROR SOLVE: " << e.what() << endl;
    return OptimizationStatus::FAILED_SEG_FAULT;
  } catch (...) {
    LOG(ERROR) << "### UNEXPECTED CPLEX ERROR SOLVE ..." << endl;
    return OptimizationStatus::FAILED_SEG_FAULT;
  }

  // Timing
  clock_gettime(CLOCK_MONOTONIC, &finish);
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

  collectCplexStatistics(cplex);

  // If cplex run success
  if (cplex_call_status) {
    // Results
    status = OptimizationStatus::SUCCESS;
    collectRawResults(opl);
    collectSolutionStatus(cplex, elapsed);
    LOG(INFO) << std::setprecision(15)
              << "SOLVER SUMMARY: Found valid solution at t = " << timestemp
              << " cplex status = " << solutionProperties_.status
              << " status = " << status
              << " objective = " << solutionProperties_.objective
              << " gap = " << solutionProperties_.gap
              << " time = " << solutionProperties_.time
              << " Solution Quality = "
              << cplex.getQuality(IloCplex::MaxPrimalInfeas) << endl;

    // Warmstart
    if (doWarmstart_ == LAST_SOLUTION_WARMSTART ||
        doWarmstart_ == BOTH_WARMSTART_STRATEGIES) {
      cplex.writeMIPStarts(tmpWarmstartFile_.c_str());
    }

    // Debug outputs
    if (print_debug_outputs_) {
      stringstream filename;
      filename << std::setprecision(15) << debugOutputFilePath_ << "/"
               << debugOutputFilePrefix_ << "solution_" << timestemp << ".txt";

      std::ofstream ofs(filename.str().c_str(), std::ofstream::out);
      opl.printSolution(ofs);
      ofs.close();

      stringstream warmstartfile;
      warmstartfile << std::setprecision(15) << debugOutputFilePath_ << "/"
                    << debugOutputFilePrefix_ << "warmstartsolution_"
                    << timestemp << ".mst";
      if (doWarmstart_ == LAST_SOLUTION_WARMSTART ||
          doWarmstart_ == BOTH_WARMSTART_STRATEGIES) {
        cplex.writeMIPStarts(warmstartfile.str().c_str());
      }
    }

  } else {  // cplex failed
    auto cplexstatus = cplex.getCplexStatus();
    if (cplexstatus == CPX_STAT_ABORT_TIME_LIM) {
      status = OptimizationStatus::FAILED_TIMEOUT;
    } else {
      status = OptimizationStatus::FAILED_NO_SOLUT;
    }
    solutionProperties_.objective = NAN;
    solutionProperties_.gap = NAN;
    solutionProperties_.time = static_cast<double>(elapsed);
    solutionProperties_.status = static_cast<int>(cplexstatus);
    LOG(INFO) << std::setprecision(15)
              << "SOLVER SUMMARY: Found no solution at t= " << timestemp
              << " cplex status = " << static_cast<int>(cplexstatus)
              << " status = " << status << endl;
  }

  return status;
}

template <class TensorType>
void CplexWrapper::getDecisionVariable(const IloOplElement& element,
                                       TensorType& outdata) {
  using namespace std;

  // cerr << "Getting decision variable: " << element.getName()
  //     << " of type: " << element.getElementType() << endl;

  outdata.setConstant(9999999);  // NAN is a warning
  if (element.getElementType() == IloOplElementType::MAP_NUM) {
    IloNumMap m = element.asNumMap();
    // cerr << "m: " << m << endl;
    getNdimDecisionVariable<TensorType, IloNumMap>(m, outdata, {});
  } else if (element.getElementType() == IloOplElementType::MAP_INT) {
    IloIntMap m = element.asIntMap();
    // cerr << "m: " << m << endl;
    getNdimDecisionVariable<TensorType, IloIntMap>(m, outdata, {});
  } else {
    throw NotImplementedException();
  }

  // cerr << "outdata: " << outdata << endl;
}

template <class TensorType, class IloMapType>
void CplexWrapper::getNdimDecisionVariable(IloMapType m, TensorType& outdata,
                                           std::vector<int> dimIdx) {
  //! NOTE copied from stream operator code. replaced outputs by eigen tensor
  //! fill
  IloInt nb = m.getNbDim();
  if (nb > 1) {
    IloMapType ma;
    IloInt i, n = m.getSize() - 1;
    for (i = 0; i < n; ++i) {
      ma = IloMapType(m.getAtAbsoluteIndex(i));
      std::vector<int> dimIdxNext = dimIdx;
      dimIdxNext.push_back(i);
      getNdimDecisionVariable<TensorType, IloMapType>(ma, outdata, dimIdxNext);
    }
    if (n >= 0) {
      ma = IloMapType(m.getAtAbsoluteIndex(i));
      std::vector<int> dimIdxNext = dimIdx;
      dimIdxNext.push_back(i);
      getNdimDecisionVariable<TensorType, IloMapType>(ma, outdata, dimIdxNext);
    }
  } else if (1 == nb) {
    IloInt i, n = m.getSize() - 1;
    for (i = 0; i < n; ++i) {
      std::vector<int> dimIdxNext = dimIdx;
      dimIdxNext.push_back(i);
      outdata(dimIdxNext) = m.eltAtAbsoluteIndex(i);
    }
    if (n >= 0) {
      std::vector<int> dimIdxNext = dimIdx;
      dimIdxNext.push_back(i);
      outdata(dimIdxNext) = m.eltAtAbsoluteIndex(i);
    }
  }
}

void CplexWrapper::collectRawResults(IloOplModel& opl) {
  //! NOTE I cannot make opl const due to the getElement access.

  const IloOplDataElements externalData = opl.makeDataElements();
  const int N = static_cast<int>(externalData.getElement("NumSteps").asInt());
  const int NrEnvironments =
      static_cast<int>(externalData.getElement("nr_environments").asInt());
  const int NrRegions =
      static_cast<int>(externalData.getElement("nr_regions").asInt());
  const int NrObstacles =
      static_cast<int>(externalData.getElement("nr_obstacles").asInt());
  const int MaxLinesObstacles =
      static_cast<int>(externalData.getElement("max_lines_obstacles").asInt());
  const int NrCarToCarCollisions =
      static_cast<int>(externalData.getElement("NumCar2CarCollisions").asInt());
  const int NrCars =
      static_cast<int>(externalData.getElement("NumCars").asInt());

  rawResults_->N = N;
  rawResults_->NrEnvironments = NrEnvironments;
  rawResults_->NrRegions = NrRegions;
  rawResults_->NrObstacles = NrObstacles;
  rawResults_->MaxLinesObstacles = MaxLinesObstacles;
  rawResults_->NrCarToCarCollisions = NrCarToCarCollisions;
  rawResults_->NrCars = NrCars;

  // TODO TOBIAS resizing here could be a performance bottleneck... allocate
  // memory only once if one of the sizes change
  rawResults_->u_x.resize(NrCars, N);
  rawResults_->u_y.resize(NrCars, N);
  rawResults_->pos_x.resize(NrCars, N);
  rawResults_->vel_x.resize(NrCars, N);
  rawResults_->acc_x.resize(NrCars, N);
  rawResults_->pos_y.resize(NrCars, N);
  rawResults_->vel_y.resize(NrCars, N);
  rawResults_->acc_y.resize(NrCars, N);
  rawResults_->pos_x_front_UB.resize(NrCars, N);
  rawResults_->pos_x_front_LB.resize(NrCars, N);
  rawResults_->pos_y_front_UB.resize(NrCars, N);
  rawResults_->pos_y_front_LB.resize(NrCars, N);
  rawResults_->notWithinEnvironmentRear.resize(NrCars, NrEnvironments, N);
  rawResults_->notWithinEnvironmentFrontUbUb.resize(NrCars, NrEnvironments, N);
  rawResults_->notWithinEnvironmentFrontLbUb.resize(NrCars, NrEnvironments, N);
  rawResults_->notWithinEnvironmentFrontUbLb.resize(NrCars, NrEnvironments, N);
  rawResults_->notWithinEnvironmentFrontLbLb.resize(NrCars, NrEnvironments, N);
  rawResults_->active_region.resize(NrCars, N, NrRegions);
  rawResults_->region_change_not_allowed_x_positive.resize(NrCars, N);
  rawResults_->region_change_not_allowed_y_positive.resize(NrCars, N);
  rawResults_->region_change_not_allowed_x_negative.resize(NrCars, N);
  rawResults_->region_change_not_allowed_y_negative.resize(NrCars, N);
  rawResults_->region_change_not_allowed_combined.resize(NrCars, N);
  rawResults_->deltacc.resize(NrCars, NrObstacles, N, MaxLinesObstacles);
  rawResults_->deltacc_front.resize(
      NrCars, NrObstacles, N, MaxLinesObstacles,
      4);  // 4 : all combinations of UBUB,  UBLB, LBUB, UBUB
  rawResults_->car2car_collision.resize(
      NrCarToCarCollisions, N,
      16);  // 16 Idxs: Rear/rear = 1..4, Rear/front = 5..8, Front/rear
            // = 9..12, Front/front = 13..16
  rawResults_->slackvars.resize(
      NrCarToCarCollisions, N,
      4);  // 4 Idxs: rear x = 1, rear y = 2, front x = 3, front y = 4
  rawResults_->slackvarsObstacle.resize(NrCars, NrObstacles, N);
  rawResults_->slackvarsObstacle_front.resize(
      NrCars, NrObstacles, N,
      4);  // 4 Idxs: rear x = 1, rear y = 2, front x = 3, front y = 4

  // Get the data out of the opl object and parse it into eigen tensors.
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("u_x"),
                                                 rawResults_->u_x);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("u_y"),
                                                 rawResults_->u_y);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("pos_x"),
                                                 rawResults_->pos_x);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("vel_x"),
                                                 rawResults_->vel_x);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("acc_x"),
                                                 rawResults_->acc_x);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("pos_y"),
                                                 rawResults_->pos_y);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("vel_y"),
                                                 rawResults_->vel_y);
  getDecisionVariable<Eigen::Tensor<double, 2> >(opl.getElement("acc_y"),
                                                 rawResults_->acc_y);
  getDecisionVariable<Eigen::Tensor<double, 2> >(
      opl.getElement("pos_x_front_UB"), rawResults_->pos_x_front_UB);
  getDecisionVariable<Eigen::Tensor<double, 2> >(
      opl.getElement("pos_x_front_LB"), rawResults_->pos_x_front_LB);
  getDecisionVariable<Eigen::Tensor<double, 2> >(
      opl.getElement("pos_y_front_UB"), rawResults_->pos_y_front_UB);
  getDecisionVariable<Eigen::Tensor<double, 2> >(
      opl.getElement("pos_y_front_LB"), rawResults_->pos_y_front_LB);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("notWithinEnvironmentRear"),
      rawResults_->notWithinEnvironmentRear);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("notWithinEnvironmentFrontUbUb"),
      rawResults_->notWithinEnvironmentFrontUbUb);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("notWithinEnvironmentFrontLbUb"),
      rawResults_->notWithinEnvironmentFrontLbUb);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("notWithinEnvironmentFrontUbLb"),
      rawResults_->notWithinEnvironmentFrontUbLb);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("notWithinEnvironmentFrontLbLb"),
      rawResults_->notWithinEnvironmentFrontLbLb);
  getDecisionVariable<Eigen::Tensor<int, 3> >(opl.getElement("active_region"),
                                              rawResults_->active_region);
  getDecisionVariable<Eigen::Tensor<int, 2> >(
      opl.getElement("region_change_not_allowed_x_positive"),
      rawResults_->region_change_not_allowed_x_positive);
  getDecisionVariable<Eigen::Tensor<int, 2> >(
      opl.getElement("region_change_not_allowed_y_positive"),
      rawResults_->region_change_not_allowed_y_positive);
  getDecisionVariable<Eigen::Tensor<int, 2> >(
      opl.getElement("region_change_not_allowed_x_negative"),
      rawResults_->region_change_not_allowed_x_negative);
  getDecisionVariable<Eigen::Tensor<int, 2> >(
      opl.getElement("region_change_not_allowed_y_negative"),
      rawResults_->region_change_not_allowed_y_negative);
  getDecisionVariable<Eigen::Tensor<int, 2> >(
      opl.getElement("region_change_not_allowed_combined"),
      rawResults_->region_change_not_allowed_combined);
  getDecisionVariable<Eigen::Tensor<int, 4> >(opl.getElement("deltacc"),
                                              rawResults_->deltacc);
  getDecisionVariable<Eigen::Tensor<int, 5> >(opl.getElement("deltacc_front"),
                                              rawResults_->deltacc_front);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("car2car_collision"), rawResults_->car2car_collision);
  getDecisionVariable<Eigen::Tensor<int, 3> >(opl.getElement("slackvars"),
                                              rawResults_->slackvars);
  getDecisionVariable<Eigen::Tensor<int, 3> >(
      opl.getElement("slackvarsObstacle"), rawResults_->slackvarsObstacle);
  getDecisionVariable<Eigen::Tensor<int, 4> >(
      opl.getElement("slackvarsObstacle_front"),
      rawResults_->slackvarsObstacle_front);
}

/**
 * @brief addRecedingHorizonWarmstart -- set the warmstartValues_ structure and
 *activate the warmstart from a given solution.
 *
 * @param WarmstartType wt -- default value: RECEDING_HORIZON_WARMSTART. If set
 *to something else, set to both warmstart strategies
 **/
void CplexWrapper::addRecedingHorizonWarmstart(
    std::shared_ptr<RawResults> warmstart, WarmstartType wt) {
  if (wt != RECEDING_HORIZON_WARMSTART) {
    doWarmstart_ = BOTH_WARMSTART_STRATEGIES;
  } else {
    doWarmstart_ = wt;
  }
  warmstartValues_ = warmstart;
}

/**
 * @brief setLastSolutionWarmstart -- activate warmstart from the previous
 *solution(s) read from an input file.
 *
 * @param WarmstartType wt -- default value: LAST_SOLUTION_WARMSTART. If set
 *to something else, set to both warmstart strategies
 **/
void CplexWrapper::setLastSolutionWarmstart(WarmstartType wt) {
  if (wt != LAST_SOLUTION_WARMSTART) {
    doWarmstart_ = BOTH_WARMSTART_STRATEGIES;
  } else {
    doWarmstart_ = wt;
  }
}

void CplexWrapper::deleteLastSolutionWarmstartFile() {
  std::ifstream infile(tmpWarmstartFile_.c_str());
  if (infile.good()) {
    LOG(INFO) << "Deleting old warmstart file...";
    remove(tmpWarmstartFile_.c_str());
  }
}

// good docu on warmstaring and the APIs:
// https://www.ibm.com/support/pages/warmstarting-lps-and-mips-having-multi-dimensional-decision-variables-starting-solutions-using-java-api
// My implementation follows this example:
// https://www.ibm.com/support/pages/node/461013
void CplexWrapper::initializeWarmstart(IloOplModel& opl, IloCplex& cplex) {
  IloNumVarMap pos_x = opl.getElement("pos_x").asNumVarMap();
  IloNumVarMap vel_x = opl.getElement("vel_x").asNumVarMap();
  IloNumVarMap acc_x = opl.getElement("acc_x").asNumVarMap();
  IloNumVarMap pos_y = opl.getElement("pos_y").asNumVarMap();
  IloNumVarMap vel_y = opl.getElement("vel_y").asNumVarMap();
  IloNumVarMap acc_y = opl.getElement("acc_y").asNumVarMap();
  IloNumVarMap u_x = opl.getElement("u_x").asNumVarMap();
  IloNumVarMap u_y = opl.getElement("u_y").asNumVarMap();
  IloNumVarMap slackvars = opl.getElement("slackvars").asNumVarMap();
  IloNumVarMap slackvarsObstacle =
      opl.getElement("slackvarsObstacle").asNumVarMap();
  IloNumVarMap slackvarsObstacle_front =
      opl.getElement("slackvarsObstacle_front").asNumVarMap();
  IloNumVarMap pos_x_front_UB = opl.getElement("pos_x_front_UB").asNumVarMap();
  IloNumVarMap pos_x_front_LB = opl.getElement("pos_x_front_LB").asNumVarMap();
  IloNumVarMap pos_y_front_UB = opl.getElement("pos_y_front_UB").asNumVarMap();
  IloNumVarMap pos_y_front_LB = opl.getElement("pos_y_front_LB").asNumVarMap();
  IloIntVarMap active_region = opl.getElement("active_region").asIntVarMap();
  IloIntVarMap region_change_not_allowed_x_positive =
      opl.getElement("region_change_not_allowed_x_positive").asIntVarMap();
  IloIntVarMap region_change_not_allowed_y_positive =
      opl.getElement("region_change_not_allowed_y_positive").asIntVarMap();
  IloIntVarMap region_change_not_allowed_x_negative =
      opl.getElement("region_change_not_allowed_x_negative").asIntVarMap();
  IloIntVarMap region_change_not_allowed_y_negative =
      opl.getElement("region_change_not_allowed_y_negative").asIntVarMap();
  IloIntVarMap region_change_not_allowed_combined =
      opl.getElement("region_change_not_allowed_combined").asIntVarMap();
  IloIntVarMap notWithinEnvironmentRear =
      opl.getElement("notWithinEnvironmentRear").asIntVarMap();
  IloIntVarMap notWithinEnvironmentFrontUbUb =
      opl.getElement("notWithinEnvironmentFrontUbUb").asIntVarMap();
  IloIntVarMap notWithinEnvironmentFrontLbUb =
      opl.getElement("notWithinEnvironmentFrontLbUb").asIntVarMap();
  IloIntVarMap notWithinEnvironmentFrontUbLb =
      opl.getElement("notWithinEnvironmentFrontUbLb").asIntVarMap();
  IloIntVarMap notWithinEnvironmentFrontLbLb =
      opl.getElement("notWithinEnvironmentFrontLbLb").asIntVarMap();
  IloIntVarMap deltacc = opl.getElement("deltacc").asIntVarMap();
  IloIntVarMap deltacc_front = opl.getElement("deltacc_front").asIntVarMap();
  IloIntVarMap car2car_collision =
      opl.getElement("car2car_collision").asIntVarMap();

  IloNumVarArray startVar(env_);
  IloNumArray startVal(env_);

  std::vector<size_t> dimIdx = {};
  size_t dimDepth = 0;

  initializeWarmstartElement(pos_x, warmstartValues_->pos_x, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(vel_x, warmstartValues_->vel_x, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(acc_x, warmstartValues_->acc_x, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(pos_y, warmstartValues_->pos_y, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(vel_y, warmstartValues_->vel_y, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(acc_y, warmstartValues_->acc_y, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(u_x, warmstartValues_->u_x, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(u_y, warmstartValues_->u_y, startVar, startVal,
                             dimDepth, dimIdx);
  initializeWarmstartElement(slackvars, warmstartValues_->slackvars, startVar,
                             startVal, dimDepth, dimIdx);
  initializeWarmstartElement(slackvarsObstacle,
                             warmstartValues_->slackvarsObstacle, startVar,
                             startVal, dimDepth, dimIdx);
  initializeWarmstartElement(slackvarsObstacle_front,
                             warmstartValues_->slackvarsObstacle_front,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(pos_x_front_UB, warmstartValues_->pos_x_front_UB,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(pos_x_front_LB, warmstartValues_->pos_x_front_LB,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(pos_y_front_UB, warmstartValues_->pos_y_front_UB,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(pos_y_front_LB, warmstartValues_->pos_y_front_LB,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(active_region, warmstartValues_->active_region,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(
      region_change_not_allowed_x_positive,
      warmstartValues_->region_change_not_allowed_x_positive, startVar,
      startVal, dimDepth, dimIdx);
  initializeWarmstartElement(
      region_change_not_allowed_x_negative,
      warmstartValues_->region_change_not_allowed_x_negative, startVar,
      startVal, dimDepth, dimIdx);
  initializeWarmstartElement(
      region_change_not_allowed_y_positive,
      warmstartValues_->region_change_not_allowed_y_positive, startVar,
      startVal, dimDepth, dimIdx);
  initializeWarmstartElement(
      region_change_not_allowed_y_negative,
      warmstartValues_->region_change_not_allowed_y_negative, startVar,
      startVal, dimDepth, dimIdx);
  initializeWarmstartElement(
      region_change_not_allowed_combined,
      warmstartValues_->region_change_not_allowed_combined, startVar, startVal,
      dimDepth, dimIdx);
  initializeWarmstartElement(notWithinEnvironmentRear,
                             warmstartValues_->notWithinEnvironmentRear,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(notWithinEnvironmentFrontUbUb,
                             warmstartValues_->notWithinEnvironmentFrontUbUb,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(notWithinEnvironmentFrontUbLb,
                             warmstartValues_->notWithinEnvironmentFrontUbLb,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(notWithinEnvironmentFrontLbUb,
                             warmstartValues_->notWithinEnvironmentFrontLbUb,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(notWithinEnvironmentFrontLbLb,
                             warmstartValues_->notWithinEnvironmentFrontLbLb,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(deltacc, warmstartValues_->deltacc, startVar,
                             startVal, dimDepth, dimIdx);
  initializeWarmstartElement(deltacc_front, warmstartValues_->deltacc_front,
                             startVar, startVal, dimDepth, dimIdx);
  initializeWarmstartElement(car2car_collision,
                             warmstartValues_->car2car_collision, startVar,
                             startVal, dimDepth, dimIdx);

  // std::cout <<"startval: " << startVal << std::endl;
  // std::cout <<"startvar: " << startVar << std::endl;

  LOG(INFO) << "Warmstart val size = " << startVal.getSize()
            << " var size = " << startVar.getSize() << std::endl;

  // NOTE FIRST end() the array THEN use them in the warmstart, otherwise the
  // warmstart is invalid!!!
  startVal.end();
  startVar.end();

  // documentation of the affort level:
  // https://www.ibm.com/support/knowledgecenter/en/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/UsrMan/topics/discr_optim/mip/para/49_mipStarts.html#User_manual.uss_solveMIP.638620__section1218612714531
  cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartSolveMIP, "combined");
  // cplex.setParam(IloCplex::RepairTries, 50);
  // https://www.ibm.com/support/knowledgecenter/en/SSSA5P_12.7.1/ilog.odms.cplex.help/CPLEX/Parameters/topics/AdvInd.html
  // cplex.setParam(IloCplex::Param::Advance, 2);
  // cplex.setParam(IloCplex::RINSHeur, 100);
}

template <class TensorType, class IloVarMapType>
void CplexWrapper::initializeWarmstartElement(
    IloVarMapType& varMap, TensorType& vals, IloNumVarArray& startVar,
    IloNumArray& startVal, size_t dimDepth, std::vector<size_t> dimIdx) {
  const auto& dim = vals.dimensions();
  if (dim.size() > dimDepth) {
    for (int i = 0; i < dim[dimDepth]; ++i) {  // loop over the current
                                               // dimension
      if (dim.size() == dimDepth + 1) {
        // We reached the actual element: add tensor element and decision
        // variable
        std::vector<size_t> dimIdxNext = dimIdx;
        dimIdxNext.push_back(i);
        startVal.add(vals(dimIdxNext));
        startVar.add(varMap.eltAtAbsoluteIndex(i));
      } else {
        // Recursive call: increase one dimension
        IloVarMapType subVarMap = varMap.getAtAbsoluteIndex(i);
        dimIdx.push_back(i);
        initializeWarmstartElement<TensorType, IloVarMapType>(
            subVarMap, vals, startVar, startVal, dimDepth + 1, dimIdx);
      }
    }
  }
}

void CplexWrapper::resetParameters(
    std::shared_ptr<ModelParameters> parameters) {
  ds_.resetParameters(parameters);
}

void CplexWrapper::collectSolutionStatus(const IloCplex& cplex,
                                         const double& solutiontime) {
  solutionProperties_.objective = static_cast<double>(cplex.getObjValue());
  solutionProperties_.gap = static_cast<double>(cplex.getMIPRelativeGap());
  solutionProperties_.time = static_cast<double>(solutiontime);
  solutionProperties_.status = static_cast<int>(cplex.getCplexStatus());
}

void CplexWrapper::collectCplexStatistics(const IloCplex& cplex) {
  solutionProperties_.NrConstraints = static_cast<int>(cplex.getNrows());
  solutionProperties_.NrBinaryVariables = static_cast<int>(cplex.getNbinVars());
  solutionProperties_.NrFloatVariables = static_cast<int>(cplex.getNcols()) -
                                         static_cast<int>(cplex.getNbinVars()) -
                                         static_cast<int>(cplex.getNintVars());
  solutionProperties_.NonZeroCoefficients = static_cast<int>(cplex.getNNZs());
  solutionProperties_.NrIterations = static_cast<int>(cplex.getNiterations());
  solutionProperties_.NrSolutionPool =
      static_cast<int>(cplex.getSolnPoolNsolns());
}

void CplexWrapper::addSOSConstraints(IloOplModel& opl) {
  IloModel model = opl.getModel();
  const IloOplDataElements externalData = opl.makeDataElements();
  const int N = static_cast<int>(externalData.getElement("NumSteps").asInt());
  const int NrRegions =
      static_cast<int>(externalData.getElement("nr_regions").asInt());
  const int NrCars =
      static_cast<int>(externalData.getElement("NumCars").asInt());

  IloIntVarMap active_region = opl.getElement("active_region").asIntVarMap();

  for (int idx_car = 0; idx_car < NrCars; ++idx_car) {
    IloIntVarMap active_region_car = active_region.getAtAbsoluteIndex(idx_car);
    for (int idx_idx = 0; idx_idx < N; ++idx_idx) {
      IloIntVarMap active_region_car_idx =
          active_region_car.getAtAbsoluteIndex(idx_idx);
      IloNumVarArray sosvar(env_, NrRegions);
      IloNumArray sosval(env_, NrRegions);
      for (int idx_reg = 0; idx_reg < NrRegions; ++idx_reg) {
        sosvar[idx_reg] = active_region_car_idx.eltAtAbsoluteIndex(idx_reg);
        sosval[idx_reg] = idx_reg + NrRegions;  // arbitrary prio
        // if(warmstartValues_->active_region(idx_car, idx_idx, idx_reg) == 1) {
        // //TODO once warmstart works, this should be a good strategy
        if (externalData.getElement("initial_region")
                .asIntMap()
                .eltAtAbsoluteIndex(idx_car) == idx_reg) {
          sosval[idx_reg] = 10000;  // TODO not sure if the weight shall be high
                                    // or low for early branching
        }
      }
      model.add(IloSOS1(model.getEnv(), sosvar, sosval));
    }
  }
}

// Branchin priorities
// 0 	no automatic priority order will be generated (default)
// 1 	decreasing cost coefficients among the variables
// 2 	increasing bound range among the variables
// 3 	increasing cost per matrix coefficient count among the variables
void CplexWrapper::setBranchingPriorities(IloOplModel& opl, IloCplex& cplex) {
  // HACK priories currently hardcoded here!!

  const IloOplDataElements externalData = opl.makeDataElements();
  const int N = static_cast<int>(externalData.getElement("NumSteps").asInt());
  const int NrEnvironments =
      static_cast<int>(externalData.getElement("nr_environments").asInt());
  const int NrRegions =
      static_cast<int>(externalData.getElement("nr_regions").asInt());
  // const int NrObstacles =
  //     static_cast<int>(externalData.getElement("nr_obstacles").asInt());
  // const int MaxLinesObstacles =
  //     static_cast<int>(externalData.getElement("max_lines_obstacles").asInt());
  // const int NrCarToCarCollisions =
  //     static_cast<int>(externalData.getElement("NumCar2CarCollisions").asInt());
  const int NrCars =
      static_cast<int>(externalData.getElement("NumCars").asInt());

  const int increase_prio = increasedBranchingPriorityValue_;
  const int default_prio = 0;
  int N_extent = branchingPriorityExtent_;
  if (N_extent > N - 1) {
    N_extent = N - 1;
  }
  if (N_extent < 1) {
    N_extent = 1;
  }

  // Active Region
  // Set branching priority for active_region: if region is possible, set prio
  // high: if region cannot be used: set prio low
  Eigen::Tensor<int, 3> active_reg_prio;  //(NrCars, N, NrRegions);
  active_reg_prio.resize(NrCars, N, NrRegions);
  active_reg_prio.setConstant(default_prio);
  auto possible_region = externalData.getElement("possible_region").asIntMap();
  for (int idx_car = 0; idx_car < NrCars; ++idx_car) {
    auto pr_car = possible_region.getAtAbsoluteIndex(idx_car);
    for (int idx_reg = 0; idx_reg < NrRegions; ++idx_reg) {
      const int pr_car_reg = pr_car->getAtAbsoluteIndex(idx_reg).asInt();
      if (pr_car_reg == 1) {
        // increase branching priority
        Eigen::array<long, 3> offset = {idx_car, 1, idx_reg};
        Eigen::array<long, 3> extent = {1, N_extent, 1};
        active_reg_prio.slice(offset, extent).setConstant(increase_prio);
      }
      // if (pr_car_reg == 0) {
      //   // decrease branching priority, is this useful??
      //   Eigen::array<long, 3> offset = {idx_car, 1, idx_reg};
      //   Eigen::array<long, 3> extent = {1, N_extent, 1};
      //   active_reg_prio.slice(offset, extent).setConstant(increase_prio);
      // }
    }
  }
  IloIntVarMap active_reg = opl.getElement("active_region").asIntVarMap();
  setBranchingPriorityFromTensor(cplex, active_reg, active_reg_prio, 0, {});

  // notWithinEnvironmentRear
  // if not too many environment changes are necessary, then branching
  // priorities are not beneficial regarding runtime
  if (NrEnvironments > 0) {
    Eigen::Tensor<int, 3> env_rear_prio;  //(NrCars, NrEnvironments, N);
    env_rear_prio.resize(NrCars, NrEnvironments, N);
    env_rear_prio.setConstant(default_prio);
    Eigen::array<long, 3> offset = {0, 0, 0};
    Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N_extent};
    env_rear_prio.slice(offset, extent).setConstant(increase_prio);
    IloIntVarMap notWithinEnvironmentRear =
        opl.getElement("notWithinEnvironmentRear").asIntVarMap();
    setBranchingPriorityFromTensor(cplex, notWithinEnvironmentRear,
                                   env_rear_prio, 0, {});
  }

  // Makes runtime worse!
  // if (NrEnvironments > 0) {
  //   // notWithinEnvironmentFrontUbUb
  //   Eigen::Tensor<int, 3> env_frontubub_prio;  //(NrCars, NrEnvironments, N);
  //   env_frontubub_prio.resize(NrCars, NrEnvironments, N);
  //   env_frontubub_prio.setConstant(default_prio);
  //   Eigen::array<long, 3> offset = {0, 0, 0};
  //   Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N_extent};
  //   env_frontubub_prio.slice(offset, extent).setConstant(increase_prio);
  //   IloIntVarMap notWithinEnvironmentFrontUbUb =
  //       opl.getElement("notWithinEnvironmentFrontUbUb").asIntVarMap();
  //   setBranchingPriorityFromTensor(cplex, notWithinEnvironmentFrontUbUb,
  //                                  env_frontubub_prio, 0, {});
  // }

  // if (NrEnvironments > 0) {
  //   // notWithinEnvironmentFrontLbUb
  //   Eigen::Tensor<int, 3> env_frontlbub_prio;  //(NrCars, NrEnvironments, N);
  //   env_frontlbub_prio.resize(NrCars, NrEnvironments, N);
  //   env_frontlbub_prio.setConstant(default_prio);
  //   Eigen::array<long, 3> offset = {0, 0, 0};
  //   Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N_extent};
  //   env_frontlbub_prio.slice(offset, extent).setConstant(increase_prio);
  //   IloIntVarMap notWithinEnvironmentFrontLbUb =
  //       opl.getElement("notWithinEnvironmentFrontLbUb").asIntVarMap();
  //   setBranchingPriorityFromTensor(cplex, notWithinEnvironmentFrontLbUb,
  //                                  env_frontlbub_prio, 0, {});
  // }

  // if (NrEnvironments > 0) {
  //   // notWithinEnvironmentFrontUbLb
  //   Eigen::Tensor<int, 3> env_frontublb_prio;  //(NrCars, NrEnvironments, N);
  //   env_frontublb_prio.resize(NrCars, NrEnvironments, N);
  //   env_frontublb_prio.setConstant(default_prio);
  //   Eigen::array<long, 3> offset = {0, 0, 0};
  //   Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N_extent};
  //   env_frontublb_prio.slice(offset, extent).setConstant(increase_prio);
  //   IloIntVarMap notWithinEnvironmentFrontUbLb =
  //       opl.getElement("notWithinEnvironmentFrontUbLb").asIntVarMap();
  //   setBranchingPriorityFromTensor(cplex, notWithinEnvironmentFrontUbLb,
  //                                  env_frontublb_prio, 0, {});
  // }

  // if (NrEnvironments > 0) {
  //   // notWithinEnvironmentFrontLbLb
  //   Eigen::Tensor<int, 3> env_frontlblb_prio;  //(NrCars, NrEnvironments, N);
  //   env_frontlblb_prio.resize(NrCars, NrEnvironments, N);
  //   env_frontlblb_prio.setConstant(default_prio);
  //   Eigen::array<long, 3> offset = {0, 0, 0};
  //   Eigen::array<long, 3> extent = {NrCars, NrEnvironments, N_extent};
  //   env_frontlblb_prio.slice(offset, extent).setConstant(increase_prio);
  //   IloIntVarMap notWithinEnvironmentFrontLbLb =
  //       opl.getElement("notWithinEnvironmentFrontLbLb").asIntVarMap();
  //   setBranchingPriorityFromTensor(cplex, notWithinEnvironmentFrontLbLb,
  //                                  env_frontlblb_prio, 0, {});
  // }
}

template <class TensorType>
void CplexWrapper::setBranchingPriorityFromTensor(IloCplex& cplex,
                                                  IloIntVarMap& varMap,
                                                  TensorType& vals,
                                                  size_t dimDepth,
                                                  std::vector<size_t> dimIdx) {
  const auto& dim = vals.dimensions();
  if (dim.size() > dimDepth) {
    for (int i = 0; i < dim[dimDepth]; ++i) {  // loop over the current dim
      if (dim.size() == dimDepth + 1) {
        // We reached the actual element: set priority
        std::vector<size_t> dimIdxNext = dimIdx;
        dimIdxNext.push_back(i);
        cplex.setPriority(varMap.eltAtAbsoluteIndex(i), vals(dimIdxNext));
      } else {
        // Recursive call: increase one dimension
        IloIntVarMap subVarMap = varMap.getAtAbsoluteIndex(i);
        dimIdx.push_back(i);
        setBranchingPriorityFromTensor<TensorType>(cplex, subVarMap, vals,
                                                   dimDepth + 1, dimIdx);
      }
    }
  }
}

void CplexWrapper::setBranchingPriorityValueExtent(int value, int extent) {
  branchingPriorityExtent_ = extent;
  increasedBranchingPriorityValue_ = value;
  LOG(INFO) << "Branching priority value set to " << value
            << " with extent = " << extent;
}

void CplexWrapper::overrideSolverSettingsDataSource(
    std::shared_ptr<ModelParameters> parameters) {
  ds_.resetCplexSolverSettingsOnly(parameters);
}

void CplexWrapper::setDebugOutputFilePath(std::string in) {
  if (!boost::filesystem::exists(in)) {
    if (!boost::filesystem::create_directories(in)) {
      LOG(ERROR) << "Could not create logging directory at: " << in.c_str();
    }
  }
  debugOutputFilePath_ = in;
}

}  // namespace cplex
}  // namespace planner
}  // namespace miqp
