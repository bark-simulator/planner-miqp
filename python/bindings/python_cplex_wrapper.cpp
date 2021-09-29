// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>

#include "python_cplex_wrapper.hpp"
#include "src/cplex_wrapper.hpp"

namespace py = pybind11;
using bark::commons::ParamsPtr;
using miqp::planner::cplex::CplexWrapper;
using miqp::planner::cplex::OptimizationStatus;
using miqp::planner::cplex::SolutionProperties;

void python_cplex_wrapper(py::module m) {
  py::class_<CplexWrapper>(m, "CplexWrapper")
      .def(py::init<const char*, const int>())
      .def("setParameterDatFileAbsolute",
           &CplexWrapper::setParameterDatFileAbsolute)
      .def("callCplex", &CplexWrapper::callCplex)
      .def("setDebugOutputFilePath", &CplexWrapper::setDebugOutputFilePath)
      .def("setDebugOutputFilePrefix", &CplexWrapper::setDebugOutputFilePrefix)
      .def("setDebugOutputPrint", &CplexWrapper::setDebugOutputPrint)
      .def("getDebugOutputParameterFilePath",
           &CplexWrapper::getDebugOutputParameterFilePath)
      .def("getSolutionProperties", &CplexWrapper::getSolutionProperties);
}

void python_optimization_status(py::module m) {
  py::enum_<OptimizationStatus>(m, "OptimizationStatus", py::arithmetic())
      .value("SUCCESS", OptimizationStatus::SUCCESS)
      .value("FAILED_NO_SOLUT", OptimizationStatus::FAILED_NO_SOLUT)
      .value("FAILED_SEG_FAULT", OptimizationStatus::FAILED_SEG_FAULT)
      .value("FAILED_TIMEOUT", OptimizationStatus::FAILED_TIMEOUT)
      .export_values();
}

void python_solution_properties(py::module m) {
  py::class_<SolutionProperties>(m, "SolutionProperties")
      .def(py::init())
      .def_readwrite("objective", &SolutionProperties::objective)
      .def_readwrite("status", &SolutionProperties::status)
      .def_readwrite("gap", &SolutionProperties::gap)
      .def_readwrite("time", &SolutionProperties::time);
}

void python_warmstart_type(py::module m) {
  py::enum_<CplexWrapper::WarmstartType>(m, "WarmstartType")
      .value("NO_WARMSTART", CplexWrapper::WarmstartType::NO_WARMSTART)
      .value("RECEDING_HORIZON_WARMSTART",
             CplexWrapper::WarmstartType::RECEDING_HORIZON_WARMSTART)
      .value("LAST_SOLUTION_WARMSTART",
             CplexWrapper::WarmstartType::LAST_SOLUTION_WARMSTART)
      .value("BOTH_WARMSTART_STRATEGIES",
             CplexWrapper::WarmstartType::BOTH_WARMSTART_STRATEGIES)
      .export_values();
}

void python_parallel_mode(py::module m) {
  py::enum_<CplexWrapper::ParallelMode>(m, "ParallelMode")
      .value("AUTO", CplexWrapper::ParallelMode::AUTO)
      .value("DETERMINISTIC", CplexWrapper::ParallelMode::DETERMINISTIC)
      .value("OPPORTUNISTIC", CplexWrapper::ParallelMode::OPPORTUNISTIC)
      .export_values();
}