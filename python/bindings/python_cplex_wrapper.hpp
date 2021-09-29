// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_BINDINGS_PYTHON_CPLEX_WRAPPER_HPP_
#define PYTHON_BINDINGS_PYTHON_CPLEX_WRAPPER_HPP_

#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

void python_cplex_wrapper(py::module m);

void python_optimization_status(py::module m);

void python_solution_properties(py::module m);

void python_warmstart_type(py::module m);

void python_parallel_mode(py::module m);

#endif  // PYTHON_BINDINGS_PYTHON_CPLEX_WRAPPER_HPP_
