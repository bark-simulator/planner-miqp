// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "python/bindings/python_planner_miqp.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"

#include "src/behavior_miqp_agent.hpp"
#include "src/dynamic_occupancy.hpp"

namespace py = pybind11;
using bark::commons::Params;
using bark::geometry::Polygon;
using bark::models::behavior::BehaviorMiqpAgent;
using bark::models::behavior::BehaviorModel;
using bark::models::behavior::DynamicOccupancy;
using bark::models::behavior::OccupancyType;
using bark::world::objects::AgentId;
using namespace bark::models::behavior;

void python_planner_miqp(py::module m) {
  py::enum_<OccupancyType>(m, "OccupancyType")
      .value("PREDICTION", OccupancyType::PREDICTION)
      .value("RULE_NO_RIGHT_PASSING", OccupancyType::RULE_NO_RIGHT_PASSING)
      .value("RULE_SAFE_DISTANCE", OccupancyType::RULE_SAFE_DISTANCE)
      .export_values();

  py::class_<DynamicOccupancy, std::shared_ptr<DynamicOccupancy>>(
      m, "DynamicOccupancy")
      .def(py::init<int, const AgentId, const Polygon&, OccupancyType, bool>())
      .def_property_readonly("id", &DynamicOccupancy::GetAgentId,
                             "get agent id.")
      .def_property_readonly("shape", &DynamicOccupancy::GetShape, "get shape.")
      .def_property_readonly("type", &DynamicOccupancy::GetOccupancyType,
                             "get occupancy type.")
      .def_property_readonly("prediction", &DynamicOccupancy::GetPrediction,
                             "get prediction.")
      .def_property_readonly("is_soft", &DynamicOccupancy::GetIsSoft,
                             "get is_soft.");

  py::class_<BehaviorMiqpAgent, BehaviorModel,
             std::shared_ptr<BehaviorMiqpAgent>>(m, "BehaviorMiqpAgent")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("GetCollisionCircleCenters",
           &BehaviorMiqpAgent::GetCollisionCircleCenters)
      .def("GetFrontLbUb", &BehaviorMiqpAgent::GetFrontLbUb)
      .def("GetLastCplexWrapper", &BehaviorMiqpAgent::GetLastCplexWrapper)
      .def("GetLastSolutionProperties",
           &BehaviorMiqpAgent::GetLastSolutionProperties)
      .def("GetCollisionRadius", &BehaviorMiqpAgent::GetCollisionRadius)
      .def("SetWarmstartType", &BehaviorMiqpAgent::SetWarmstartType)
      .def_property_readonly("last_trajectories_all_cars",
                             &BehaviorMiqpAgent::GetLastTrajectoriesAllCars,
                             "planned trajectories of all agents.")
      .def_property_readonly("ref_trajectories",
                             &BehaviorMiqpAgent::GetRefTrajectories,
                             "reference trajectories.")
      .def_property_readonly("reference_lines",
                             &BehaviorMiqpAgent::GetReferenceLines,
                             "reference lines.")
      .def_property_readonly(
          "ref_trajectories_longer_horizon",
          &BehaviorMiqpAgent::GetRefTrajectoriesLongerHorizon,
          "reference trajectories with longer horizon.")
      .def_property_readonly(
          "convex_shrinked_env_polygons_all_cars",
          &BehaviorMiqpAgent::GetConvexShrinkedEnvPolygonsAllCars,
          "decomposed convex polygons from planner.")
      .def_property_readonly("env_poly",
                             &BehaviorMiqpAgent::GetEnvironmentPolygon,
                             "environment polygon")
      .def_property_readonly("last_planning_success",
                             &BehaviorMiqpAgent::GetLastPlanningSuccess,
                             "true if last planning was successfull")
      .def_property_readonly("last_dynamic_occupancies",
                             &BehaviorMiqpAgent::GetLastDynamicOccupancies,
                             "last dynamic occupancies")
      .def("__repr__",
           [](const BehaviorMiqpAgent& m) {
             return "bark.behavior.BehaviorMiqpAgent";
           })
      .def(py::pickle(
          [](const BehaviorMiqpAgent& b) {
            // We throw away other information such as last trajectories
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new BehaviorMiqpAgent(
                PythonToParams(t[0].cast<py::tuple>()));
          }));
}
