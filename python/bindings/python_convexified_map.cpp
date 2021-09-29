// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>

#include "common/map/convexified_map.hpp"
#include "python_convexified_map.hpp"

namespace py = pybind11;
using bark::commons::ParamsPtr;
using bark::geometry::Polygon;
using miqp::common::map::ConvexifiedMap;

void python_convexified_map(py::module m) {
  py::class_<ConvexifiedMap, std::shared_ptr<ConvexifiedMap>>(m,
                                                              "ConvexifiedMap")
      .def(py::init<const ParamsPtr&, const Polygon&, const double,
                    const double, const double, const double>())
      .def("Convert", &ConvexifiedMap::Convert)
      .def("GetIntersectingConvexPolygons",
           &ConvexifiedMap::GetIntersectingConvexPolygons)
      .def_property_readonly("map_nonconvex_polygon",
                             &ConvexifiedMap::GetMapNonConvexPolygon,
                             "input map polygon.")
      .def_property_readonly("map_convex_polygons",
                             &ConvexifiedMap::GetMapConvexPolygons,
                             "decomposed convex polygons.");
}