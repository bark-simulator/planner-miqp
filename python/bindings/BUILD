package(default_visibility = ["//visibility:public"])
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

pybind_library(
  name = "planner_miqp",
  srcs = ["python_planner_miqp.cpp"],
  hdrs = ["python_planner_miqp.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//src:behavior_miqp",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT", "-DPLANNER_MIQP"],
)

pybind_library(
  name = "convexified_map",
  srcs = ["python_convexified_map.cpp"],
  hdrs = ["python_convexified_map.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//common/map:convexified_map",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
)

pybind_library(
  name = "cplex_wrapper",
  srcs = ["python_cplex_wrapper.cpp"],
  hdrs = ["python_cplex_wrapper.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//src:planner_miqp",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
  data = ["//cplexmodel:cplex_models_filegroup"],
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
)

# the following rules are only for testing purposes
pybind_extension(
  name = "miqp",
  srcs = ["python_module.cpp"],
  deps = [
    "//python/bindings:convexified_map",
    "//python/bindings:cplex_wrapper",
    "@bark_project//bark/world:world",
    "@bark_project//bark/python_wrapper:behavior",
  ],
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT", "-DPLANNER_MIQP"],
)