package(default_visibility = ["//visibility:public"])

cc_library(
  name = "planner_miqp",
  srcs = ["python_planner_miqp.cpp"],
  hdrs = ["python_planner_miqp.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//src:behavior_miqp",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT", "-DPLANNER_MIQP"],
)

cc_library(
  name = "convexified_map",
  srcs = ["python_convexified_map.cpp"],
  hdrs = ["python_convexified_map.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//common/map:convexified_map",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
)

cc_library(
  name = "cplex_wrapper",
  srcs = ["python_cplex_wrapper.cpp"],
  hdrs = ["python_cplex_wrapper.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//src:planner_miqp",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
  data = ["@cplex_models//:cplex_models_filegroup"],
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
)

# the following rules are only for testing purposes
cc_binary(
  name = "miqp.so",
  srcs = ["python_module.cpp"],
  deps = [
    "//python/bindings:convexified_map",
    "//python/bindings:cplex_wrapper",
    "@bark_project//bark/world:world",
    "@bark_project//bark/python_wrapper:behavior",
  ],
  linkshared = 1,
  linkstatic = 1,
  copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT", "-DPLANNER_MIQP"],
)