# TODO can I somehow link in this test against the so file?
cc_test(
    name = "miqp_planner_c_api_test",
    srcs = [
        "miqp_planner_c_api_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
    deps = [
        "@gtest//:main",
        "//src:miqp_planner_c_api_test_cclibrary",
    ],
    tags = ["exclusive"],
)

cc_test(
    name = "cplex_wrapper_test",
    srcs = [
        "cplex_wrapper_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
    deps = [
        "@gtest//:main",
        "//src:planner_miqp",
    ],
    tags = ["exclusive"],
)

cc_test(
    name = "miqp_planner_test",
    srcs = [
        "miqp_planner_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
    deps = [
        "@gtest//:main",
        "//src:planner_miqp",
    ],
    tags = ["exclusive"],
)

cc_test(
    name = "behavior_miqp_agent_test",
    srcs = [
        "behavior_miqp_agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
    deps = [
        "@gtest//:main",
        "//src:behavior_miqp",
        "@bark_project//bark/commons/params:params",
        "@bark_project//bark/world/tests:make_test_xodr_map",
        "@bark_project//bark/models/execution/interpolation:interpolation",
        "@bark_project//bark/models/behavior/constant_acceleration:constant_acceleration",

    ],
    tags = ["exclusive"],
)

py_test(
  name = "py_behavior_miqp_agent_test",
  srcs = ["py_behavior_miqp_agent_test.py"],
  data = ['//python/bindings:miqp.so',
          '@bark_project//bark:generate_core',
          '//common/tests:xodr_data',
          "//common/simulator:simulator"],
  imports = ['../python/bindings'],
  deps = ["@bark_project//bark/runtime/commons:parameters",
          "@bark_project//bark/runtime:runtime"],
  tags = ["exclusive"],
)

py_test(
  name = "py_miqp_simulation_test",
  srcs = ["py_miqp_simulation_test.py"],
  data = ['//python/bindings:miqp.so',
          '@bark_project//bark:generate_core',
          '//common/tests:xodr_data',
          "//common/simulator:simulator"],
  imports = ['../python/bindings'],
  deps = ["@bark_project//bark/runtime/commons:parameters",
          "@bark_project//bark/runtime:runtime"],
  tags = ["exclusive"],
)