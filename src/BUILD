cc_library(
    name = "miqp_planner_c_api_test_cclibrary",
    srcs = [
        "miqp_planner_c_api.cpp",
    ],
    hdrs = [
        "miqp_planner_c_api.h",
        ],
    visibility = ["//visibility:public"],
    deps = [":planner_miqp",],
    copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT", "-DPLANNER_MIQP_CAPI_NO_APOLLO"],
)

cc_binary(
    name = "libmiqp_planner_c_api.so",
    srcs = [
        "miqp_planner_c_api.cpp",
        "miqp_planner_c_api.h",
        ],
    visibility = ["//visibility:public"],
    deps = [":planner_miqp",],
    copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
    linkshared = 1,
    linkstatic = 1,
)

cc_library(
    name = "behavior_miqp",
    srcs = [
        "behavior_miqp_agent.cpp",
    ],
    hdrs = [
        "behavior_miqp_agent.hpp",
        "dynamic_occupancy.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_miqp",
    ],
    copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
)

cc_library(
    name = "planner_miqp",
    srcs = [
        "model_input_data_source.cpp",
        "cplex_wrapper.cpp",
        "miqp_planner.cpp",
    ],
    hdrs = [
        "cplex_wrapper.hpp",
        "model_input_data_source.hpp",
        "miqp_planner_data.hpp",
        "miqp_planner_settings.h",
        "miqp_planner.hpp",
        "miqp_settings_from_param_server.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bark_project//bark:include",
        "@com_github_glog_glog//:glog",
        "@boost//:polygon",
        "@boost//:geometry",
        "@boost//:filesystem",
        "@local_cplex_installation//:cplex",
        "@com_github_eigen_eigen//:eigen",
        "//common/geometry:geometry",
        "//common/dynamic:dynamic_conversion",
        "//common/map:convexified_map",
        "//common/parameter:parameters",
        "//common/parameter:parameter_preparer",
        "//common/reference:reference_trajectory_generator",
    ],
    data = ["//cplexmodel:cplex_models_filegroup"],
    copts = ["-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"],
)

# cc_library(
#     name = "miqp_planner_data",
#     hdrs = [
#         "behavior_miqp_agent.hpp",
#         "cplex_wrapper.hpp",
#         "model_input_data_source.hpp",
#         "miqp_planner_data.hpp",
#     ],
#     visibility = ["//visibility:public"]
# )

