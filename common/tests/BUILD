
filegroup(
   name="xodr_data",
   srcs=glob(["data/**"]),
   visibility = ["//visibility:public"],
)

cc_test(
    name = "geometry_test",
    srcs = [
        "geometry_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//common/geometry:geometry",
        "@bark_project//bark/geometry:geometry",
        "@gtest//:main",
    ],
)

cc_test(
    name = "convexified_map_test",
    srcs = [
        "convexified_map_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:main",
        "//common/map:convexified_map",
        "@bark_project//bark/commons/params:params",
    ],
)

cc_test(
    name = "reference_trajectory_generator_test",
    srcs = [
        "reference_trajectory_generator_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:main",
        "//common/reference:reference_trajectory_generator",
        "@bark_project//bark/commons/params:params",
        "@bark_project//bark/models/dynamic:dynamic",
        "@bark_project//bark/geometry:geometry",
    ],
)

cc_test(
    name = "parameter_preparer_test",
    srcs = [
        "parameter_preparer_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:main",
        "//common/parameter:parameter_preparer",
        "@bark_project//bark/commons/params:params",
    ],
)

cc_test(
    name = "regions_test",
    srcs = [
        "regions_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:main",
        "//common/parameter:parameters",
        "//common/parameter:parameter_preparer",
        "@bark_project//bark/commons/params:params",
    ],
)

cc_test(
    name = "fitting_polynomial_parameters_test",
    srcs = [
        "fitting_polynomial_parameters_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:main",
        "//common/parameter:parameters",
    ],
)
