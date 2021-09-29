cc_library(
    name = "parameters",
    hdrs = [
        "limits_parameters.hpp",
        "linearization_parameters.hpp",
        "vehicle_parameters.hpp",
        "regions.hpp",
        "fitting_polynomial_parameters.hpp",
    ],
    srcs = [
        "regions.cpp",
    ],
    deps = ["@com_github_eigen_eigen//:eigen"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "parameter_preparer",
    srcs = [
        "parameter_preparer.cpp",
    ],
    hdrs = [
        "parameter_preparer.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bark_project//bark:include",
        "@com_github_glog_glog//:glog",
        "@boost//:math",
        ":parameters",
        "//common/math:math"
    ],
)