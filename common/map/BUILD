cc_library(
    name = "convexified_map",
    srcs = [
        "convexified_map.cpp",
    ],
    hdrs = [
        "convexified_map.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bark_project//bark:include",
        "@com_github_glog_glog//:glog",
        "@boost//:polygon",
        "@boost//:geometry",
        "//common/geometry:geometry",
    ],
)