workspace(name = "planner_miqp")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

load("//util:deps.bzl", "planner_miqp_dependencies")
planner_miqp_dependencies()

load("@bark_project//tools:deps.bzl", "bark_dependencies")
bark_dependencies()

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python") 