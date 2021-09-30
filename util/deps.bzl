load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

def planner_miqp_dependencies():
    _maybe(
    git_repository,
    name = "bark_project",
    commit="53562ac52fd948e8f5b7dec025bf909922c0afac",
    remote = "https://github.com/bark-simulator/bark",
    )
    
    _maybe(
      git_repository,
      name = "com_github_glog_glog",
      commit = "c5dcae830670bfaea9573fa7b700e862833d14ff",
      remote = "https://github.com/google/glog"
    )

    _maybe(
    http_archive,
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file_content = """
cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    copts = ["-Iexternal/gtest/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)""",
    strip_prefix = "googletest-release-1.7.0",
    )

    _maybe(
        http_archive,
        # Need Eigen 3.4 (which is in development) for STL-compatible iterators
        name = "com_github_eigen_eigen",
        build_file_content = """
cc_library(
    name = 'eigen',
    srcs = [],
    includes = ['.'],
    hdrs = glob(['Eigen/**', 'unsupported/**']),
    visibility = ['//visibility:public'],
)""",
        sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
        strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
        urls = [
            "https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip",
        ],
    )

    _maybe(
    git_repository,
    name = "com_github_nelhage_rules_boost",
    commit = "fb9f3c9a6011f966200027843d894923ebc9cd0b",
    remote = "https://github.com/nelhage/rules_boost"
    )

    _maybe(
    native.new_local_repository,
    name = "local_cplex_installation",
    path = "/opt/ibm/ILOG/CPLEX_Studio1210/opl",
    build_file_content = """
cc_library(
    name = "cplex",
    srcs = [
        "lib/x86-64_linux/static_pic/libopl.a",
        "lib/x86-64_linux/static_pic/libiljs.a",
        "lib/x86-64_linux/static_pic/libilocplex.a",
        "lib/x86-64_linux/static_pic/libcp.a",
        "lib/x86-64_linux/static_pic/libconcert.a",
        "bin/x86-64_linux/libcplex12100.so",
        "bin/x86-64_linux/libicuuc.so",
        "bin/x86-64_linux/libicui18n.so",
        "bin/x86-64_linux/libicuio.so",
        "bin/x86-64_linux/libicudata.so",
        "bin/x86-64_linux/liboplnl1.so",
    ],
    hdrs = glob(["include/**/*.h"]),
    visibility = ["//visibility:public"],
    includes = ["include"],
)""",
    )

    _maybe(
    http_archive,
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file_content = """
cc_library(
    name = "pybind11",
    hdrs = glob([
        "include/**/**/*.h",
    ]),
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include/"
)
"""
    )

    _maybe(
    native.new_local_repository,
    name = "python_linux",
    path = "./python/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
    """
    )

    _maybe(
      new_git_repository,
      name = "com_github_spline",
      commit = "619c634ef5f6f2df1508c767f979eb4b7bf9c66a",
      remote = "https://github.com/ttk592/spline",
      build_file_content = """
cc_library(
    name = 'spline',
    srcs = [],
    includes = ['.'],
    hdrs = ["src/spline.h"],
    visibility = ['//visibility:public'],
)
    """
    )
    
def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)