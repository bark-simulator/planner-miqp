py_library(
  name = "simulator",
  srcs = ["simulator.py"],
  visibility = ["//visibility:public"],
  data = ['//python/bindings:miqp.so',
          '@bark_project//bark:generate_core',
          '//common/tests:xodr_data',
          "//common/viewer:custom_matplotlib_viewer"],
  imports = ['../python/bindings'],
  deps = ["@bark_project//bark/runtime/commons:parameters",
          "@bark_project//bark/runtime:runtime"],
)

