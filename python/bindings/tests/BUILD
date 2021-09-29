package(default_visibility = ["//visibility:public"])

py_test(
   name = "python_import_test",
   srcs = ["python_import_test.py"],
   data = ["//python/bindings:miqp.so"],
   imports = ["../../../python/bindings/"],
   deps=["@bark_project//bark/runtime/commons:commons"],
)

py_test(
  name = "py_pickle_tests",
  srcs = ["py_pickle_tests.py"],
  data = ['@bark_project//bark:generate_core'],
  deps = ["@bark_project//bark/runtime/commons:commons"]
)
