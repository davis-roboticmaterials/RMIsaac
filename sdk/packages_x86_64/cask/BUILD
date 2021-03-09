
filegroup(
  name = "libcask_module.so_release", 
  srcs = ["libcask_module.so"],
  visibility = ["//visibility:public"],
)
 
filegroup(
  name = "pycask.so_release", 
  srcs = ["pycask.so"],
  visibility = ["//visibility:public"],
)
 
py_library( 
name = "pycask", 
srcs = [], 
visibility = ["//visibility:public"], 
data = [":pycask.so_release"], 
imports = [":pycask.so_release"], 
)