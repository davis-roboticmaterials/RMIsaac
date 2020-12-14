
load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app")

isaac_py_app(
    name = "realsense_cartographer",
    srcs = [
        "realsense_cartographer.py"
    ],
    data = [
        "chair_bot.lua",
        "realsense_cartographer.app.json",
        "//packages:py_init",
    ],
    modules = [
        "perception:stereo_visual_odometry",
        "lidar_slam:cartographer",
        "navigation",
        "realsense",
        "utils",
        "viewers",
        "sight",
        "rgbd_processing",
    ],
    deps = [
        "//engine/pyalice",
    ],
)