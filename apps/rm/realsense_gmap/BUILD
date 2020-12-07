
load("//engine/build:isaac.bzl", "isaac_app", "isaac_py_app")

isaac_py_app(
    name = "realsense_gmap",
    srcs = [
        "realsense_gmap.py"
    ],
    data = [
        "odometry.py",
        "realsense_gmap.app.json",
        "//packages:py_init",
    ],
    modules = [
        "perception:stereo_visual_odometry",
        "lidar_slam:g_mapping",
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

isaac_py_app(
    name = "realsense_gmap_python",
    srcs = [
        "realsense_gmap_python.py"
    ],
    data = [
        "websight_config.json",
        "odometry.py",
        "//packages:py_init",
    ],
    modules = [
        "perception:stereo_visual_odometry",
        "lidar_slam:g_mapping",
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
