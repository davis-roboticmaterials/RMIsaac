from engine.pyalice import Application, Node, Codelet, Message
from odometry_spoofer import OdometrySpoofer

if __name__ == "__main__":
    app = Application(name="realsense_gmap",
                      modules=[
                          "perception:stereo_visual_odometry",
                          "lidar_slam:g_mapping",
                          "navigation",
                          "realsense",
                          "rgbd_processing",
                          "utils",
                          "sight",
                          "viewers",
                      ])

    # ========== Nodes ========== #
    # Realsense camera
    camera = app.add("camera").add(app.registry.isaac.RealsenseCamera)

    # Pose Initializer
    pose_initializer = app.add("pose_initializer").add(app.registry.isaac.alice.PoseInitializer)
    lidar_initializer = app.add("lidar_initializer").add(app.registry.isaac.alice.PoseInitializer)

    # Split IR channels into left and right
    splitter_left = app.add("camera_splitter_left").add(app.registry.isaac.utils.ColorCameraProtoSplitter)
    splitter_right = app.add("camera_splitter_right").add(app.registry.isaac.utils.ColorCameraProtoSplitter)

    # Image flattener (flattens depth to LIDAR approximation)
    flattener = app.add("image_flattener")
    depth_flattener = flattener.add(app.registry.isaac.rgbd_processing.DepthImageFlattening)
    # throttler = flattener.add(app.registry.isaac.alice.Throttle)
    
    # Visual Odometry tracker
    tracker = app.add("tracker").add(app.registry.isaac.StereoVisualOdometry)

    # Odometry Spoofer that takes the pose from the tracker and spoofs vel/acc data
    odometry = app.add("odometry").add(OdometrySpoofer)

    # Gmapper
    # Apperently nodes are initialized in alphabetical order. So since I cannot figure out 
    # how to change the start order, adding a 'z' in front of the gmapping seems to work...
    gmap = app.add('zgmapping').add(app.registry.isaac.lidar_slam.GMapping)
    scan_to_map = app.add("zrange_scan_to_observation_map").add(app.registry.isaac.navigation.RangeScanToObservationMap)

    # Websight Viewers
    viewer = app.add("viewers")
    color_viewer = viewer.add(app.registry.isaac.viewers.ColorCameraViewer)
    depth_viewer = viewer.add(app.registry.isaac.viewers.DepthCameraViewer)
    map_viewer = viewer.add(app.registry.isaac.sight.SightWidget, "Map2")
    scan_viewer = viewer.add(app.registry.isaac.sight.SightWidget, "Current Scan Data")

    # ========== Edges ========== #
    # Connect left IR to tracker
    app.connect(camera, "left_ir", splitter_left, "color_camera")
    app.connect(splitter_left, "image", tracker, "left_image")
    app.connect(splitter_left, "intrinsics", tracker, "left_intrinsics")

    # Connect right IR to tracker
    app.connect(camera, "right_ir", splitter_right, "color_camera")
    app.connect(splitter_right, "image", tracker, "right_image")
    app.connect(splitter_right, "intrinsics", tracker, "right_intrinsics")

    # Connect viewer elements
    app.connect(camera, "left_ir", color_viewer, "color_listener")
    app.connect(camera, "depth", depth_viewer, "depth_listener")

    # Connect depth flattener
    # app.connect(camera, "depth", throttler, "input")
    # app.connect(throttler, "output", depth_flattener, "depth")
    app.connect(camera, "depth", depth_flattener, "depth")

    # Connect GMap
    app.connect(tracker, "left_camera_pose", odometry, "pose")
    app.connect(depth_flattener, "flatscan", gmap, "flatscan")
    app.connect(depth_flattener, "flatscan", scan_to_map, "flatscan")
    app.connect(odometry, "odometry", gmap, "odometry")

    # ========== Config ========== #
    # ----- Camera Config ----- #
    camera.config.cols = 640
    camera.config.rows = 360
    camera.config.depth_framerate = 30
    camera.config.ir_framerate = 30
    camera.config.enable_depth = True
    camera.config.enable_ir_stereo = True
    camera.config.enable_color = False
    camera.config.enable_depth_laser = False
    camera.config.align_to_color = False  
    camera.config.frame_queue_size = 2
    camera.config.auto_exposure_priority = False

    # ----- Pose Initializer Config ----- #
    pose_initializer.config.lhs_frame = "robot"
    pose_initializer.config.rhs_frame = "realsense"
    pose_initializer.config.pose = {
        "translation": [0, 0, 0]
    }

    lidar_initializer.config.lhs_frame = "robot"
    lidar_initializer.config.rhs_frame = "lidar"
    lidar_initializer.config.pose = {
        "translation": [0, 0, 0]
    }

    # ----- Splitter Configs ----- #
    splitter_left.config.only_pinhole = False
    splitter_right.config.only_pinhole = False

    # ----- Tracker Config ----- #
    tracker.config.horizontal_stereo_camera = True
    tracker.config.process_imu_readings = False
    tracker.config.lhs_camera_frame = "left_ir_camera"
    tracker.config.rhs_camera_frame = "right_ir_camera"

    # ----- Viewer Config ----- #
    color_viewer.config.reduce_scale = 2

    depth_viewer.config.reduce_scale = 2
    depth_viewer.config.min_visualization_depth: 0.2
    depth_viewer.config.max_visualization_depth: 5.0
    depth_viewer.config.camera_name = "Realsense"
    depth_viewer.config.colormap = [
          [128,   0,   0],
          [255,   0,   0],
          [255, 255,   0],
          [0,   255, 255],
          [0,     0, 255],
          [0,     0, 128]
        ]

    # ----- Websight Config (load from file) ----- #
    app.load("apps/rm/realsense_gmap/websight_config.json")

    # ----- Flattener Config ----- #
    # depth_flattener.config.min_distance = 0.13
    # depth_flattener.config.max_distance = 10
    # depth_flattener.config.height_min = 0.0
    # depth_flattener.config.height_max = 10.0
    depth_flattener.config.ground_frame = "robot"
    depth_flattener.config.camera_frame = "realsense"
    # depth_flattener.config.range_delta = 0.015
    # depth_flattener.config.sector_delta = 0.008

    # throttler.config.data_channel = "input"
    # throttler.config.output_channel = "output"
    # throttler.config.minimum_interval = 0.1
    # throttler.config.use_signal_channel = False

    # ----- GMap Config ----- #
    # gmap.node.start_order = 200
    # gmap.config.use_pose_tree = False
    # gmap.config.file_path = "/tmp"
    # gmap.config.build_map_period = 1.
    # gmap.config.map_x_max = 10.0
    # gmap.config.map_x_min = -10.0
    # gmap.config.map_y_max = 10.0
    # gmap.config.map_y_min = -10.0
    # gmap.config.map_resolution = 0.005
    # gmap.config.max_range = 12
    # gmap.config.map_update_range = 10
    # gmap.config.number_particles = 40
    # gmap.config.linear_distance = 0.4
    # gmap.config.angular_distance = 0.14

    app.run()
