from engine.pyalice import Application
from pose_to_edge import PoseToEdge
from pprint import pprint

def main():
    app = Application(app_filename="apps/rm/realsense_cartographer/realsense_cartographer.app.json")

    pose_to_edge = app.nodes["pose_tree_injector"].add(PoseToEdge, "PoseToEdge")
    app.connect(app.nodes["visual_odometry_tracker"]["StereoVisualOdometry"], "left_camera_pose", pose_to_edge, "pose")
    app.connect(pose_to_edge, "edge", app.nodes["pose_tree_injector"]["PoseMessageInjector"], "pose")

    comp = app.nodes['visual_odometry_tracker']["StereoVisualOdometry"]
    pprint(comp.config.get_config())
    
    app.run()

if __name__ == '__main__':
    main()