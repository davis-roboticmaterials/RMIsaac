from engine.pyalice import Application
from odometry import Odometry

from pprint import pprint

def main():
    app = Application(app_filename="apps/rm/realsense_gmap/realsense_gmap.app.json")

    odometry = app.nodes["odometry"].add(Odometry, "Odometry")
    app.connect(app.nodes["visual_odometry_tracker"]["StereoVisualOdometry"], "left_camera_pose", odometry, "pose")
    app.connect(odometry, "odometry", app.nodes["gmapping"]["GMapping"], "odometry")

    gmap = app.nodes['gmapping']["GMapping"]
    pprint(gmap.config.get_config())

    app.run()

if __name__ == '__main__':
    main()