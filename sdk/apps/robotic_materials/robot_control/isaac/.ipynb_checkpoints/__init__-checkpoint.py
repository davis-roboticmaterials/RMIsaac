import ipywidgets as widgets

from isaac.isaac_arm import IsaacArm
from isaac.isaac_camera import IsaacCamera
from isaac.isaac_effector import IsaacEffector

import os
# set the current working directory to the deployed package folder. This is required by isaac.
os.chdir("/home/davis/deploy/davis/rm_isaac_bridge-pkg")
os.getcwd()

from engine.pyalice import Application
from IPython.display import display

class RMIsaacBridge(object):
    def __init__(self):
        return
    
    def add_component(self, component, component_config):
        if component == 'isaac_ur10':
            self.arm = IsaacArm(component_config)
        elif component == 'isaac_camera':
            self.camera = IsaacCamera(component_config)
        elif component == 'isaac_smarthand':
            self.effector = IsaacEffector(component_config)

    def initialize_app(self):
        app = Application(name="rm_isaac_bridge")
        app.load(filename="packages/navsim/apps/navsim_tcp.subgraph.json", prefix="simulation")
        
        if hasattr(self, 'arm'):
            app = self.arm.connect_app(app)
        if hasattr(self, 'camera'):
            app = self.camera.connect_app(app)
        if hasattr(self, 'effector'):
            app = self.effector.connect_app(app)
        
        self._app = app
    
    def start_app(self):
        
        available_widgets = []
        if hasattr(self, 'camera') and self.camera.config['widgets']:
            available_widgets.append(widgets.HBox([self.camera.color_widget, self.camera.depth_widget]))
        if hasattr(self, 'arm') and self.arm.config['widgets']:
            available_widgets.append(self.arm.joint_widget.panel)
        if hasattr(self, 'effector') and self.effector.config['widgets']:
            available_widgets.append(self.effector.finger_widget.panel)
        
        if len(available_widgets) > 0:
            display(widgets.VBox(available_widgets))
        
        self._app.start()
        
    def stop_app(self):
        self._app.stop()