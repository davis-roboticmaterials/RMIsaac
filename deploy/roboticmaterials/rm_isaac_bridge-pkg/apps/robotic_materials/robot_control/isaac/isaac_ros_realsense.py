from collections import deque  
import numpy as np
import time
import os

import io, ipywidgets
from PIL import Image
import matplotlib.pyplot as plt
import ipywidgets as widgets

from isaac.utilities import *

# set the current working directory to the deployed package folder. This is required by isaac.
os.chdir("/home/davis/deploy/davis/rm_isaac_bridge-pkg")

from packages.pyalice import Codelet

cmap = plt.get_cmap('jet')

class IsaacRosRealsense(object):
    def __init__(self, config):
        self.config = config
        self._timeout = config['timeout']
        
        self.color = np.zeros((720,1280))
        self.depth = np.zeros((720,1280))
        
        self._widgets = self.config['widgets']
        self._stream_color = self.config['stream_color']
        self._stream_depth = self.config['stream_depth']
        
        if self._widgets:
            self.color_widget = ipywidgets.Image(height=310, width=640)
            self.depth_widget = ipywidgets.Image(height=310, width=640)
        
        self._command_queue = deque()
        
    def command(self, action, payload=None):
        if action not in ['get_depth', 'get_color']:
            raise ValueError(action+' is not a valid action type')

        command = Command(action, payload)
        self._command_queue.append(command)
        
        elapsed = 0
        while command.response is None and elapsed < self._timeout:
            elapsed += 0.01
            time.sleep(0.01)
            
        return command.response
    
    def enable_color_stream(self):
        self._stream_color = True
    def disable_color_stream(self):
        self._stream_color = False
        
    def enable_depth_stream(self):
        self._stream_depth = True
    def disable_depth_stream(self):
        self._stream_depth = False
        
    def enable_all_streams(self):
        self._stream_color = True
        self._stream_depth = True
    def disable_all_streams(self):
        self._stream_color = False
        self._stream_depth = False
        
    def _ColorReciever(self):
        parent = self
        class ColorReciever(Codelet):
            def start(self):
                self.rx = self.isaac_proto_rx("ColorCameraProto", "color")
                self.tick_on_message(self.rx)
            
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'get_color':
                    msg = self.rx.message
                    tensor = msg.tensor
                    test_img = tensor
                    parent.color = tensor
                    command = parent._command_queue.popleft()
                    command.response = parent.color
                    
                elif parent._stream_color:
                    msg = self.rx.message
                    tensor = msg.tensor
                    parent.color = tensor
                    
                else:
                    return
                
                if parent._widgets:
                    bts = io.BytesIO()
                    Image.fromarray(parent.color).save(bts, 'png')
                    parent.color_widget.value = bts.getvalue()
                        
                    
        return ColorReciever
    
    def _DepthReciever(self):
        parent = self
        class DepthReciever(Codelet):
            def start(self):
                self.rx = self.isaac_proto_rx("ColorCameraProto", "depth")
                self.tick_on_message(self.rx)
            
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'get_depth':
                    msg = self.rx.message
                    tensor = msg.tensor
                    parent.depth = tensor
                    command = parent._command_queue.popleft()
                    command.response = parent.depth
                    
                elif parent._stream_depth:
                    msg = self.rx.message
                    tensor = msg.tensor
                    parent.depth = tensor
                    
                else:
                    return
                
                if parent._widgets:
                    min_depth = np.min(parent.depth)
                    max_depth = np.max(parent.depth)
                    
                    normalized_1channel = np.array(255*(parent.depth-min_depth)/(max_depth-min_depth), dtype=np.uint8)
                    rgba_depth = np.array(255*cmap(normalized_1channel), dtype=np.uint8)
                    
                    bts = io.BytesIO()
                    Image.fromarray(rgba_depth).save(bts, 'png')
                    parent.depth_widget.value = bts.getvalue()
                    
        return DepthReciever
    
    def connect_app(self, app):
        
        ros_interface = app.nodes["ros_perception.subgraph"]["interface"]
        ros_converters = app.nodes["ros_perception.ros_converters"]

        ros_converters["RosToDepth"].config.channel_name = "/camera/depth/image_rect_raw"
        ros_converters["RosToImage"].config.channel_name = "/camera/color/image_raw"

        color_in_node = app.add('color_input')
        color_in_node.add(self._ColorReciever(), 'color_reciever')
        app.connect(ros_interface, 'color', color_in_node['color_reciever'], 'color')
        
        depth_in_node = app.add('depth_input')
        depth_in_node.add(self._DepthReciever(), 'depth_reciever')
        app.connect(ros_interface, 'depth', depth_in_node['depth_reciever'], 'depth')
        
        return app