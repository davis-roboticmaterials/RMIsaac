from collections import deque  
import numpy as np
import time
import os

import json

from isaac.utilities import *

# set the current working directory to the deployed package folder. This is required by isaac.
os.chdir("/home/davis/deploy/davis/rm_isaac_bridge-pkg")

from engine.pyalice import Codelet
from engine.pyalice.gui.composite_widget import CompositeWidget


class IsaacArm:
    def __init__(self, config):
        self.config = config
        self._timeout = config['timeout']
        
        self._widgets = self.config['widgets']
        self._stream_joints = self.config['stream_joints']
        
        self.joint_names = self._load_kinematics(config['arm_type'])
        self.joints = CompositeArray(self.joint_names, 'position', [0]*len(self.joint_names))

        if self._widgets:
            self.joint_widget = CompositeWidget(self.joint_names, 'position', [[-2*np.pi, 2*np.pi]] * len(self.joint_names))
        
        self._command_queue = deque()
        
        
    def _load_kinematics(self, arm_type):
        valid_kinematics = ['ur10', 'kinova_j2n7']
        if arm_type not in valid_kinematics:
            raise ValueError('No valid kinematic file found for '+arm_type+'. Valid kinematics exist for '+', '.join(valid_kinematics))
            
        self._kinematic_file = "apps/assets/kinematic_trees/{}.kinematic.json".format(arm_type)
        joint_names = []
        with open(self._kinematic_file,'r') as fd:
            kt = json.load(fd)
            for link in kt['links']:
                if 'motor' in link and link['motor']['type'] != 'constant':
                    joint_names.append(link['name'])
                    
        return joint_names
    
    def command(self, action, payload=None):
        if action not in ['set_joint_angles', 'get_joint_angles']:
            raise ValueError(action+' is not a valid action type')
        
        if type(payload) in [list, np.ndarray]:
            payload = CompositeArray(self.joint_names, 'position', payload)
            
        command = Command(action, payload)
        self._command_queue.append(command)
        
        elapsed = 0
        while command.response is None and elapsed < self._timeout:
            elapsed += 0.01
            time.sleep(0.01)
            
        return command.response
        
    def enable_joint_stream(self):
        self._stream_joints = True
    def disable_joint_stream(self):
        self._stream_joints = False
    
    def enable_all_streams(self):
        self._stream_joints = True
    def disable_all_streams(self):
        self._stream_joints = False
        
    def _JointReciever(self):
        parent = self
        class JointReciever(Codelet):
            def start(self):
                self.rx = self.isaac_proto_rx("CompositeProto", "state")
                self.tick_on_message(self.rx)
            
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'get_joint_angles':
                    msg = self.rx.message
                    parent.joints.composite = msg
                    command = parent._command_queue.popleft()
                    command.response = parent.joints.values
                    
                elif parent._stream_joints:
                    msg = self.rx.message
                    parent.joints.composite = msg
                    
                else:
                    return
                
                if parent._widgets:
                    parent.joint_widget.composite = msg
                    
                    
        return JointReciever
    
    def _JointTransmitter(self):
        parent = self
        class JointTransmitter(Codelet):
            def start(self):
                self.tx = self.isaac_proto_tx("CompositeProto", "command")
                self.tick_periodically(0.03)
                
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'set_joint_angles':
                    self.tx._msg = parent._command_queue[0].payload.composite
                    self.tx.publish()
                    command = parent._command_queue.popleft()
                    command.response = True
                
                elif parent._widgets and parent._stream_joints:
                    self.tx._msg = parent.joint_widget.composite
                    self.tx.publish()
                    
        return JointTransmitter
        
    def connect_app(self, app):
        # load dependency subgraphs
        app.load(filename="packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr")
        simulation_interface = app.nodes["simulation.interface"]
        lqr_interface = app.nodes["lqr.subgraph"]["interface"]
        
        # configs
        app.nodes["lqr.kinematic_tree"]["KinematicTree"].config.kinematic_file = self._kinematic_file
        lqr_planner = app.nodes["lqr.local_plan"]["MultiJointLqrPlanner"]
        lqr_planner.config.speed_min = [-self.config['joint_speed']] * len(self.joint_names)
        lqr_planner.config.speed_max = [self.config['joint_speed']] * len(self.joint_names)
        lqr_planner.config.acceleration_min = [-self.config['joint_accel']] * len(self.joint_names)
        lqr_planner.config.acceleration_max = [self.config['joint_accel']] * len(self.joint_names)
        
        # create nodes
        joints_in_node = app.add("joints_input")
        joints_in_node.add(self._JointReciever(), 'joints_reciever')
        
        joints_out_node = app.add("joints_output")
        joints_out_node.add(self._JointTransmitter(), 'joints_transmitter')
                
        # connect edges
        app.connect(simulation_interface["output"], "joint_state", lqr_interface, "joint_state")
        app.connect(simulation_interface["output"], "joint_state", joints_in_node['joints_reciever'], "state")
        
        app.connect(joints_out_node['joints_transmitter'], "command", lqr_interface, "joint_target")
        app.connect(lqr_interface, "joint_command", simulation_interface["input"], "joint_position")
        
        return app