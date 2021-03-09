from collections import deque  
import numpy as np
import time
import os

import json

from isaac.utilities import *

# set the current working directory to the deployed package folder. This is required by isaac.
os.chdir("/home/davis/deploy/davis/rm_isaac_bridge-pkg")

from packages.pyalice import Codelet
from packages.pyalice.gui.composite_widget import CompositeWidget


class IsaacSimSmarthand:
    def __init__(self, config):
        self.config = config
        self._timeout = config['timeout']
        
        self._widgets = self.config['widgets']
        self._stream_articulations = self.config['stream_articulations']
        
        # Since isaac does not support closed kinematics (4-bar linkage), there are 4 dof, where
        # left_finger == left_finger_upper and right_finger == right_finger_upper
        self.joint_names = self._load_kinematics(config['effector_type'])
        self.joints = CompositeArray(self.joint_names, 'position', [0]*len(self.joint_names))

        if self._widgets:
            self.finger_widget = CompositeWidget(self.joint_names, 'position', [[-np.pi/2, np.pi/2]]*6)
        
        self._command_queue = deque()

    def _load_kinematics(self, effector_type):
        valid_kinematics = ['smarthand']
        if effector_type not in valid_kinematics:
            raise ValueError('No valid kinematic file found for '+effector_type+'. Valid kinematics exist for '+', '.join(valid_kinematics))
            
        self._kinematic_file = "apps/assets/kinematic_trees/{}.kinematic.json".format(effector_type)
        joint_names = []
        with open(self._kinematic_file,'r') as fd:
            kt = json.load(fd)
            for link in kt['links']:
                if 'motor' in link and link['motor']['type'] != 'constant':
                    joint_names.append(link['name'])
                    
        return joint_names
        
    def command(self, action, payload=None):
        if action not in ['get_articulation_angles', 'set_articulation_angles']:
            raise ValueError(action+' is not a valid action type')
        
        if type(payload) in [list, np.ndarray]:
            if len(list(payload)) == 2:
                payload = [payload[0], payload[0], payload[0], payload[1], payload[1], payload[1]]
            payload = CompositeArray(self.joint_names, 'position', payload)
            
        command = Command(action, payload)
        self._command_queue.append(command)
        
        elapsed = 0
        while command.response is None and elapsed < self._timeout:
            elapsed += 0.01
            time.sleep(0.01)
            
        return command.response
        
    def enable_articulation_stream(self):
        self._stream_articulations = True
    def disable_articulation_stream(self):
        self._stream_articulations = False
    
    def enable_all_streams(self):
        self._stream_articulations = True
    def disable_all_streams(self):
        self._stream_articulations = False
        
    def _ArticulationReciever(self):
        parent = self
        class JointReciever(Codelet):
            def start(self):
                self.rx = self.isaac_proto_rx("CompositeProto", "state")
                self.tick_on_message(self.rx)
            
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'get_articulation_angles':
                    msg = self.rx.message
                    parent.joints.composite = msg
                    command = parent._command_queue.popleft()
                    values = parent.joints.values
                    command.response = values
                    
                elif parent._stream_articulations:
                    msg = self.rx.message
                    parent.joints.composite = msg
                    
                else:
                    return
                
                if parent._widgets:
                    parent.finger_widget.composite = msg
                    
                    
        return JointReciever
    
    def _ArticulationTransmitter(self):
        parent = self
        class JointTransmitter(Codelet):
            def start(self):
                self.tx = self.isaac_proto_tx("CompositeProto", "command")
                self.tick_periodically(0.03)
                
            def tick(self):
                if len(parent._command_queue) > 0 and parent._command_queue[0].action == 'set_articulation_angles':
                    self.tx._msg = parent._command_queue[0].payload.composite
                    self.tx.publish()
                    command = parent._command_queue.popleft()
                    command.response = True
                
                elif parent._widgets and parent._stream_articulations:
                    self.tx._msg = parent.finger_widget.composite
                    self.tx.publish()
                    
        return JointTransmitter
        
    def connect_app(self, app):
        # load dependency subgraphs
        app.load(filename="packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr_gripper")
        simulation_interface = app.nodes["simulation.interface"]
        lqr_interface = app.nodes["lqr_gripper.subgraph"]["interface"]
        
        # configs
        app.nodes["lqr_gripper.kinematic_tree"]["KinematicTree"].config.kinematic_file = self._kinematic_file
        lqr_planner = app.nodes["lqr_gripper.local_plan"]["MultiJointLqrPlanner"]
        lqr_planner.config.speed_min = [-self.config['joint_speed']] * len(self.joint_names)
        lqr_planner.config.speed_max = [self.config['joint_speed']] * len(self.joint_names)
        lqr_planner.config.acceleration_min = [-self.config['joint_accel']] * len(self.joint_names)
        lqr_planner.config.acceleration_max = [self.config['joint_accel']] * len(self.joint_names)
        
        # create nodes
        joints_in_node = app.add("effector_articulation_input")
        joints_in_node.add(self._ArticulationReciever(), 'articulation_reciever')
        
        joints_out_node = app.add("effector_articulation_output")
        joints_out_node.add(self._ArticulationTransmitter(), 'articulation_transmitter')
                
        # connect edges
        app.connect(simulation_interface["output"], "effector_articulation_states", lqr_interface, "joint_state")
        app.connect(simulation_interface["output"], "effector_articulation_states", joints_in_node['articulation_reciever'], "state")
        
        app.connect(joints_out_node['articulation_transmitter'], "command", lqr_interface, "joint_target")
        app.connect(lqr_interface, "joint_command", simulation_interface["input"], "effector_articulation_positions")
        
        return app