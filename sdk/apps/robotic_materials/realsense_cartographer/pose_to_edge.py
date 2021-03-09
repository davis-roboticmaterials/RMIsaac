from engine.pyalice import Application, Node, Codelet, Message
from engine.pyalice.CapnpMessages import get_capnp_proto_schemata
import math
from pprint import pprint

CAPNP_DICT = get_capnp_proto_schemata()    # proto name to proto schema

def proto(proto_name):
    if proto_name is None:
        return None
    assert proto_name in CAPNP_DICT, 'Could not load specified message type of %r'\
        '. Is it mis-spelling or missing capnp file?' % proto_name

    return CAPNP_DICT[proto_name].new_message()

class PoseToEdge(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("Pose3dProto", "pose")
        self.tx = self.isaac_proto_tx("PoseTreeEdgeProto", "edge")

        self.tick_on_message(self.rx)
    
    def tick(self):
        rx_message = self.rx.message

        pose3 = rx_message.proto
        tx_message = self.tx.init()
        edge = tx_message.proto

        edge.pose = pose3
        edge.lhs = self.config.lhs_frame
        edge.rhs = self.config.rhs_frame

        self.tx.publish()