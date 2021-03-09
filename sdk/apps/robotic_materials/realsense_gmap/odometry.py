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


# ===== Quaterion to Euler angle conversions ===== #
def quaternionToEulerX(q):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)

    return math.atan2(sinr_cosp, cosr_cosp)

def quaternionToEulerY(q):
    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        return math.copysign(math.pi / 2, sinp); # use 90 degrees if out of range
    else:
        return math.asin(sinp)

def quaternionToEulerZ(q):
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


# ===== Proto Builders ===== #
def pose_to_proto(poseProto, x, y, phi):
    # converts (x,y) translation and rotation angle to Pose2dProto

    poseProto.translation = proto("Vector2dProto")
    poseProto.translation.x = x
    poseProto.translation.y = y
    poseProto.rotation = proto("SO2dProto")
    poseProto.rotation.q = proto("Vector2dProto")
    poseProto.rotation.q.x = math.cos(phi)
    poseProto.rotation.q.y = math.sin(phi)

    return poseProto

def odometry_to_proto(odometryProto, pose2dProto, omega, speedX, speedY, accelX, accelY):
    # Assigns odometry values to a proto

    speed = proto("Vector2dProto")
    speed.x = speedX
    speed.y = speedY

    acceleration = proto("Vector2dProto")
    acceleration.x = accelX
    acceleration.y = accelY

    odometryProto.odomTRobot = pose2dProto
    odometryProto.speed = speed
    odometryProto.angularSpeed = omega
    odometryProto.acceleration = acceleration
    odometryProto.odometryFrame = "realsense"
    odometryProto.robotFrame = "robot"

    return odometryProto


class Odometry(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("Pose3dProto", "pose")
        self.tx = self.isaac_proto_tx("Odometry2Proto", "odometry")

        # Save old odometry and aquired time to derive speed and acceleration
        initialPoseProto = pose_to_proto(proto("Pose2dProto"), 0, 0, 0)
        self.previous_odometry = odometry_to_proto(proto("Odometry2Proto"), initialPoseProto, 0, 0, 0, 0, 0)
        self.previous_acqtime = None

        self.tick_on_message(self.rx)
    
    def tick(self):
        rx_message = self.rx.message

        acqtime = rx_message.acqtime/1000000000 # convert to seconds
        if self.previous_acqtime is None:
            self.previous_acqtime = acqtime
            return
        elif acqtime == self.previous_acqtime:
            # Repeat message, do nothing
            return
        temporal_difference = acqtime - self.previous_acqtime
        
        # ----- Extract current camera pose from message ----- #
        pose3dProto = rx_message.proto

        # ----- Pose ----- #
        phi = quaternionToEulerZ(pose3dProto.rotation.q)
        pose2dProto = pose_to_proto(proto("Pose2dProto"), pose3dProto.translation.x, pose3dProto.translation.y, phi)
        
        # ----- Speed ----- #
        speedX = (pose3dProto.translation.x - self.previous_odometry.odomTRobot.translation.x)/temporal_difference
        speedY = (pose3dProto.translation.y - self.previous_odometry.odomTRobot.translation.y)/temporal_difference

        # angular speed
        previous_phi = math.atan2(self.previous_odometry.odomTRobot.rotation.q.y, self.previous_odometry.odomTRobot.rotation.q.x)
        omega = (phi - previous_phi)/temporal_difference

        # ----- Acceleration ----- #
        accelX = (speedX - self.previous_odometry.speed.x)/temporal_difference
        accelY = (speedY - self.previous_odometry.speed.y)/temporal_difference


         # ----- Build and publish message ----- #
        tx_message = self.tx.init()
        odometry = odometry_to_proto(tx_message.proto, pose2dProto, omega, speedX, speedY, accelX, accelY)
        self.tx.publish()

        self.previous_odometry = odometry
        self.previous_acqtime = acqtime

        # print(temporal_difference)
        # pprint(odometry.to_dict())

        