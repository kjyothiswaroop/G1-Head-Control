import json
import math
import numpy as np
import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ZMQ_HOST  = "localhost"
ZMQ_PORT  = 5556
ZMQ_TOPIC = "pose"

# vr_orientation is float[12]: [left_wxyz, right_wxyz, head_wxyz]
HEAD_QUAT_OFFSET = 8

DTYPE_MAP = {
    "f32": np.float32,
    "f64": np.float64,
    "i32": np.int32,
    "i64": np.int64,
    "bool": np.bool_,
}


def unpack_pose_message(raw: bytes, topic: str) -> dict:
    payload = raw[len(topic):]
    header = json.loads(payload[:1024].rstrip(b"\x00"))
    binary = payload[1024:]

    arrays = {}
    offset = 0
    for field in header["fields"]:
        dtype = DTYPE_MAP[field["dtype"]]
        shape = field["shape"]
        nbytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        arrays[field["name"]] = np.frombuffer(
            binary[offset : offset + nbytes], dtype=dtype
        ).reshape(shape)
        offset += nbytes
    return arrays


class HeadPicoBridge(Node):
    def __init__(self):
        super().__init__("head_pico_bridge")

        self._pub = self.create_publisher(JointState, "head/target", 10)

        ctx = zmq.Context()
        self._sock = ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://{ZMQ_HOST}:{ZMQ_PORT}")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, ZMQ_TOPIC)

        self.create_timer(0.02, self._poll)  # 50 Hz
        self.get_logger().info(f"Head PICO bridge started (tcp://{ZMQ_HOST}:{ZMQ_PORT})")

    def _poll(self):
        try:
            raw = self._sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        try:
            arrays = unpack_pose_message(raw, ZMQ_TOPIC)
        except Exception as e:
            self.get_logger().warn(f"Failed to unpack message: {e}")
            return

        ori = arrays.get("vr_orientation")
        if ori is None or ori.size < 12:
            self.get_logger().warn("vr_orientation missing or too short")
            return

        ori = ori.flatten()
        w = float(ori[HEAD_QUAT_OFFSET])
        x = float(ori[HEAD_QUAT_OFFSET + 1])
        y = float(ori[HEAD_QUAT_OFFSET + 2])
        z = float(ori[HEAD_QUAT_OFFSET + 3])

        pitch, yaw = quat_to_pitch_yaw(w, x, y, z)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ["pitch", "yaw"]
        msg.position = [pitch, yaw]
        self._pub.publish(msg)


def quat_to_pitch_yaw(w, x, y, z):
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw   = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = HeadPicoBridge()
    rclpy.spin(node)
    rclpy.shutdown()
