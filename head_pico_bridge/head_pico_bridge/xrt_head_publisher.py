import json
import time
import numpy as np
import zmq
from scipy.spatial.transform import Rotation as sRot

try:
    import xrobotoolkit_sdk as xrt
except ImportError as e:
    raise ImportError("xrobotoolkit_sdk not found — ensure XRobotToolkit is installed") from e

ZMQ_PORT = 5556
TOPIC    = "pose"
RATE_HZ  = 50

NECK_IDX = 12
ROOT_IDX = 0

# Matches OFFSETS[0] and OFFSETS[3] from pico_manager_thread_server.py
ROOT_OFFSET = sRot.from_euler("xyz", [0, 0, -90], degrees=True)
NECK_OFFSET = sRot.from_euler("xyz", [0, 0, -90], degrees=True)


def _unity_to_robot(qxyzw: np.ndarray, offset: sRot) -> np.ndarray:
    return (sRot.from_quat(qxyzw, scalar_first=False) * offset).as_quat(scalar_first=True)


def _neck_quat_relative_to_root(body_poses: np.ndarray) -> np.ndarray:
    root_q = _unity_to_robot(body_poses[ROOT_IDX, 3:], ROOT_OFFSET)
    neck_q = _unity_to_robot(body_poses[NECK_IDX, 3:], NECK_OFFSET)
    rel = sRot.from_quat(root_q, scalar_first=True).inv() * sRot.from_quat(neck_q, scalar_first=True)
    return rel.as_quat(scalar_first=True).astype(np.float32)


def _apply_calibration(q: np.ndarray, ref: sRot) -> np.ndarray:
    return (ref.inv() * sRot.from_quat(q, scalar_first=True)).as_quat(scalar_first=True).astype(np.float32)


def _build_message(vr_orientation: np.ndarray) -> bytes:
    fields = [{"name": "vr_orientation", "dtype": "f32", "shape": [12]}]
    header = json.dumps({"fields": fields, "version": 3, "count": 1}).encode().ljust(1024, b"\x00")
    return TOPIC.encode() + header + vr_orientation.tobytes()


def main():
    print("Initializing XRT SDK...")
    xrt.init()

    print("Waiting for body tracking data...")
    while not xrt.is_body_data_available():
        time.sleep(0.05)
    print("Body tracking ready. Press A to calibrate zero pose.")

    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{ZMQ_PORT}")
    print(f"Publishing on tcp://*:{ZMQ_PORT} (topic: {TOPIC!r})")

    calibration_ref: sRot | None = None
    prev_a = False
    dt = 1.0 / RATE_HZ

    while True:
        t0 = time.monotonic()

        body_poses = np.array(xrt.get_body_joints_pose())  # (24, 7)
        head_q = _neck_quat_relative_to_root(body_poses)   # [qw, qx, qy, qz]

        # Rising edge on A button → set calibration reference
        a_pressed = bool(xrt.get_A_button())
        if a_pressed and not prev_a:
            calibration_ref = sRot.from_quat(head_q, scalar_first=True)
            print("Calibrated: current pose is now zero.")
        prev_a = a_pressed

        if calibration_ref is None:
            print("Waiting for calibration — press A to set zero pose.", flush=True)
            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
            continue

        head_q = _apply_calibration(head_q, calibration_ref)

        vr_orientation = np.zeros(12, dtype=np.float32)
        vr_orientation[0] = 1.0   # left identity
        vr_orientation[4] = 1.0   # right identity
        vr_orientation[8:12] = head_q

        sock.send(_build_message(vr_orientation))

        elapsed = time.monotonic() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)


if __name__ == "__main__":
    main()
