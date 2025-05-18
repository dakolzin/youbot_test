#!/usr/bin/env python3
import math, socket
from typing import Tuple, Optional, List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros, tf2_geometry_msgs               # noqa: F401

from moveit_py import RobotState
from moveit_py.core import RobotModelLoader

# ---------------- параметры ---------------------------------------------------
CAM_HOST, CAM_PORT = "127.0.0.1", 6000
SERVER_IP, SERVER_PORT = "0.0.0.0", 5000

CAM_FRAME, BASE_FRAME = "color_cam", "base_link"
EE_LINK, ARM_GROUP    = "gripper_tcp", "youbot_arm"

CHILD_FRAME_ORIG, CHILD_FRAME_FLIPPED = "grasp_target", "grasp_target_flipped"

TIMEOUT_TCP = 5.0
TF_TIMEOUT  = 0.5
IK_TIMEOUT  = 0.05      # локальный IK – 50 мс
BROADCAST_RATE = 0.1
# -----------------------------------------------------------------------------


def quat_to_mat(q: Tuple[float, float, float, float]) -> list[list[float]]:
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [[1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy)],
            [2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]]


def mat_to_quat(m: list[list[float]]) -> Tuple[float, float, float, float]:
    r00, r01, r02 = m[0]; r10, r11, r12 = m[1]; r20, r21, r22 = m[2]
    trace = r00 + r11 + r22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx, qy, qz = (r21 - r12)/s, (r02 - r20)/s, (r10 - r01)/s
    elif r00 >= r11 and r00 >= r22:
        s = math.sqrt(1+r00-r11-r22) * 2
        qw = (r21 - r12)/s; qx = 0.25*s; qy = (r01+r10)/s; qz = (r02+r20)/s
    elif r11 >= r22:
        s = math.sqrt(1+r11-r00-r22) * 2
        qw = (r02 - r20)/s; qx = (r01+r10)/s; qy = 0.25*s; qz = (r12+r21)/s
    else:
        s = math.sqrt(1+r22-r00-r11) * 2
        qw = (r10 - r01)/s; qx = (r02+r20)/s; qy = (r12+r21)/s; qz = 0.25*s
    norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    return qx/norm, qy/norm, qz/norm, qw/norm


def read_pose() -> PoseStamped:
    with socket.create_connection((CAM_HOST, CAM_PORT), TIMEOUT_TCP) as s:
        data = s.recv(256).decode().strip()
    vals = list(map(float, data.split()))
    if len(vals) != 7:
        raise ValueError(f"bad pose line: {data!r}")
    x, y, z, qx, qy, qz, qw = vals
    ps = PoseStamped()
    ps.header.frame_id = CAM_FRAME
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx, qy, qz, qw
    return ps


class PoseBridge(Node):
    def __init__(self):
        super().__init__("pose_bridge")

        # TF ----------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # --- локальный IK -------------------------------------------
        loader = RobotModelLoader(self)
        self.robot_model = loader.get_model()
        self.group = self.robot_model.get_joint_model_group(ARM_GROUP)
        self.robot_state = RobotState(self.robot_model)
        self.robot_state.set_to_default_values()
        self.get_logger().info(f"Local IK ready ({self.group.get_name()})")

        # TCP-сервер --------------------------------------------------
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv_sock.bind((SERVER_IP, SERVER_PORT))
        self.srv_sock.listen(); self.srv_sock.setblocking(False)
        self.get_logger().info(f"TCP server on {SERVER_IP}:{SERVER_PORT}")

        self.create_timer(0.05, self.poll_once)
        self.last_tf: Optional[TransformStamped] = None
        self.last_tf_flip: Optional[TransformStamped] = None
        self.create_timer(BROADCAST_RATE, self.pub_last)

    # ---------- IK-фильтр ------------------------------------------
    def ik_feasible(self, pose: PoseStamped) -> bool:
        # 10 попыток, общий лимит IK_TIMEOUT
        return self.robot_state.set_from_ik(
            self.group, pose.pose, EE_LINK, attempts=10, timeout=IK_TIMEOUT)

    # ---------- TF периодический репаблиш --------------------------
    def pub_last(self):
        if self.last_tf and self.last_tf_flip:
            now = self.get_clock().now().to_msg()
            self.last_tf.header.stamp = now
            self.last_tf_flip.header.stamp = now
            self.br.sendTransform([self.last_tf, self.last_tf_flip])

    # ---------- приём клиентов ------------------------------------
    def poll_once(self):
        try:
            conn, addr = self.srv_sock.accept()
        except BlockingIOError:
            return
        with conn:
            self.get_logger().info(f"client {addr} connected")
            try:
                cam_pose = read_pose()
                cam_pose.header.stamp = self.get_clock().now().to_msg()

                if not self.tf_buffer.can_transform(
                        BASE_FRAME, CAM_FRAME, rclpy.time.Time(),
                        timeout=Duration(seconds=TF_TIMEOUT)):
                    raise RuntimeError("TF unavailable")

                base_pose: PoseStamped = self.tf_buffer.transform(
                    cam_pose, BASE_FRAME,
                    timeout=Duration(seconds=TF_TIMEOUT))

                if not self.ik_feasible(base_pose):
                    self.get_logger().info("IK unreachable – skipped")
                    conn.sendall(b"# IK_FAILED\n"); return

                # --- TF original ----------------------------------
                t = TransformStamped()
                t.header.frame_id = BASE_FRAME; t.child_frame_id = CHILD_FRAME_ORIG
                t.transform.translation.x = base_pose.pose.position.x
                t.transform.translation.y = base_pose.pose.position.y
                t.transform.translation.z = base_pose.pose.position.z
                t.transform.rotation      = base_pose.pose.orientation

                # --- TF flipped -----------------------------------
                q = base_pose.pose.orientation
                R = quat_to_mat((q.x, q.y, q.z, q.w))
                x_orig, y_orig, z_orig = R[0], R[1], R[2]
                x_new = [-z_orig[0], -z_orig[1], -z_orig[2]]
                q_new = mat_to_quat([[x_new[0], y_orig[0], x_orig[0]],
                                     [x_new[1], y_orig[1], x_orig[1]],
                                     [x_new[2], y_orig[2], x_orig[2]]])

                t_flip = TransformStamped()
                t_flip.header.frame_id = BASE_FRAME; t_flip.child_frame_id = CHILD_FRAME_FLIPPED
                t_flip.transform.translation.x = base_pose.pose.position.x
                t_flip.transform.translation.y = base_pose.pose.position.y
                t_flip.transform.translation.z = base_pose.pose.position.z
                (t_flip.transform.rotation.x,
                 t_flip.transform.rotation.y,
                 t_flip.transform.rotation.z,
                 t_flip.transform.rotation.w) = q_new

                self.last_tf, self.last_tf_flip = t, t_flip
                self.br.sendTransform([t, t_flip])

                p = base_pose.pose
                line = (f"{p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                        f"{p.orientation.x:.6f} {p.orientation.y:.6f} "
                        f"{p.orientation.z:.6f} {p.orientation.w:.6f}\n")
                conn.sendall(line.encode())
                self.get_logger().info("sent pose & both TF frames")

            except Exception as e:
                self.get_logger().error(f"# failed: {e}")
                conn.sendall((f"# failed: {e}\n").encode())


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
