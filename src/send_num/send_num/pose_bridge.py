#!/usr/bin/env python3
"""
pose_bridge.py – читает позу из камеры, конвертирует в base_link,
шлёт её по TCP (порт 5000) и публикует TF‑фрейм grasp_target.
Фрейм остаётся видимым, пока не придёт новая точка.
"""

import socket
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs  # только для импорта

# ---------- параметры ---------------------------------------------------------
CAM_HOST, CAM_PORT     = "127.0.0.1", 6000
SERVER_IP, SERVER_PORT = "0.0.0.0", 5000
CAM_FRAME  = "color_cam"
BASE_FRAME = "base_link"
CHILD_FRAME = "grasp_target"
TIMEOUT_TCP = 5.0
TF_TIMEOUT  = 0.5
BROADCAST_RATE = 0.1            # 10 Гц
# ------------------------------------------------------------------------------


def read_pose() -> PoseStamped:
    """Получает строку 'x y z qx qy qz qw' и формирует PoseStamped в CAM_FRAME."""
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

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        self.last_tf: TransformStamped | None = None

        # TCP‑сервер
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv_sock.bind((SERVER_IP, SERVER_PORT))
        self.srv_sock.listen()
        self.srv_sock.setblocking(False)
        self.get_logger().info(f"TCP server on {SERVER_IP}:{SERVER_PORT}")

        # таймеры
        self.create_timer(0.05, self.poll_once)          # приём клиентов
        self.create_timer(BROADCAST_RATE, self.pub_last) # переиздание TF

    # --- переиздать хранимый трансформ ----------------------------------------
    def pub_last(self):
        if self.last_tf is None:
            return
        self.last_tf.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.last_tf)

    # --- приём клиента --------------------------------------------------------
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
                    raise RuntimeError("TF transform unavailable")

                base_pose: PoseStamped = self.tf_buffer.transform(
                    cam_pose, BASE_FRAME,
                    timeout=Duration(seconds=TF_TIMEOUT))

                # формируем TransformStamped и сохраняем
                t = TransformStamped()
                t.header.frame_id = BASE_FRAME
                t.child_frame_id = CHILD_FRAME
                t.transform.translation.x = base_pose.pose.position.x
                t.transform.translation.y = base_pose.pose.position.y
                t.transform.translation.z = base_pose.pose.position.z
                t.transform.rotation = base_pose.pose.orientation
                self.last_tf = t                                    # запомнили
                self.pub_last()                                    # сразу отправили

                p = base_pose.pose
                line = (f"{p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                        f"{p.orientation.x:.6f} {p.orientation.y:.6f} "
                        f"{p.orientation.z:.6f} {p.orientation.w:.6f}\n")
                conn.sendall(line.encode())
                self.get_logger().info(f"sent pose & TF: {line.strip()}")

            except Exception as e:
                err = f"# failed: {e}"
                self.get_logger().error(err)
                conn.sendall((err + '\n').encode())


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
