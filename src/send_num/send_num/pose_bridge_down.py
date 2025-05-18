#!/usr/bin/env python3
"""
pose_bridge.py
--------------
Получает grasp-позу из камеры, переводит её в base_link и публикует три TF-фрейма:

  • grasp_target            – ориентация без изменений
  • grasp_target_flipped    – X_new = –Z_orig, Y_new =  Y_orig, Z_new =  X_orig
  • grasp_target_down       – фикс. ориентация: локальная Z смотрит вниз

TCP-клиенту (порт 5000) возвращается поза grasp_target_down
в виде строки 'x y z qx qy qz qw'.
"""

import math
import socket
from typing import Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs  # лишь чтобы подтянуть зависимости

# ---------- параметры ---------------------------------------------------------
CAM_HOST, CAM_PORT     = "127.0.0.1", 6000   # сервер SBG-скрипта
SERVER_IP, SERVER_PORT = "0.0.0.0", 5000     # этот TCP-сервер
CAM_FRAME  = "color_cam"
BASE_FRAME = "base_link"
CHILD_FRAME_ORIG    = "grasp_target"
CHILD_FRAME_FLIPPED = "grasp_target_flipped"
CHILD_FRAME_DOWN    = "grasp_target_down"
TIMEOUT_TCP = 5.0
TF_TIMEOUT  = 0.5
BROADCAST_RATE = 0.1           # 10 Гц
# ------------------------------------------------------------------------------


# --- матричные utils ----------------------------------------------------------
def quat_to_mat(q: Tuple[float, float, float, float]) -> list[list[float]]:
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)],
    ]


def mat_to_quat(m: list[list[float]]) -> Tuple[float, float, float, float]:
    r00, r01, r02 = m[0]
    r10, r11, r12 = m[1]
    r20, r21, r22 = m[2]
    trace = r00 + r11 + r22
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 >= r11 and r00 >= r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 >= r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/norm, qy/norm, qz/norm, qw/norm)
# ------------------------------------------------------------------------------


def read_pose() -> PoseStamped:
    """Читает строку 'x y z qx qy qz qw' от SBG-скрипта и формирует PoseStamped."""
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


# ---------- фикс. кватернион: Z инструмента смотрит вниз ----------------------
# Поворот на 180° вокруг оси X переводит локальную Z в −Z base_link.
Q_Z_DOWN = (1.0, 0.0, 0.0, 0.0)        # (qx, qy, qz, qw)
# ------------------------------------------------------------------------------


class PoseBridge(Node):
    def __init__(self):
        super().__init__("pose_bridge")

        # TF -инфраструктура
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br          = tf2_ros.TransformBroadcaster(self)

        # TCP-сервер
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv_sock.bind((SERVER_IP, SERVER_PORT))
        self.srv_sock.listen()
        self.srv_sock.setblocking(False)
        self.get_logger().info(f"TCP server on {SERVER_IP}:{SERVER_PORT}")

        # таймеры
        self.create_timer(0.05, self.poll_once)  # проверка клиентов
        self.create_timer(BROADCAST_RATE, self.pub_last)

        # кеш последних TF для периодического паблиша
        self.last_tf: TransformStamped | None = None
        self.last_tf_flip: TransformStamped | None = None
        self.last_tf_down: TransformStamped | None = None

    # ---------- периодический репаблиш ---------------------------------------
    def pub_last(self):
        if not (self.last_tf and self.last_tf_flip and self.last_tf_down):
            return
        now = self.get_clock().now().to_msg()
        self.last_tf.header.stamp      = now
        self.last_tf_flip.header.stamp = now
        self.last_tf_down.header.stamp = now
        self.br.sendTransform([self.last_tf,
                               self.last_tf_flip,
                               self.last_tf_down])

    # ---------- обработка входящего TCP-клиента -------------------------------
    def poll_once(self):
        try:
            conn, addr = self.srv_sock.accept()
        except BlockingIOError:
            return

        with conn:
            self.get_logger().info(f"client {addr} connected")
            try:
                # 1) читаем позу из камеры
                cam_pose = read_pose()
                cam_pose.header.stamp = self.get_clock().now().to_msg()

                # 2) переводим в base_link
                if not self.tf_buffer.can_transform(
                        BASE_FRAME, CAM_FRAME, rclpy.time.Time(),
                        timeout=Duration(seconds=TF_TIMEOUT)):
                    raise RuntimeError("TF transform unavailable")

                base_pose: PoseStamped = self.tf_buffer.transform(
                    cam_pose, BASE_FRAME,
                    timeout=Duration(seconds=TF_TIMEOUT))

                # --- 2.1 исходный TF ------------------------------------------
                t_orig = TransformStamped()
                t_orig.header.frame_id  = BASE_FRAME
                t_orig.child_frame_id   = CHILD_FRAME_ORIG
                t_orig.transform.translation.x = base_pose.pose.position.x
                t_orig.transform.translation.y = base_pose.pose.position.y
                t_orig.transform.translation.z = base_pose.pose.position.z
                t_orig.transform.rotation      = base_pose.pose.orientation

                # --- 2.2 flipped TF ------------------------------------------
                q_orig = (
                    base_pose.pose.orientation.x,
                    base_pose.pose.orientation.y,
                    base_pose.pose.orientation.z,
                    base_pose.pose.orientation.w,
                )
                R_orig = quat_to_mat(q_orig)

                # столбцы ориентационной матрицы = оси X,Y,Z исходного фрейма
                x_o = [R_orig[0][0], R_orig[1][0], R_orig[2][0]]
                y_o = [R_orig[0][1], R_orig[1][1], R_orig[2][1]]
                z_o = [R_orig[0][2], R_orig[1][2], R_orig[2][2]]

                # новое: X = –Z_orig, Y = Y_orig, Z = X_orig
                R_new = [
                    [-z_o[0],  y_o[0],  x_o[0]],
                    [-z_o[1],  y_o[1],  x_o[1]],
                    [-z_o[2],  y_o[2],  x_o[2]],
                ]
                q_flip = mat_to_quat(R_new)

                t_flip = TransformStamped()
                t_flip.header.frame_id  = BASE_FRAME
                t_flip.child_frame_id   = CHILD_FRAME_FLIPPED
                t_flip.transform.translation.x = base_pose.pose.position.x
                t_flip.transform.translation.y = base_pose.pose.position.y
                t_flip.transform.translation.z = base_pose.pose.position.z
                (t_flip.transform.rotation.x,
                 t_flip.transform.rotation.y,
                 t_flip.transform.rotation.z,
                 t_flip.transform.rotation.w) = q_flip

                # --- 2.3 down-ориентация (Z вниз) ----------------------------
                t_down = TransformStamped()
                t_down.header.frame_id  = BASE_FRAME
                t_down.child_frame_id   = CHILD_FRAME_DOWN
                t_down.transform.translation.x = base_pose.pose.position.x
                t_down.transform.translation.y = base_pose.pose.position.y
                t_down.transform.translation.z = base_pose.pose.position.z
                (t_down.transform.rotation.x,
                 t_down.transform.rotation.y,
                 t_down.transform.rotation.z,
                 t_down.transform.rotation.w) = Q_Z_DOWN

                # 3) сохраняем и публикуем все три TF сразу
                self.last_tf      = t_orig
                self.last_tf_flip = t_flip
                self.last_tf_down = t_down
                self.br.sendTransform([t_orig, t_flip, t_down])

                # 4) формируем строку для клиента: используем down-позу
                line = (f"{base_pose.pose.position.x:.6f} "
                        f"{base_pose.pose.position.y:.6f} "
                        f"{base_pose.pose.position.z:.6f} "
                        f"{Q_Z_DOWN[0]:.6f} {Q_Z_DOWN[1]:.6f} "
                        f"{Q_Z_DOWN[2]:.6f} {Q_Z_DOWN[3]:.6f}\n")
                conn.sendall(line.encode())
                self.get_logger().info("sent pose_down & all TF frames")

            except Exception as e:
                err = f"# failed: {e}"
                self.get_logger().error(err)
                try:
                    conn.sendall((err + '\n').encode())
                except Exception:
                    pass


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
