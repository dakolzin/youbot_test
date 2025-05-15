#!/usr/bin/env python3
"""
pose_bridge.py – копит позы от камеры, при каждом TCP-подключении отдаёт
следующую позу и публикует TF grasp_target.
"""

import socket, threading, queue, time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs  # импорт только ради зависимостей

# ---------- параметры ---------------------------------------------------------
CAM_HOST, CAM_PORT     = "127.0.0.1", 6000          # камера-сервер
SERVER_IP, SERVER_PORT = "0.0.0.0", 5000            # этот сервер
CAM_FRAME  = "color_cam"
BASE_FRAME = "base_link"
CHILD_FRAME = "grasp_target"
TF_TIMEOUT     = 0.5
BROADCAST_RATE = 0.1                               # 10 Гц
QUEUE_SIZE     = 100                               # max точек в буфере
# ------------------------------------------------------------------------------


def read_pose_blocking() -> PoseStamped:
    """Блокирующе читает одну строку 'x y z qx qy qz qw' от камеры."""
    with socket.create_connection((CAM_HOST, CAM_PORT)) as s:
        data = b''
        while not data.endswith(b'\n'):
            chunk = s.recv(256)
            if not chunk:
                raise RuntimeError("Камера закрыла соединение")
            data += chunk
    vals = list(map(float, data.decode().strip().split()))
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
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br          = tf2_ros.TransformBroadcaster(self)

        # очередь поз
        self.pose_q: queue.Queue[PoseStamped] = queue.Queue(maxsize=QUEUE_SIZE)

        # TCP-сервер
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv_sock.bind((SERVER_IP, SERVER_PORT))
        self.srv_sock.listen()
        self.srv_sock.setblocking(False)
        self.get_logger().info(f"TCP server on {SERVER_IP}:{SERVER_PORT}")

        # таймеры ROS-событий
        self.create_timer(0.05, self._poll_accept)   # приём клиентов
        self.create_timer(BROADCAST_RATE, self._pub_last)  # переиздание TF

        self.last_tf: TransformStamped | None = None

        # фоновый поток чтения камеры
        threading.Thread(target=self._camera_loop, daemon=True).start()

    # ---------- цикл чтения камеры -------------------------------------------
    def _camera_loop(self):
        while rclpy.ok():
            try:
                ps = read_pose_blocking()
                self.pose_q.put(ps, block=True)  # блокируемся, если очередь полна
            except Exception as e:
                self.get_logger().error(f"camera read error: {e}")
                time.sleep(0.5)

    # ---------- публикация TF -------------------------------------------------
    def _pub_last(self):
        if not self.last_tf:
            return
        self.last_tf.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.last_tf)

    # ---------- приём клиентов ------------------------------------------------
    def _poll_accept(self):
        try:
            conn, addr = self.srv_sock.accept()
        except BlockingIOError:
            return
        threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True).start()

    def _handle_client(self, conn: socket.socket, addr):
        with conn:
            self.get_logger().info(f"client {addr} connected")
            try:
                # берём следующую позу (блокируется, пока не появится)
                cam_pose: PoseStamped = self.pose_q.get(block=True)
                cam_pose.header.stamp = self.get_clock().now().to_msg()

                # трансформируем в base_link
                if not self.tf_buffer.can_transform(
                        BASE_FRAME, CAM_FRAME, rclpy.time.Time(),
                        timeout=Duration(seconds=TF_TIMEOUT)):
                    raise RuntimeError("TF transform unavailable")

                base_pose: PoseStamped = self.tf_buffer.transform(
                    cam_pose, BASE_FRAME,
                    timeout=Duration(seconds=TF_TIMEOUT))

                # формируем и сохраняем TF
                t = TransformStamped()
                t.header.frame_id  = BASE_FRAME
                t.child_frame_id   = CHILD_FRAME
                t.transform.translation.x = base_pose.pose.position.x
                t.transform.translation.y = base_pose.pose.position.y
                t.transform.translation.z = base_pose.pose.position.z
                t.transform.rotation      = base_pose.pose.orientation
                self.last_tf = t
                self._pub_last()   # сразу опубликовать

                # строка для клиента
                p = base_pose.pose
                line = (f"{p.position.x:.6f} {p.position.y:.6f} {p.position.z:.6f} "
                        f"{p.orientation.x:.6f} {p.orientation.y:.6f} "
                        f"{p.orientation.z:.6f} {p.orientation.w:.6f}\n")
                conn.sendall(line.encode())
                self.get_logger().info(f"sent pose: {line.strip()} "
                                       f"(queue size: {self.pose_q.qsize()})")

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
