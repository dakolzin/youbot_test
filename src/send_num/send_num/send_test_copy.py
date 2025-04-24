#!/usr/bin/env python3
# coding: utf-8
"""
ROS2-нода: подписывается на /joint_states и на
/youbot_gripper_controller/state, дублирует arm+gripper по TCP.

IP робота и порт задаются через флаги:
  --ip    (default: 192.168.1.12)
  --port  (default: 5000)

Пример запуска:
ros2 run send_num send_test_copy -- --ip 127.0.0.1 --port 5000
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import socket, json, threading, argparse

# разбор аргументов
parser = argparse.ArgumentParser(description='TCP-клиент для youBot-TCP-сервера')
parser.add_argument('--ip',    default='192.168.1.12',
                    help='IP-адрес youBot-сервера (default: %(default)s)')
parser.add_argument('--port',  type=int, default=5000,
                    help='TCP-порт сервера (default: %(default)s)')
args = parser.parse_args()

YOUBOT_IP = args.ip
PORT      = args.port

ARM_JOINTS = [
    "arm_joint_1","arm_joint_2","arm_joint_3",
    "arm_joint_4","arm_joint_5"
]


class JointStateTcpClient(Node):
    def __init__(self):
        super().__init__("joint_state_tcp_client")
        self._lock = threading.Lock()
        self._sock = None
        self._connect()

        # arm – из /joint_states
        self.create_subscription(JointState,
                                 "/joint_states",
                                 self._on_joint_state,
                                 10)
        # gripper – из контроллера
        self.create_subscription(JointTrajectoryControllerState,
                                 "/youbot_gripper_controller/state",
                                 self._on_gripper_state,
                                 10)

    def _connect(self):
        if self._sock:
            try: self._sock.close()
            except: pass
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._sock.connect((YOUBOT_IP, PORT))
            self._sock.settimeout(0.1)
            self.get_logger().info(f"Connected to {YOUBOT_IP}:{PORT}")
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            self._sock = None

    def _send(self, pkt: dict):
        data = (json.dumps(pkt) + "\n").encode()
        with self._lock:
            if not self._sock:
                self._connect()
            if not self._sock:
                return
            try:
                self._sock.sendall(data)
            except Exception as e:
                self.get_logger().error(f"Send error: {e}")
                self._connect()

    def _on_joint_state(self, msg: JointState):
        arm_names, arm_pos = [], []
        for n, p in zip(msg.name, msg.position):
            if n in ARM_JOINTS:
                arm_names.append(n)
                arm_pos.append(p)
        if arm_names:
            pkt = {
                "source": "arm",
                "joint_names": arm_names,
                "positions": arm_pos
            }
            self._send(pkt)

    def _on_gripper_state(self, msg: JointTrajectoryControllerState):
        pkt = {
            "source": "gripper",
            "joint_names": msg.joint_names,
            "positions": list(msg.actual.positions)
        }
        self._send(pkt)


def main():
    rclpy.init()
    node = JointStateTcpClient()
    try:
        rclpy.spin(node)
    finally:
        if node._sock:
            try: node._sock.close()
            except: pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
