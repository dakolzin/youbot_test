#!/usr/bin/env python3
"""
ik_pose_checker.py — Проверка IK для youBot с расширенной диагностикой.
- Получает позу по TCP с порта 6000 (pose_server)
- Переводит в base_link через TF2
- Передаёт в IK-сервис MoveIt2 с seed по рабочим суставам
- Логирует все ключевые этапы и результат IK (error_code, решение)
"""

import socket
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
import tf2_ros
import tf2_geometry_msgs  # НЕ УДАЛЯТЬ!

CAM_HOST, CAM_PORT = "127.0.0.1", 6000
CAM_FRAME  = "color_cam"
BASE_FRAME = "base_link"
TF_TIMEOUT = 0.5
TIMEOUT_TCP = 5.0

# Только рабочие суставы!
ARM_JOINT_NAMES = [
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5"
]

def read_pose() -> PoseStamped:
    """Чтение строки 'x y z qx qy qz qw' -> PoseStamped в CAM_FRAME."""
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

def filter_joint_state(src, arm_joint_names):
    name = []
    position = []
    velocity = []
    effort = []
    for i, n in enumerate(src.name):
        if n in arm_joint_names:
            name.append(n)
            position.append(src.position[i])
            if src.velocity:
                velocity.append(src.velocity[i])
            if src.effort:
                effort.append(src.effort[i])
    msg = JointState()
    msg.header = src.header
    msg.name = name
    msg.position = position
    if velocity: msg.velocity = velocity
    if effort: msg.effort = effort
    return msg

class IKPoseChecker(Node):
    def __init__(self):
        super().__init__("ik_pose_checker")

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # IK сервис MoveIt2
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /compute_ik service...")

        # Храним последние joint_states
        self.joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.pose_counter = 0
        self.timer = self.create_timer(0.5, self.check_next_pose)

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def check_next_pose(self):
        try:
            if self.joint_state is None:
                self.get_logger().warn("Ждём /joint_states ...")
                return

            cam_pose = read_pose()
            self.get_logger().info(
                f"[{self.pose_counter}] Получена точка (CAM): [{cam_pose.pose.position.x:.3f} {cam_pose.pose.position.y:.3f} {cam_pose.pose.position.z:.3f}]"
            )

            # Преобразовать в base_link
            try:
                base_pose: PoseStamped = self.tf_buffer.transform(
                    cam_pose, BASE_FRAME, timeout=Duration(seconds=TF_TIMEOUT)
                )
            except Exception as e:
                self.get_logger().error(f"TF transform error: {e}")
                self.pose_counter += 1
                return

            p = base_pose.pose
            self.get_logger().info(
                f"[{self.pose_counter}] Преобразовано в {BASE_FRAME}: [{p.position.x:.3f} {p.position.y:.3f} {p.position.z:.3f}]"
            )

            # Фильтруем seed по рабочим суставам
            filtered_seed = filter_joint_state(self.joint_state, ARM_JOINT_NAMES)
            self.get_logger().info(
                f"Seed joint_state: names={filtered_seed.name}, positions={filtered_seed.position}"
            )

            # Передать в IK-решатель
            req = GetPositionIK.Request()
            req.ik_request.group_name = "youbot_arm"
            req.ik_request.pose_stamped = base_pose
            req.ik_request.timeout = Duration(seconds=2.0).to_msg()
            req.ik_request.robot_state.joint_state = filtered_seed
            self.get_logger().info(f"[{self.pose_counter}] Преобразованная точка передана в решатель (IK с seed)")

            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            res = future.result()

            # Диагностика результата IK
            if not res or res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().warn(f"[{self.pose_counter}] IK НЕ РЕШИЛ, пропуск точки")
                if res:
                    self.get_logger().warn(f"IK error_code: {res.error_code.val}, {res.error_code}")
                    self.get_logger().warn(f"IK response: {res}")
            else:
                self.get_logger().info(f"[{self.pose_counter}] IK решил, точка выполнима")
                js = res.solution.joint_state
                self.get_logger().info(
                    f"Ответ IK: names={js.name}, positions={js.position}"
                )
            self.pose_counter += 1
        except Exception as e:
            self.get_logger().error(f"failed: {e}")
            self.pose_counter += 1

def main():
    rclpy.init()
    node = IKPoseChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
