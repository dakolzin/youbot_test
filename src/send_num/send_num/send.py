import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import socket
import json

class TCPBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')
        
        # Подключаемся к серверу (убедитесь, что на 127.0.0.1:5000 уже запущен сервер)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 5000))
        self.get_logger().info('TCP-клиент подключился к 127.0.0.1:5000')

        # Подписка на состояние "руки" (arm)
        self.arm_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/youbot_arm_controller/state',
            self.arm_callback,
            10
        )

        # Подписка на состояние "губок" (gripper)
        self.gripper_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/youbot_gripper_controller/state',
            self.gripper_callback,
            10
        )

    def arm_callback(self, msg: JointTrajectoryControllerState):
        """
        Обрабатываем состояние робо-руки,
        передаём по TCP positions из msg.actual.positions
        """
        positions = list(msg.actual.positions)
        data_dict = {
            'source': 'arm',  # метка, откуда данные
            'joint_names': msg.joint_names,
            'actual_positions': positions
        }
        self.send_data(data_dict)

    def gripper_callback(self, msg: JointTrajectoryControllerState):
        """
        Обрабатываем состояние губок,
        передаём по TCP positions из msg.actual.positions
        """
        positions = list(msg.actual.positions)
        data_dict = {
            'source': 'gripper',  # метка, откуда данные
            'joint_names': msg.joint_names,
            'actual_positions': positions
        }
        self.send_data(data_dict)

    def send_data(self, data_dict):
        """
        Универсальный метод отправки JSON по сокету.
        """
        data_str = json.dumps(data_dict)
        self.sock.sendall(data_str.encode('utf-8'))


def main(args=None):
    rclpy.init(args=args)
    node = TCPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()  # Закрываем TCP-соединение
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
