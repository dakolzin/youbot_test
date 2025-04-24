#!/usr/bin/env python3
# coding: utf-8
"""
Отправляет две команды:
  1) Манипулятор (arm)
  2) Губки (gripper)
Каждый JSON оканчивается \n
"""
import socket, json, time

YOUBOT_IP = "192.168.1.12"   # подставьте IP youBot
PORT      = 5000

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((YOUBOT_IP, PORT))

    arm = {
        "source": "arm",
        "joint_names": [
            "arm_joint_1", "arm_joint_2", "arm_joint_3",
            "arm_joint_4", "arm_joint_5"
        ],
        "positions": [0.0, 0.0, -1.2158, 0.0, 0.0]
    }
    gripper = {
        "source": "gripper",
        "joint_names": [
            "gripper_finger_joint_l", "gripper_finger_joint_r"
        ],
        "positions": [0.01, 0.01]   # 15 мм
    }

    s.sendall((json.dumps(arm) + "\n").encode())
    time.sleep(0.05)
    s.sendall((json.dumps(gripper) + "\n").encode())
    s.close()

if __name__ == "__main__":
    main()
