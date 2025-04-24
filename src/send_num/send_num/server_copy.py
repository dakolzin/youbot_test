#!/usr/bin/env python
# coding: utf-8
"""
Принимает JSON-поток от ROS 2-сервера, публикует команды youBot
в формате brics_actuator/JointPositions.
"""

import rospy
import socket
import json

from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg   import JointState

SERVER_IP = "192.168.1.20"     # IP компьютера с ROS 2
PORT      = 5000

# ─── helpers ───────────────────────────────────────────────────────────────────
def make_joint_positions(names, values, unit):
    msg = JointPositions()
    now = rospy.Time.now()
    for n, v in zip(names, values):
        jv = JointValue()
        jv.timeStamp = now
        jv.joint_uri = n
        jv.unit      = unit
        jv.value     = v
        msg.positions.append(jv)
    return msg

# ─── main ──────────────────────────────────────────────────────────────────────
def main():
    rospy.init_node("tcp_state_to_youbot")

    pub_arm_cmd     = rospy.Publisher("/arm_1/arm_controller/position_command",
                                      JointPositions, queue_size=3)
    pub_gripper_cmd = rospy.Publisher("/arm_1/gripper_controller/position_command",
                                      JointPositions, queue_size=3)
    pub_js          = rospy.Publisher("/arm_1/joint_states", JointState, queue_size=3)

    s = None
    buf = b""

    rate = rospy.Rate(200)     # читаем часто, но публикацию можно задросселировать
    while not rospy.is_shutdown():
        try:
            # ── (re)connect ──
            if s is None:
                rospy.loginfo("Подключаюсь к %s:%d …", SERVER_IP, PORT)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((SERVER_IP, PORT))
                s.settimeout(0.01)
                buf = b""

            # ── read ──
            try:
                chunk = s.recv(4096)
                if not chunk:
                    raise RuntimeError("Соединение закрыто сервером")
                buf += chunk
            except socket.timeout:
                pass                                # нет новых данных – ок

            # ── split by '\n' ──
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                pkt = json.loads(line.decode())
                names = pkt["joint_names"]
                pos   = pkt["positions"]
                src   = pkt["source"]

                if src == "arm":
                    pub_arm_cmd.publish(
                        make_joint_positions(names, pos, "rad")
                    )
                elif src == "gripper":
                    pub_gripper_cmd.publish(
                        make_joint_positions(names, pos, "m")
                    )

                # зеркалим в joint_states для rqt_plot
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name, js.position = names, pos
                pub_js.publish(js)

        except Exception as e:
            rospy.logerr("Ошибка: %s. Пытаюсь переподключиться…", e)
            if s:
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                s.close()
            s = None
            rospy.sleep(1.0)

        rate.sleep()


if __name__ == "__main__":
    main()
