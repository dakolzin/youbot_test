#!/usr/bin/env python
# -*- coding: utf-8 -*-

# у 11 другие топики

# youbot@youbot-desktop:~/script_ws$ rosrun script_ws server.py

# <pre>root@pod:/youbot# ros2 run send_num send_test_copy</pre>

# root@pod:/youbot# ros2 run send_num youbot

# root@pod:/youbot# ros2 run youbot_moveit script


import rospy
import socket
import json

from brics_actuator.msg import JointPositions, JointValue   # команды
from sensor_msgs.msg import JointState                     # (опц.) зеркалим в joint_states

TCP_PORT = 5000

def make_joint_positions(names, values, unit):
    """Собираем сообщение brics_actuator/JointPositions"""
    msg = JointPositions()
    now = rospy.Time.now()
    for n, v in zip(names, values):
        jv = JointValue()
        jv.timeStamp = now
        jv.joint_uri = n
        jv.unit = unit
        jv.value = v
        msg.positions.append(jv)
    return msg

def main():
    rospy.init_node("tcp_cmd_to_youbot")

    # паблишеры команд
    pub_arm_cmd     = rospy.Publisher("/youbot1/arm_1/arm_controller/position_command",
                                      JointPositions, queue_size=3)
    pub_gripper_cmd = rospy.Publisher("/youbot1/arm_1/gripper_controller/position_command",
                                      JointPositions, queue_size=3)

    # (необязательно) дублируем в /arm_1/joint_states для наглядности
    pub_js          = rospy.Publisher("/arm_1/joint_states",
                                      JointState, queue_size=3)

    # простой TCP‑сервер
    srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv_sock.bind(("0.0.0.0", TCP_PORT))
    srv_sock.listen(1)
    rospy.loginfo("TCP‑сервер слушает %d", TCP_PORT)

    while not rospy.is_shutdown():
        client, addr = srv_sock.accept()
        rospy.loginfo("Клиент %s:%d подключён", *addr)

        try:
            while not rospy.is_shutdown():
                data = client.recv(4096)
                if not data:
                    rospy.loginfo("Клиент разорвал соединение")
                    break

                pkt = json.loads(data.decode("utf-8"))
                rospy.loginfo("Получено: %s", pkt)

                names = pkt.get("joint_names", [])
                pos   = pkt.get("positions", [])
                src   = pkt.get("source", "")      # 'arm' | 'gripper'

                # публикуем команду
                if src == "arm":
                    arm_msg = make_joint_positions(names, pos, "rad")
                    pub_arm_cmd.publish(arm_msg)

                elif src == "gripper":
                    grip_msg = make_joint_positions(names, pos, "m")  # пальцы—поступательные
                    pub_gripper_cmd.publish(grip_msg)

                else:
                    rospy.logwarn("Неизвестный source=%s", src)

                # (опционально) дублируем в JointState — удобно смотреть в rqt_plot
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name     = names
                js.position = pos
                pub_js.publish(js)

        except Exception as e:
            rospy.logerr("Ошибка при обработке: %s", e)

        client.close()

    srv_sock.close()

if __name__ == "__main__":
    main()
