# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
import time
import numpy as np

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt

import sys
import os

from pose_design.my_widgets import MySlider, Point

from std_msgs.msg import String
from nao_sensor_msgs.msg import Touch
from std_msgs.msg import ColorRGBA
from nao_command_msgs.msg import ChestLed
from nao_command_msgs.msg import JointStiffnesses
from nao_command_msgs.msg import JointPositions as JPOUT
from nao_sensor_msgs.msg import JointPositions as JPIN

from pose_design.application import Window

import threading

global data


class PoseDesign(Node):
    def __init__(self):
        global data
        data = 777
        super().__init__('PoseDesign')
        self.PublisherPose = self.create_publisher(JPOUT, '/effectors/joint_positions', 10)
        #self.Application_make(receive=self.receive_pose, send=self.send_pose)
        # self.SubscriptionPose = self.create_subscription(JPIN, '/sensors/joint_positions', self.receive_pose, 10)
        #self.SubscriptionPose = threading.Thread(target=self.create_subscription,
        #                                        args=(JPIN, '/sensors/joint_positions', self.rcv, 10))
        #self.SubscriptionPose.start()

        #self.SubscriptionPose = threading.Thread(target=self.run_my_thread, args=(self.rcv, self))
        #self.SubscriptionPose.start()




        '''self.subscription = self.create_subscription(JPIN, '/sensors/joint_positions', self.rcv, 10)
        self.sub = self.create_subscription(Touch, 'sensors/touch', self.rcv, 10)'''

        # self.SubscriptionPose.start()
        self.Application_make(receive=self.receive_pose, send=self.send_pose)
        self.Application_run()

    @staticmethod
    def run_my_thread(rcv_function, node):
        global data
        node.create_subscription(JPIN, '/sensors/joint_positions', data, 10)
        while(True):
            pass


    def rcv(self, msg):
        global data
        data = msg
        print('YES')


    def Application_make(self, send, receive):
        self.app = QApplication(sys.argv)
        self.window = Window(send=send, receive=receive)

    def Application_run(self):
        self.window.show()
        sys.exit(self.app.exec_())  # корректное завершение

    def receive_pose(self):
        pass
#        open('data.txt', 'w').close()
#        os.system('ros2 topic echo /sensors/joint_positions > data.txt')
#        print('HI')
#        os.system('cp data.txt output.txt')
#        os.system('rm data.txt')



    def send_pose(self):
        pose = self.window.getPose()
        positions_msg = JPOUT()
        positions_msg.indexes = range(25)
        positions_msg.positions = list(map(np.deg2rad, pose))
        print('send:', positions_msg)
        self.PublisherPose.publish(positions_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseDesign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
