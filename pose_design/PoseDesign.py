import numpy as np
from pathlib import Path
import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon

import rclpy
from rclpy.node import Node
from nao_command_msgs.msg import JointStiffnesses
from nao_command_msgs.msg import JointPositions as JPOUT
from nao_command_msgs.msg import JointStiffnesses

from pose_design.application import Window



class PoseDesign(Node):
    def __init__(self):
        super().__init__('pose_design') # make node
        self.PublisherPose = self.create_publisher(JPOUT, '/effectors/joint_positions', 10) # position publisher
        self.PublisherStiffness = self.create_publisher(JointStiffnesses,
                                                        'effectors/joint_stiffnesses', 10) # stiffness publisher
        self.applicationMake(send=self.send_pose) 
        self.applicationRun()

    # make window appliction with icon and window
    def applicationMake(self, send):
        self.app = QApplication(sys.argv) # make application
        image = Path(Path(Path.cwd(), 'src', 'pose_design', 'pose_design'), 'image.jpeg') # image path
        self.app.setWindowIcon(QIcon(str(image))) # set icon
        self.window = Window(send=send) # make window

    # start gui
    def applicationRun(self):
        self.window.show() 
        sys.exit(self.app.exec_())  # correct exit

    # send pose to robot's topics from window 
    def send_pose(self):
        joint_msg = JointStiffnesses()
        joint_msg.indexes = range(25)
        joint_msg.stiffnesses = [1.0] * 25
        self.PublisherStiffness.publish(joint_msg) # make stiffness 1
        pose = self.window.getPose()[0]
        positions_msg = JPOUT()
        positions_msg.indexes = range(25)
        positions_msg.positions = list(map(np.deg2rad, pose))
        self.PublisherPose.publish(positions_msg) # sending





def main(args=None):
    rclpy.init(args=args)
    node = PoseDesign() # make node
    try:
        rclpy.spin(node) # run node
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
