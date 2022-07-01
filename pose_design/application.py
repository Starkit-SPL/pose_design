from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt

import sys

from pose_design.my_widgets import MySlider, Point

joints_names = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw',
                'LElbowRoll', 'LWristYaw', 'LHipYawPitch', 'LHipRoll', 'LHipPitch',
                'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch',
                'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll',
                'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'LHand', 'RHand']

# This range is for NAO5, check for NAO6
joints_ranges = [range(-115, 115), range(-36, 28), range(-118, 118),
                 range(-17, 75), range(-118, 118), range(-87, -3), range(-103, 103),
                 range(-64, 41), range(-20, 44), range(-87, 26), range(-4, 120),
                 range(-67, 51), range(-21, 43), range(-44, 20), range(-87, 26),
                 range(-4, 120), range(-67, 51), range(-43, 21), range(-118, 118),
                 range(-75, 17), range(-118, 118), range(3, 87), range(-103, 103)]


class Joint:
    def __init__(self, name=None, range_list=None, slider=None, SLD=None):
        if SLD is None:
            self.num = joints_names.index(name)
            self.name = name
            self.range = joints_ranges[self.num]  # degrees
            self.slider = slider
        else:
            self.slider = SLD
            self.name = SLD.name
            self.num = joints_names.index(self.name)
            self.range = SLD.slider_range

    def getValue(self):
        return self.slider.value()

    def setValue(self, value):
        self.slider.set(value)


class Window(QMainWindow):
    def __init__(self, receive, send):
        super(Window, self).__init__()

        self.send = send
        self.receive = receive
        self.setWindowTitle("Simple program")
        self.setGeometry(1200, 250, 600, 950)  # (300, 250) от левого верхнего угла
        # ширина, высота

        self.buttons = self.make_buttons()

        self.sliders = self.make_sliders()
        self.joints = self.make_joints()

    def make_buttons(self):
        class Buttons:
            apply = self.make_button('Apply', Point(50, 890))
            send_pose = self.make_button('Send Pose', Point(250, 890), self.send)
            #receive_pose = self.make_button('Receive Pose', Point(450, 890), self.receive)

        return Buttons
        # return {'apply': button_apply, 'receive': button_receive_pose, 'send': button_send_pose}

    def make_button(self, name='Test button', point=Point(), signal=None):
        btn = QtWidgets.QPushButton(self)
        btn.move(point.x, point.y)
        btn.setText(name)
        btn.adjustSize()
        if signal is not None:
            btn.clicked.connect(signal)
        return btn


    def make_sliders(self):
        class Sliders:
            SldHeadYaw = self.make_slider(name='HeadYaw', point=Point(275, 25),
                                          sld_range=joints_ranges[joints_names.index('HeadYaw')])
            SldHeadPitch = self.make_slider(name='HeadPitch', point=Point(275, 135),
                                            sld_range=joints_ranges[joints_names.index('HeadPitch')])
            SldLShoulderPitch = self.make_slider(name='LShoulderPitch', point=Point(50, 245),
                                                 sld_range=joints_ranges[joints_names.index('LShoulderPitch')])
            SldRShoulderPitch = self.make_slider(name='RShoulderPitch', point=Point(450, 245),
                                                 sld_range=joints_ranges[joints_names.index('RShoulderPitch')])
            SldLHipYawPitch = self.make_slider(name='LHipYawPitch', point=Point(260, 245),
                                               sld_range=joints_ranges[joints_names.index('LHipYawPitch')])
            SldLShoulderRoll = self.make_slider(name='LShoulderRoll', point=Point(50, 355),
                                             sld_range=joints_ranges[joints_names.index('LShoulderRoll')])
            SldRShoulderRoll = self.make_slider(name='RShoulderRoll', point=Point(450, 355),
                                             sld_range=joints_ranges[joints_names.index('RShoulderRoll')])
            SldLElbowYaw = self.make_slider(name='LElbowYaw', point=Point(50, 465),
                                             sld_range=joints_ranges[joints_names.index('LElbowYaw')])
            SldRElbowYaw = self.make_slider(name='RElbowYaw', point=Point(450, 465),
                                             sld_range=joints_ranges[joints_names.index('RElbowYaw')])
            SldLElbowRoll = self.make_slider(name='LElbowRoll', point=Point(50, 575),
                                             sld_range=joints_ranges[joints_names.index('LElbowRoll')])
            SldRElbowRoll = self.make_slider(name='RElbowRoll', point=Point(450, 575),
                                             sld_range=joints_ranges[joints_names.index('RElbowRoll')])
            SldLWristYaw = self.make_slider(name='LWristYaw', point=Point(50, 685),
                                             sld_range=joints_ranges[joints_names.index('LWristYaw')])
            SldRWristYaw = self.make_slider(name='RWristYaw', point=Point(450, 685),
                                             sld_range=joints_ranges[joints_names.index('RWristYaw')])
            SldLHipRoll = self.make_slider(name='LHipRoll', point=Point(200, 355),
                                             sld_range=joints_ranges[joints_names.index('LHipRoll')])
            SldLHipPitch = self.make_slider(name='LHipPitch', point=Point(200, 465),
                                             sld_range=joints_ranges[joints_names.index('LHipPitch')])
            SldLKneePitch = self.make_slider(name='LKneePitch', point=Point(200, 575),
                                             sld_range=joints_ranges[joints_names.index('LKneePitch')])
            SldLAnklePitch = self.make_slider(name='LAnklePitch', point=Point(200, 685),
                                             sld_range=joints_ranges[joints_names.index('LAnklePitch')])
            SldLAnkleRoll = self.make_slider(name='LAnkleRoll', point=Point(200, 795),
                                             sld_range=joints_ranges[joints_names.index('LAnkleRoll')])

            SldRHipRoll = self.make_slider(name='RHipRoll', point=Point(325, 355),
                                        sld_range=joints_ranges[joints_names.index('RHipRoll')])
            SldRHipPitch = self.make_slider(name='RHipPitch', point=Point(325, 465),
                                         sld_range=joints_ranges[joints_names.index('RHipPitch')])
            SldRKneePitch = self.make_slider(name='RKneePitch', point=Point(325, 575),
                                          sld_range=joints_ranges[joints_names.index('RKneePitch')])
            SldRAnklePitch = self.make_slider(name='RAnklePitch', point=Point(325, 685),
                                           sld_range=joints_ranges[joints_names.index('RAnklePitch')])
            SldRAnkleRoll = self.make_slider(name='RAnkleRoll', point=Point(325, 795),
                                          sld_range=joints_ranges[joints_names.index('RAnkleRoll')])

        return Sliders

    def make_slider(self, name='Test slider', point=Point(), sld_range=range(0, 1)):
        sld = MySlider(self, self.buttons.apply, Point(point.x, point.y), sld_range, name)
        return sld

    def getPose(self):
        pose = [0] * 25
        for joint in self.JointsList:
            pose[joint.num] = joint.getValue()
        return pose

    def setPose(self, pose):
        for joint in self.JointsList:
            joint.setValue(pose[joint.num])

    def make_joints(self):
        class Joints:
            HeadYaw = Joint(name=self.sliders.SldHeadYaw.name,
                            range_list=self.sliders.SldHeadYaw.slider_range, slider=self.sliders.SldHeadYaw)
            HeadPitch = Joint(name=self.sliders.SldHeadPitch.name,
                              range_list=self.sliders.SldHeadPitch.slider_range, slider=self.sliders.SldHeadPitch)
            LShoulderPitch = Joint(name=self.sliders.SldLShoulderPitch.name,
                                   range_list=self.sliders.SldLShoulderPitch.slider_range,
                                   slider=self.sliders.SldLShoulderPitch)
            RShoulderPitch = Joint(name=self.sliders.SldRShoulderPitch.name,
                                   range_list=self.sliders.SldRShoulderPitch.slider_range,
                                   slider=self.sliders.SldRShoulderPitch)
            LHipYawPitch = Joint(name=self.sliders.SldLHipYawPitch.name,
                                 range_list=self.sliders.SldLHipYawPitch.slider_range,
                                 slider=self.sliders.SldLHipYawPitch)
            LShoulderRoll = Joint(SLD=self.sliders.SldLShoulderRoll)
            RShoulderRoll = Joint(SLD=self.sliders.SldRShoulderRoll)
            LElbowYaw = Joint(SLD=self.sliders.SldLElbowYaw)
            RElbowYaw = Joint(SLD=self.sliders.SldRElbowYaw)
            LElbowRoll = Joint(SLD=self.sliders.SldLElbowRoll)
            RElbowRoll = Joint(SLD=self.sliders.SldRElbowRoll)
            LWristYaw = Joint(SLD=self.sliders.SldLWristYaw)
            RWristYaw = Joint(SLD=self.sliders.SldRWristYaw)

            LHipRoll = Joint(SLD=self.sliders.SldLHipRoll)
            LHipPitch = Joint(SLD=self.sliders.SldLHipPitch)
            LKneePitch = Joint(SLD=self.sliders.SldLKneePitch)
            LAnklePitch = Joint(SLD=self.sliders.SldLAnklePitch)
            LAnkleRoll = Joint(SLD=self.sliders.SldLAnkleRoll)
            RHipRoll = Joint(SLD=self.sliders.SldRHipRoll)
            RHipPitch = Joint(SLD=self.sliders.SldRHipPitch)
            RKneePitch = Joint(SLD=self.sliders.SldRKneePitch)
            RAnklePitch = Joint(SLD=self.sliders.SldRAnklePitch)
            RAnkleRoll = Joint(SLD=self.sliders.SldRAnkleRoll)


        self.JointsList = [Joints.HeadYaw, Joints.HeadPitch, Joints.LShoulderPitch, Joints.RShoulderPitch,
                           Joints.LHipYawPitch, Joints.RHipPitch, Joints.LShoulderRoll, Joints.RShoulderRoll,
                           Joints.LElbowYaw, Joints.RElbowYaw, Joints.RElbowRoll, Joints.LElbowRoll,
                           Joints.LWristYaw, Joints.RWristYaw, Joints.LHipRoll, Joints.LHipPitch, Joints.LKneePitch,
                           Joints.LAnklePitch, Joints.LAnkleRoll, Joints.RHipRoll, Joints.RHipPitch, Joints.RKneePitch,
                           Joints.RAnklePitch, Joints.RAnkleRoll]
        return Joints


'''def Application(send, receive):
    app = QApplication(sys.argv)
    window = Window(send, receive)
    window.show()
    sys.exit(app.exec_())  # корректное завершение'''

'''
if __name__ == "__main__":
    application()'''
