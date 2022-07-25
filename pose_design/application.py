from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QScrollArea
from pathlib import Path
from pose_design.interpolation import Head, LAnkle, RAnkle
import os
import time

import sys

from pose_design.my_widgets import MySlider, Point, Limits, MyEditLine

joints_names = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw',
                'LElbowRoll', 'LWristYaw', 'LHipYawPitch', 'LHipRoll', 'LHipPitch',
                'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch',
                'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll',
                'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'LHand', 'RHand']

start_positions = [5.835615623007271e-17
                   - 6.631492244224546e-09,
                   5.46203182238969e-07,
                   -3.0801841112426676e-12,
                   1.016807331666314e-07,
                   -1.1239626473980024e-07,
                   3.875324239288602e-08,
                   -3.6887286114506423e-07,
                   3.108395389972429e-07,
                   -4.951590426571784e-07,
                   -5.005954903936072e-07,
                   -5.14188116085279e-07,
                   -3.088730693434627e-07,
                   -3.1083948215382406e-07,
                   -4.951590995005972e-07,
                   -5.00595547237026e-07,
                   -5.141881729286979e-07,
                   3.0887309776517213e-07,
                   5.46203182238969e-07,
                   3.0789979940659684e-12,
                   -1.016807331666314e-07,
                   1.1239625763437289e-07,
                   -3.875324239288602e-08,
                   -2.2158275214678724e-11,
                   -2.2157387036259024e-11,
                   ]
start_positions[joints_names.index('LElbowRoll')] = -4
start_positions[joints_names.index('RElbowRoll')] = 3

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

        self._createActions()
        self._createMenuBar()

        self.scroll = QScrollArea(self)
        self.scroll.setGeometry(445, 25, 150, 180)
        self.list = QListWidget(self)
        self.list.itemDoubleClicked.connect(self.DoubleTouch)
        self.list.itemClicked.connect(self.SingleTouch)
        self.scroll.setWidget(self.list)

        #self._createToolBars()

        #self.fileMenu.addAction()
        '''self.TabBar = QtWidgets.QTabBar(self)
        self.TabBar.addTab('smth')
        self.TabBar.show()'''

        #self.FirstTab.setGeometry(20, 20, 100, 30)
        #self.FirstTab.setTabText(0, 'smth')
        #self.FirstTab.setTabText('smth')
        #file, _ = QtWidgets.QFileDialog.getOpenFileName(None, 'Open File', './', "Image (*.png *.jpg *jpeg)")
        #print(file)
        #self.FileDialog = QtWidgets.QFileDialog(self)
        #self.FileDialog.setGeometry(0, 0, 100, 30)
        #self.FileDialog.DialogLabel('File')
        #self.FileDialog.setFileMode(QtWidgets.QFileDialog.directory())


        self.HeadLimits = Head
        self.LAnkleLimits = LAnkle
        self.RAnkleLimits = RAnkle

        self.send = send
        self.receive = receive
        self.setWindowTitle("Pose Designer")
        self.setGeometry(1200, 250, 600, 950)  # (300, 250) от левого верхнего угла
        # ширина, высота

        self.buttons = self.make_buttons()

        self.filename = MyEditLine(window=self, name='File name:', state=Point(450, 790), scale=Point(100, 30))

        self.duration = MyEditLine(window=self, name='Duration:', state=Point(50, 100), scale=Point(80, 30), init_value=0)

        self.sliders = self.make_sliders()
        self.joints = self.make_joints()

    def SingleTouch(self):
        print('single')

    def DoubleTouch(self, item):
        filename = item.text()
        filePath = self.filelistPaths[filename]
        pose = [0] * 25
        f = open(filePath, 'r')
        num = 0
        for line in f:
            if num < len(pose):
                pose[num] = int(line)
            else:
                print('here')
                self.duration.edit_line.setText(line)
            num += 1

        f.close()
        self.setPose(pose)

    def _createMenuBar(self):
        menuBar = self.menuBar()
        # File menu
        fileMenu = QtWidgets.QMenu("File", self)
        menuBar.addMenu(fileMenu)
        fileMenu.addAction(self.savePoseAction)
        fileMenu.addAction(self.loadPoseAction)
        # Edit menu
        '''editMenu = menuBar.addMenu("Edit")
        editMenu.addAction(self.copyAction)
        editMenu.addAction(self.pasteAction)
        editMenu.addAction(self.cutAction)'''
        # Help menu


    def _createActions(self):
        self.savePoseFolder = Path.cwd()
        self.savePoseAction = QtWidgets.QAction(self)
        self.savePoseAction.setText("Save directory")
        self.savePoseAction.triggered.connect(self._savePoseActionClick)

        self.loadPoseFolder = Path.cwd()
        self.loadPoseAction = QtWidgets.QAction(self)
        self.loadPoseAction.setText("Load directory")
        self.loadPoseAction.triggered.connect(self._loadPoseActionClick)

    def _savePoseActionClick(self):
        home = Path.cwd()
        dialog = QtWidgets.QFileDialog(self)
        viewMode = QtWidgets.QFileDialog.FileMode.DirectoryOnly
        dir = dialog.getExistingDirectory(None, 'Select Save directory', str(home.parent))
        self.savePoseFolder = dir

    def _loadPoseActionClick(self):
        home = Path.cwd()
        dialog = QtWidgets.QFileDialog(self)
        viewMode = QtWidgets.QFileDialog.FileMode.DirectoryOnly
        dir = dialog.getExistingDirectory(None, 'Select Load directory', str(home.parent))
        self.loadPoseFolder = dir



    def save(self):
        fn = self.filename.edit_line.text()
        home = Path.cwd()
        if fn is None or fn == '':
            filename = Path(self.savePoseFolder, 'pose.txt')
        else:
            filename = Path(self.savePoseFolder, fn)
        pose, duration = self.getPose()
        f = open(filename, 'w')
        for tmp in pose:
            f.write(str(tmp) + '\n')
        f.write(self.duration.edit_line.text() + '\n')
        f.close()

    def load(self):
        fn = self.filename.edit_line.text()
        home = Path.cwd()
        if fn is None or fn == '':
            filename = Path(self.loadPoseFolder, 'pose.txt')
        else:
            filename = Path(self.loadPoseFolder, fn)
        pose = [0] * 25
        f = open(filename, 'r')
        num = 0
        for line in f:
            if num < len(pose):
                pose[num] = int(line)
            else:
                print('here')
                self.duration.edit_line.setText(line)
            num += 1

        f.close()
        self.setPose(pose)

    def loadFromName(self, filename):
        pose = [0] * 25
        f = open(filename, 'r')
        num = 0
        for line in f:
            if num < len(pose):
                pose[num] = int(line)
            else:
                self.duration.edit_line.setText(line)
            num += 1
        f.close()
        self.setPose(pose)

    def loadDirectory(self):
        self.list.clear()
        self.loadDirPath = self.loadPoseFolder
        self.filelistPaths = {}
        self.filelistNames = []
        for root, dirs, files in os.walk(self.loadDirPath):
            for file in files:
                self.filelistPaths[str(file)] = os.path.join(root, file)
                self.filelistNames.append(file)
        self.filelistNames.sort()
        self.list.addItems(self.filelistNames)

    def playPoses(self):
        for file in self.filelistNames:
            path =  self.filelistPaths[file]
            print(path)
            self.loadFromName(path)
            self.send()
            time.sleep(1)

    def make_buttons(self):
        class Buttons:
            apply = self.make_button('Apply', Point(50, 890))
            send_pose = self.make_button('Send Pose', Point(250, 890), self.send)
            save = self.make_button('Save', Point(450, 890), self.save)
            load = self.make_button('Load', Point(450, 850), self.load)
            loadDir = self.make_button('loadDir', Point(50, 840), self.loadDirectory)
            play = self.make_button('play', Point(50, 790), self.playPoses)
            # receive_pose = self.make_button('Receive Pose', Point(450, 890), self.receive)

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
                                          sld_range=joints_ranges[joints_names.index('HeadYaw')],
                                          start_value=start_positions[joints_names.index('HeadYaw')])
            SldHeadPitch = self.make_slider(name='HeadPitch', point=Point(275, 135),
                                            sld_range=joints_ranges[joints_names.index('HeadPitch')],
                                            start_value=start_positions[joints_names.index('HeadPitch')],
                                            limits=Limits(slider=SldHeadYaw, limit=self.HeadLimits))
            SldLShoulderPitch = self.make_slider(name='LShoulderPitch', point=Point(50, 245),
                                                 sld_range=joints_ranges[joints_names.index('LShoulderPitch')],
                                                 start_value=start_positions[joints_names.index('LShoulderPitch')])
            SldRShoulderPitch = self.make_slider(name='RShoulderPitch', point=Point(450, 245),
                                                 sld_range=joints_ranges[joints_names.index('RShoulderPitch')],
                                                 start_value=start_positions[joints_names.index('RShoulderPitch')])
            SldLHipYawPitch = self.make_slider(name='LHipYawPitch', point=Point(260, 245),
                                               sld_range=joints_ranges[joints_names.index('LHipYawPitch')],
                                               start_value=start_positions[joints_names.index('LHipYawPitch')])
            SldLShoulderRoll = self.make_slider(name='LShoulderRoll', point=Point(50, 355),
                                                sld_range=joints_ranges[joints_names.index('LShoulderRoll')],
                                                start_value=start_positions[joints_names.index('LShoulderRoll')])
            SldRShoulderRoll = self.make_slider(name='RShoulderRoll', point=Point(450, 355),
                                                sld_range=joints_ranges[joints_names.index('RShoulderRoll')],
                                                start_value=start_positions[joints_names.index('RShoulderRoll')])
            SldLElbowYaw = self.make_slider(name='LElbowYaw', point=Point(50, 465),
                                            sld_range=joints_ranges[joints_names.index('LElbowYaw')],
                                            start_value=start_positions[joints_names.index('LElbowYaw')])
            SldRElbowYaw = self.make_slider(name='RElbowYaw', point=Point(450, 465),
                                            sld_range=joints_ranges[joints_names.index('RElbowYaw')],
                                            start_value=start_positions[joints_names.index('RElbowYaw')])
            SldLElbowRoll = self.make_slider(name='LElbowRoll', point=Point(50, 575),
                                             sld_range=joints_ranges[joints_names.index('LElbowRoll')],
                                             start_value=start_positions[joints_names.index('LElbowRoll')])
            SldRElbowRoll = self.make_slider(name='RElbowRoll', point=Point(450, 575),
                                             sld_range=joints_ranges[joints_names.index('RElbowRoll')],
                                             start_value=start_positions[joints_names.index('RElbowRoll')])
            SldLWristYaw = self.make_slider(name='LWristYaw', point=Point(50, 685),
                                            sld_range=joints_ranges[joints_names.index('LWristYaw')],
                                            start_value=start_positions[joints_names.index('LWristYaw')])
            SldRWristYaw = self.make_slider(name='RWristYaw', point=Point(450, 685),
                                            sld_range=joints_ranges[joints_names.index('RWristYaw')],
                                            start_value=start_positions[joints_names.index('RWristYaw')])
            SldLHipRoll = self.make_slider(name='LHipRoll', point=Point(200, 355),
                                           sld_range=joints_ranges[joints_names.index('LHipRoll')],
                                           start_value=start_positions[joints_names.index('LHipRoll')])
            SldLHipPitch = self.make_slider(name='LHipPitch', point=Point(200, 465),
                                            sld_range=joints_ranges[joints_names.index('LHipPitch')],
                                            start_value=start_positions[joints_names.index('LHipPitch')])
            SldLKneePitch = self.make_slider(name='LKneePitch', point=Point(200, 575),
                                             sld_range=joints_ranges[joints_names.index('LKneePitch')],
                                             start_value=start_positions[joints_names.index('LKneePitch')])
            SldLAnklePitch = self.make_slider(name='LAnklePitch', point=Point(200, 685),
                                              sld_range=joints_ranges[joints_names.index('LAnklePitch')],
                                              start_value=start_positions[joints_names.index('LAnklePitch')])
            SldLAnkleRoll = self.make_slider(name='LAnkleRoll', point=Point(200, 795),
                                             sld_range=joints_ranges[joints_names.index('LAnkleRoll')],
                                             start_value=start_positions[joints_names.index('LAnkleRoll')],
                                             limits=Limits(slider=SldLAnklePitch, limit=self.LAnkleLimits))

            SldRHipRoll = self.make_slider(name='RHipRoll', point=Point(325, 355),
                                           sld_range=joints_ranges[joints_names.index('RHipRoll')],
                                           start_value=start_positions[joints_names.index('RHipRoll')])
            SldRHipPitch = self.make_slider(name='RHipPitch', point=Point(325, 465),
                                            sld_range=joints_ranges[joints_names.index('RHipPitch')],
                                            start_value=start_positions[joints_names.index('RHipPitch')])
            SldRKneePitch = self.make_slider(name='RKneePitch', point=Point(325, 575),
                                             sld_range=joints_ranges[joints_names.index('RKneePitch')],
                                             start_value=start_positions[joints_names.index('RKneePitch')])
            SldRAnklePitch = self.make_slider(name='RAnklePitch', point=Point(325, 685),
                                              sld_range=joints_ranges[joints_names.index('RAnklePitch')],
                                              start_value=start_positions[joints_names.index('RAnklePitch')])
            SldRAnkleRoll = self.make_slider(name='RAnkleRoll', point=Point(325, 795),
                                             sld_range=joints_ranges[joints_names.index('RAnkleRoll')],
                                             start_value=start_positions[joints_names.index('RAnkleRoll')],
                                             limits=Limits(slider=SldRAnklePitch, limit=self.RAnkleLimits))

        return Sliders

    def make_slider(self, name='Test slider', point=Point(), sld_range=range(0, 1), start_value=0, limits=None):
        sld = MySlider(self, self.buttons.apply, Point(point.x, point.y), sld_range, name, start_value, limits)
        return sld

    def check_joint_pose(self, pose, joint_pose, limits):
        value = pose[joint_pose]
        local_range = limits.limit.getValueRange(limits.slider.value())
        if value >= max(local_range):
            value = max(local_range)
        elif value <= min(local_range):
            value = min(local_range)
        return value


    def getPose(self):
        pose = [1] * 25
        for joint in self.JointsList:
            pose[joint.num] = joint.getValue()
        pose[1] = self.check_joint_pose(pose, 1, Limits(slider=self.sliders.SldHeadPitch, limit=self.HeadLimits))
        pose[12] = self.check_joint_pose(pose, 12, Limits(slider=self.sliders.SldLAnklePitch, limit=self.LAnkleLimits))
        pose[17] = self.check_joint_pose(pose, 17, Limits(slider=self.sliders.SldRAnklePitch, limit=self.RAnkleLimits))
        return pose, self.duration.edit_line.text()

    def setPose(self, pose):
        pose[1] = self.check_joint_pose(pose, 1, Limits(slider=self.sliders.SldHeadPitch, limit=self.HeadLimits))
        pose[12] = self.check_joint_pose(pose, 12, Limits(slider=self.sliders.SldLAnklePitch, limit=self.LAnkleLimits))
        pose[17] = self.check_joint_pose(pose, 17, Limits(slider=self.sliders.SldRAnklePitch, limit=self.RAnkleLimits))
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
