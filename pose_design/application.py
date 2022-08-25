import os
import time

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import  QMainWindow
from pathlib import Path

from pose_design.myWidgets import MySlider, Point, Limits
from pose_design.jointsClass import _makeJoints
from pose_design.slidersClass import _makeSliders
from pose_design.makers import _createActions, _createMenuBar, _makeScrollArea, _makeButtons, _makeEditLines
from pose_design.interpolation import Head, LAnkle, RAnkle
from pose_design.constants import jointsRanges, jointsNames

class Window(QMainWindow):
    def __init__(self, send):
        super(Window, self).__init__()
        self.send = send
        self.setWindowTitle("Pose Designer")
        self.setGeometry(1200, 250, 600, 950)
        
        _createActions(self)
        _createMenuBar(self)
        _makeScrollArea(self)
        self.buttons = _makeButtons(self)
        _makeEditLines(self)
        self.sliders = _makeSliders(self)
        self.joints = _makeJoints(self)

    def SingleTouch(self):
        pass
    
    def readPoseFromFile(self, filename):
        """get joints positions and durations from path"""
        pose = [0] * 25
        f = open(filename, 'r')
        num = 0
        for line in f:
            if num < len(pose) - 1:
                pose[num] = int(line)
            elif num == len(pose):
                pD = int(line)
            elif num == len(pose) + 1:
                cD = int(line)
            num += 1
        f.close()
        return {'pose': pose, 'poseDuration': pD, 'changeDuration': cD}
        

    def DoubleTouch(self, item):
        filename = item.text()
        filePath = self.filelistPaths[filename]
        self.setPose(self.readPoseFromFile(filename=filePath))

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
        """click on save button"""
        fn = self.filename.edit_line.text()
        if fn is None or fn == '':
            filename = Path(self.savePoseFolder, 'pose.txt')
        else:
            filename = Path(self.savePoseFolder, fn)
        pose, poseDuration, changeDuration = self.getPose()
        f = open(filename, 'w')
        for tmp in pose:
            f.write(str(tmp) + '\n')
        f.write(poseDuration + '\n')
        f.write(changeDuration + '\n')
        f.close()

    def load(self):
        """click on load button"""
        fn = self.filename.edit_line.text()
        if fn is None or fn == '':
            filename = Path(self.loadPoseFolder, 'pose.txt')
        else:
            filename = Path(self.loadPoseFolder, fn)
        self.setPose(self.readPoseFromFile(filename=filename))

    def loadDirectory(self):
        """click on loadDir button"""
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
        
    def setSendSleep(self, pose_, dur):
        self.setPose(pose_)
        self.send()
        time.sleep(dur)

    def playPoses(self):
        arr = []
        for file in self.filelistNames:
            path =  self.filelistPaths[file]
            arr.append(self.readPoseFromFile(path))
        
        self.setSendSleep(arr[0], int(arr[0]['poseDuration'])/1000)
        for i in range(len(arr) - 1):
            cur = arr[i]
            next = arr[i + 1]
            delta = [next['pose'][j] - cur['pose'][j] for j in range(len(cur['pose']))]
            n = int(cur['changeDuration']) // 12
            n = n if n > 0 else 1
            dPose = list(map(lambda t: t / n, delta))
            for q in range(n - 1):
                cur['pose'] = [cur['pose'][j] + dPose[j] for j in range(len(dPose))]
                self.checkPoseReality(cur['pose'])
                self.setSendSleep(cur, 1/1000)
            self.setSendSleep(next, int(next['poseDuration'])/1000)

    def makeButton(self, name='Test button', point=Point(), signal=None):
        btn = QtWidgets.QPushButton(self)
        btn.move(point.x, point.y)
        btn.setText(name)
        btn.adjustSize()
        if signal is not None:
            btn.clicked.connect(signal)
        return btn

    def makeSlider(self, name='Test slider', point=Point(), sldRange=range(0, 1), startValue=0, limits=None):
        sld = MySlider(self, self.buttons.apply, Point(point.x, point.y), sldRange, name, startValue, limits)
        return sld

    def checkJointPose(self, pose, jointPose, limits):
        """check joints onto allowed interval"""
        value = pose[jointPose]
        local_range = limits.limit.getValueRange(limits.slider.value())
        if value >= max(local_range):
            value = max(local_range)
        elif value <= min(local_range):
            value = min(local_range)
        return value
    
            
    def checkPoseReality(self, pose):
        pose[1] = self.checkJointPose(pose, 1, Limits(slider=self.sliders.SldHeadPitch, limit=Head))
        pose[12] = self.checkJointPose(pose, 12, Limits(slider=self.sliders.SldLAnklePitch, limit=LAnkle))
        pose[17] = self.checkJointPose(pose, 17, Limits(slider=self.sliders.SldRAnklePitch, limit=RAnkle))
        for i in range(23):
            cur = pose[i]
            range_ = jointsRanges[i]
            if cur >= max(range_):
                cur = max(range_)
            if cur <= min(range_):
                cur = min(range_)
            pose[i] = cur

    def getPose(self):
        """get values from sliders and durations from GUI"""
        pose = [0] * 25
        for joint in self.JointsList:
            pose[joint.num] = joint.getValue()
        pose[1] = self.checkJointPose(pose, 1, Limits(slider=self.sliders.SldHeadPitch, limit=Head))
        pose[12] = self.checkJointPose(pose, 12, Limits(slider=self.sliders.SldLAnklePitch, limit=LAnkle))
        pose[17] = self.checkJointPose(pose, 17, Limits(slider=self.sliders.SldRAnklePitch, limit=RAnkle))
        return pose, self.poseDuration.edit_line.text(), self.changeDuration.edit_line.text()

    def setPose(self, pose_):
        """set sliders and durations on GUI"""
        pose = pose_['pose']
        poseDuration = pose_['poseDuration']
        changeDuration = pose_['changeDuration']
        pose[1] = self.checkJointPose(pose, 1, Limits(slider=self.sliders.SldHeadPitch, limit=Head))
        pose[12] = self.checkJointPose(pose, 12, Limits(slider=self.sliders.SldLAnklePitch, limit=LAnkle))
        pose[17] = self.checkJointPose(pose, 17, Limits(slider=self.sliders.SldRAnklePitch, limit=RAnkle))
        for joint in self.JointsList:
            joint.setValue(int(pose[joint.num]))
        self.poseDuration.edit_line.setText(str(poseDuration))
        self.changeDuration.edit_line.setText(str(changeDuration))
