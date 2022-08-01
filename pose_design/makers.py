from pathlib import Path

from PyQt5.QtWidgets import QListWidget, QScrollArea
from PyQt5 import QtWidgets

from pose_design.myWidgets import Point, MyEditLine


def _createMenuBar(cls):
    menuBar = cls.menuBar()
    fileMenu = QtWidgets.QMenu("File", cls)
    menuBar.addMenu(fileMenu)
    fileMenu.addAction(cls.savePoseAction)
    fileMenu.addAction(cls.loadPoseAction)


def _createActions(cls):
    cls.savePoseFolder = Path.cwd()
    cls.savePoseAction = QtWidgets.QAction(cls)
    cls.savePoseAction.setText("Save directory")
    cls.savePoseAction.triggered.connect(cls._savePoseActionClick)

    cls.loadPoseFolder = Path.cwd()
    cls.loadPoseAction = QtWidgets.QAction(cls)
    cls.loadPoseAction.setText("Load directory")
    cls.loadPoseAction.triggered.connect(cls._loadPoseActionClick)


def _makeScrollArea(cls):
    cls.scroll = QScrollArea(cls)
    cls.scroll.setGeometry(445, 25, 150, 180)
    cls.list = QListWidget(cls)
    cls.list.itemDoubleClicked.connect(cls.DoubleTouch)
    cls.list.itemClicked.connect(cls.SingleTouch)
    cls.scroll.setWidget(cls.list)


def _makeEditLines(cls):
    cls.filename = MyEditLine(
        window=cls, name='File name:', state=Point(450, 790), scale=Point(100, 30))
    cls.poseDuration = MyEditLine(window=cls, name='poseDuration:', state=Point(50, 100),
                                  scale=Point(80, 30), initValue=0)
    cls.changeDuration = MyEditLine(window=cls, name='changeDuration:', state=Point(50, 30),
                                    scale=Point(80, 30), initValue=0)


def _makeButtons(cls):
    class Buttons:
        apply = cls.makeButton('Apply', Point(50, 890))
        send_pose = cls.makeButton('Send Pose', Point(250, 890), cls.send)
        save = cls.makeButton('Save', Point(450, 890), cls.save)
        load = cls.makeButton('Load', Point(450, 850), cls.load)
        loadDir = cls.makeButton('loadDir', Point(50, 840), cls.loadDirectory)
        play = cls.makeButton('play', Point(50, 790), cls.playPoses)

    return Buttons
