from PyQt5.QtWidgets import (QWidget, QSlider, QHBoxLayout,
                             QLabel, QApplication, QLineEdit, QLabel)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
import sys



class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class MyEditLine:
    def __init__(self, window, name, state, scale, init_value=None):
        self.name_line = QLabel(window)
        self.name_line.setGeometry(state.x, state.y, scale.x, scale.y - 10)
        self.name_line.setText(name)
        self.name_line.adjustSize()
        self.edit_line = QLineEdit(window)
        self.edit_line.setGeometry(state.x, state.y + 20, scale.x, scale.y)
        if init_value is not None:
            self.edit_line.setText(str(init_value))


class Limits:
    def __init__(self, slider, limit):
        self.slider = slider
        self.limit = limit


class MySlider:

    def __init__(self, window, button_apply, point, slider_range, name, start_value, limits=None):
        self.window = window
        self.point = point
        self.slider_range = list(slider_range)
        self.start_value = start_value
        self.limits = limits
        self.button_apply = button_apply
        self.name = name
        self.make_line()
        self.make_name()
        self.make_slider()
        self.make_signals()

    def make_name(self):
        self.label = QLabel(self.window)
        self.label.move(self.point.x, self.point.y - 25)
        self.label.setText(self.name)
        self.label.adjustSize()

    def make_line(self):
        self.line = QLineEdit(self.window)
        self.line.move(self.point.x, self.point.y + 30)
        self.line.setText(str(int(self.start_value)))

    def make_slider(self):
        self.slider = QSlider(Qt.Horizontal, self.window)
        self.slider.setRange(self.slider_range[0], self.slider_range[-1])
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.setPageStep(5)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        self.slider.move(self.point.x, self.point.y)
        self.slider.setGeometry(self.point.x, self.point.y, 50, 20)
        self.slider.setValue(int(self.start_value))
        self.slider.adjustSize()

    def value(self):
        return int(self.line.text())
        #return self.slider.value()

    def set(self, value):
        self.slider.setValue(value)

    def make_signals(self):
        self.slider.valueChanged.connect(self.updateLine)
        self.button_apply.clicked.connect(self.updateSlider)

    def updateLine(self):
        value = self.slider.value()
        if self.limits is not None:
            local_range = self.limits.limit.getValueRange(self.limits.slider.value())
            if value >= max(local_range):
                value = max(local_range)
            elif value <= min(local_range):
                value = min(local_range)
        self.line.setText(str(int(value)))

    def updateSlider(self):
        value = self.line.text()
        if int(value) not in self.slider_range:
            value = self.slider_range[0]
        self.slider.setValue(int(value))
