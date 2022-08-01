from PyQt5.QtWidgets import QSlider, QLabel, QLineEdit, QLabel
from PyQt5.QtCore import Qt

class Point:
    """pair (x, y)"""
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

class MyEditLine:
    """line with label"""
    
    def __init__(self, window, name, state, scale, initValue=None):
        self.nameLine = QLabel(window)
        self.nameLine.setGeometry(state.x, state.y, scale.x, scale.y - 10)
        self.nameLine.setText(name)
        self.nameLine.adjustSize()
        self.edit_line = QLineEdit(window)
        self.edit_line.setGeometry(state.x, state.y + 20, scale.x, scale.y)
        if initValue is not None:
            self.edit_line.setText(str(initValue))

class Limits:
    """slider with current limits"""
    
    def __init__(self, slider, limit):
        self.slider = slider
        self.limit = limit
class MySlider:
    """slider with line and label"""

    def __init__(self, window, buttonApply, point, sliderRange, name, startValue, limits=None):
        self.window = window
        self.point = point
        self.sliderRange = list(sliderRange)
        self.startValue = startValue
        self.limits = limits
        self.buttonApply = buttonApply
        self.name = name
        self.makeLine()
        self.makeName()
        self.makeSlider()
        self.makeSignals()

    def makeName(self):
        self.label = QLabel(self.window)
        self.label.move(self.point.x, self.point.y)
        self.label.setText(self.name)
        self.label.adjustSize()

    def makeLine(self):
        self.line = QLineEdit(self.window)
        self.line.move(self.point.x, self.point.y + 55)
        self.line.setText(str(int(self.startValue)))

    def makeSlider(self):
        self.slider = QSlider(Qt.Horizontal, self.window)
        self.slider.setRange(self.sliderRange[0], self.sliderRange[-1])
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.setPageStep(5)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        self.slider.move(self.point.x, self.point.y)
        self.slider.setGeometry(self.point.x, self.point.y + 25, 50, 20)
        self.slider.setValue(int(self.startValue))
        self.slider.adjustSize()

    def value(self):
        return int(self.line.text())

    def set(self, value):
        self.slider.setValue(value)
        self.line.setText(str(int(value)))
        print(self.name, self.line.text())

    def makeSignals(self):
        self.slider.valueChanged.connect(self.updateLine)
        self.buttonApply.clicked.connect(self.updateSlider)

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
        if int(value) not in self.sliderRange:
            value = self.sliderRange[0]
        self.slider.setValue(int(value))
