import matplotlib as matplotlib
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np

HeadYaw = [-119.52, -87.49, -62.45, -51.74, -43.32, -27.85, 0.0, 27.85, 43.32, 51.74, 62.45, 87.49, 119.52]
HeadPitchMin = [-25.73, -18.91, -24.64, -27.5, -31.4, -38.5, -38.5, -38.5, -31.4, -27.5, -24.64, -18.91, -25.73]
HeadPitchMax = [18.91, 11.46, 17.19, 18.91, 21.2, 24.18, 29.51, 24.18, 21.2, 18.91, 17.19, 11.46, 18.91]
HeadPitchRange = [-38.5, 29.5]

LAnklePitch = [-68.15, -48.13, -40.11, -25.78, 5.73, 20.05, 52.87]
LAnkleRollMin = [-2.86, -10.31, -22.8, -22.8, -22.8, -22.8, 0.0]
LAnkleRollMax = [4.3, 9.74, 12.61, 44.06, 44.06, 31.54, 2.86]
LAnkleRollRange = [-22.79, 44.06]

RAnklePitch = [-68.15, -48.13, -40.11, -25.78, 5.73, 20.05, 52.87]
RAnkleRollMin = [-4.3, -9.74, -12.61, -44.06, -44.06, -31.54, -2.86]
RAnkleRollMax = [2.86, 10.31, 22.8, 22.8, 22.8, 22.8, 0.0]
RAnkleRollRange = [-44.06, 22.80]


class JointInterpolation:
    def __init__(self, x, y_bottom, y_top, range_):
        self.x = x
        self.y_bottom = y_bottom
        self.y_top = y_top
        self.range_ = range_
        self.cs_bottom = CubicSpline(x, y_bottom)
        self.cs_top = CubicSpline(x, y_top)

    def draw(self):
        x_test = np.linspace(min(self.x), max(self.x), 100)
        plt.plot(x_test, list(map(self.getValueMin, x_test)), c='r')
        plt.plot(x_test, list(map(self.getValueMax, x_test)), c='b')
        plt.scatter(self.x, self.y_bottom, c='r')
        plt.scatter(self.x, self.y_top, c='b')
        plt.show()

    def getValueMin(self, x_):
        tmp = self.cs_bottom(x_)
        if tmp >= max(self.range_):
            tmp = max(self.range_)
        elif tmp <= min(self.range_):
            tmp = min(self.range_)
        return tmp

    def getValueMax(self, x_):
        tmp = self.cs_top(x_)
        if tmp >= max(self.range_):
            tmp = max(self.range_)
        elif tmp <= min(self.range_):
            tmp = min(self.range_)
        return tmp

    def getValueRange(self, x_):
        return [self.getValueMin(x_), self.getValueMax(x_)]


'''x = []
y_bottom = []
y_top = []
f = open('pose_design/data_to_interpolation.txt', 'r')
for line in f:
    tmp = list(map(float, line.split()))
    x.append(tmp[0])
    y_bottom.append(tmp[1])
    y_top.append(tmp[2])
f.close()
print(x, y_bottom, y_top)'''

Head = JointInterpolation(HeadYaw, HeadPitchMin, HeadPitchMax, HeadPitchRange)
LAnkle = JointInterpolation(LAnklePitch, LAnkleRollMin, LAnkleRollMax, LAnkleRollRange)
RAnkle = JointInterpolation(RAnklePitch, RAnkleRollMin, RAnkleRollMax, RAnkleRollRange)

