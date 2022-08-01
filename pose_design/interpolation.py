from scipy.interpolate import CubicSpline

from pose_design.constants import HeadYaw, HeadPitchMin, HeadPitchMax, HeadPitchRange, LAnklePitch, \
LAnkleRollMin, LAnkleRollMax, LAnkleRollRange, RAnklePitch, RAnkleRollMin, RAnkleRollMax, RAnkleRollRange
class JointInterpolation:
    """
    allowing values for correleted joints
    
    using cubicSpline interpolation with minMax limiter
    """
    
    def __init__(self, x, y_bottom, y_top, range_):
        self.x = x
        self.y_bottom = y_bottom
        self.y_top = y_top
        self.range_ = range_
        self.cs_bottom = CubicSpline(x, y_bottom)
        self.cs_top = CubicSpline(x, y_top)

    def getValueMin(self, x_):
        """get min value in current position"""
        tmp = self.cs_bottom(x_)
        if tmp >= max(self.range_):
            tmp = max(self.range_)
        elif tmp <= min(self.range_):
            tmp = min(self.range_)
        return tmp

    def getValueMax(self, x_):
        """get max value in current position"""
        tmp = self.cs_top(x_)
        if tmp >= max(self.range_):
            tmp = max(self.range_)
        elif tmp <= min(self.range_):
            tmp = min(self.range_)
        return tmp

    def getValueRange(self, x_):
        """get allowed range of joint in current position(analise other joints)"""
        return [self.getValueMin(x_), self.getValueMax(x_)]

Head = JointInterpolation(HeadYaw, HeadPitchMin, HeadPitchMax, HeadPitchRange)
LAnkle = JointInterpolation(LAnklePitch, LAnkleRollMin, LAnkleRollMax, LAnkleRollRange)
RAnkle = JointInterpolation(RAnklePitch, RAnkleRollMin, RAnkleRollMax, RAnkleRollRange)

