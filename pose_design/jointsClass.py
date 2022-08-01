from pose_design.constants import jointsNames, jointsRanges



class Joint:
    def __init__(self, name=None, rangeList=None, slider=None, SLD=None):
        if SLD is None:
            self.num = jointsNames.index(name)
            self.name = name
            self.range = jointsRanges[self.num]  # degrees
            self.slider = slider
        else:
            self.slider = SLD
            self.name = SLD.name
            self.num = jointsNames.index(self.name)
            self.range = SLD.sliderRange

    def getValue(self):
        return self.slider.value()

    def setValue(self, value):
        self.slider.set(value)
           
def _makeJoints(cls):
    """associate joints with sliders"""
    class Joints:
        HeadYaw = Joint(SLD=cls.sliders.SldHeadYaw)
        HeadPitch = Joint(SLD=cls.sliders.SldHeadPitch)
        LShoulderPitch = Joint(SLD=cls.sliders.SldLShoulderPitch)
        RShoulderPitch = Joint(SLD=cls.sliders.SldRShoulderPitch)
        LHipYawPitch = Joint(SLD=cls.sliders.SldLHipYawPitch)
        LShoulderRoll = Joint(SLD=cls.sliders.SldLShoulderRoll)
        RShoulderRoll = Joint(SLD=cls.sliders.SldRShoulderRoll)
        LElbowYaw = Joint(SLD=cls.sliders.SldLElbowYaw)
        RElbowYaw = Joint(SLD=cls.sliders.SldRElbowYaw)
        LElbowRoll = Joint(SLD=cls.sliders.SldLElbowRoll)
        RElbowRoll = Joint(SLD=cls.sliders.SldRElbowRoll)
        LWristYaw = Joint(SLD=cls.sliders.SldLWristYaw)
        RWristYaw = Joint(SLD=cls.sliders.SldRWristYaw)
        LHipRoll = Joint(SLD=cls.sliders.SldLHipRoll)
        LHipPitch = Joint(SLD=cls.sliders.SldLHipPitch)
        LKneePitch = Joint(SLD=cls.sliders.SldLKneePitch)
        LAnklePitch = Joint(SLD=cls.sliders.SldLAnklePitch)
        LAnkleRoll = Joint(SLD=cls.sliders.SldLAnkleRoll)
        RHipRoll = Joint(SLD=cls.sliders.SldRHipRoll)
        RHipPitch = Joint(SLD=cls.sliders.SldRHipPitch)
        RKneePitch = Joint(SLD=cls.sliders.SldRKneePitch)
        RAnklePitch = Joint(SLD=cls.sliders.SldRAnklePitch)
        RAnkleRoll = Joint(SLD=cls.sliders.SldRAnkleRoll)

    cls.JointsList = [Joints.HeadYaw, Joints.HeadPitch, Joints.LShoulderPitch, Joints.RShoulderPitch,
                        Joints.LHipYawPitch, Joints.RHipPitch, Joints.LShoulderRoll, Joints.RShoulderRoll,
                        Joints.LElbowYaw, Joints.RElbowYaw, Joints.RElbowRoll, Joints.LElbowRoll,
                        Joints.LWristYaw, Joints.RWristYaw, Joints.LHipRoll, Joints.LHipPitch, Joints.LKneePitch,
                        Joints.LAnklePitch, Joints.LAnkleRoll, Joints.RHipRoll, Joints.RHipPitch, Joints.RKneePitch,
                        Joints.RAnklePitch, Joints.RAnkleRoll]
    return Joints

