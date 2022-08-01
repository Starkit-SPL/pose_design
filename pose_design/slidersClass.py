from pose_design.constants import jointsNames, jointsRanges, startPositions
from pose_design.interpolation import Head, RAnkle, LAnkle
from pose_design.myWidgets import Point, Limits


def _makeSliders(cls):
    class Sliders:
        SldHeadYaw = cls.makeSlider(name='HeadYaw', point=Point(275, 25),
                                    sldRange=jointsRanges[jointsNames.index(
                                        'HeadYaw')],
                                    startValue=startPositions[jointsNames.index('HeadYaw')])
        SldHeadPitch = cls.makeSlider(name='HeadPitch', point=Point(275, 135),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'HeadPitch')],
                                      startValue=startPositions[jointsNames.index(
                                          'HeadPitch')],
                                      limits=Limits(slider=SldHeadYaw, limit=Head))
        SldLShoulderPitch = cls.makeSlider(name='LShoulderPitch', point=Point(50, 245),
                                           sldRange=jointsRanges[jointsNames.index(
                                               'LShoulderPitch')],
                                           startValue=startPositions[jointsNames.index('LShoulderPitch')])
        SldRShoulderPitch = cls.makeSlider(name='RShoulderPitch', point=Point(450, 245),
                                           sldRange=jointsRanges[jointsNames.index(
                                               'RShoulderPitch')],
                                           startValue=startPositions[jointsNames.index('RShoulderPitch')])
        SldLHipYawPitch = cls.makeSlider(name='LHipYawPitch', point=Point(260, 245),
                                         sldRange=jointsRanges[jointsNames.index(
                                             'LHipYawPitch')],
                                         startValue=startPositions[jointsNames.index('LHipYawPitch')])
        SldLShoulderRoll = cls.makeSlider(name='LShoulderRoll', point=Point(50, 355),
                                          sldRange=jointsRanges[jointsNames.index(
                                              'LShoulderRoll')],
                                          startValue=startPositions[jointsNames.index('LShoulderRoll')])
        SldRShoulderRoll = cls.makeSlider(name='RShoulderRoll', point=Point(450, 355),
                                          sldRange=jointsRanges[jointsNames.index(
                                              'RShoulderRoll')],
                                          startValue=startPositions[jointsNames.index('RShoulderRoll')])
        SldLElbowYaw = cls.makeSlider(name='LElbowYaw', point=Point(50, 465),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'LElbowYaw')],
                                      startValue=startPositions[jointsNames.index('LElbowYaw')])
        SldRElbowYaw = cls.makeSlider(name='RElbowYaw', point=Point(450, 465),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'RElbowYaw')],
                                      startValue=startPositions[jointsNames.index('RElbowYaw')])
        SldLElbowRoll = cls.makeSlider(name='LElbowRoll', point=Point(50, 575),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'LElbowRoll')],
                                       startValue=startPositions[jointsNames.index('LElbowRoll')])
        SldRElbowRoll = cls.makeSlider(name='RElbowRoll', point=Point(450, 575),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'RElbowRoll')],
                                       startValue=startPositions[jointsNames.index('RElbowRoll')])
        SldLWristYaw = cls.makeSlider(name='LWristYaw', point=Point(50, 685),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'LWristYaw')],
                                      startValue=startPositions[jointsNames.index('LWristYaw')])
        SldRWristYaw = cls.makeSlider(name='RWristYaw', point=Point(450, 685),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'RWristYaw')],
                                      startValue=startPositions[jointsNames.index('RWristYaw')])
        SldLHipRoll = cls.makeSlider(name='LHipRoll', point=Point(200, 355),
                                     sldRange=jointsRanges[jointsNames.index(
                                         'LHipRoll')],
                                     startValue=startPositions[jointsNames.index('LHipRoll')])
        SldLHipPitch = cls.makeSlider(name='LHipPitch', point=Point(200, 465),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'LHipPitch')],
                                      startValue=startPositions[jointsNames.index('LHipPitch')])
        SldLKneePitch = cls.makeSlider(name='LKneePitch', point=Point(200, 575),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'LKneePitch')],
                                       startValue=startPositions[jointsNames.index('LKneePitch')])
        SldLAnklePitch = cls.makeSlider(name='LAnklePitch', point=Point(200, 685),
                                        sldRange=jointsRanges[jointsNames.index(
                                            'LAnklePitch')],
                                        startValue=startPositions[jointsNames.index('LAnklePitch')])
        SldLAnkleRoll = cls.makeSlider(name='LAnkleRoll', point=Point(200, 795),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'LAnkleRoll')],
                                       startValue=startPositions[jointsNames.index(
                                           'LAnkleRoll')],
                                       limits=Limits(slider=SldLAnklePitch, limit=LAnkle))
        SldRHipRoll = cls.makeSlider(name='RHipRoll', point=Point(325, 355),
                                     sldRange=jointsRanges[jointsNames.index(
                                         'RHipRoll')],
                                     startValue=startPositions[jointsNames.index('RHipRoll')])
        SldRHipPitch = cls.makeSlider(name='RHipPitch', point=Point(325, 465),
                                      sldRange=jointsRanges[jointsNames.index(
                                          'RHipPitch')],
                                      startValue=startPositions[jointsNames.index('RHipPitch')])
        SldRKneePitch = cls.makeSlider(name='RKneePitch', point=Point(325, 575),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'RKneePitch')],
                                       startValue=startPositions[jointsNames.index('RKneePitch')])
        SldRAnklePitch = cls.makeSlider(name='RAnklePitch', point=Point(325, 685),
                                        sldRange=jointsRanges[jointsNames.index(
                                            'RAnklePitch')],
                                        startValue=startPositions[jointsNames.index('RAnklePitch')])
        SldRAnkleRoll = cls.makeSlider(name='RAnkleRoll', point=Point(325, 795),
                                       sldRange=jointsRanges[jointsNames.index(
                                           'RAnkleRoll')],
                                       startValue=startPositions[jointsNames.index(
                                           'RAnkleRoll')],
                                       limits=Limits(slider=SldRAnklePitch, limit=RAnkle))

    return Sliders
