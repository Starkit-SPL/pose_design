#from pose_design.interpolation import Head, LAnkle, RAnkle

# Data from: http://doc.aldebaran.com/2-8/family/nao_technical/joints_naov6.html

HeadYaw = [-119.52, -87.49, -62.45, -51.74, -43.32, -27.85, 0.0, 27.85, 43.32, 51.74, 62.45, 87.49, 119.52]
HeadPitchMin = [-25.73, -18.91, -24.64, -27.5, -31.4, -
                38.5, -38.5, -38.5, -31.4, -27.5, -24.64, -18.91, -25.73]
HeadPitchMax = [18.91, 11.46, 17.19, 18.91, 21.2, 24.18,
                29.51, 24.18, 21.2, 18.91, 17.19, 11.46, 18.91]
HeadPitchRange = [-38.5, 29.5]

LAnklePitch = [-68.15, -48.13, -40.11, -25.78, 5.73, 20.05, 52.87]
LAnkleRollMin = [-2.86, -10.31, -22.8, -22.8, -22.8, -22.8, 0.0]
LAnkleRollMax = [4.3, 9.74, 12.61, 44.06, 44.06, 31.54, 2.86]
LAnkleRollRange = [-22.79, 44.06]

RAnklePitch = [-68.15, -48.13, -40.11, -25.78, 5.73, 20.05, 52.87]
RAnkleRollMin = [-4.3, -9.74, -12.61, -44.06, -44.06, -31.54, -2.86]
RAnkleRollMax = [2.86, 10.31, 22.8, 22.8, 22.8, 22.8, 0.0]
RAnkleRollRange = [-44.06, 22.80]

jointsNames = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw',
               'LElbowRoll', 'LWristYaw', 'LHipYawPitch', 'LHipRoll', 'LHipPitch',
               'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch',
               'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll',
               'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'LHand', 'RHand']

startPositions = [5.835615623007271e-17, - 6.631492244224546e-09, 5.46203182238969e-07,
                  -3.0801841112426676e-12, 1.016807331666314e-07, -1.1239626473980024e-07,
                  3.875324239288602e-08, -3.6887286114506423e-07, 3.108395389972429e-07,
                  -4.951590426571784e-07, -5.005954903936072e-07, -5.14188116085279e-07,
                  -3.088730693434627e-07, -3.1083948215382406e-07, -4.951590995005972e-07,
                  -5.00595547237026e-07, -5.141881729286979e-07, 3.0887309776517213e-07,
                  5.46203182238969e-07, 3.0789979940659684e-12, -1.016807331666314e-07,
                  1.1239625763437289e-07, -3.875324239288602e-08, -2.2158275214678724e-11,
                  -2.2157387036259e-11
                  ]
startPositions[jointsNames.index('LElbowRoll')] = -4
startPositions[jointsNames.index('RElbowRoll')] = 3

jointsRanges = [
range( -112 ,  111 ),
range( -35 ,  27 ),
range( -115 ,  114 ),
range( -17 ,  72 ),
range( -115 ,  114 ),
range( -85 ,  -4 ),
range( -100 ,  99 ),
range( -63 ,  39 ),
range( -20 ,  42 ),
range( -85 ,  25 ),
range( -4 ,  116 ),
range( -65 ,  49 ),
range( -21 ,  41 ),
range( -43 ,  19 ),
range( -85 ,  25 ),
range( -4 ,  116 ),
range( -65 ,  49 ),
range( -42 ,  20 ),
range( -115 ,  114 ),
range( -73 ,  16 ),
range( -115 ,  114 ),
range( 3 ,  84 ),
range( -100 ,  99 ),
]

