from pose_design.interpolation import Head, LAnkle, RAnkle

Head, LAnkle, RAnkle = Head, LAnkle, RAnkle

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

jointsRanges = [range(-115, 115), range(-36, 28), range(-118, 118),
                 range(-17, 75), range(-118, 118), range(-87, -3), range(-103, 103),
                 range(-64, 41), range(-20, 44), range(-87, 26), range(-4, 120),
                 range(-67, 51), range(-21, 43), range(-44, 20), range(-87, 26),
                 range(-4, 120), range(-67, 51), range(-43, 21), range(-118, 118),
                 range(-75, 17), range(-118, 118), range(3, 87), range(-103, 103)]