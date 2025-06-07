import quaternion
import RotationConversion
import numpy as np

testQ2 = { 's':1,'i':2,'j':3,'k':4}
testQ = { 's':0.707106781,'i':0,'j':0,'k':0.707106781}
q1 = quaternion.Quaternion_RotationIdentity()
q2 = quaternion.sQuaternion_Copy(q1)
q3 = quaternion.Quaternion_Multiply(testQ,q1)
q4 = quaternion.Quaternion_Normalize(testQ2)

print(q1)
print(q2)
print(q3)
print(q4)
print('Rotation Tests')

dcm = RotationConversion.FormDCMFromQuaternion(testQ)
euler = RotationConversion.ExtractEulerAnglesFromDCM(dcm)
testEuler = {'NAV_FRAME_EULER_ROLL':0.0,
             'NAV_FRAME_EULER_PITCH':90.0*np.pi/180.0,
             'NAV_FRAME_EULER_HEADING':0.0
             }
qq = RotationConversion.FormQuaternionFromEulerAngles(testEuler)

print(dcm)
print(euler)
print(qq)