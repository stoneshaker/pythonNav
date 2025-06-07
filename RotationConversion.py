import numpy as np

#include "RotationConversion.h"
#include "CommonDefines.h"
#include "MathLibrary.h"

#---------------------------------------------------------------------------------------------

def FormQuaternionFromEulerAngles(EulerAttitude):

    cRoll    = np.cos(
         EulerAttitude['NAV_FRAME_EULER_ROLL'] / 2.0);
    sRoll    = np.sin(
         EulerAttitude['NAV_FRAME_EULER_ROLL'] / 2.0);
    cPitch   = np.cos(
         EulerAttitude['NAV_FRAME_EULER_PITCH'] / 2.0);
    sPitch   = np.sin(
         EulerAttitude['NAV_FRAME_EULER_PITCH'] / 2.0);
    cHeading = np.cos(
         EulerAttitude['NAV_FRAME_EULER_HEADING'] / 2.0);
    sHeading = np.sin(
         EulerAttitude['NAV_FRAME_EULER_HEADING'] / 2.0);


    result = {'s':(cRoll*cPitch*cHeading + sRoll*sPitch*sHeading),
      'i':(sRoll*cPitch*cHeading - cRoll*sPitch*sHeading),
      'j':(cRoll*sPitch*cHeading + sRoll*cPitch*sHeading),
      'k':(cRoll*cPitch*sHeading - sRoll*sPitch*cHeading)
      }
    return result

#---------------------------------------------------------------------------------------------


def FormDCMFromQuaternion(q):

   result = np.identity(3)
   ss = q['s'] * q['s'];
   ii = q['i'] * q['i'];
   jj = q['j'] * q['j'];
   kk = q['k'] * q['k'];

   jk = 2.0 * q['j'] * q['k'];
   ij = 2.0 * q['i'] * q['j'];
   ik = 2.0 * q['i'] * q['k'];
   si = 2.0 * q['s'] * q['i'];
   sj = 2.0 * q['s'] * q['j'];
   sk = 2.0 * q['s'] * q['k'];


   result[0][0] = ss + ii - jj - kk;
   result[0][1] = ij - sk;
   result[0][2] = ik + sj;
   result[1][0] = ij + sk;
   result[1][1] = ss - ii + jj - kk;
   result[1][2] = jk - si;
   result[2][0] = ik - sj;
   result[2][1] = jk + si;
   result[2][2] = ss - ii - jj + kk;
   
   return result


#---------------------------------------------------------------------------------------------

def ExtractEulerAnglesFromDCM(DCM):

   result = {'NAV_FRAME_EULER_ROLL' : np.arctan2(DCM[2][1],DCM[2][2]),
             'NAV_FRAME_EULER_PITCH' : -np.arcsin(DCM[2][0]),
             'NAV_FRAME_EULER_HEADING' : np.arctan2(DCM[1][0],DCM[0][0])
             }
   return result

#---------------------------------------------------------------------------------------------

def FormQuaternionUpdate(Increment):

   
   # Create a quaternion that represents a small Euler angle update.

   # Quaternion update defined as Ac + As*SigmaX i + As*SigmaY j +
   # As*SigmaZ k as in Weston
   result = {'s':1,'i':0,'j':0,'k':0}
   IncrementMagnitude = np.sqrt(
                  Increment[0] * Increment[0] +
                  Increment[1] * Increment[1] +
                  Increment[2] * Increment[2])


   if (IncrementMagnitude != 0.0):
      As = np.sin(IncrementMagnitude / 2.0)
      Ac = np.cos(IncrementMagnitude / 2.0)
      As /= IncrementMagnitude;

      result['s'] = Ac;
      result['i'] = As * Increment[0]
      result['j'] = As * Increment[1]
      result['k'] = As * Increment[2]
   else:
      result['s'] = 1.0
      result['i'] = 0.0
      result['j'] = 0.0
      result['k'] = 0.0
      
   return result

#---------------------------------------------------------------------------------------------

