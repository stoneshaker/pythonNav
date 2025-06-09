import numpy as np
import math
import sys
sys.path.append('..\\')
import NEF
import EarthModel
inert = NEF.NEFInertialInputData()
print(inert.TimeOfValidity)
print(inert.BodyAngularRates)
NEFout = NEF.NEFOutput()
print(NEFout.EulerAngleAttitude)
0.0
{'BODY_FRAME_X': 0, 'BODY_FRAME_Y': 0, 'BODY_FRAME_Z': 0}
{'NAV_FRAME_EULER_ROLL': 0.0, 'NAV_FRAME_EULER_PITCH': 0.0, 'NAV_FRAME_EULER_HEADING': 0.0}
pPosition = {
  'GEODETIC_POSITION_LATITUDE': 27.975608*math.pi/180.0,
  'GEODETIC_POSITION_LONGITUDE': -82.382481*math.pi/180.0,
  'GEODETIC_POSITION_HEIGHT': 5
}

pVelocity = {
  'NAV_FRAME_NORTH': 0,
  'NAV_FRAME_EAST':0,
  'NAV_FRAME_DOWN':0
  }

pAttitude = {
    'NAV_FRAME_EULER_ROLL':0.0,
    'NAV_FRAME_EULER_PITCH':90.0*np.pi/180.0,
    'NAV_FRAME_EULER_HEADING':0.0
    }
pInitial = {
    'TimeOfValidity':1024.0,
    'Position':pPosition,
    'Velocity':pVelocity,
    'Attitude':pAttitude
    }
pEarthModel = EarthModel.EarthModel()
pEarthModel.EarthModel_SetParametersToWGS84()
pEarthModel.EarthModel_CalculateGravityAndEarthRadii(pPosition, np.sin(pPosition['GEODETIC_POSITION_LATITUDE']))
GeodeticPositionInit =  {
    'GEODETIC_POSITION_LATITUDE': 0.0,
    'GEODETIC_POSITION_LONGITUDE': 0.0,
    'GEODETIC_POSITION_HEIGHT': 0.0
    }

NavFrameVelocityInit = {
  'NAV_FRAME_NORTH':0.0,
  'NAV_FRAME_EAST':0.0,
  'NAV_FRAME_DOWN':0.0
  }

OutputInit = {
  'GeodeticPosition':GeodeticPositionInit,
  'CosLatitude':1.0,
  'SinLatitude':0.0,
  'NavFrameVelocity':NavFrameVelocityInit,
  'EarthModel':pEarthModel
  } 
  
#pStateData = {
#  ['Output']:OutputInit
#  }

print(pEarthModel.TransverseRadiusOfCurvature)
print(OutputInit['EarthModel'].TransverseRadiusOfCurvature)

state = NEF.NEFStateData()
 

 