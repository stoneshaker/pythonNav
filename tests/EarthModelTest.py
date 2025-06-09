import numpy as np
import math
import sys
sys.path.append('..\\')
import EarthModel

pPosition = {
  'GEODETIC_POSITION_LATITUDE': 27.975608*math.pi/180.0,
  'GEODETIC_POSITION_LONGITUDE': -82.382481*math.pi/180.0,
  'GEODETIC_POSITION_HEIGHT': 5
}
SinLatitude = math.sin(pPosition['GEODETIC_POSITION_LATITUDE'])

GeodeticPositionInit = {
  'GEODETIC_POSITION_LATITUDE': 0,
  'GEODETIC_POSITION_LONGITUDE': 0,
  'GEODETIC_POSITION_HEIGHT': 0
  }
  
NavFrameVelocityInit = {
  'NAV_FRAME_NORTH': 0,
  'NAV_FRAME_EAST':0,
  'NAV_FRAME_DOWN':0
  }

pEarthModel = EarthModel.EarthModel()
pEarthModel.EarthModel_SetParametersToWGS84()
pEarthModel.EarthModel_CalculateGravityAndEarthRadii(pPosition, SinLatitude)

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