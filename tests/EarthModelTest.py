#import numpy as np
import math

EARTH_ROTATION_RATE = 7.292115e-05
MAJOR_ECCENTRICITY_SQUARED = 6.69437999013779969797476e-3
SEMI_MAJOR_AXIS = 6378137.0

class EarthModel:
    def __init__(self):
        # Constants
        self.MajorEccentricitySquared = 0.0  # e^2
        self.SemiMajorAxis = 0.0  # metres
        self.EarthRotationRate = 0.0  # rads/s

        # Position dependant
        self.GravitationalAcceleration = 0.0  # metres per second squared
        self.MeridianRadiusOfCurvature = 0.0  # metres
        self.TransverseRadiusOfCurvature = 0.0  # metres
        self.MeanRadiusOfCurvature = 0.0  # metres
        
    def EarthModel_CalculateGravityAndEarthRadii(self, pPosition, SinLatitude):
        Sin2Latitude = math.sin(2 * pPosition['GEODETIC_POSITION_LATITUDE'])
        SinLatitudeSquared = SinLatitude * SinLatitude
        MajorEccentricitySinLatitudeSquared = self.MajorEccentricitySquared * SinLatitudeSquared
        Sin2LatitudeSquared = Sin2Latitude * Sin2Latitude
        Gravity_At_Zero_Meters_AMSL_At_Equator = 9.780318

        self.GravitationalAcceleration = Gravity_At_Zero_Meters_AMSL_At_Equator * (1 + 5.3024e-3 * SinLatitudeSquared - 5.9e-6 * Sin2LatitudeSquared) 
        Intermediate = 1 - MajorEccentricitySinLatitudeSquared
        self.MeridianRadiusOfCurvature = self.SemiMajorAxis * (1 - self.MajorEccentricitySquared) / math.sqrt(Intermediate * Intermediate * Intermediate)
        self.TransverseRadiusOfCurvature = self.SemiMajorAxis / math.sqrt(1 - MajorEccentricitySinLatitudeSquared)
        self.MeanRadiusOfCurvature = math.sqrt(self.MeridianRadiusOfCurvature * self.TransverseRadiusOfCurvature)


    def EarthModel_SetParametersToWGS84(self):
        self.EarthRotationRate = EARTH_ROTATION_RATE
        self.MajorEccentricitySquared = MAJOR_ECCENTRICITY_SQUARED
        self.SemiMajorAxis = SEMI_MAJOR_AXIS
	
def PositionComputation_Initialise(pStateData, pPosition):
    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'] = float(pPosition['GEODETIC_POSITION_LATITUDE'])
    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LONGITUDE'] = float(pPosition['GEODETIC_POSITION_LONGITUDE'])
    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] = float(pPosition['GEODETIC_POSITION_HEIGHT'])
    pStateData.Output.CosLatitude = math.cos(float(pPosition['GEODETIC_POSITION_LATITUDE']))
    pStateData.Output.SinLatitude = math.sin(float(pPosition['GEODETIC_POSITION_LATITUDE']))
    return pStateData
	
def PositionComputation_Run(pStateData, pInertialInput):
    # Do vertical position update
    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] -= \
        pInertialInput.TimeDelta * \
        pStateData.Output.NavFrameVelocity['NAV_FRAME_DOWN']

    # Update Horizontal position elements
    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'] += \
        pInertialInput.TimeDelta * \
        pStateData.Output.NavFrameVelocity['NAV_FRAME_NORTH'] / \
        (pStateData.Output.EarthModel.MeridianRadiusOfCurvature + \
         pStateData.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'])

    pStateData.Output.CosLatitude = \
        math.cos(float(pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE']))

    pStateData.Output.SinLatitude = \
        math.sin(float(pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE']))

    pStateData.Output.GeodeticPosition['GEODETIC_POSITION_LONGITUDE'] += \
        pInertialInput.TimeDelta * \
        pStateData.Output.NavFrameVelocity['NAV_FRAME_EAST'] / \
        ((pStateData.Output.EarthModel.TransverseRadiusOfCurvature + \
          pStateData.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT']) * \
         pStateData.Output.CosLatitude)
    return pStateData

import math

def EarthModel_CalculateGravityAndEarthRadii(pEarthModel, pPosition, SinLatitude):
    Sin2Latitude = math.sin(2 * pPosition['GEODETIC_POSITION_LATITUDE'])
    SinLatitudeSquared = SinLatitude * SinLatitude
    MajorEccentricitySinLatitudeSquared = pEarthModel.MajorEccentricitySquared * SinLatitudeSquared
    Sin2LatitudeSquared = Sin2Latitude * Sin2Latitude
    Gravity_At_Zero_Meters_AMSL_At_Equator = 9.780318

    pEarthModel.GravitationalAcceleration = Gravity_At_Zero_Meters_AMSL_At_Equator * (1 + 5.3024e-3 * SinLatitudeSquared - 5.9e-6 * Sin2LatitudeSquared) 
    Intermediate = 1 - MajorEccentricitySinLatitudeSquared
    pEarthModel.MeridianRadiusOfCurvature = pEarthModel.SemiMajorAxis * (1 - pEarthModel.MajorEccentricitySquared) / math.sqrt(Intermediate * Intermediate * Intermediate)
    pEarthModel.TransverseRadiusOfCurvature = pEarthModel.SemiMajorAxis / math.sqrt(1 - MajorEccentricitySinLatitudeSquared)
    pEarthModel.MeanRadiusOfCurvature = math.sqrt(pEarthModel.MeridianRadiusOfCurvature * pEarthModel.TransverseRadiusOfCurvature)
    return pEarthModel
	
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

pEarthModel = EarthModel()
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