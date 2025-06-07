import numpy as np
import math
import EarthModel
import RotationConversion
#include "NEF.h"
#include "CommonDefines.h"
#include "RotationConversion.h"
#include "VectorTypes.h"
#include "MathLibrary.h"

SIMPLE_GRAVITY               = 9.80665
QBOOST_ACC_THRESHOLD         = 1.5*SIMPLE_GRAVITY
VEL_Q_BOOST_FACTOR           = 500.0
VEL_Q_BOOST_UPPER_LIMIT      = 1000.0

DEG2RAD                      = np.pi/180
QBOOST_RATE_THRESHOLD        = (10*DEG2RAD)    
ATTITUDE_Q_BOOST_FACTOR      = 50.0
ATTITUDE_Q_BOOST_UPPER_LIMIT = 1000.0


#float NorthVelQBoost
#float EastVelQBoost
#float DownVelQBoost

#float BodyX_AttitudeQ_Boost
#float BodyY_AttitudeQ_Boost
#float BodyZ_AttitudeQ_Boost

# Inertial input interface
class NEFInertialInputData:
   # True or False indicates the validity of this inertial data.
   self.DataValid = False

   # Units - seconds, this time represents the end of
   # the IMU measurement period.
   self.TimeOfValidity = 0.0

   # Units - seconds, this time represents the duration
   # of the IMU measurement period. The rates and
   # accelerations that are measured are assumed to be
   # the average rates and accelerations over this period.
   #short_float TimeDelta
   self.TimeDelta = 0.0

   # Body Frame Vector,
   #   BODY_FRAME_X = radians per second
   #   BODY_FRAME_Y = radians per second
   #   BODY_FRAME_Z = radians per second
   self.BodyAngularRates = {
     'BODY_FRAME_X':0,
     'BODY_FRAME_Y':0,
     'BODY_FRAME_Z':0
     }

   # Body Frame Vector,
   #   BODY_FRAME_X = metres per second squared
   #   BODY_FRAME_Y = metres per second squared
   #   BODY_FRAME_Z = metres per second squared
   self.BodyAccelerations = {
     'BODY_FRAME_X':0,
     'BODY_FRAME_Y':0,
     'BODY_FRAME_Z':0
     }
   
# Output interface
class NEFOutput:
   # units - seconds
   self.TimeOfValidity = 0.0

   # units - trig ratio (-1..1)
   self.CosLatitude = 1.0

   # units - trig ratio (-1..1)
   self.SinLatitude = 0.0

   # Geodetic Position Vector,
   #   GEODETIC_POSITION_LATITUDE  = radians,
   #   GEODETIC_POSITION_LONGITUDE = radians,
   #   GEODETIC_POSITION_HEIGHT    = metres
   self.GeodeticPosition = {
     'GEODETIC_POSITION_LATITUDE':0.0,
     'GEODETIC_POSITION_LONGITUDE':0.0,
     'GEODETIC_POSITION_HEIGHT':0.0
     }

   # Nav Frame Vector,
   #   NAV_FRAME_NORTH = metres per second
   #   NAV_FRAME_EAST  = metres per second
   #   NAV_FRAME_DOWN  = metres per second
   self.NavFrameVelocity = {
     'NAV_FRAME_NORTH':0.0,
     'NAV_FRAME_EAST':0.0,
     'NAV_FRAME_DOWN':0.0
     }

   # Nav Frame Euler Vector,
   #   NAV_FRAME_EULER_ROLL    = radians
   #   NAV_FRAME_EULER_PITCH   = radians
   #   NAV_FRAME_EULER_HEADING = radians
   self.EulerAngleAttitude = {
     'NAV_FRAME_EULER_ROLL':0.0,
     'NAV_FRAME_EULER_PITCH':0.0,
     'NAV_FRAME_EULER_HEADING':0.0
     }

   # Direction Cosine Matrix
   self.BodyToNavFrameDCM = np.identity(3)

   # Body Frame Vector,
   #   BODY_FRAME_X = radians per second
   #   BODY_FRAME_Y = radians per second
   #   BODY_FRAME_Z = radians per second
   self.RawAngularRates = {
     'BODY_FRAME_X':0.0,
     'BODY_FRAME_Y':0.0,
     'BODY_FRAME_Z':0.0
     }

   # Body Frame Vector,
   #   BODY_FRAME_X = radians per second
   #   BODY_FRAME_Y = radians per second
   #   BODY_FRAME_Z = radians per second
   self.CorrectedAngularRates = {
     'BODY_FRAME_X':0.0,
     'BODY_FRAME_Y':0.0,
     'BODY_FRAME_Z':0.0
     }

   # Body Frame Vector,
   #   BODY_FRAME_X = metres per second squared
   #   BODY_FRAME_Y = metres per second squared
   #   BODY_FRAME_Z = metres per second squared
   self.RawAccelerometerReadings = {
     'BODY_FRAME_X':0.0,
     'BODY_FRAME_Y':0.0,
     'BODY_FRAME_Z':0.0
     }

   # Nav Frame Vector,
   #   NAV_FRAME_NORTH = metres per second squared
   #   NAV_FRAME_EAST  = metres per second squared
   #   NAV_FRAME_DOWN  = metres per second squared
   self.CorrectedNavFrameAccelerations = {
     'NAV_FRAME_NORTH':0.0,
     'NAV_FRAME_EAST':0.0,
     'NAV_FRAME_DOWN':0.0
     }

   self.EarthModel = EarthModel()
   
class NEFStateData:
  def __init__(self):
   # To save space the output data also represents state data.
   # The output data should be copied before being used by
   # processes other than the NEF.
    self.Output = NEFOutput()
    self.BodyToNavQuaternionAttitude = np.quaternion(1,0,0,0)
    self.NavToBodyFrameDCM = np.identity(3)
    self.GeodeticPosition = GeodeticPositionInit
    self.TransportRate = [0] * 3
    self.EarthRate = [0] * 3
    self.BodyFrameRate = [0] * 3
    self.LastInertialInputValid = False
    self.CosLatitude = 1.0
    self.SinLatitude = 0.0
    self.NavFrameVelocity = NavFrameVelocityInit
    self.EarthModel = pEarthModel
    self.LastValidInertialInput = NEFInertialInputData()
    self.AccelerationLimit = 0.0
    self.AngularRateLimit = 0.0

#---------------------
# Position Computation
#---------------------

  def PositionComputation_Initialize(self, pPosition):

   # Initialize the vector position
   self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'] = pPosition['GEODETIC_POSITION_LATITUDE']

   self.Output.GeodeticPosition['GEODETIC_POSITION_LONGITUDE'] = pPosition['GEODETIC_POSITION_LONGITUDE']

   self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] = pPosition['GEODETIC_POSITION_HEIGHT']

   self.Output.CosLatitude = np.cos(pPosition['GEODETIC_POSITION_LATITUDE'])
   self.Output.SinLatitude = np.sin(pPosition['GEODETIC_POSITION_LATITUDE'])


  def PositionComputation_Run(self, pInertialInput):

   # Do vertical position update
   self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] -= \
      pInertialInput.TimeDelta * \
            self.Output.NavFrameVelocity['NAV_FRAME_DOWN']

   # Update Horizontal position elements
   self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'] += \
      pInertialInput.TimeDelta * \
            self.Output.NavFrameVelocity['NAV_FRAME_NORTH'] / \
       (self.Output.EarthModel.MeridianRadiusOfCurvature + \
        self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'])

   self.Output.CosLatitude = \
      np.cos(self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'])

   self.Output.SinLatitude = \
      np.sin(self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'])

   self.Output.GeodeticPosition['GEODETIC_POSITION_LONGITUDE'] += \
      pInertialInput.TimeDelta * \
          self.Output.NavFrameVelocity['NAV_FRAME_EAST'] / \
     ((self.Output.EarthModel.TransverseRadiusOfCurvature + \
       self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT']) * \
          self.Output.CosLatitude)


  def PositionComputation_AddCorrections(self, pCorrections):

   # Add the Position correction from the Navigation Filter if it is valid
   if (pCorrections.bOneOffCorrectionsValid):
   
      self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'] += \
         pCorrections.GeodeticPositionCorrection['GEODETIC_POSITION_LATITUDE']

      self.Output.GeodeticPosition['GEODETIC_POSITION_LONGITUDE'] += \
         pCorrections.GeodeticPositionCorrection['GEODETIC_POSITION_LONGITUDE']

      self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] += \
         pCorrections.GeodeticPositionCorrection['GEODETIC_POSITION_HEIGHT']

      self.Output.CosLatitude =  \
         np.cos(self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'])

      self.Output.SinLatitude =  \
         np.sin(self.Output.GeodeticPosition['GEODETIC_POSITION_LATITUDE'])
   
#--------------------
# Velocity Processing
#--------------------

  def VelocityProcessing_FrameRateComputation(self):

   # Calculate Transport Rate
   self.TransportRate['NAV_FRAME_NORTH'] = \
      self.Output.NavFrameVelocity['NAV_FRAME_EAST'] / \
      (self.Output.EarthModel.TransverseRadiusOfCurvature + \
        self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'])

   self.TransportRate['NAV_FRAME_EAST'] = \
      -self.Output.NavFrameVelocity['NAV_FRAME_NORTH'] / \
      (self.Output.EarthModel.MeridianRadiusOfCurvature + \
        self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'])

   self.TransportRate['NAV_FRAME_DOWN'] =  \
      -self.Output.NavFrameVelocity['NAV_FRAME_EAST'] * \
      self.Output.SinLatitude / (self.Output.CosLatitude *  \
      (self.Output.EarthModel.TransverseRadiusOfCurvature + \
         self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT']))

   # Resolve Earth Rate Correction to Geographic Frame
   self.EarthRate['NAV_FRAME_NORTH'] =  \
      self.Output.EarthModel.EarthRotationRate * \
                              self.Output.CosLatitude

   self.EarthRate['NAV_FRAME_EAST'] = 0.0

   self.EarthRate['NAV_FRAME_DOWN'] = \
      -self.Output.EarthModel.EarthRotationRate * \
                              self.Output.SinLatitude

   # Determine body frame rates
   NavFrameRate = self.EarthRate + self.TransportRate

   self.BodyFrameRate = np.matmul(self.NavToBodyFrameDCM, NavFrameRate)


  def VelocityProcessing_Initialize(self, Velocity):

    self.Output.NavFrameVelocity = Velocity

    VelocityProcessing_FrameRateComputation(pStateData)


  def VelocityProcessing_VelocityComputation(
                  self, pInertialInput, pCorrections):
   #struct s3dVector GravitationalAcceleration
   #struct s3dVector NavFrameCoriolisAcceleration
   #struct s3dVector CorrectedBodyFrameAcceleration
   #struct s3dVector CorrectedNavFrameAcceleration
   #struct s3dVector NavFrameVelocityIncrement
   #struct s3dVector NavFrameCoriolis
   #struct s3dVector RotationCorrection

   #float NorthAccAbs
   #float EastAccAbs
   #float DownAccAbs


   # Calculate Acceleration due to Gravity at current height - this
   # is simplified, the error is accounted for in the navigation filter.
   # short_float HeightRatioSqd = 1.0
   # should be... (full version used at the moment)
   HeightRatio = 1.0 + \
      (self.Output.GeodeticPosition['GEODETIC_POSITION_HEIGHT'] / \
               self.Output.EarthModel.MeanRadiusOfCurvature)

   HeightRatioSqd = HeightRatio * HeightRatio


   GravitationalAcceleration['NAV_FRAME_NORTH'] = 0.0

   GravitationalAcceleration['NAV_FRAME_EAST'] = 0.0

   GravitationalAcceleration['NAV_FRAME_DOWN'] = \
      self.Output.EarthModel.GravitationalAcceleration / HeightRatioSqd
   
   # Calculate Coriolis Correction
   NavFrameCoriolis = self.EarthRate * 2.0
   NavFrameCoriolis = self.TransportRate + NavFrameCoriolis

   NavFrameCoriolisAcceleration = np.cross(NavFrameCoriolis, \
               self.Output.NavFrameVelocity)

   # Add the Accelerometer Bias correction to the inertial accelerations
   # if it is valid.
   CorrectedBodyFrameAcceleration = pInertialInput.BodyAccelerations

   if (pCorrections.bContinuousCorrectionsValid):
      CorrectedBodyFrameAcceleration = CorrectedBodyFrameAcceleration + \
               pCorrections.AccelerometerBiasCorrection

   # Calculate and apply the rotation correction.
   RotationCorrection = s3dVector_CrossProduct( 
                              self.Output.CorrectedAngularRates,
                              CorrectedBodyFrameAcceleration)

   RotationCorrection = RotationCorrection * 0.5 * pInertialInput.TimeDelta

   CorrectedBodyFrameAcceleration = CorrectedBodyFrameAcceleration - RotationCorrection

   # Resolve corrected body frame accelerations in to geographic frame.
   CorrectedNavFrameAcceleration = np.matmul(self.Output.BodyToNavFrameDCM, \
                                             CorrectedBodyFrameAcceleration)

   # Add the Coriolis and Gravity terms to the Nav Frame Acceleration.
   CorrectedNavFrameAcceleration = CorrectedNavFrameAcceleration + GravitationalAcceleration
   
   #/*****************************************************************/
   #/* get the acc magnitudes - avoid using fabs() function because  */
   #/* it doesn't work properly!                                     */
   #/*****************************************************************/   

   NorthAccAbs = CorrectedNavFrameAcceleration['NAV_FRAME_NORTH']
     
   if (NorthAccAbs < 0):
      NorthAccAbs = -NorthAccAbs

   EastAccAbs = CorrectedNavFrameAcceleration['NAV_FRAME_EAST']
   
   if (EastAccAbs < 0):
      EastAccAbs = -EastAccAbs

   DownAccAbs = CorrectedNavFrameAcceleration['NAV_FRAME_DOWN']
   
   if (DownAccAbs < 0):
      DownAccAbs = -DownAccAbs

   #/*---------------------------------------------------------------*/

   #/**************************************************************/
   #/* if above the acc threshold apply proportional Q boost,     */
   #/* subject to an upper limit                                  */
   #/**************************************************************/

   # North axis                     
   if (NorthAccAbs > QBOOST_ACC_THRESHOLD):
      NorthVelQBoost = (NorthAccAbs/QBOOST_ACC_THRESHOLD)*VEL_Q_BOOST_FACTOR
   
      #NorthVelQBoost = 1.0  uncomment to turn Q boost off
 
      if (NorthVelQBoost > VEL_Q_BOOST_UPPER_LIMIT):
         NorthVelQBoost = VEL_Q_BOOST_UPPER_LIMIT

   else:
         NorthVelQBoost = 1.0
      
   # East axis
   if (EastAccAbs > QBOOST_ACC_THRESHOLD):
      EastVelQBoost = (EastAccAbs/QBOOST_ACC_THRESHOLD)*VEL_Q_BOOST_FACTOR
   
      #EastVelQBoost = 1.0  uncomment to turn Q boost off

      if (EastVelQBoost > VEL_Q_BOOST_UPPER_LIMIT):
         EastVelQBoost = VEL_Q_BOOST_UPPER_LIMIT

   else:
      EastVelQBoost = 1.0
   
   # Down axis      
   if (DownAccAbs > QBOOST_ACC_THRESHOLD):
      DownVelQBoost = (DownAccAbs/QBOOST_ACC_THRESHOLD)*VEL_Q_BOOST_FACTOR
      
      #DownVelQBoost = 1.0  uncomment to turn Q boost off
      
      if (DownVelQBoost > VEL_Q_BOOST_UPPER_LIMIT):
         DownVelQBoost = VEL_Q_BOOST_UPPER_LIMIT

   else:
      DownVelQBoost = 1.0
        
   CorrectedNavFrameAcceleration = CorrectedNavFrameAcceleration - NavFrameCoriolisAcceleration

   # Save the corrected nav frame acceleration.
   self.Output.CorrectedNavFrameAccelerations = CorrectedNavFrameAcceleration

   # Turn the corrected Nav Frame Accelerations into a velocity increment.
   NavFrameVelocityIncrement = CorrectedNavFrameAcceleration * pInertialInput.TimeDelta

   # Integrate the Nav Frame Velocity.
   self.Output.NavFrameVelocity = self.Output.NavFrameVelocity + NavFrameVelocityIncrement

   # Copy the raw accelerations to the output interface.
   self.Output.RawAccelerometerReadings = pInertialInput.BodyAccelerations


  def VelocityProcessing_Run(
                  self,
                  const struct sNEFInertialInputData* pInertialInput,
                  const struct sNEFContinuousCorrections* pCorrections)

   VelocityProcessing_VelocityComputation(
                                    pStateData,
                                    pInertialInput,
                                    pCorrections)
   VelocityProcessing_FrameRateComputation(pStateData)


  def VelocityProcessing_AddCorrections(self, pCorrections)

   # Add the Geographic Velocity correction from the Navigation Filter.
   if (pCorrections.bOneOffCorrectionsValid):
      self.Output.NavFrameVelocity = self.Output.NavFrameVelocity + \
                                     pCorrections.NavFrameVelocityCorrection

      VelocityProcessing_FrameRateComputation(pStateData)


#----------
# Strapdown
#----------

  def Strapdown_AttitudeChange(self):
    # Normalize the attitude quaternion
    Quaternion_Normalize(self.BodyToNavQuaternionAttitude)

   # Update the Direction Cosine Matrices.
    self.Output.BodyToNavFrameDCM = FormDCMFromQuaternion(self.BodyToNavQuaternionAttitude)
               )

    self.NavToBodyFrameDCM = np.transpose(self.Output.BodyToNavFrameDCM)

   # Update the Euler Angles
    self.Output.EulerAngleAttitude = ExtractEulerAnglesFromDCM(self.Output.BodyToNavFrameDCM)
}

  def Strapdown_Initialize(self, pAttitude):
    FormQuaternionFromEulerAngles(
               pAttitude,
               self.BodyToNavQuaternionAttitude)

    Strapdown_AttitudeChange(pStateData)

  def Strapdown_Run(self, pInertialInput, pCorrections):

    #struct s3dVector GyroRate
    #struct s3dVector CorrectedGyroIncrements
    #struct sQuaternion QuaternionUpdate
    #struct sQuaternion NewAttitudeQuaternion

    #float XRateAbs
    #float YRateAbs
    #float ZRateAbs

    # Write the raw body rate to the Output data.
    self.Output.RawAngularRates = pInertialInput.BodyAngularRates

    # Add the gyro bias correction if it is valid.
    GyroRate = pInertialInput.BodyAngularRates

    if (pCorrections.bContinuousCorrectionsValid):
        pCorrections.GyroDriftCorrection + GyroRate

   #/*****************************************************************/
   #/* get the rate magnitudes - avoid using fabs() function because */
   #/* it doesn't work properly!                                     */
   #/*****************************************************************/
   
   XRateAbs = GyroRate['BODY_FRAME_X']

   if(XRateAbs < 0):
      XRateAbs = -XRateAbs

   YRateAbs = GyroRate['BODY_FRAME_Y']

   if(YRateAbs < 0):
      YRateAbs = -YRateAbs

   ZRateAbs = GyroRate['BODY_FRAME_Z']

   if(ZRateAbs < 0):
      ZRateAbs = -ZRateAbs

   #/*------------------------------------------------------------*/

   #/**************************************************************/
   #/* if above the rate threshold apply proportional Q boost,    */
   #/* subject to an upper limit                                  */
   #/**************************************************************/

   # X axis
   if (XRateAbs > QBOOST_RATE_THRESHOLD):
      BodyX_AttitudeQ_Boost = (XRateAbs/QBOOST_RATE_THRESHOLD)*ATTITUDE_Q_BOOST_FACTOR
      
      #BodyX_AttitudeQ_Boost = 1.0  uncomment to turn Q boost off

      if(BodyX_AttitudeQ_Boost > ATTITUDE_Q_BOOST_UPPER_LIMIT):
         BodyX_AttitudeQ_Boost = ATTITUDE_Q_BOOST_UPPER_LIMIT

   else:
      BodyX_AttitudeQ_Boost = 1.0

   # Y axis
   if (YRateAbs > QBOOST_RATE_THRESHOLD):
   
      BodyY_AttitudeQ_Boost = (YRateAbs/QBOOST_RATE_THRESHOLD)*ATTITUDE_Q_BOOST_FACTOR
      
      #BodyY_AttitudeQ_Boost = 1.0 uncomment to turn Q boost off
      
      if(BodyY_AttitudeQ_Boost > ATTITUDE_Q_BOOST_UPPER_LIMIT):
      
         BodyY_AttitudeQ_Boost = ATTITUDE_Q_BOOST_UPPER_LIMIT
      
   

   else:
   
      BodyY_AttitudeQ_Boost = 1.0
   

   # Z axis
   if (ZRateAbs > QBOOST_RATE_THRESHOLD):
   
      BodyZ_AttitudeQ_Boost = (ZRateAbs/QBOOST_RATE_THRESHOLD)*ATTITUDE_Q_BOOST_FACTOR
      
      #BodyZ_AttitudeQ_Boost = 1.0 uncomment to turn Q boost off
      
      if(BodyZ_AttitudeQ_Boost > ATTITUDE_Q_BOOST_UPPER_LIMIT):
      
         BodyZ_AttitudeQ_Boost = ATTITUDE_Q_BOOST_UPPER_LIMIT
      
   

   else:
      BodyZ_AttitudeQ_Boost = 1.0


   # Subtract the frame rate adjustment.
   GyroRate = GyroRate - self.BodyFrameRate

   # Write the corrected rate to the output data.
   self.Output.CorrectedAngularRates = GyroRate

   # Turn corrected rates into increments.
   CorrectedGyroIncrements = GyroRate * pInertialInput.TimeDelta

   # Integrate the quaternion for Body to Nav Frame conversion
   FormQuaternionUpdate(
               CorrectedGyroIncrements,
               QuaternionUpdate)

   Quaternion_Multiply(
               self.BodyToNavQuaternionAttitude,
               QuaternionUpdate,
               NewAttitudeQuaternion)

   self.BodyToNavQuaternionAttitude = NewAttitudeQuaternion

   # Update the Attitude members to reflect new attitude
   Strapdown_AttitudeChange(pStateData)
}

  def Strapdown_AddCorrections(self, pCorrections)
   #struct sQuaternion NavFrameMisalignmentQuaternionUpdate
   #struct sQuaternion NewAttitudeQuaternion
   #struct s3dVector NavFrameMisalignmentCorrection

   
   NavFrameMisalignmentCorrection = pCorrections.NavFrameMisalignmentCorrection

   NavFrameMisalignmentCorrection = NavFrameMisalignmentCorrection * -1.0
 
   # Apply geographic misalignment corrections from Navigation Filter
   # if they are valid.
   if (pCorrections.bOneOffCorrectionsValid):
      FormQuaternionUpdate(
                  NavFrameMisalignmentCorrection,
                  NavFrameMisalignmentQuaternionUpdate)

      Quaternion_Multiply(
                  NavFrameMisalignmentQuaternionUpdate,
                  self.BodyToNavQuaternionAttitude,
                  NewAttitudeQuaternion)

      self.BodyToNavQuaternionAttitude = NewAttitudeQuaternion

      # Push change through to other state data.
      Strapdown_AttitudeChange(self)

#---------------------------------------------------
# Public functions - Constructors and Copy routines.
#---------------------------------------------------
#---------------------------------------------------

#--------------
# Copy Routines
#--------------

def sNEFOutput_Copy(
            const struct sNEFOutput* pSource,
            struct sNEFOutput* pDestination)
{
   pDestination = pSource
}

def sNEFContinuousCorrections_Copy(
            const struct sNEFContinuousCorrections* pSource,
            struct sNEFContinuousCorrections* pDestination)

   pDestination = pSource

#------------------------------
# Public Functions - Operations
#------------------------------
#------------------------------

def NEF_Constructor(self):
    EarthModel_SetParametersToWGS84(&self.Output.EarthModel)

    # Set limits on inertial data.
    self.AccelerationLimit = 100.0 # default to 10g
    self.AngularRateLimit = 2.0    # default to 360 degrees a second


#---------------------------------------------------------------------------------------------

def NEF_EarthModelSetParameters(self):
    EarthModel_SetParametersToWGS84(
                  self.Output.EarthModel)

#---------------------------------------------------------------------------------------------

def NEF_Initialize(self, pInitializationData):
   PositionComputation_Initialize(
                  self,
                  pInitializationData.Position)

   EarthModel_CalculateGravityAndEarthRadii(
                  self.Output.EarthModel,
                  self.Output.GeodeticPosition,
                  self.Output.SinLatitude)

   Strapdown_Initialize(
                  self,
                  &pInitializationData.Attitude)

   VelocityProcessing_Initialize(
                  self,
                  pInitializationData.Velocity)

   self.Output.TimeOfValidity = pInitializationData.TimeOfValidity

   # Initialize the last valid inertial input
   self.LastInertialInputValid = False
   self.LastValidInertialInput.TimeOfValidity =
                           pInitializationData.TimeOfValidity

#---------------------------------------------------------------------------------------------

def NEF_SetInertialLimits(self):

   self.AccelerationLimit = 300.0 # m/s/s
   self.AngularRateLimit =  157.0 # rads/s

#---------------------------------------------------------------------------------------------

def NEF_NavigationUpdate(self, pInertialInput, pCorrections)

    Spike = False


   EarthModel_CalculateGravityAndEarthRadii(
                  self.Output.EarthModel,
                  self.Output.GeodeticPosition,
                  self.Output.SinLatitude)

   # Check the validity of the inertial input...

   # Do the spike check
   for (i = 0 i < 3 i++):
      if ((sf_abs(pInertialInput.BodyAccelerations[i]) >
                                       self.AccelerationLimit) ||
          (sf_abs(pInertialInput.BodyAngularRates[i]) >
                                       self.AngularRateLimit)):
          Spike = True

   # Only update the inertial data if it is valid
   if ((Spike == False) && (pInertialInput.DataValid == True)):
      s3dVector_Copy(pInertialInput.BodyAccelerations,
               self.LastValidInertialInput.BodyAccelerations)

      s3dVector_Copy(
               pInertialInput.BodyAngularRates,
               self.LastValidInertialInput.BodyAngularRates)

      self.LastInertialInputValid = True

   self.LastValidInertialInput.TimeDelta = 
      (pInertialInput.TimeOfValidity -
                  self.LastValidInertialInput.TimeOfValidity)

   self.LastValidInertialInput.TimeOfValidity =
                                 pInertialInput.TimeOfValidity

   if (self.LastInertialInputValid == True):
      Strapdown_Run(
               pStateData,
               &self.LastValidInertialInput,
               pCorrections)

      VelocityProcessing_Run(
               pStateData,
               &self.LastValidInertialInput,
               pCorrections)

      PositionComputation_Run(
               pStateData,
               &self.LastValidInertialInput)

      self.Output.TimeOfValidity = pInertialInput.TimeOfValidity
}

#---------------------------------------------------------------------------------------------

def NEF_AddCorrections(self, pCorrections):

   PositionComputation_AddCorrections(self, pCorrections)

   EarthModel_CalculateGravityAndEarthRadii(
               self.Output.EarthModel,
               self.Output.GeodeticPosition,
               self.Output.SinLatitude)

   Strapdown_AddCorrections(self, pCorrections)

   VelocityProcessing_AddCorrections(self, pCorrections)


#---------------------------------------------------------------------------------------------

def NEF_GetOutput(self, pOutput)

   sNEFOutput_Copy(self.Output, pOutput)


#---------------------------------------------------------------------------------------------

