class SINAV_VALIDITY_INIT_WORD_TYPE: 
    def __init__(self):
       self.Heading_Valid  = False 
       self.Attitude_Valid = False
       self.Velocity_Valid = False
       self.Position_Valid = False


class SHC_INIT_MSG_TYPE:
   def __init__(self):
       self.Validity = SINAV_VALIDITY_INIT_WORD_TYPE()        
       self.Latitude = 0.0      
       self.Longitude = 0.0
       self.Altitude = 0.0
       self.North_Velocity = 0.0
       self.East_Velocity = 0.0
       self.Down_Velocity = 0.0
       self.Roll = 0.0
       self.Pitch = 0.0
       self.Heading = 0.0

#----------------
# SN_IMU_MSG_TYPE
#----------------
class SN_IMU_MSG_TYPE:
    def __init__(self):
       self.TimeOfValidity = 0.0
       self.BodyXRate = 0.0
       self.BodyYRate = 0.0
       self.BodyZRate = 0.0
       self.AccX = 0.0
       self.AccY = 0.0
       self.AccZ = 0.0

class FROM_GPS_INTEGER_TIME_MARK_PULSE_TYPE:
   def __init__(self):
       self.UTC_Valid = False
       self.Data_TTFF_Suitable = False
       self.Data_Block_Valid = False
   
       self.GPS_Time_MSW = 0.0
       self.GPS_Time_LSW = 0.0
       self.UTC_Time_MSW = 0.0
       self.UTC_Time_LSW = 0.0
       self.Delta_Time = 0.0
       self.Time_Mark_Counter = 0.0
       self.Latitude = 0.0
       self.Longitude = 0.0
       self.Position_ECEF_X = 0.0
       self.Position_ECEF_Y = 0.0
       self.Position_ECEF_Z = 0.0
       self.Altitude_MSL = 0.0
       self.Altitude_ABS = 0.0
       self.Velocity_East = 0.0
       self.Velocity_North = 0.0
       self.Velocity_Up = 0.0
       self.Time_Of_Validity = 0.0
       self.Accel_East = 0.0
       self.Accel_North = 0.0
       self.Accel_Up = 0.0
       self.Attitude_Pitch = 0.0
       self.Attitude_Roll = 0.0
       self.True_Heading = 0.0
       self.Magnetic_Variation = 0.0
       self.Recoverable_MDZ = 0.0
       self.Nonrecoverable_MDZ = 0.0
       self.GPS_Stat_And_FOM = 0.0
       self.Est_Horizontal_Error = 0.0
       self.Est_Vertical_Error = 0.0
       self.Configuration = 0.0
       self.Time_FOM_Year = 0.0
       self.Day_Of_Year = 0.0
       self.SAAS_Status = 0.0