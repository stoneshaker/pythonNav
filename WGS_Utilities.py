import math

def ECEF_To_Geodetic(Pos_X, Pos_Y, Pos_Z):

#   long_float Sin_Theta;
#   long_float Sin_Theta_Cubed;
#   long_float Cos_Theta;
#   long_float Cos_Theta_Cubed;

#   long_float P;
#   long_float Theta;
#   long_float R_Curve;  # Raduis of Curvature in Prime Vertical

   Semi_Major_Axis = 6378137.0
   Semi_Minor_Axis = 6356752.3142
   f = (Semi_Major_Axis - Semi_Minor_Axis) / Semi_Major_Axis
   Ecc_Sq = 2.0 * f - (f * f)  # Ellipsiod Eccentricity Squared
   Ecc_Dash_Sq = ((Semi_Major_Axis * Semi_Major_Axis) - \
             (Semi_Minor_Axis * Semi_Minor_Axis)) / \
                         (Semi_Minor_Axis * Semi_Minor_Axis)


   P       = math.sqrt((Pos_X * Pos_X) + (Pos_Y * Pos_Y))
   Theta   = math.atan2((Pos_Z * Semi_Major_Axis), (P * Semi_Minor_Axis))

   Sin_Theta = math.sin(Theta)
   Sin_Theta_Cubed = Sin_Theta * Sin_Theta * Sin_Theta
   Cos_Theta = math.cos(Theta)
   Cos_Theta_Cubed = Cos_Theta * Cos_Theta * Cos_Theta

   Lat = math.atan2(
            (Pos_Z + Ecc_Dash_Sq * Semi_Minor_Axis * Sin_Theta_Cubed),
            (P - Ecc_Sq * Semi_Major_Axis * Cos_Theta_Cubed))

   Long = math.atan2(Pos_Y , Pos_X)

   R_Curve = Semi_Major_Axis / \
                  math.sqrt(1.0 - Ecc_Sq * (math.sin(Lat) * math.sin(Lat)))

   Height = (P / math.cos(Lat)) - R_Curve
   return Lat, Long, Height
   
# End of ECEF_To_Geodetic

def Geodetic_To_ECEF(Lat, Long, Height):

   Semi_Major_Axis = 6378137.0
   Semi_Minor_Axis = 6356752.3142
   f = (Semi_Major_Axis - Semi_Minor_Axis) / Semi_Major_Axis
   Ecc_Sq = 2.0 * f - (f * f)  # Ellipsiod Eccentricity Squared
   
#  long_float R_Curve;  # Radius of Curvature in Prime Vertical

   
   R_Curve = Semi_Major_Axis / \
                  math.sqrt(1.0 - Ecc_Sq * (math.sin(Lat) * math.sin(Lat)))
   
   Pos_X = (R_Curve + Height) * math.cos(Lat) * math.cos(Long)
   Pos_Y = (R_Curve + Height) * math.cos(Lat) * math.sin(Long)
   Pos_Z = (R_Curve * (1.0 - Ecc_Sq) + Height) * math.sin(Lat)
   
   return Pos_X, Pos_Y, Pos_Z

# End of Geodetic_To_ECEF;


def NED_To_ECEF(Lat, Long, North, East, Down):

   X = -(math.sin(Lat) * math.cos(Long) * North) - \
         (math.sin(Long) * East) - \
         (math.cos(Lat) * math.cos(Long) * Down)
   
   Y = -(math.sin(Lat) * math.sin(Long) * North) + \
         (math.cos(Long) * East) - \
         (math.cos(Lat) * math.sin(Long) * Down)
      
   Z = math.cos(Lat) * North - math.sin(Lat) * Down
   
   return X, Y, Z
   

# End of NED_to_ECEF


def ECEF_To_NED(Lat, Long, X, Y, Z):

      North = -(math.sin(Lat) * math.cos(Long) * X) - \
                (math.sin(Lat) * math.sin(Long) * Y) + \
                (math.cos(Lat) * Z)
    
      East = -(math.sin(Long) * X) + (math.cos(Long) * Y)
         
      Down = -(math.cos(Lat) * math.cos(Long) * X) - \
            (math.cos(Lat) * math.sin(Long) * Y) - \
            (math.sin(Lat) * Z)
            
      return North, East, Down

# End of ECEF_To_NED;