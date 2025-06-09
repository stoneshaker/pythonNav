import math
import sys
sys.path.append('..\\')
import WGS_Utilities

Lat = 27.975608 * math.pi/180.0
Long = -82.382481 * math.pi/180.0
Height = 5.0

PosX, PosY, PosZ = WGS_Utilities.Geodetic_To_ECEF(Lat, Long, Height)
print('ECEF Position: x =',PosX,'y =',PosY,'z =',PosZ)
lat, lon, h = WGS_Utilities.ECEF_To_Geodetic(PosX, PosY, PosZ)
print('Geodetic Position: Lat =',lat*180.0/math.pi,'Lon =',lon*180.0/math.pi,'Height  =',h)

North = 100.0
East = 100.0
Down = 0.0
X, Y, Z = WGS_Utilities.NED_To_ECEF(Lat, Long, North, East, Down)
print(X, Y, Z)
N, E, D = WGS_Utilities.ECEF_To_NED(Lat, Long, X, Y, Z)
print(N, E, D)