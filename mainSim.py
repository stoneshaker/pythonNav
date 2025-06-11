import NEF
import numpy as np
import pandas as pd
from enum import Enum
import math
import csv
import Messages as msg

NavFileOut = open('C:\\Users\\ShaneStone\\source\\nav\\pythonNav\\data\\Scenario1h\\pyNav_Solution.csv','w') 
SN_CONTROL_NEF_STATUS_TYPE = Enum('SN_CONTROL_NEF_STATUS_TYPE', [('NEF_NOT_INITIALIZED', 1),
                                                 ('NEF_WAITING_FOR_VALID_INIT_DATA', 2),
												 ('NEF_NAVIGATING', 3)])
IMUToBodyDCM = np.identity(3)
Input_Init_Message = msg.SHC_INIT_MSG_TYPE()
GPS_Nav_Solution = msg.FROM_GPS_INTEGER_TIME_MARK_PULSE_TYPE()

def System_Initialization(Input_Init_Message, GPS_Nav_Solution):
#    LCNApplication_ConstructObjects() 
    NEF_Status = SN_CONTROL_NEF_STATUS_TYPE.NEF_NOT_INITIALIZED
    Input_Init_Message.Validity.Heading_Valid = False
    Input_Init_Message.Validity.Attitude_Valid = False
    Input_Init_Message.Validity.Velocity_Valid = False
    Input_Init_Message.Validity.Position_Valid = False
    GPS_Nav_Solution.Position_ECEF_X - 0.0
    GPS_Nav_Solution.Position_ECEF_Y - 0.0
    GPS_Nav_Solution.Position_ECEF_Z - 0.0
#    AHRS_Initialize()
    print('Initialization Complete')
    return NEF_Status, Input_Init_Message, GPS_Nav_Solution

NEF_Status, Input_Init_Message, GPS_Nav_Solution = System_Initialization(Input_Init_Message, GPS_Nav_Solution)
print('NEF_NAVIGATING =',NEF_Status.NEF_NAVIGATING.value)
print('Input Init Message Validition',Input_Init_Message.Validity.Heading_Valid)
#arr = np.genfromtxt("C:\\Users\\ShaneStone\\source\\nav\\pythonNav\\data\\Scenario1h\\Scenario1h_IMU600Hz.log",
#                    delimiter=",")
IMUdata = np.genfromtxt('C:\\Users\\ShaneStone\\source\\nav\\pythonNav\\data\\Scenario1h\\Scenario1h_IMU600Hz.log',comments='#',skip_header=1,invalid_raise=False)
imu_length = len(IMUdata)
EndSim_Time = IMUdata[-1,1]
print(IMUdata[0:10][:])
print(imu_length)
print(np.shape(IMUdata))
print('EndSim Time =',EndSim_Time)
