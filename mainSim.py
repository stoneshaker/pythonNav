import NEF
import numpy as np
import pandas as pd
from enum import Enum
import math

NEF_Status = Enum('SN_CONTROL_NEF_STATUS_TYPE', [('NEF_NOT_INITIALIZED', 1),
                                                 ('NEF_WAITING_FOR_VALID_INIT_DATA', 2),
												 ('NEF_NAVIGATING', 3)])
IMUToBodyDCM = np.identity(3)
print('NEF_NAVIGATING =',NEF_Status.NEF_NAVIGATING.value)

#arr = np.genfromtxt("C:\\Users\\ShaneStone\\source\\nav\\pythonNav\\data\\Scenario1h\\Scenario1h_IMU600Hz.log",
#                    delimiter=",")
IMUdata = np.genfromtxt('C:\\Users\\ShaneStone\\source\\nav\\pythonNav\\data\\Scenario1h\\Scenario1h_IMU600Hz.log',comments='#',skip_header=1,invalid_raise=False)


print(IMUdata[0:10][:])
