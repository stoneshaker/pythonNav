import NEF
import numpy as np
from enum import Enum
import math

NEF_Status = Enum('SN_CONTROL_NEF_STATUS_TYPE', [('NEF_NOT_INITIALIZED', 1),
                                                 ('NEF_WAITING_FOR_VALID_INIT_DATA', 2),
												 ('NEF_NAVIGATING', 3)])
IMUToBodyDCM = np.Identity(3)
print('NEF_NAVIGATING =',NEF_Status.NEF_NAVIGATING.value)