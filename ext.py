import numpy as np

"""
Parameters
"""

START_POS = np.asarray([1500, 1500, 500])
END_POS = np.asarray([700, 510, 150])
END_NORM=np.asarray([0, 1, 0])
CAMERA_CENTER = np.asarray([700, 510, 150])
DRONE_START_POS = np.asarray([1500, 1500, 2.1])
SAMPLE_FREQUENCY=20


"""

"""


# debug
#CONTROL_POINT = Vec3D.Vec3D(1200, 1000, 300)

HOST = '172.26.210.160'
PORT = 7777
SPEED=1
FREQUENCY=40