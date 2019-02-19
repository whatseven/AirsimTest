import numpy as np

"""
Parameters
"""

START_POS = np.asarray([600, -500, 112])
END_POS = np.asarray([-900, 600, 220])
END_NORM=np.asarray([1, 0, 0])
CAMERA_CENTER = np.asarray([-900, 600, 220])
DRONE_START_POS = START_POS
SAMPLE_FREQUENCY=20

ROUTE_START_POS=[-300,-1300,450]
ROUTE_END_POS=[1100,1100,1050]
ROUTE_STEP=200

g_log_level=["move to","after moving","camera","process","debug"]
"""

"""


# debug
#CONTROL_POINT = Vec3D.Vec3D(1200, 1000, 300)

HOST = '172.26.210.160'
PORT = 7777
SPEED=1
FREQUENCY=40