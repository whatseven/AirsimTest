import time

import numpy

from ext import g_log_level


def print_current_pos(state):
    current_pos = state.kinematics_estimated.position
    log_file("debug", "pos:"
             + str(current_pos.x_val) + "_"
             + str(current_pos.y_val) + "_"
             + str(current_pos.z_val))

def log_file(log_type, log_msg):
    if log_type in g_log_level:
        if type(log_msg)==numpy.ndarray or type(log_msg)==list:
            print(time.asctime(time.localtime(time.time()))
                  + "=====" + log_type + "---:" + log_msg)
        else:
            print(time.asctime(time.localtime(time.time()))
                  + "=====" + log_type + "---:" + log_msg)