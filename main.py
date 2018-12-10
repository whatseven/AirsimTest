import math
import os
import time

import Vec3D
from BezierClass import BezierCurve
import matplotlib
import matplotlib.pyplot as plt
import socket
import numpy as np

import _thread
import airsim

from ext import *

PARAMETERS = (0, 0.5, 1)

"""
Utils
"""


def log_file(log_type, log_msg):
    print(time.asctime(time.localtime(time.time()))
          + "=====" + log_type + "---:" + log_msg)


def calculate_path(start_pos, end_pos, end_norm):
    end_to_start = start_pos - end_pos
    max_length = np.dot(end_to_start, end_norm)
    control_points = []
    for i in range(1, 6):
        control_points.append(end_pos + end_norm * (max_length / 5 * i))

    return control_points


def generateCurvePoint(v_control_point):
    curve = BezierCurve()

    points = [Vec3D.Vec3D(START_POS[0], START_POS[1], START_POS[2]),
              Vec3D.Vec3D(v_control_point[0], v_control_point[1], v_control_point[2]),
              Vec3D.Vec3D(END_POS[0], END_POS[1], END_POS[2])]

    for index in range(0, 3):
        curve.append_point(points[index])
    x = []
    y = []
    z = []
    for item in curve.draw():
        x.append(item.x)
        y.append(item.y)
        z.append(item.z)
    # ax = plt.subplot(111, projection='3d')

    # ax.scatter(x[0:], y[0:], z[0:], c='y')

    # ax.set_zlabel('Z')
    # ax.set_ylabel('Y')
    # ax.set_xlabel('X')
    # plt.show()
    return x, y, z


def socket_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(2)
    try:
        while True:
            conn, add = server.accept()
            log_file("accept", "")
            while True:
                buf = conn.recv(1024).decode()
                data = buf.split("_")
                if data[0] == "syncP":
                    pos_x = data[4]
                    pos_y = data[5]
                    pos_z = data[6]
                    log_file("syncp:", pos_x + "," + pos_y + "," + pos_z)
                elif data[0] == "syncG":
                    pass

    except KeyboardInterrupt:
        print("you have CTRL+C,Now quit")
        server.close()


def socket_client():
    pass


class AirsimDemo:
    def __init__(self):
        """
        Global control data
        """
        pos_pitch = ""
        pos_roll = ""
        pos_yaw = ""

        pos_x = ""
        pos_y = ""
        pos_z = ""

        gim_pitch = ""
        gim_raw = ""
        gim_yaw = ""

        self.camera_center = CAMERA_CENTER

        """
        File dir
        """
        self.reporter = None
        self.reporter_dir = "reporter"
        self.data_dir = os.path.join(os.getcwd(), "image")

        try:
            os.makedirs(self.data_dir)
            os.makedirs(self.reporter_dir)
        except OSError:
            if not os.path.isdir(self.data_dir):
                raise
            if not os.path.isdir(self.reporter_dir):
                raise

        """
        Airsim controller
        """
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        log_file("Airsim connect succssful", "")

        self.client.takeoffAsync().join()

    def world_to_local(self, world_pos):
        pos_diff = DRONE_START_POS
        return np.append((world_pos - pos_diff)[:2], -world_pos[2] + pos_diff[2]) / 100

    def local_to_world(self, local_pos):
        pos_diff = DRONE_START_POS
        np.asarray([local_pos * 100 + pos_diff[:2], -local_pos * 100 + pos_diff])
        return local_pos * 100 + pos_diff

    def calculate_camera_to_center(self, v_pos_local):
        direction = self.world_to_local(self.camera_center) - v_pos_local
        direction_in_camera = np.asarray([direction[0],direction[1] ,-direction[2]])
        direction_in_camera_norm = direction_in_camera / np.linalg.norm(direction_in_camera)
        pitch = math.asin(direction_in_camera_norm[2])

        if direction_in_camera_norm[1]<0 and direction_in_camera_norm[0]<0:
            yaw = -math.atan(direction_in_camera_norm[0] / direction_in_camera_norm[1])-math.pi/2
        elif direction_in_camera_norm[1]>0 and direction_in_camera_norm[0]>0:
            yaw = math.atan(direction_in_camera_norm[1] / direction_in_camera_norm[0])
        elif direction_in_camera_norm[1] < 0 and direction_in_camera_norm[0] > 0:
            yaw = -math.atan(-direction_in_camera_norm[1] / direction_in_camera_norm[0])
        else:
            yaw = math.pi/2-math.atan(direction_in_camera_norm[1] / -direction_in_camera_norm[0])

        return [pitch, 0, yaw]

    def start(self, v_points, v_index):
        """
        Data reporter
        """
        try:
            self.reporter = open(os.path.join(self.reporter_dir, str(v_index) + ".txt"), "w")
        except Exception as e:
            print(e)

        """
        Fly to first point
        """
        start_pos_local = self.world_to_local(v_points[0])
        self.client.moveToPositionAsync(start_pos_local[0], start_pos_local[1], start_pos_local[2]
                                        , SPEED).join()

        self.client.hoverAsync().join()
        state = self.client.getMultirotorState()
        current_pos = state.kinematics_estimated.position
        log_file("Ready to start", "pos:"
                 + str(current_pos.x_val) + "_"
                 + str(current_pos.y_val) + "_"
                 + str(current_pos.z_val))

        """
        Start flying
        """
        next_index = 1
        while next_index < v_points.shape[0]:
            state = self.client.getMultirotorState()
            log_file("move to:" + str(v_points[next_index]), "now:" + str())
            current_pos = np.asarray([state.kinematics_estimated.position.x_val
                                         , state.kinematics_estimated.position.y_val
                                         , state.kinematics_estimated.position.z_val])

            next_direction_local = self.world_to_local(v_points[next_index]) - current_pos
            distance = np.linalg.norm(next_direction_local)
            next_direction_local_norm = next_direction_local / distance
            next_duration = distance / SPEED

            self.client.moveByVelocityAsync(next_direction_local_norm[0] * SPEED
                                            , next_direction_local_norm[1] * SPEED
                                            , next_direction_local_norm[2] * SPEED
                                            , next_duration
                                            , airsim.DrivetrainType.ForwardOnly
                                            , airsim.YawMode(False,0)).join()

            """
            Calculate camera heading
            """
            current_pos_state = self.client.getMultirotorState().kinematics_estimated
            current_pos_local = np.asarray([current_pos_state.position.x_val
                                               , current_pos_state.position.y_val
                                               , current_pos_state.position.z_val])

            camera_angle = self.calculate_camera_to_center(current_pos_local)
            self.client.simSetCameraOrientation(""
                                                , airsim.to_quaternion(
                    camera_angle[0],0,camera_angle[2]-airsim.to_eularian_angles(current_pos_state.orientation)[2]))

            """
            Generate Image
            """
            responses = self.client.simGetImages([
                airsim.ImageRequest("", airsim.ImageType.Scene)  # scene vision image in png format
            ])


            image_filename = os.path.normpath(os.path.join(self.data_dir,str(v_index)+"_"+ str(next_index)) + '.png')
            airsim.write_file(image_filename, responses[0].image_data_uint8)
            log_file("Image saved", str(image_filename))

            """
            Report
            """
            self.reporter.write(str(next_direction_local_norm[0] * SPEED) + "_"
                                + str(next_direction_local_norm[1] * SPEED) + "_"
                                + str(next_direction_local_norm[2] * SPEED) + "_"
                                + str(next_duration) + "_"
                                + str(image_filename))

            """
            Logging
            """
            state = self.client.getMultirotorState()
            log_file("after moving:", "(" + str(state.kinematics_estimated.position.x_val) + ","
                     + str(state.kinematics_estimated.position.y_val) + ","
                     + str(state.kinematics_estimated.position.z_val) + ")"
                     + "\nangle:" + str(state.kinematics_estimated.orientation.x_val) + ","
                     + str(state.kinematics_estimated.orientation.y_val) + ","
                     + str(state.kinematics_estimated.orientation.z_val)
                     )
            next_index += 1

        state = self.client.getMultirotorState()

        self.reporter.close()
        self.reporter = None

    def finish(self):
        self.client.hoverAsync().join()
        self.client.armDisarm(False)
        self.client.reset()

        # that's enough fun for now. let's quit cleanly
        self.client.enableApiControl(False)


if __name__ == '__main__':

    # Client to send the pose data and point
    # _thread.start_new_thread(socket_client)

    # Server to receive current states and position
    # _thread.start_new_thread(socket_server)

    airsim_demo = AirsimDemo()

    for idx, control_point in enumerate(calculate_path(START_POS, END_POS, END_NORM)):
        x, y, z = generateCurvePoint(control_point)
        points = np.asarray([x, y, z]).T

        airsim_demo.start(points, idx)

    airsim_demo.finish()
