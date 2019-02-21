import math
import os
import time
from datetime import datetime
from threading import Timer

import Vec3D
from BezierClass import BezierCurve
import matplotlib
import matplotlib.pyplot as plt
import socket
import numpy as np

import _thread
import airsim

from debug import log_file, print_current_pos
from ext import *

PARAMETERS = (0, 0.5, 1)

"""
Utils
"""

g_x=0
g_y=0
g_z=0

g_current_orientation_pitch=0
g_current_orientation_roll=0
g_current_orientation_yaw=0

g_next_duration=1


def calculate_path(start_pos, end_pos, end_norm):
    end_to_start = start_pos - end_pos
    max_length = np.dot(end_to_start, end_norm)
    control_points = []
    for i in range(1, 6):
        control_points.append(end_pos + end_norm * (max_length / 5 * i))

    return end_pos + end_norm * (max_length / 5)


def generateCurvePoint(v_control_point,v_start_pos):
    curve = BezierCurve()

    points = [Vec3D.Vec3D(v_start_pos[0], v_start_pos[1], v_start_pos[2]),
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

class AirsimDemo:
    def __init__(self):
        self.camera_center=CAMERA_CENTER

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
        Data reporter
        """
        try:
            self.reporter = open(os.path.join(self.reporter_dir, datetime.now().strftime('%m-%d-%H-%M-%S') + ".csv"), "w")
        except Exception as e:
            print(e)

        """
        Airsim controller
        """
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        #self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        log_file("Airsim connect succssful", "")

        self.client.takeoffAsync().join()
        #self.client.hoverAsync().join()

    #
    # Util
    #
    def world_to_local(self, world_pos):
        pos_diff = DRONE_START_POS
        return np.append((world_pos - pos_diff)[:2], -world_pos[2] + pos_diff[2]) / 100

    def local_to_world(self, local_pos):
        pos_diff = DRONE_START_POS

        return np.append(local_pos[:2] * 100 + pos_diff[:2], -local_pos[2] * 100 + pos_diff[2])

    def calculate_camera_to_center(self,v_pos_local):
        direction = self.world_to_local(self.camera_center) - v_pos_local
        direction_in_camera = np.asarray([direction[0], direction[1], -direction[2]])
        direction_in_camera_norm = direction_in_camera / np.linalg.norm(direction_in_camera)
        pitch = math.asin(direction_in_camera_norm[2])

        yaw = math.atan(direction_in_camera_norm[1] / direction_in_camera_norm[0])

        if direction_in_camera_norm[0] < 0:
            yaw += math.pi

        return [pitch, 0, yaw]

    #
    # control function
    #

    def start(self, v_points, v_id_route):
        print_current_pos(self.client.getMultirotorState())

        self.prepare_start(v_id_route, v_points[0])

        """
        Start flying
        """
        next_index = 1
        #Debug
        #d_current_pos=self.world_to_local(v_points[0])
        #d_pos_list=[d_current_pos]
        while next_index < v_points.shape[0]-1:
            identifier=str(v_id_route)+"_"+str(next_index)

            self.move_to_next(next_index, v_points)

            self.calculate_camera(identifier)

            image_filename = self.generate_image(identifier)

            self.report_and_logging(identifier,image_filename)
            next_index += 1


        state = self.client.getMultirotorState()

        self.reporter.write("\n")

    def report_and_logging(self,v_identifier, image_filename):
        """
        Report
        """
        global g_x,g_y,g_z,g_current_orientation_pitch,g_current_orientation_roll,g_current_orientation_yaw,g_next_duration
        self.reporter.write(v_identifier + "," + str(g_x) + ","
                            + str(g_y) + ","
                            + str(g_z) + ","
                            + str(g_current_orientation_pitch) + ","
                            + str(g_current_orientation_roll) + ","
                            + str(g_current_orientation_yaw) + ","
                            + str(g_next_duration) + ","
                            + str(image_filename) + ",")
        """
        Logging
        """
        state = self.client.getMultirotorState()
        log_file("position", "after moving:(" + str(state.kinematics_estimated.position.x_val) + ","
                 + str(state.kinematics_estimated.position.y_val) + ","
                 + str(state.kinematics_estimated.position.z_val) + ")"
                 + "\nvelocity:" + str(g_x) + ","
                 + str(g_y) + ","
                 + str(g_z) + "\n"
                 + "duration:" + str(g_next_duration)
                 )

    def generate_image(self, v_identifier):
        """
        Generate Image
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene)  # scene vision image in png format
        ])
        image_filename = os.path.normpath(os.path.join(self.data_dir, str(v_identifier)) + '.png')
        airsim.write_file(image_filename, responses[0].image_data_uint8)
        # log_file("Image saved", str(image_filename))
        return image_filename

    def calculate_camera(self,v_identifier):
        """
        Calculate camera heading
        """
        current_pos_state = self.client.getMultirotorState().kinematics_estimated
        current_pos_local = np.asarray([current_pos_state.position.x_val
                                           , current_pos_state.position.y_val
                                           , current_pos_state.position.z_val])
        # print("now:"+str(current_pos_local))

        camera_angle = self.calculate_camera_to_center(current_pos_local)
        global g_current_orientation_pitch,g_current_orientation_roll,g_current_orientation_yaw
        g_current_orientation_pitch = camera_angle[0]
        g_current_orientation_roll = 0
        g_current_orientation_yaw = camera_angle[2]-airsim.to_eularian_angles(current_pos_state.orientation)[2]
        #g_current_orientation_yaw = camera_angle[2]

        # log_file("camera",
        #          v_identifier+":desired orientation:" + str(g_current_orientation_pitch) + ","
        #          + str(g_current_orientation_yaw))

        self.client.simSetCameraOrientation("front_center", airsim.to_quaternion(
            g_current_orientation_pitch, g_current_orientation_roll, g_current_orientation_yaw))

        # state = self.client.getMultirotorState()
        # orientation = airsim.to_eularian_angles(state.kinematics_estimated.orientation)
        # log_file("camera", v_identifier + ":camera after pitch:" + str(orientation[0]) + "," + str(orientation[2]))
        #
        # self.client.rotateToYawAsync(math.degrees(g_current_orientation_yaw), 5,margin=0).join()
        #
        # state = self.client.getMultirotorState()
        # orientation = airsim.to_eularian_angles(state.kinematics_estimated.orientation)
        # log_file("camera", v_identifier+":actual orientation:" + str(orientation[0]) + "," + str(orientation[2]))

    def move_to_next(self, next_index, v_points):
        state = self.client.getMultirotorState()
        current_pos = np.asarray([state.kinematics_estimated.position.x_val
                                     , state.kinematics_estimated.position.y_val
                                     , state.kinematics_estimated.position.z_val])
        current_pos_world=self.local_to_world(current_pos)
        current_distance=math.inf
        nearest_point_index=0
        for i in range(max(next_index-10,0),min(next_index+10,len(v_points))):
            t_distance=np.sum(np.power(v_points[i]-current_pos_world,2))
            if t_distance<current_distance:
                current_distance=t_distance
                nearest_point_index=i

        next_points=v_points[next_index]

        log_file("debug" , str(nearest_point_index)+"_"+ str(next_index))
        next_direction_local = self.world_to_local(next_points) - current_pos

        #
        # Velocity based
        #
        global g_x,g_y,g_z,g_current_orientation_pitch,g_current_orientation_roll,g_current_orientation_yaw,g_next_duration
        g_x = next_direction_local[0] / SPEED
        g_y = next_direction_local[1] / SPEED
        g_z = next_direction_local[2] / SPEED
        g_next_duration = 1
        self.client.moveByVelocityAsync(g_x
                                        , g_y
                                        , g_z
                                        , g_next_duration
                                        , airsim.DrivetrainType.ForwardOnly
                                        , airsim.YawMode(False, 0)).join()

        #
        # Control Angle based
        #
        # global g_x, g_y, g_z, g_next_duration
        # g_x = next_direction_local[0] / SPEED
        # g_y = 0
        # g_z = next_direction_local[2] / SPEED
        # self.client.moveByAngleThrottleAsync(g_x,g_y,g_z,0,g_next_duration).join()

    def prepare_start(self, v_id_route, v_start_point):

        """
        Fly to first point
        """
        start_pos_local = self.world_to_local(v_start_point)
        log_file("debug", "Fly to:"
                 + str(start_pos_local[0]) + "_"
                 + str(start_pos_local[1]) + "_"
                 + str(start_pos_local[2]))
        self.client.moveToPositionAsync(start_pos_local[0], start_pos_local[1], start_pos_local[2]
                                        , SPEED).join()
        # self.client.hoverAsync().join()
        print_current_pos(self.client.getMultirotorState())

    def finish(self):
        self.client.hoverAsync().join()
        self.client.armDisarm(False)
        self.client.reset()

        self.reporter.close()

        # that's enough fun for now. let's quit cleanly
        self.client.enableApiControl(False)

        #self.s.close()


if __name__ == '__main__':
    # Sample the route
    start_poses=[]
    x=np.linspace(ROUTE_START_POS[0],ROUTE_END_POS[0],(abs(ROUTE_END_POS[0]-ROUTE_START_POS[0]))/ROUTE_STEP).reshape((-1,1))
    y=np.linspace(ROUTE_START_POS[1],ROUTE_END_POS[1],(abs(ROUTE_END_POS[1]-ROUTE_START_POS[1]))/ROUTE_STEP).reshape((-1,1))
    z=np.linspace(ROUTE_START_POS[2],ROUTE_END_POS[2],(abs(ROUTE_END_POS[2]-ROUTE_START_POS[2]))/ROUTE_STEP).reshape((-1,1))
    for tz in z:
        for ty in y:
            for tx in x:
                start_poses.append(np.asarray([tx,ty,tz]))

    #
    # Init the model
    #
    airsim_demo = AirsimDemo()

    for id_route, start_pos in enumerate(start_poses):
        control_point=calculate_path(start_pos, END_POS, END_NORM)
        x, y, z = generateCurvePoint(control_point, start_pos)
        points = np.asarray([x, y, z]).T
        airsim_demo.start(points[0], id_route)
        log_file("process",str(id_route)+"/"+str(len(start_poses)))
        # for idx, control_point in enumerate(calculate_path(start_pos, END_POS, END_NORM)):
        #     if idx==4:
        #         x, y, z = generateCurvePoint(control_point, start_pos)
        #         points = np.asarray([x, y, z]).T
        #
        #         airsim_demo.start(points, idx,id_route)

    airsim_demo.finish()
