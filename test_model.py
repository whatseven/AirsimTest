import math
import os

import airsim
import torch
import torchvision

from DroneLstm import DroneNet
from debug import log_file, print_current_pos
from ext import CAMERA_CENTER, DRONE_START_POS, SPEED, test_model_image_dir
import numpy as np

from ext1 import *

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)

class AirsimTest:
    def __init__(self):
        self.camera_center=CAMERA_CENTER

        self.model=DroneNet(useRNN)
        self.model.load_state_dict(torch.load(model_name))
        self.model=self.model.to(device)
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

    def start(self):
        for i in range(1000000):
            image=self.generate_image()

            self.model.eval()
            with torch.set_grad_enabled(False):
                score=self.model(image)
                preds=np.ones_like(score.cpu())
                preds[score.cpu()<0]=0
                preds=preds[0]

                vx=(1 if preds[0] else -1)*SPEED*0.1
                vy=(1 if preds[1] else -1)*SPEED*0.1
                vz=(1 if preds[2] else -1)*SPEED*0.1

            self.client.moveByVelocityAsync(-vx,-vy,-vz,1, airsim.DrivetrainType.ForwardOnly
                                        , airsim.YawMode(False, 0)).join()

            self.calculate_camera()
            print(i)

    def generate_image(self):
        """
        Generate Image
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)  # scene vision image in png format
        ])
        img1d = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgba = img1d.reshape(responses[0].height, responses[0].width, 4)
        image=np.delete(img_rgba, 3, axis=2)

        transform=torchvision.transforms.Compose([
            torchvision.transforms.ToPILImage(),
            torchvision.transforms.Resize((224,224)),
            torchvision.transforms.ToTensor(),
            torchvision.transforms.Normalize(mean=[0.485,0.456,0.406],
                                             std=[0.229,0.224,0.225])
        ])
        image=transform(image).unsqueeze(0).unsqueeze(0)
        image=image.to(device)
        return image

    def calculate_camera(self):
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

    def prepare_start(self):

        """
        Fly to first point
        """
        state = self.client.getMultirotorState()
        current_pos = np.asarray([state.kinematics_estimated.position.x_val
                                     , state.kinematics_estimated.position.y_val
                                     , state.kinematics_estimated.position.z_val])
        self.client.moveToPositionAsync(current_pos[0], current_pos[1], -7
                                        , SPEED).join()
        # self.client.hoverAsync().join()
        print_current_pos(self.client.getMultirotorState())

    def finish(self):
        self.client.hoverAsync().join()
        self.client.armDisarm(False)
        self.client.reset()

        # that's enough fun for now. let's quit cleanly
        self.client.enableApiControl(False)

        #self.s.close()

demo=AirsimTest()
demo.prepare_start()
demo.start()

