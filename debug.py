import airsim
import math
import numpy as np

CAMERA_CENTER = np.asarray([700, 510, 150])
DRONE_START_POS = np.asarray([1500, 1500, 2.1])

client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

def world_to_local(world_pos):
    pos_diff = DRONE_START_POS
    return np.append((world_pos - pos_diff)[:2], -world_pos[2] + pos_diff[2]) / 100


client.moveToPositionAsync(0,0,-5,1,60).join()


current_pos_state = client.getMultirotorState().kinematics_estimated
current_pos_local = np.asarray([current_pos_state.position.x_val
                                   , current_pos_state.position.y_val
                                   , current_pos_state.position.z_val])

direction = world_to_local(CAMERA_CENTER) - current_pos_local
direction_in_camera = np.asarray([direction[0],direction[1] ,-direction[2]])
direction_in_camera_norm = direction_in_camera / np.linalg.norm(direction_in_camera)
pitch = math.asin(direction_in_camera_norm[2])
yaw = math.atan(direction_in_camera_norm[1] / direction_in_camera_norm[0])
camera_angle=airsim.to_quaternion(pitch,0,yaw+airsim.to_eularian_angles(current_pos_state.orientation)[2] )
client.simSetCameraOrientation("", camera_angle)

client.moveByVelocityAsync(1, 1, 1, 0.1, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()