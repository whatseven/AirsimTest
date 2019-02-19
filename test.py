import airsim
import math
import numpy as np

DRONE_START_POS=np.asarray([600, -500, 112])
camera_center=np.asarray([-900, 600, 220])

def calculate_camera_to_center(v_pos_local):
    direction = world_to_local(camera_center) - v_pos_local
    direction_in_camera = np.asarray([direction[0], direction[1], -direction[2]])
    direction_in_camera_norm = direction_in_camera / np.linalg.norm(direction_in_camera)
    pitch = math.asin(direction_in_camera_norm[2])

    yaw = math.atan(direction_in_camera_norm[1] / direction_in_camera_norm[0])

    if direction_in_camera_norm[0]<0:
        yaw+=math.pi

    return [pitch,0,yaw]

def world_to_local(world_pos):
    pos_diff = DRONE_START_POS
    return np.append((world_pos - pos_diff)[:2], -world_pos[2] + pos_diff[2]) / 100

client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

client.moveToPositionAsync(0,0,-1,1,60).join()


current_pos_state = client.getMultirotorState().kinematics_estimated
current_pos=np.asarray([current_pos_state.position.x_val, current_pos_state.position.y_val, current_pos_state.position.z_val])
pitch,_,yaw=calculate_camera_to_center(current_pos)
camera_angle=airsim.to_quaternion(pitch,0,yaw-airsim.to_eularian_angles(current_pos_state.orientation)[2])
client.simSetCameraOrientation("", camera_angle)

client.rotateToYawAsync(100).join()
airsim.to_eularian_angles(current_pos_state.orientation)[2]

client.moveByAngleThrottleAsync(1, 0, 0, 0, 3).join()