# script to test the GJK algorithm
# generating random arm pose and compare the result

from arm_control.robot import Robot
from arm_control.urdf import URDF
from arm_control.collision import GJK

import trimesh
import numpy as np
from loguru import logger as log
import time

from copy import deepcopy


def detect_collision_gjk(collison_object_list):
    for i in range(len(collison_object_list) - 1):
        for j in range(i + 2, len(collison_object_list)):
            detector = GJK()
            result = detector.super_check(
                collison_object_list[i], collison_object_list[j], 3
            )
            if result:
                return True
    return False


def detect_collison_fcl(mesh_list):
    for i in range(len(collison_object_list) - 1):
        for j in range(i + 2, len(collison_object_list)):
            manager = trimesh.collision.CollisionManager()
            manager.add_object(str(i), mesh_list[i])
            result = manager.in_collision_single(mesh_list[j])
            if result:
                return True
    return False


log.remove()
TEST_NUM = 1000
error_count = 0

## Read file
file_path = "./urdf/uR10.urdf"
urdf = URDF()
urdf.get_urdf_parameters(file_path)
for joint in urdf.joint_list:
    print(f'name of joint: {joint["name"]}')
print(f"Number of joints: {len(urdf.joint_list)}")
link_list = urdf.link_list
for link in link_list:
    print(f'name of link: {link["name"]}')
print(f"Number of links: {len(link_list)}")
## Get the robot for forward kinematics function
robot = Robot()
robot.joint_list = urdf.joint_list
time_start = time.time()
collison_manager = trimesh.collision.CollisionManager()
mesh_list = []
for i in range(len(link_list) - 1):
    mesh = trimesh.load(link_list[i]["collision_mesh_path"], force="mesh")
    mesh_list.append(mesh)
    collison_manager.add_object(str(i), mesh)

for ii in range(TEST_NUM):
    random_theta_list = np.random.uniform(-np.pi, np.pi, len(urdf.joint_list))
    random_theta_list[-2:] = 0  # set the last two joints to zero
    frame_history = robot.forward_kine(random_theta_list)

    collison_object_list = []
    temp_mesh_list = []
    temp_mesh_list.append(deepcopy(mesh_list[0]))
    collison_object_list.append(mesh_list[0].sample(1000))
    # collison_manager.add_object("0", mesh)
    for i in range(1, len(frame_history)):
        collison_manager.set_transform(str(i), frame_history[i - 1])
        # collison_manager.add_object(str(i), mesh)
        sample_points = (
            deepcopy(mesh_list[i]).apply_transform(frame_history[i - 1]).sample(1000)
        )
        collison_object_list.append(sample_points)
        temp_mesh_list.append(
            deepcopy(mesh_list[i]).apply_transform(frame_history[i - 1])
        )
    result_gjk = detect_collision_gjk(collison_object_list)
    result_fcl = detect_collison_fcl(temp_mesh_list)
    error_count += result_gjk != result_fcl
    print(f"current k: {ii}, gjk result: {result_gjk}, fcl result: {result_fcl}")
    print(f"error rate: {error_count/(ii+1)}")
time_end = time.time()
print(f"Time cost: {time_end - time_start}")
print(f"Avg time cost: {(time_end - time_start) / TEST_NUM}")
print(f"Error rate: {error_count / TEST_NUM}")
