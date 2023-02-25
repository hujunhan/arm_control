# coding= utf8
"""
Explore the workspace of a robot
"""

from arm_control.myfk import Robot
from arm_control.myurdf import URDF
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import time

## Read file
file_path='./urdf/uR10e.urdf'
urdf=URDF()
urdf.get_urdf_parameters(file_path)

## Get the robot for forward kinematics function
robot=Robot()
robot.joint_list=urdf.joint_list
N=len(urdf.joint_list)
for joint in urdf.joint_list:
    print(f'limit of joint: {joint["bounds"]}')
    
## Get the sample joint angles (randomly)
sample_num=1000
sample_joint_list=np.random.uniform([-np.pi]*N,[np.pi]*N,[sample_num,N])
sample_step=1.0
x=np.arange(-np.pi,np.pi,sample_step)
y=np.arange(-np.pi,np.pi,sample_step)
z=np.arange(-np.pi,np.pi,sample_step)
i=np.arange(-np.pi,np.pi,sample_step)
j=np.arange(-np.pi,np.pi,sample_step)

X,Y,Z,I,J=np.meshgrid(x,y,z,i,j)
j6=np.zeros(X.shape)
j7=np.zeros(X.shape)
# flatten XYZIJ and combine them into a matrix
sample_joint_list=np.concatenate((X.reshape(-1,1),Y.reshape(-1,1),Z.reshape(-1,1),I.reshape(-1,1),J.reshape(-1,1),j6.reshape(-1,1),j7.reshape(-1,1)),axis=1)
sample_num=len(sample_joint_list)

print(len(sample_joint_list))

## Apply forward kinematics to get the end-effector position
for i in range(sample_num):
    frame_history= robot.forward_kine(sample_joint_list[i])
    if i==0:
        ee_pos_list=np.array([frame_history[-1][0:3,3]])
    else:
        ee_pos_list=np.concatenate((ee_pos_list,[frame_history[-1][0:3,3]]),axis=0)

## Plot the end-effector position
fig=plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(ee_pos_list[:,0],ee_pos_list[:,1],ee_pos_list[:,2])
ax.set_aspect('equal')
plt.show()