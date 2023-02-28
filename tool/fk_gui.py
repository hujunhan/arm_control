from arm_control.urdf import URDF
from arm_control.robot import Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import time

## Read file
file_path='./urdf/uR10e.urdf'
urdf=URDF()
urdf.get_urdf_parameters(file_path)
for joint in urdf.joint_list:
    print(f'name of joint: {joint["name"]}')
print(f'Number of joints: {len(urdf.joint_list)}')

## Get the robot for forward kinematics function
robot=Robot()
robot.joint_list=urdf.joint_list

## Initial joint angles
theta_list=[0,0,0,0,0,0,0]

## Set up the figure
fig=plt.figure()
N=len(theta_list)
fig.subplots_adjust(left=0.1, right=0.7, bottom=0.1, top=1.0)
ax = fig.add_subplot(projection='3d')

def update(val):
    theta_list=[slider.val for slider in slider_list]
    frame_history= robot.forward_kine(theta_list)
    
    ax.cla()
    x=[0]
    y=[0]
    z=[0]
    for i in range(len(frame_history)):
        x.append(frame_history[i][0,3])
        y.append(frame_history[i][1,3])
        z.append(frame_history[i][2,3])
    for i in range(len(x)-1):
        ax.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
    ax.set_aspect('equal')
    ax.set_xlim(-1,1)
    ax.set_ylim(-1,1)
    ax.set_zlim(-1,1)

## Add sliders
ax_list=[]
slider_list=[]
for i in range(N):
    axe=fig.add_axes([0.8, 1-0.9*(i+1)/N, 0.15, 0.8/N])
    slider=Slider(axe, f'joint{i}', -np.pi, np.pi, valinit=theta_list[i])
    slider.on_changed(update)
    slider_list.append(slider)
    

## Plot the robot in the initial position
a=time.time()
frame_history= robot.forward_kine(theta_list)
x=[0]
y=[0]
z=[0]
for i in range(len(frame_history)):
    x.append(frame_history[i][0,3])
    y.append(frame_history[i][1,3])
    z.append(frame_history[i][2,3])
    

for i in range(len(x)-1):
    ax.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
ax.set_aspect('equal')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)
b=time.time()    
print(f'Forward kinematics time: {b-a}')
plt.show()