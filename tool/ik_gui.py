from arm_control.urdf import URDF
from arm_control.robot import Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import time
file_path='./urdf/uR10e.urdf'
urdf=URDF()
urdf.get_urdf_parameters(file_path)
for joint in urdf.joint_list:
    # print(joint['rotation'])
    # print(joint['bounds'])
    print(f'name of joint: {joint["name"]}')
    print(f'joint bounds: {joint["bounds"]}')
print(f'Number of joints: {len(urdf.joint_list)}')
robot=Robot()
robot.joint_list=urdf.joint_list
frame_history= robot.forward_kine_sympy()
robot.jacobian_xyz(frame_history[-1])
robot.joint_list=urdf.joint_list
theta_list_current=np.asarray([0,0.1,0.2,0.3,0.4,0,0])
theta_list_target=np.asarray([1,0.2,0.3,0.4,0.5,0,0])

print(f'current position: {robot.fk_lambda(theta_list_current)[0:3,3]}')
print(f'target position: {robot.fk_lambda(theta_list_target)[0:3,3]}')

angle_history=robot.ik_transpose(theta_list_current,robot.fk_lambda(theta_list_target)[0:3,3],max_iteration=1000)

print(f'angle_history: {angle_history}')


## Use the matplotlib to plot the robot
# animate the robot moving from the initial position to the target position
fig=plt.figure()
ax = fig.add_subplot(projection='3d')
line_list=[]
for i in range(len(robot.joint_list)):
    line,=ax.plot([],[],[])
    line_list.append(line)
ax.set_aspect('equal')
# ax.set_xlim(-2,2)
# ax.set_ylim(-2,2)
# ax.set_zlim(-2,2)
def animate(i):
    frame_history= robot.forward_kine(angle_history[i])
    x=[0]
    y=[0]
    z=[0]
    for i in range(len(frame_history)):
        x.append(frame_history[i][0,3])
        y.append(frame_history[i][1,3])
        z.append(frame_history[i][2,3])
        

    for i in range(len(x)-1):
        # ax.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
        line_list[i].set_data([x[i],x[i+1]],[y[i],y[i+1]])
        line_list[i].set_3d_properties([z[i],z[i+1]])


import matplotlib.animation as animation
ani = animation.FuncAnimation(fig, animate, frames=len(angle_history), interval=100)
plt.show()