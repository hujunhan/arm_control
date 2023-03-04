from arm_control.urdf import URDF
from arm_control.robot import Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


def get_line_points(start_point,end_point,n_points=100):
    t=np.linspace(0.01,1,n_points)
    via_points=[]
    for i in range(len(t)):
        via_points.append(start_point*(1-t[i])+end_point*t[i])
    return via_points

def get_circle_points(start_point,radius,n_points=100):
    t=np.linspace(0,2*np.pi,n_points)
    via_points=[]
    x = 16/32*np.sin(t)**3
    y = (13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t))/32
    for i in range(len(t)):
        via_points.append(start_point+np.asarray([x[i],0,y[i]]))
    return via_points

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
theta_list_current=np.asarray([1,-1.1,1.2,-1.3,-1.4,0,0])
theta_list_target=np.asarray([1,-1.2,-1.3,2.4,-1.5,1,0])

current_position=robot.fk_lambda(theta_list_current)[0:3,3]
target_position=robot.fk_lambda(theta_list_target)[0:3,3]
print(f'current position: {current_position}')
print(f'target position: {target_position}')

## interpolate the robot to move from the current position to the target position
via_points=get_circle_points(current_position,target_position)
# print(f'via_points: {via_points}')
angle_history=[theta_list_current]
for i in range(len(via_points)):
    temp_history=robot.ik_transpose(angle_history[-1],via_points[i],max_iteration=1000)
    if len(temp_history)>0:
        angle_history.append(temp_history[-1])
    # angle_history.append(temp_history[-1])
# print(f'angle_history: {angle_history}')


## Use the matplotlib to plot the robot
# animate the robot moving from the initial position to the target position
fig=plt.figure()
ax = fig.add_subplot(projection='3d')
# robot arm lines
arm_line_list=[]
for i in range(len(robot.joint_list)):
    line,=ax.plot([],[],[])
    arm_line_list.append(line)
    
# end effector lines
end_effector_line_list=[]    
for i in range(len(angle_history)):
    line,=ax.plot([],[],[],'r')
    end_effector_line_list.append(line)
ax.view_init(elev=17, azim=90, roll=0)
ax.set_aspect('equal')
axis_limit=1.5
ax.set_xlim(-axis_limit,axis_limit)
ax.set_ylim(-axis_limit,axis_limit)
ax.set_zlim(-axis_limit,axis_limit)
end_effector_xyz_history=[]
def animate(i):
    frame_history= robot.forward_kine(angle_history[i])
    x=[0]
    y=[0]
    z=[0]
    for ii in range(len(frame_history)):
        x.append(frame_history[ii][0,3])
        y.append(frame_history[ii][1,3])
        z.append(frame_history[ii][2,3])
    end_effector_xyz_history.append([x[-1],y[-1],z[-1]])
    if i>0:
        from_point=end_effector_xyz_history[-2]
        to_point=end_effector_xyz_history[-1]
        end_effector_line_list[i].set_data([from_point[0],to_point[0]],[from_point[1],to_point[1]])
        end_effector_line_list[i].set_3d_properties([from_point[2],to_point[2]])
    for ii in range(len(x)-1):
        # ax.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
        arm_line_list[ii].set_data([x[ii],x[ii+1]],[y[ii],y[ii+1]])
        arm_line_list[ii].set_3d_properties([z[ii],z[ii+1]])


import matplotlib.animation as animation
ani = animation.FuncAnimation(fig, animate, frames=len(angle_history), interval=100)
plt.show()