from arm_control.urdf import URDF
from arm_control.robot import Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import trimesh
import time
from arm_control.collision import GJK

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

## Initial joint angles
theta_list = [0, 0, 0, 0, 0, 0, 0]

## Set up the figure
fig = plt.figure()
N = len(theta_list)
fig.subplots_adjust(left=0.1, right=0.7, bottom=0.1, top=1.0)
ax = fig.add_subplot(projection="3d")

# mesh = o3d.io.read_triangle_mesh(str(link_list[1]["collision_mesh_path"]))
mesh = trimesh.load(link_list[1]["collision_mesh_path"], force="mesh")
sample_points = mesh.sample(1000)
# ax.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2])
# plt.show()
# ax.plot_trisurf(
#     mesh.vertices[:, 0],
#     mesh.vertices[:, 1],
#     triangles=mesh.faces,
#     Z=mesh.vertices[:, 2],
# )
# plt.show()


def update(val):
    theta_list = [slider.val for slider in slider_list]
    frame_history = robot.forward_kine(theta_list)

    ax.cla()
    collison_object_list = []
    ## plot the mesh of the robot
    mesh = trimesh.load(link_list[0]["collision_mesh_path"], force="mesh")
    ax.plot_trisurf(
        mesh.vertices[:, 0],
        mesh.vertices[:, 1],
        triangles=mesh.faces,
        Z=mesh.vertices[:, 2],
    )
    collison_object_list.append(mesh.sample(1000))
    for i in range(1, len(frame_history)):
        mesh = trimesh.load(link_list[i]["collision_mesh_path"], force="mesh")
        mesh.apply_transform(frame_history[i - 1])
        sample_points = mesh.sample(1000)
        collison_object_list.append(sample_points)
        ax.plot_trisurf(
            mesh.vertices[:, 0],
            mesh.vertices[:, 1],
            triangles=mesh.faces,
            Z=mesh.vertices[:, 2],
        )
    result = detect_collision(collison_object_list)
    if result is not False:
        ax.title.set_text(f"Collision between {result[0]} and {result[1]}")
    else:
        ax.title.set_text("No collision")
    ax.set_aspect("equal")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)


def detect_collision(collison_object_list):
    for i in range(len(collison_object_list) - 1):
        for j in range(i + 2, len(collison_object_list)):
            detector = GJK()
            result = detector.super_check(
                collison_object_list[i], collison_object_list[j]
            )
            if result:
                print(f"collision between {i} and {j}")
                return [i, j]
    return False


## Add sliders
ax_list = []
slider_list = []
for i in range(N):
    axe = fig.add_axes([0.8, 1 - 0.9 * (i + 1) / N, 0.15, 0.8 / N])
    slider = Slider(axe, f"joint{i}", -np.pi, np.pi, valinit=theta_list[i])
    slider.on_changed(update)
    slider_list.append(slider)


## Plot the robot in the initial position
a = time.time()
frame_history = robot.forward_kine(theta_list)


## plot the mesh of the robot
mesh = trimesh.load(link_list[0]["collision_mesh_path"], force="mesh")
ax.plot_trisurf(
    mesh.vertices[:, 0],
    mesh.vertices[:, 1],
    triangles=mesh.faces,
    Z=mesh.vertices[:, 2],
)
for i in range(1, len(frame_history)):
    mesh = trimesh.load(link_list[i]["collision_mesh_path"], force="mesh")
    mesh.apply_transform(frame_history[i - 1])
    ax.plot_trisurf(
        mesh.vertices[:, 0],
        mesh.vertices[:, 1],
        triangles=mesh.faces,
        Z=mesh.vertices[:, 2],
    )

ax.set_aspect("equal")
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
b = time.time()
print(f"Forward kinematics time: {b-a}")
plt.show()
