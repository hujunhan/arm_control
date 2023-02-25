from myurdf import URDF
import numpy as np
import time
class Robot:
    def __init__(self) -> None:
        self.joint_list=[]
        pass
    
    def forward_kine(self,theta_list):
        """Forward kinematics of the robot"""
        ## Calculate the transformation matrix using the origin_translation and origin_orientation and rotation
        all_frame_matrix=np.eye(4)
        frame_history=[]
        for i,joint in enumerate(self.joint_list):
            
            origin_translation=joint['origin_translation']
            origin_orientation=joint['origin_orientation']
            rotation=joint['rotation']
            translation=joint['translation']
            theta=theta_list[i]
            
            frame_matrix = np.eye(4)
            
            # get the translation matrix
            trans_x,trans_y,trans_z=origin_translation
            translation_matrix = np.array([[1, 0, 0, trans_x], [0, 1, 0, trans_y], [0, 0, 1, trans_z], [0, 0, 0, 1]])
            frame_matrix=np.dot(frame_matrix,translation_matrix)
            
            # get the rotation matrix
            roll, pitch, yaw=origin_orientation
            temp_matrix=np.eye(4)
            rotation_matrix=np.dot(self.rz_matrix(yaw), np.dot(self.ry_matrix(pitch), self.rx_matrix(roll)))
            temp_matrix[:3,:3]=rotation_matrix
            frame_matrix=np.dot(frame_matrix,temp_matrix)
            
            # get the theta matrix
            if rotation:
                x,y,z=rotation
                c=np.cos(theta)
                s=np.sin(theta)
                theta_matrix=np.asarray(self._axis_rotation_matrix_formula(x,y,z,c,s))
                temp_matrix[:3,:3]=theta_matrix
                frame_matrix=np.dot(frame_matrix,temp_matrix)
            
            all_frame_matrix=np.dot(all_frame_matrix,frame_matrix)
            frame_history.append(all_frame_matrix)
            # print(f'frane_matrix{i}:\n{frame_matrix}')
        print(f'All frame matrix:\n{all_frame_matrix}')
        return frame_history
            
    
    @staticmethod
    def rx_matrix(theta):
        """Rotation matrix around the X axis"""
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

    @staticmethod
    def rz_matrix(theta):
        """Rotation matrix around the Z axis"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
    @staticmethod
    def ry_matrix(theta):
        """Rotation matrix around the Y axis"""
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
    @staticmethod
    def _axis_rotation_matrix_formula(x, y, z, c, s):
        return [
            [x ** 2 + (1 - x ** 2) * c, x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [x * y * (1 - c) + z * s, y ** 2 + (1 - y ** 2) * c, y * z * (1 - c) - x * s],
            [x * z * (1 - c) - y * s, y * z * (1 - c) + x * s, z ** 2 + (1 - z ** 2) * c]
        ]
if __name__ == '__main__':
    file_path='./urdf/uR10e.urdf'
    urdf=URDF()
    urdf.get_urdf_parameters(file_path)
    for joint in urdf.joint_list:
        # print(joint['rotation'])
        # print(joint['bounds'])
        print(f'name of joint: {joint["name"]}')
    print(f'Number of joints: {len(urdf.joint_list)}')
    robot=Robot()
    robot.joint_list=urdf.joint_list
    theta_list=[0,0,0,0,0,0,0]
    # theta_list=[np.pi/3,np.pi/3]
    
    import matplotlib.pyplot as plt
    ax = plt.figure().add_subplot(projection='3d')
    a=time.time()
    frame_history= robot.forward_kine(theta_list)
    x=[0]
    y=[0]
    z=[0]
    for i in range(len(frame_history)):
        x.append(frame_history[i][0,3])
        y.append(frame_history[i][1,3])
        z.append(frame_history[i][2,3])
        
    # ax.plot(x,y,z,color = plt.cm.rainbow(np.linspace(0, 1, len(x))))
    

    for i in range(len(x)-1):
        ax.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
    ax.set_aspect('equal')
    
    b=time.time()    
    print(f'Forward kinematics time: {b-a}')
    plt.show()