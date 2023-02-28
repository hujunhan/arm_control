from arm_control.urdf import URDF
import numpy as np
import time
import sympy as sym
class Robot:
    def __init__(self) -> None:
        self.joint_list=[]
        self.symbolic=False
        pass
    
    def forward_kine(self,theta_list):
        """Forward kinematics of the robot"""
        ## Calculate the transformation s'y'm using the origin_translation and origin_orientation and rotation
        self.symbolic=False
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
        # print(f'All frame matrix:\n{all_frame_matrix}')
        return frame_history
    
    ## TODO: use sympy to calculate the jacobian matrix
    # do the same thing as forward_kine function, but use sympy to calculate the jacobian matrix
    def forward_kine_sympy(self,theta_list):
        self.symbolic=True
        ## Calculate the transformation matrix using the origin_translation and origin_orientation and rotation
        all_frame_matrix=sym.Matrix.eye(4)
        frame_history=[]
        for i,joint in enumerate(self.joint_list):

            origin_translation=sym.Matrix(joint['origin_translation'])
            origin_orientation=sym.Matrix(joint['origin_orientation'])
            rotation=joint['rotation']
            translation=joint['translation']
            theta=sym.symbols('theta'+str(i))

            frame_matrix = sym.Matrix.eye(4)

            # get the translation matrix
            trans_x,trans_y,trans_z=origin_translation
            translation_matrix = sym.Matrix([[1, 0, 0, trans_x], [0, 1, 0, trans_y], [0, 0, 1, trans_z], [0, 0, 0, 1]])
            frame_matrix=frame_matrix*translation_matrix

            # get the rotation matrix
            roll, pitch, yaw=origin_orientation
            temp_matrix=sym.Matrix.eye(4)
            rotation_matrix=self.rz_matrix(yaw)*self.ry_matrix(pitch)*self.rx_matrix(roll)
            temp_matrix[:3,:3]=rotation_matrix
            frame_matrix=frame_matrix*temp_matrix

            # get the theta matrix
            if rotation:
                x,y,z=rotation
                c=sym.cos(theta)
                s=sym.sin(theta)
                theta_matrix=sym.Matrix(self._axis_rotation_matrix_formula(x,y,z,c,s))
                temp_matrix[:3,:3]=theta_matrix
                frame_matrix=frame_matrix*temp_matrix

            all_frame_matrix=all_frame_matrix*frame_matrix
            frame_history.append(all_frame_matrix)
            # print(f'frane_matrix{i}:\n{frame_matrix}')
        # print(f'All frame matrix:\n{all_frame_matrix}')
        # all_frame_matrix = all_frame_matrix.subs(theta_dict)
        return frame_history
        
    

    def rx_matrix(self,theta):
        """Rotation matrix around the X axis"""
        if self.symbolic:
            return sym.Matrix([
                [1, 0, 0],
                [0, sym.cos(theta), -sym.sin(theta)],
                [0, sym.sin(theta), sym.cos(theta)]
            ])
        else:
            return np.array([
                [1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)]
            ])
    def rz_matrix(self,theta):
        """Rotation matrix around the Z axis"""
        if self.symbolic:
            return sym.Matrix([
                [sym.cos(theta), -sym.sin(theta), 0],
                [sym.sin(theta), sym.cos(theta), 0],
                [0, 0, 1]
            ])
        else:
            return np.array([
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]
            ])
    def ry_matrix(self,theta):
        """Rotation matrix around the Y axis"""
        if self.symbolic:
            return sym.Matrix([
                [sym.cos(theta), 0, sym.sin(theta)],
                [0, 1, 0],
                [-sym.sin(theta), 0, sym.cos(theta)]
            ])
        else:
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
    VISUALIZE=True
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
    theta_list=[0,np.pi/3,0,0,0,0,0]
    theta_dict = {sym.symbols('theta'+str(i)):theta_list[i] for i in range(len(theta_list))}
    
    # theta_list=[np.pi/3,np.pi/3]
    
    frame_history= robot.forward_kine_sympy(theta_list)
    print(frame_history[-1].subs(theta_dict)[0:3,3])
    if VISUALIZE:
        import matplotlib.pyplot as plt
        ax = plt.figure().add_subplot(projection='3d')
        a=time.time()
        frame_history= robot.forward_kine(theta_list)
        print(frame_history[-1][0:3,3])
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
        