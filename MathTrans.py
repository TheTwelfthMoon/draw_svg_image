import numpy as np
from math import cos, sin, sqrt, atan2
from typing import Union

class MathTrans():
    @staticmethod
    def eulerTrans2tf(euler: list, trans: list) -> np.ndarray:
        R = MathTrans.euler2rot(euler)
        R = R.tolist()
        R[0].append(trans[0])
        R[1].append(trans[1])
        R[2].append(trans[2])
        R.append([0,0,0,1])
        return np.array(R)
    
    @staticmethod
    def tf2eulerTrans(tf: np.ndarray) -> Union[np.ndarray, np.ndarray]:
        trans = tf[:3,3]
        euler = MathTrans.rot2euler(tf[:3,:3])
        return euler, trans

    @staticmethod
    def rot2euler(rot: np.ndarray) -> np.ndarray:
        sy = sqrt(rot[0,0] * rot[0,0] +  rot[1,0] * rot[1,0])
    
        singular = sy < 1e-6
    
        if  not singular :
            x = atan2(rot[2,1] , rot[2,2])
            y = atan2(-rot[2,0], sy)
            z = atan2(rot[1,0], rot[0,0])
        else :
            x = atan2(-rot[1,2], rot[1,1])
            y = atan2(-rot[2,0], sy)
            z = 0
    
        return np.array([z, y, x])

    @staticmethod
    def euler2rot(euler: list) -> np.ndarray:
        rotx = np.array([[1, 0, 0],
                    [0, cos(euler[2]), -sin(euler[2])],
                    [0, sin(euler[2]), cos(euler[2])]])

        roty = np.array([[cos(euler[1]), 0, sin(euler[1])],
                    [0, 1, 0],
                    [-sin(euler[1]), 0, cos(euler[1])]])

        rotz = np.array([[cos(euler[0]), -sin(euler[0]), 0],
                        [sin(euler[0]), cos(euler[0]), 0],
                        [0, 0, 1]])
        
        R = rotz@roty@rotx

        return R

    @staticmethod
    def quatTrans2tf(quat: list, trans: list) -> np.ndarray:
        q = -1*np.array([quat[3], quat[0], quat[1], quat[2]])
        rMatrix = [[1-2*(q[2]**2 + q[3]**2), 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2])],
                    [2*(q[1]*q[2]+q[0]*q[3]), 1-2*(q[1]**2 + q[3]**2), 2*(q[2]*q[3]-q[0]*q[1])],
                    [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2)]]

        for i,el in enumerate(trans):
            rMatrix[i].append(el)
        rMatrix.append([0,0,0,1])

        return np.array(rMatrix)