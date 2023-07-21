#!/usr/bin/python3
import motorcortex
import math
import time
from robot_control import to_radians
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np

from scipy.spatial.transform import Rotation as R

import gcodeparser

class calibration_ws():
    @staticmethod
    def getTfByPoints(p1, p2, p3):
        # x = p1[0]
        # y = p1[1]
        # z = p1[2]

        # yx_ = p2[0] - x
        # yy_ = p2[1] - y
        # yz_ = p2[2] - z

        # xx = p3[0] - x
        # xy = p3[1] - y
        # xz = p3[2] - z

        x = p2[0]
        y = p2[1]
        z = p2[2]

        yx_ = p1[0] - x
        yy_ = p1[1] - y
        yz_ = p1[2] - z

        xx = p3[0] - x
        xy = p3[1] - y
        xz = p3[2] - z

        # zx = xy * yz_
        # zy = -xz * yx_
        # zz = xx*yy_

        zx = (xy*yz_ - xz*yy_)
        zy = -(xx*yz_ - xz*yx_)
        zz = (xx*yy_ - xy*yx_)

        yx = zy*xz - zz*xy
        yy = -(zx*xz-zz*xx)
        yz = zx*xy - zy*xx

        # yx = zy*xz
        # yy = -zz*xx
        # yz = zx*xy

        x_v_n = np.linalg.norm(np.array([xx, xy, xz]))
        [xx, xy, xz] = np.array([xx, xy, xz])/x_v_n

        y_v_n = np.linalg.norm(np.array([yx, yy, yz]))
        [yx, yy, yz] = np.array([yx, yy, yz])/y_v_n

        z_v_n = np.linalg.norm(np.array([zx, zy, zz]))
        [zx, zy, zz] = np.array([zx, zy, zz])/z_v_n

        tf = np.array([[xx, yx, zx, x],
                    [xy, yy, zy, y],
                    [xz, yz, zz, z],
                    [0, 0, 0, 1]])
        
        print("tf", tf)
        
        return tf
    
    @staticmethod
    def transformGCodePointToPlaneWithNormal(matrix, points, angle):
        # x = matrix[0:3, 0]
        # y = matrix[0:3, 1]
        # z = matrix[0:3, 2]
        # antiNormal = z*-1
        # antiNormal = [0, angle, math.pi]# - angle]
        # angle_dg = [0,0,0]
        # for i in range(0,3):
            
        #     angle_dg[i] = math.degrees(angle[i])

        antiNormal = angle
        for point_set in points:
            for i in range(len(point_set["points"])):
                point_m = point_set["points"][i][0:3] + [1]
                point_set["points"][i] = matrix.dot(point_m).tolist()[0:3] + antiNormal
        return points