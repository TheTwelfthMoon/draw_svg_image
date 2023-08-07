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

import MathTrans

import gcodeparser

class NormalWs():
    @staticmethod
    def normal_calculation(p1, p2, p3):
        a = p1
        b = p2
        c = p3

        ba = [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
        bc = [c[0]-b[0], c[1]-b[1], c[2]-b[2]]

        print("bc", bc)
        print("ba", ba)

        normal = (np.array(np.cross(bc, ba)))
        antinormal = normal*(-1)
        print("antinormal", antinormal)
        norm_antinormal = [(antinormal[0]/math.sqrt((antinormal[0]**2)+(antinormal[1]**2)+(antinormal[2]**2))), (antinormal[1]/math.sqrt((antinormal[0]**2)+(antinormal[1]**2)+(antinormal[2]**2))), 
                       (antinormal[2]/math.sqrt((antinormal[0]**2)+(antinormal[1]**2)+(antinormal[2]**2)))]
        print("antinormnormal", norm_antinormal)
        
        print("modX", (bc[2])/math.sqrt(((bc[0])**2)+((bc[1])**2)+((bc[2])**2)))
        newX = [((bc[0])/math.sqrt(((bc[0])**2)+((bc[1])**2)+((bc[2])**2))), ((bc[1])/math.sqrt(((bc[0])**2)+((bc[1])**2)+((bc[2])**2))), 
                ((bc[2])/math.sqrt(((bc[0])**2)+((bc[1])**2)+((bc[2])**2)))]
        
        newY = np.cross(newX, norm_antinormal)
        
        newMatrix = [[newX[0], newY[0], norm_antinormal[0]], [newX[1], newY[1], norm_antinormal[1]], [newX[2], newY[2], norm_antinormal[2]]]

        newMatrix = np.array(newMatrix)
        print("new_matrix", newMatrix)
        print("Х:", newMatrix[:, 0])
        print("У:",newMatrix[:, 1])
        print("Z:",newMatrix[:, 2])

        euler = MathTrans.MathTrans.rot2euler(newMatrix)

        angle = [0, 0, 0]
        for i in range(0,3):
            
            angle[i] = math.degrees(euler[i])
        return angle
