#!/usr/bin/python3

from re import X
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

class ConvertGcode2motorcortex():
    @staticmethod
    def sign(a):
        if a < 0:
            return -1
        else:
            return 1
        
    @staticmethod
    def circleToThreePoint(point1, point2, radius, direction):
        d = math.sqrt((point1[0] - point2[0])**2 + (point1[1]-point2[1])**2)
        h = math.sqrt(radius**2 - (d/2)**2)

        x01 = point1[0] + (point2[0]-point1[0])/2 + h * (point2[1] - point1[1]) / d
        y01 = point1[1] + (point2[1]-point1[1])/2 - h * (point2[0] - point1[0]) / d

        x02 = point1[0] + (point2[0]-point1[0])/2 - h * (point2[1] - point1[1]) / d
        y02 = point1[1] + (point2[1]-point1[1])/2 + h * (point2[0] - point1[0]) / d

        v1 = [point2[0]-point1[0], point2[1]-point1[1]]
        v2 = [x01 - point1[0], y01 - point1[1]]
        v3 = [x02 - point1[0], y02 - point1[1]]


        angle1 = v1[0]*v2[1] - v1[1]*v2[0]
        angle2 = v1[0]*v3[1] - v1[1]*v3[0]

        if (ConvertGcode2motorcortex.sign(angle1) == direction):
            center = [x01, y01]
        else: 
            center = [x02, y02]

        if (point1[0] == point2[0]):
            p1 = [point1[0] - radius, (point1[1] + point2[0])/2]
            p2 = [point1[0] + radius, (point1[1] + point2[0])/2]
        else:
            x = (point1[0] + point2[0]) / 2
            y_d = math.sqrt(radius**2 - (center[0] - x)**2)
            y1 = center[1] + y_d
            y2 = center[1] - y_d
            p1 = [x, y1]
            p2 = [x, y2]

        angle3 = v1[0]*p1[1] - v1[1]*p1[0]
        if (ConvertGcode2motorcortex.sign(angle3) == direction):
            point3 = p1
        else:
            point3 = p2

        return point3
    
    @staticmethod
    def convert_gcode2motorcortex(scale_, z_up, array):
        points = []
        currentPath = {"type": "", "points":[]}
        scale = scale_ / 10

        lines = gcodeparser.GcodeParser(array).lines
        for line in lines:
            try:
                if (line.command == ('G', 21)):
                    print("Metric system")
                elif (line.command == ('G', 90)):
                    print("Absolyte coordinate")
                elif (line.command == ('G', 0)):
                    if (currentPath["type"] != "L"):
                        points.append(currentPath)
                        currentPath = {"type": "L", "points":[]}
                    
                    currentPath["points"].append([(line.get_param('X')*scale), (line.get_param('Y')*scale), z_up] + to_radians([0, 0.0, 180.0]))
                    currentPath["points"].append([(line.get_param('X')*scale), (line.get_param('Y')*scale), 0] + to_radians([0, 0.0, 180.0]))
                    


                elif (line.command == ('G', 1)):
                    if (currentPath["type"] != 'L'):
                        points.append(currentPath)
                        currentPath = {"type": 'L', "points":[]}
                           
                    currentPath["points"].append([(line.get_param('X')*scale)+100, (line.get_param('Y')*scale)+100, 0] + to_radians([0, 0.0, 180.0]))
                    

                elif (line.command == ('G', 2)):
                    points.append(currentPath)
                    currentPath = {"type": "C", "points":[]}
                    
                    lastPoint = points[-1]['points'][-1]
                    if len(lastPoint) == 3:
                        lastPoint = lastPoint[-1]
                    
                        point3 = ConvertGcode2motorcortex.circleToThreePoint(lastPoint[:2], [(line.get_param('X')*scale), (line.get_param('Y')*scale)], line.get_param('R') * scale, 1)
                    
                    p1 = lastPoint[:2] + [0] + to_radians([0, 0.0, 180.0])
                    p2 = point3 + [0] + to_radians([0, 0.0, 180.0])
                    p3 = [(line.get_param('X')*scale), (line.get_param('Y')*scale), 0] + to_radians([0, 0.0, 180.0])
                    

                    currentPath["points"] = [p1, p2, p3]

                elif (line.command == ('G', 3)):
                    points.append(currentPath)
                    currentPath = {"type": "C", "points":[]}

                    lastPoint = points[-1]['points'][-1]
                    if len(lastPoint) == 3:
                        lastPoint = lastPoint[-1]
                    point3 = ConvertGcode2motorcortex.circleToThreePoint(lastPoint[:2], [(line.get_param('X')*scale), (line.get_param('Y')*scale)], (line.get_param('R')*scale), -1)

                
                    p1 = lastPoint[:2] + [0] + to_radians([0, 0.0, 180.0])
                    p2 = point3 + [0] + to_radians([0, 0.0, 180.0])
                    p3 = [(line.get_param('X')*scale), (line.get_param('Y')*scale), 0] + to_radians([0, 0.0, 180.0])
                    

                    currentPath["points"] = [p1, p2, p3]
            except BaseException:
                pass    
        points.append(currentPath)

        return points
    
    
    
