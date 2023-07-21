#!/usr/bin/python3
from symbol import subscript
import motorcortex
import math
import time
from robot_control import to_radians
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np

from scipy.spatial.transform import Rotation as R

import cv2
from cv_bridge import CvBridge
import os
import rospy
import roslib
import rospkg
import tf
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64, Float64MultiArray
import calibration_workspace

import gcodeparser

import matplotlib.pyplot as plt

roslib.load_manifest('svg_convert_srv')
from svg_convert_srv.srv import srv_svg_convert


def sendProgram(robot, motion_program):
    # send the program
    program_sent = motion_program.send("example1").get()
    print(program_sent.status)
    robot_play_state = robot.play()
    # try to play the program
    if robot_play_state == InterpreterStates.PROGRAM_RUN_S.value:
        print("Playing program")
    elif robot_play_state == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
        print("Can not play program, Robot is not at start")
        print("Moving to start")
        if robot.moveToStart(100):
            print("Move to start completed")
            robot_play_state_start = robot.play()
            if robot_play_state_start == InterpreterStates.PROGRAM_RUN_S.value:
                time.sleep(0.1)
                print("Playing program")
            elif robot_play_state_start == InterpreterStates.PROGRAM_IS_DONE.value:
                # pass
                time.sleep(0.1)
                print("Program is done")
            else:
                raise RuntimeError("Failed to play program, state: %s" % robot.getState())
        else:
            raise RuntimeError('Failed to move to start')
    elif robot_play_state == InterpreterStates.PROGRAM_IS_DONE.value:
        time.sleep(0.1)
        print("Program is done")
    else:
        raise RuntimeError("Failed to play program, state: %s" % robot.getState())

    # waiting until the program is finished

    while robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
        time.sleep(0.1)

        

def callback(point_plane:Float64MultiArray):
    print("Принято!")
    # print(point_plane)

  
    waypoint_list = []
    point_finish = []
    for point in point_plane.point.data:
        
        
         point_finish.append(point)

    # Горизонтальная плоскость    
    for i in range(0,3):
        point_finish[i] = point_finish[i]/1000
    
    
    # Вертикальная плоскость
    # for i in range(0,1):
    #     point_finish[i] = point_finish[i]/10

    # for i in range(1,3):
    #     point_finish[i] = point_finish[i]/1000
    

    # for i in range(2,3):
    #     point_finish[i] = (point_finish[i]+200)/1000


        
    for i in range(3,6):
        point_finish[i] = math.radians(point_finish[i])
    
    
    x_array.append(point_finish[1])
    y_array.append(point_finish[2])
    
    print("Получено", point_finish)
    # waypoint_list.append(Waypoint(point_finish, smoothing_factor=0.01))

    joint_params = joint_subscription.read()
    value = joint_params[0].value
    reference_joint_coord = value

    waypoint_list.append(Waypoint(point_finish, smoothing_factor=0.01))
    # point_finish.clear()  
    
        
    motion_program_plane.addMoveL(waypoint_list, 0.5, 0.5)#, ref_joint_coord_rad=reference_joint_coord)
    # motion_program_plane.addWait(2.0)
        # if move['type'] == "C":
        #     waypoint_list = []
        #     for point in move['points']:
        #         waypoint_list.append(Waypoint(point, smoothing_factor=0.001))
        #     motion_program_plane.addMoveC(waypoint_list, 0, 0.2, 0.2)


    # motion_program_plane = MotionProgram(req, motorcortex_types)

    sendProgram(robot, motion_program_plane)

    answer = "Yes!"
    print('Точка отрисована!')
    return answer



def listener():
 
    # c = manipulation()
    # c.connect()
    rospy.init_node('listener', anonymous=True)
    s = rospy.Service('point_service', srv_svg_convert, callback)
   
    # rospy.Subscriber('points_plane', Float64MultiArray, callback)
    rospy.spin()
    
    

if __name__ == '__main__':
    parameter_tree = motorcortex.ParameterTree()
    motorcortex_types = motorcortex.MessageTypes()

    

    try:
        req, sub = motorcortex.connect('ws://10.42.0.1:5568:5567', motorcortex_types, parameter_tree,
                                        timeout_ms=1000, certificate="",
                                        login="", password="")
        
        # req, sub = motorcortex.connect('wss://192.168.2.100:5568:5567', motorcortex_types, parameter_tree,
        #                                 timeout_ms=1000, certificate="mcx.cert.pem",
        #                                 login="admin", password="vectioneer")

        subscription = sub.subscribe(['root/Control/fkActualToolCoord/toolCoordinates'], 'group1', 5)
        joint_subscription = sub.subscribe(['root/Control/fkActualToolCoord/jointPositions'], 'group2', 5)
        subscription.get()

        print("Request connection is etablished")
    except Exception as e:
        print(f"Failed to establish connection: {e}")
        # return False
        
    robot = RobotCommand(req, motorcortex_types)
    

    if robot.engage():
        print('Robot is at Engage')
    else:
        print('Failed to set robot to Engage')

    robot.stop()
    robot.reset()
    
    motion_program_start = MotionProgram(req, motorcortex_types)
    start_position_jnt = Waypoint([math.radians(0.0), math.radians(0.0), math.radians(90.0), math.radians(0.0), math.radians(0.0)])
    motion_program_start.addMoveJ([start_position_jnt], 0.3, 0.3)

    motion_program_plane = MotionProgram(req, motorcortex_types)
    
    sendProgram(robot, motion_program_start)

    x_array = []
    y_array = []
    
    listener()

    plt.plot(x_array,y_array)
    plt.show()