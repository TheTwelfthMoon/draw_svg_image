#!/usr/bin/python3
import svg_to_gcode
import math
import time


from svg_to_gcode.svg_parser import parse_file
from svg_to_gcode.compiler import Compiler, interfaces

import cv2
from cv_bridge import CvBridge
import os
import rospy

import rospkg
import tf
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64, Float64MultiArray
import SVG_GCODE_convert
import GCODE_MC_convert
import calibration_workspace
import normal
# from svg_convert_package.srv import svg_convert_server
import roslib
roslib.load_manifest('svg_convert_srv')
from svg_convert_srv.srv import *

def main():

    # rospy.init_node('automatic_server')
    
    rospy.wait_for_service('point_service')
    try:
        point_client= rospy.ServiceProxy('point_service', srv_svg_convert)
        # answer = "Listen"
        # result = point_service(answer)
        # callback(result.point)
        
    except rospy.ServiceException:
         pass
    
    # s = rospy.Service('point_service', srv_svg_convert, automatic)
    # pub = rospy.Publisher('points_plane', Float64MultiArray, queue_size=10)
    # rate = rospy.Rate(10) # 10hz
    print("Ожидаю подключения")
    # rospy.spin()
    
    

    rospack = rospkg.RosPack()
    pkgDir = rospack.get_path('data_pkg')
    image = os.path.join(pkgDir, 'data', 'logo_AR_blue.svg')
    array = []
    svg_con_gcode = SVG_GCODE_convert.convert_svg2gcode.convert_svg2gcode(image)

    array.append(svg_con_gcode)
    array_command_gc = '\n'.join(array)
    points = GCODE_MC_convert.convert_gcode2motorcortex.convert_gcode2motorcortex(1, 0.02, array_command_gc)

# Место для ноды чтения точек плоскости с манипулятора


    #Горизнтальная плоскость
    # p1 = [ -0.347, -0.026, 0.088]
    # p2 = [-0.288, -0.026, 0.088]
    # p3 = [-0.288, 0.072, 0.088]

    # p1 = [ -347, -26, 88]
    # p2 = [-288, -26, 88]
    # p3 = [-288, 72, 88]

    # Вертикальная поверхность
    # p1 = [0.356, 0.160, 0.3]
    # p2 = [0.352, 0.158, 0.142]
    # p3 = [0.378, 0.078, 0.142]

    # p1 = [356, 160, 300]
    # p2 = [356, 160, 142]
    # p3 = [356, 257, 142]


    # random
    p1 = [-442, 133, 172]
    p2 = [-404, 121, 92]
    p3 = [-419, -4, 92]


    # angle = [145.45, 0.69, 180]

    angle = normal.normal_ws.normal_calculation(p1, p2, p3)

    print("angle:", angle)
    # print(type(angle))

    
    tf_table = calibration_workspace.calibration_ws.getTfByPoints(p1, p2, p3)
    p = calibration_workspace.calibration_ws.transformGCodePointToPlaneWithNormal(tf_table, points, angle)

    # print(points_plane)

    for move in p:
        if move['type'] == "L":
            for point in move['points']:
                my_msg = Float64MultiArray()
                my_msg.data = point
                # pub.publish(my_msg)
                
                print("Отправлено", point)
                point_client(my_msg)
                # rospy.spin()
                # time.sleep(0.1)
                
                # return srv_svg_convertResponse(my_msg)
                
                
                # service = rospy.Service("svg_convert_service", svg_convert_server, callback_service)
                # rospy.spin()
                # time.sleep(0.1)
                


    

    print("Всё отправлено!")


# callback_service(answer):
#     return


if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass

