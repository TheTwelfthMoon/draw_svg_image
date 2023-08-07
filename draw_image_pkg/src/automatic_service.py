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
import roslib

import rospkg
import tf
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64, Float64MultiArray
import SVG_GCODE_convert
import GCODE_MC_convert
import calibration_workspace
import normal

# roslib.load_manifest('draw_image')
from draw_image_pkg.srv import *

def main():

    rospy.wait_for_service('point_service')
    try:
        point_client= rospy.ServiceProxy('point_service', draw_image)

    except rospy.ServiceException:
         pass
    
    print("Ожидаю подключения")

    rospack = rospkg.RosPack()
    pkgDir = rospack.get_path('data_pkg')
    image = os.path.join(pkgDir, 'data', 'logo_AR_blue.svg')
    array = []
    svg_con_gcode = SVG_GCODE_convert.ConvertSvg2gcode.convert_svg2gcode(image)

    array.append(svg_con_gcode)
    array_command_gc = '\n'.join(array)
    points = GCODE_MC_convert.ConvertGcode2motorcortex.convert_gcode2motorcortex(1, 0.02, array_command_gc)

# Место для ноды чтения точек плоскости с манипулятора


    #Горизнтальная плоскость
    # p1 = [ -0.347, -0.026, 0.088]
    # p2 = [-0.288, -0.026, 0.088]
    # p3 = [-0.288, 0.072, 0.088]

    # Вертикальная поверхность
    # p1 = [0.356, 0.160, 0.3]
    # p2 = [0.352, 0.158, 0.142]
    # p3 = [0.378, 0.078, 0.142]

    # random
    p1 = [-442, 133, 172]
    p2 = [-404, 121, 92]
    p3 = [-419, -4, 92]

    angle = normal.NormalWs.normal_calculation(p1, p2, p3)

    print("angle:", angle)
 
    tf_table = calibration_workspace.CalibrationWs.getTfByPoints(p1, p2, p3)
    p = calibration_workspace.CalibrationWs.transformGCodePointToPlaneWithNormal(tf_table, points, angle)

    for move in p:
        if move['type'] == "L":
            for point in move['points']:
                my_msg = Float64MultiArray()
                my_msg.data = point
                
                print("Отправлено", point)
                point_client(my_msg)


    print("Всё отправлено!")

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass

