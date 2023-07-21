#!/usr/bin/python3
import svg_to_gcode


from svg_to_gcode.svg_parser import parse_file
from svg_to_gcode.compiler import Compiler, interfaces

import cv2
from cv_bridge import CvBridge
import os
import rospy
import roslib
import rospkg
import tf
from std_msgs.msg import String


# def parse():
#     gcode_compiler = Compiler(interfaces.Gcode, movement_speed=1000, cutting_speed=300, pass_depth=5)

#     rospack = rospkg.RosPack()
#     pkgDir = rospack.get_path('svg_convert_package')
#     File = os.path.join(pkgDir, 'data', 'logo_AR_blue.svg')
#     # curves = parse_file("logo_AR_blue.svg") # Parse an svg file into geometric curves
#     curves = parse_file(File)
#     gcode_compiler.append_curves(curves)
#     R = gcode_compiler.compile(passes=2)

#     return R


# def convert_svg2gcode():
    
#     pub = rospy.Publisher('/convert_gcode', String, queue_size=10)
#     rospy.init_node('motorcortex_proxy')
#     r = rospy.Rate(10) # 10hz
    
#     # print(type(R)) 
#     while not rospy.is_shutdown():
        
#         # R = gcode_compiler.compile(passes=2)
#         W = parse()
#         pub.publish(str(W))
#         r.sleep()



# if __name__ == '__main__':
#     # it is good practice to maintain
#     # a 'try'-'except' clause
#     try:
#         convert_svg2gcode()
#     except rospy.ROSInterruptException:
#         pass


class convert_svg2gcode():
    
    @staticmethod
    def convert_svg2gcode(File):
        gcode_compiler = Compiler(interfaces.Gcode, movement_speed=1000, cutting_speed=300, pass_depth=5)

        
        # curves = parse_file("logo_AR_blue.svg") # Parse an svg file into geometric curves
        curves = parse_file(File)
        gcode_compiler.append_curves(curves)
        # gcode_compiler.compile_to_file("w_drawing.gcode", passes=2)
        gc = gcode_compiler.compile(passes=2)

        return gc

