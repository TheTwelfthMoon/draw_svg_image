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


class ConvertSvg2gcode():
    
    @staticmethod
    def convert_svg2gcode(File):
        gcode_compiler = Compiler(interfaces.Gcode, movement_speed=1000, cutting_speed=300, pass_depth=5)

        curves = parse_file(File)
        gcode_compiler.append_curves(curves)
        gc = gcode_compiler.compile(passes=2)

        return gc

