#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg

from rostest_example.msg import Line, ScanPoints

class WallDetector:
    def __init__(self):
        self.line_pub = rospy.Publisher("line", Line)
        rospy.Subscriber("points", ScanPoints, self.wall_detector_callback)

    def wall_detector_callback(self, message):
        x, y = np.array(message.x), np.array(message.y) # Receives points as x and y arrays, elements correspond to each other.
        m, b = np.polyfit(x, y, 1) # Fits line to data
        
        line = Line() # Custom message -- just contains slope/intercept value fields
        line.m = m
        line.b = b
        
        self.line_pub.publish(line)

if __name__ == "__main__":
    rospy.init_node('wall_detector')
    wall_detector = WallDetector()
    rospy.spin()