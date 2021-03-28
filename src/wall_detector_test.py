#!/usr/bin/env python2

import sys, unittest, time
import numpy as np
import rospy, rostest
from rostest_example.msg import Line, ScanPoints

class TestWallDetector(unittest.TestCase):
    def __init__(self, *args):
        super(TestWallDetector, self).__init__(*args)
    
    def callback(self, data):
        self.line_params = [data.m, data.b] # Store to check values in test cases
        self.done = True # Set done to true in order to end waiting loop

    # First test case
    def test_wall_detector_1(self):
        self.done = False
        
        # Create test node which will be publishing and subscribing to topics that wall_detector interfaces with.
        rospy.init_node("wall_detector_test")
        pub = rospy.Publisher("points", ScanPoints)

        gt_m, gt_b = 1, 10 # The ground truth slope and intercept of the line that WallDetector should find.
        # Generate data points. Can also add random noise here to test for robustness.
        x = np.array([1, 2, 3, 4, 5, 6])
        y = gt_m * x + gt_b

        # Create test message for publication
        points = ScanPoints()
        points.x = x.tolist()
        points.y = y.tolist()

        sub = rospy.Subscriber("line", Line, self.callback)
        
        # Wait ten seconds -- can use ROS blocking functions instead
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.done and time.time() < timeout_t:
            pub.publish(points)
            time.sleep(0.1)
        
        self.assertAlmostEqual(gt_m, self.line_params[0])
        self.assertAlmostEqual(gt_b, self.line_params[1])

    # Second test case
    def test_wall_detector_2(self):
        self.done = False

        rospy.init_node("wall_detector_test")
        pub = rospy.Publisher("points", ScanPoints)

        gt_m, gt_b = -3, -1 # The ground truth slope and intercept of the line that WallDetector should find.
        x = np.array([1, 2, 3, 4, 5, 6])
        y = gt_m * x + gt_b

        # Create test message for publication
        points = ScanPoints()
        points.x = x.tolist()
        points.y = y.tolist()

        sub = rospy.Subscriber("line", Line, self.callback)
        
        # Wait ten seconds -- can use ROS blocking functions instead
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.done and time.time() < timeout_t:
            pub.publish(points)
            time.sleep(0.1)
        
        self.assertAlmostEqual(gt_m, self.line_params[0])
        self.assertAlmostEqual(gt_b, self.line_params[1])

if __name__ == '__main__':
    rostest.rosrun("rostest_example", "wall_detector_test", TestWallDetector, sys.argv)