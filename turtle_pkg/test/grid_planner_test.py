#! /usr/bin/env python3
import rospy
PKG = 'turtle_pkg'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.
from turtle_pkg.grid_path_planning import planning
import sys
import unittest

class GridPlannerTest(unittest.TestCase):
    ## assert for list
    def test_four_values(self):
        #rospy.init_node('test_pkg')
        ox=  [0, 10, 10, 0, 0]
        oy = [0, 0, 10, 10, 0]
        px_check = [2.5, 4.0, 5.5, 7.0, 7.0, 5.5, 4.0, 2.5, 2.5, 4.0, 5.5, 7.0, 7.0, 5.5, 4.0, 2.5]
        py_check = [2.5, 2.5, 2.5, 2.5, 4.0, 4.0, 4.0, 4.0, 5.5, 5.5, 5.5, 5.5, 7.0, 7.0, 7.0, 7.0]
        reso = 1.5
        
        px, py = planning(ox, oy, reso)
        
        self.assertListEqual(px, px_check)
        self.assertListEqual(py, py_check)



if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, 'gridplannertest', GridPlannerTest)