#! /usr/bin/env python

import rospy
import os
import rospkg

def main():
    rospy.init_node('python_server')
    rospack = rospkg.RosPack()
    print(rospack.get_path('robot_gui_bridge'))
    path=rospack.get_path('robot_gui_bridge')
    print('server is ready')
    os.system('cd '+path+'; cd gui; python3 -m http.server')
    
    

if __name__ == "__main__":
    main()