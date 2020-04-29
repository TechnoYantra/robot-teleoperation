#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty

from move_base_msgs.msg import MoveBaseActionResult




def main():
    rospy.init_node('start_goal_server')
    def start_move(request):
        print('done')
        data = MoveBaseActionResult()
        data.status.status = 3
        pub.publish(data)
        
    server = rospy.Service('/start_goal_server', Empty, start_move)
    pub = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size=10)
    
    

    rospy.spin()


if __name__ == "__main__":
    main()