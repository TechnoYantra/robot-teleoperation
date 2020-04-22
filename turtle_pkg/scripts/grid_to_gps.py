#! /usr/bin/env python

import rospy
from random import *
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionResult

# global variable


def feedback_cb(msg):
    if (msg.status.status == 3):
        rospy.loginfo('result is success')
        publish_next_goal()





def publish_next_goal():
    gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
    gps_data = NavSatFix()
    gps_data.header.frame_id='/world'
    x = randint(1,100)
    latitude=49.9000869191+(0.00000100*x)
    gps_data.latitude=latitude
    gps_data.longitude=8.89990548393
    rospy.loginfo('publishing next goal')
    gps_pub.publish(gps_data)


def main():
    rospy.init_node('grid_to_gps')
    gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
    move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,feedback_cb)
    gps_data = NavSatFix()
    gps_data.header.frame_id='/world'
    gps_data.latitude=49.9000871955
    gps_data.longitude=8.89990548393

    rate=rospy.Rate(10) # 10 hz
    rospy.loginfo('publishing first goal')
    gps_pub.publish(gps_data)
    rospy.spin()

if __name__ == '__main__':
    main()
