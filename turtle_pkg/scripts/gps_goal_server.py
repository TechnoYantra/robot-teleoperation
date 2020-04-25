#! /usr/bin/env python

import rospy
import actionlib
from turtle_pkg.srv import GpsPoints, GpsPointsResponse
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction
from sensor_msgs.msg import NavSatFix



class GPS_Goal:
    def __init__(self):
        rospy.init_node('gps_goal_server')
        self.service = rospy.Service('/gps_goal_server', GpsPoints, self.gps_move_base)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.success = False
        rospy.spin()


    def gps_move_base(self,request):
        gps_pub = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=10)
        gps_data = NavSatFix()
        self.success = False
        gps_data.header.frame_id='/world'
        gps_data.latitude=request.lat
        rospy.loginfo('latitude is : '+str(request.lat))
        gps_data.longitude=request.lng
        rospy.loginfo('longitude is : '+str(request.lng))
        rospy.logerr('number is '+str(request.lng))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)

        self.client.wait_for_result()

        if (self.client.get_result().status.status == 3):
            return True

        return False

    def feedback_cb(self,msg):
        if (msg.status.status == 3):
            rospy.loginfo('Goal Reached')


if __name__ == "__main__":
    gps = GPS_Goal()
