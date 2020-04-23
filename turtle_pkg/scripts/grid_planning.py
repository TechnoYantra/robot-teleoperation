#! /usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionResult



class GridPathPlannig:
    def __init__(self):
        rospy.init_node('grid_path_planning')

        self.lat = 49.9000869191
        self.lng = 8.89990548393
        move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.feedback_cb)

        rospy.spin()





    def feedback_cb(self,msg):
        if (msg.status.status == 3):
            rospy.loginfo('result is success')
            self.publish_next_goal()



    def publish_next_goal(self):
        gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        self.lat=self.lat+0.00000004000
        gps_data.latitude=self.lat
        rospy.loginfo('latitude is : '+str(self.lat))
        gps_data.longitude=self.lng
        rospy.loginfo('longitude is : '+str(self.lng))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)






if __name__ == '__main__':
    grdpthpln= GridPathPlannig()

