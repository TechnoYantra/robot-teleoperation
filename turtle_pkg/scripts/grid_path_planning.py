#! /home/hunter/development/anaconda3/bin/python

import rospy
from grid_based_sweep_path_planner import planning
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionResult



class GridPathPlannig:
    def __init__(self):
        rospy.init_node('grid_path_planning')

        lat = 49.9000869191
        lng = 8.89990548393
        ox = [lat, lat+0.0000200, lat+0.0000200, lat, lat]
        oy = [lng, lng, lng+0.0000200, lng+0.0000200, lng]
        reso = 0.0000025

        self.px, self.py = planning(ox, oy, reso)

        self.i = 0
        move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.feedback_cb)

        rospy.spin()



        

    def feedback_cb(self,msg):
        if (msg.status.status == 3):
            rospy.loginfo('result is success')
            self.publish_next_goal(self.i)

    

    def publish_next_goal(self,i):
        gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        gps_data.latitude=self.px[self.i]
        gps_data.longitude=self.py[self.i]
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)
        self.i = self.i+1


    

    

if __name__ == '__main__':
    grdpthpln= GridPathPlannig()

