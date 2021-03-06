#! /home/hunter/development/anaconda3/bin/python

import rospy
from grid_based_sweep_path_planner import planning
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty



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
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.feedback_cb)

        rospy.spin()





    def feedback_cb(self,msg):
        if (msg.status.status == 3):
            rospy.loginfo('result is success')
            rospy.logwarn('clearing costmap')
            ret = self.clear_costmap()
            self.publish_next_goal(self.i)
        elif (msg.status.status == 2):
            rospy.loginfo('there is some error so clearing the costmap')
            ret = self.clear_costmap()
            self.publish_previous_goal(self.i)
        elif (msg.status.status == 4):
            rospy.loginfo('there is some warning so clearing the costmap')
            ret = self.clear_costmap()
            self.publish_previous_goal(self.i)

    def publish_previous_goal(self,i):
        gps_pub = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        gps_data.latitude=self.px[self.i]
        rospy.loginfo('latitude is : '+str(self.px[self.i]))
        gps_data.longitude=self.py[self.i]
        rospy.loginfo('longitude is : '+str(self.py[self.i]))
        rospy.logerr('number is '+str(self.i))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)



    def publish_next_goal(self,i):
        self.i = self.i+1
        gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        rospy.logerr('number is '+str(self.i))
        gps_data.latitude=self.px[self.i]
        rospy.loginfo('latitude is : '+str(self.px[self.i]))
        gps_data.longitude=self.py[self.i]
        rospy.loginfo('longitude is : '+str(self.py[self.i]))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)






if __name__ == '__main__':
    grdpthpln= GridPathPlannig()

