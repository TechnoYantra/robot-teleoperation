#! /home/hunter/development/anaconda3/bin/python

import rospy
from grid_based_sweep_path_planner import planning
from sensor_msgs.msg import NavSatFix
from turtle_pkg.msg import GpsCoordinates
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty



class GridPathPlannig:
    def __init__(self):
        rospy.init_node('grid_path_planning')

        self.lat = 49.9000869191
        self.lng = 8.89990548393
        self.sub = rospy.Subscriber('/lat_lng_sub', GpsCoordinates, self.feedback)

        self.i = 0
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        self.move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.feedback_cb)

        rospy.spin()

    def feedback(self, msg):
        ox = [msg.lat1, msg.lat2, msg.lat3, msg.lat4, msg.lat1]
        oy = [msg.lng1, msg.lng2, msg.lng3, msg.lng4, msg.lng1]

        reso = 3.0
        rospy.loginfo('got points')
        print(ox)
        print(oy)

        self.ix, self.iy = planning(ox, oy, reso)
        self.px = []
        self.py = []
        i = 0
        while i <=(len(self.ix)-1):
            self.px.append(self.ix[i])
            self.py.append(self.iy[i])
            i = i + 3 

        self.end_point = len(self.px)
        print(self.px)
        print('--------')
        print(self.py)

        self.publish_previous_goal(self.i)







    def feedback_cb(self,msg):
        if (msg.status.status == 3):
            rospy.loginfo('result is success')
            rospy.logwarn('clearing costmap')
            #ret = self.clear_costmap()
            self.publish_next_goal(self.i)
        elif (msg.status.status == 2):
            rospy.loginfo('there is some error so clearing the costmap')
            #ret = self.clear_costmap()
            self.publish_previous_goal(self.i)
        elif (msg.status.status == 4):
            rospy.loginfo('there is some warning so clearing the costmap')
            #ret = self.clear_costmap()
            self.publish_previous_goal(self.i)

    def publish_previous_goal(self,i):
        gps_pub = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        gps_data.latitude=self.lat+(0.0000010000*self.px[self.i])
        rospy.loginfo('latitude is : '+str(self.lat+(0.0000010000*self.px[self.i])))
        gps_data.longitude=self.lng+(0.0000010000*self.py[self.i])
        rospy.loginfo('longitude is : '+str(self.lng+(0.0000010000*self.py[self.i])))
        rospy.logerr('number is '+str(self.i))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)



    def publish_next_goal(self,i):
        self.i = self.i+1
        gps_pub = rospy.Publisher('/gps_goal_fix',NavSatFix,queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        rospy.logerr('number is '+str(self.i))
        gps_data.latitude=self.lat+(0.0000010000*self.px[self.i])
        rospy.loginfo('latitude is : '+str(self.lat+(0.0000010000*self.px[self.i])))
        gps_data.longitude=self.lng+(0.0000010000*self.py[self.i])
        rospy.loginfo('longitude is : '+str(self.lng+(0.0000010000*self.py[self.i])))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)






if __name__ == '__main__':
    grdpthpln= GridPathPlannig()

