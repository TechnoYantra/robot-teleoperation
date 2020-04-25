#!/usr/bin/env python

import rospy
from grid_based_sweep_path_planner import planning
from turtle_pkg.msg import GpsCoordinates
from sensor_msgs.msg import NavSatFix



def feedback_cb(msg):
    ox = [msg.lat1, msg.lat2, msg.lat3, msg.lat4, msg.lat1]
    oy = [msg.lng1, msg.lng2, msg.lng3, msg.lng4, msg.lng1]

    reso = 5.0
    rospy.loginfo('got points')

    px, py = planning(ox, oy, reso)
    print(px)
    print('--------')
    print(py)

    start_publishing_gps_goal(px, py)


def start_publishing_gps_goal(px, py):
    for x in range(px):
        gps_pub = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=10)
        gps_data = NavSatFix()
        gps_data.header.frame_id='/world'
        gps_data.latitude=px[x]
        rospy.loginfo('latitude is : '+str(px[x]))
        gps_data.longitude=py[x]
        rospy.loginfo('longitude is : '+str(py[x]))
        rospy.logerr('number is '+str(x))
        rospy.loginfo('publishing next goal')
        gps_pub.publish(gps_data)
        wait_for_succeed()

def wait_for_succeed():
    pass

def move_feedback_cb(msg):
    pass



def main():
    rospy.init_node('lat_lng_node')
    move_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,move_feedback_cb)

    sub = rospy.Subscriber('/lat_lng_sub', GpsCoordinates, feedback_cb)
    rospy.spin()

if __name__ == "__main__":
    main()



