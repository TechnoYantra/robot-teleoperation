#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from turtle_pkg.msg import GpsCoordinates
from geometry_msgs.msg import Twist

class IntPublisher():

    def __init__(self):
        rospy.init_node('int_publisher')
        self.sub = rospy.Subscriber('int_publisher',Float32MultiArray, self.feedback_cb)
        self.yug_sub = rospy.Subscriber('/yug_pose', Twist, self.yug_feedback)
        self.i = 0
        self.data = Float32MultiArray()
        rospy.spin()


    def yug_feedback(self,msg):
        self.data.data.append(msg.linear.x)
        self.data.data.append(msg.linear.y)
        print('x is ')
        print(msg.linear.x)
        print('y is ')
        print('msg.linear.y')
        self.i = self.i+1
        print('done')
        if (self.i>=3):
            pub = rospy.Publisher('int_publisher', Float32MultiArray, queue_size=10)
            print(self.data)
            print('ready')
            pub.publish(self.data)

    def feedback_cb(self,data):
        msg = GpsCoordinates()
        msg.lat1 = data.data[0]
        msg.lat2 = data.data[2] 
        msg.lat3 = data.data[4] 
        msg.lat4 = data.data[6]

        msg.lng1 = data.data[1]
        msg.lng2 = data.data[3] 
        msg.lng3 = data.data[5] 
        msg.lng4 = data.data[7]
        gps_pub = rospy.Publisher('/lat_lng_sub', GpsCoordinates, queue_size=10)
        gps_pub.publish(msg)

        rospy.loginfo("publishing to the gps")

if __name__ == "__main__":
    int_pub = IntPublisher()
    