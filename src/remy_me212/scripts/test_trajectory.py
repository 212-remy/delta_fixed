#!/usr/bin/env python

import rospy
from remy_me212.msg import DeltaState, DeltaStateArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool
from time import sleep


def node():
    rospy.init_node('test_trajectory')
    pub = rospy.Publisher('state_arrays', DeltaStateArray, queue_size=10)
    width=100
    depth = -925
    p1 = PointStamped()
    p1.point = Point(width, width, depth)
    p1.header.stamp = rospy.Time.now()
    p1.header.frame_id = "robot_base_plate"
    p2 = PointStamped()
    p2.point = Point(width, -width, depth)
    p2.header.stamp = rospy.Time.now()
    p2.header.frame_id = "robot_base_plate"
    p3 = PointStamped()
    p3.point = Point(-width, -width, depth)
    p3.header.stamp = rospy.Time.now()
    p3.header.frame_id = "robot_base_plate"
    p4 = PointStamped()
    p4.point = Point(-width, width, depth)
    p4.header.stamp = rospy.Time.now()
    p4.header.frame_id = "robot_base_plate"
    states = DeltaStateArray([
        DeltaState(position=p1, actuator=Bool(True)),
        DeltaState(position=p2, actuator=Bool(False)),
        DeltaState(position=p3, actuator=Bool(True)),
        DeltaState(position=p4, actuator=Bool(False)),
    ])
    while not rospy.is_shutdown():
        pub.publish(states)
        rospy.sleep(5)


def main():
    node()


if __name__ == "__main__":
    main()