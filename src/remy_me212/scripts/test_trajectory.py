#!/usr/bin/env python

import rospy
from remy_me212.msg import DeltaState, DeltaStateArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from time import sleep


def node():
    rospy.init_node('test_trajectory')
    pub = rospy.Publisher('state_arrays', DeltaStateArray, queue_size=10)
    states = DeltaStateArray([
        DeltaState(position=Point(50, 50, -850), actuator=Bool(True)),
        DeltaState(position=Point(50, -50, -950), actuator=Bool(False)),
        DeltaState(position=Point(-50, -50, -850), actuator=Bool(True)),
        DeltaState(position=Point(-50, 50, -950), actuator=Bool(False)),
    ])
    while not rospy.is_shutdown():
        pub.publish(states)
        rospy.sleep(5)


def main():
    node()


if __name__ == "__main__":
    main()