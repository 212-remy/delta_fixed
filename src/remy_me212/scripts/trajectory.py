#!/usr/bin/env python

import rospy
from remy_me212.msg import DeltaState, DeltaStateArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import numpy as np
from Queue import Queue

# We want to define a state message, which contains a point (the location of the end effector) and a boolean to
# describe the state of the end effector. The input to this trajectory node will then be an array of states. To
# transition from one state to another, the node will generate a series of interpolated points from the current position
# to the next position. It will send move coordinates to the odrive node, and after moving through that subtrajectory
# this node will tell the gripper to change state (if applicable). So the delta robot will move, then grip, then move,
# then drop, then move, etc.

global delta_position_pub, actuator_pub, current_actuator_state, current_position, queue
neutral = Point(x=0, y=0, z=-750)


def trajectory(data):
    global queue
    for state in data.states:
        queue.put(state)


def queue_handle():
    global queue, delta_position_pub, actuator_pub, current_actuator_state, current_position
    if not queue.empty():
        state = queue.get()
        rospy.sleep(.1)
        delta_position_pub.publish(neutral)
        while l2_distance(current_position,  (neutral.x, neutral.y, neutral.z)) > 10:
            rospy.sleep(.1)
        delta_position_pub.publish(state.position)
        state_position = (state.position.x, state.position.y, state.position.z)
        while l2_distance(current_position, state_position) > 10:
            rospy.sleep(.1)
        actuator_pub.publish(state.actuator)
        current_actuator_state = state.actuator.data


def update_current_position(data):
    global current_position
    current_position = (data.x, data.y, data.z)


def l2_distance(p1, p2):
    return np.sqrt(sum((b-a)**2 for a, b in zip(p1, p2)))


def node():
    global delta_position_pub, actuator_pub, current_actuator_state, current_position, queue
    rospy.init_node('trajectory')
    queue = Queue()

    current_actuator_state = False
    delta_position_pub = rospy.Publisher('desired_position', Point, queue_size=10)
    actuator_pub = rospy.Publisher('actuator', Bool, queue_size=10)
    sub = rospy.Subscriber('state_arrays', DeltaStateArray, trajectory)
    delta_position_subscriber = rospy.Subscriber("delta_position", Point, update_current_position)

    while not rospy.is_shutdown():
        queue_handle()
        rospy.sleep(.1)


def main():
    try:
        node()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()