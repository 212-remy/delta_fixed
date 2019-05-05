#!/usr/bin/env python

import rospy
from remy_me212.msg import DeltaState, DeltaStateArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool
import numpy as np
from Queue import Queue
import tf

# We want to define a state message, which contains a point (the location of the end effector) and a boolean to
# describe the state of the end effector. The input to this trajectory node will then be an array of states. To
# transition from one state to another, the node will generate a series of interpolated points from the current position
# to the next position. It will send move coordinates to the odrive node, and after moving through that subtrajectory
# this node will tell the gripper to change state (if applicable). So the delta robot will move, then grip, then move,
# then drop, then move, etc.

delta_position_pub = None
actuator_pub = None
current_actuator_state = None
current_position = None
queue = None
neutral = Point(x=0, y=0, z=-750)
transformer = tf.TransformerROS()


def trajectory(data):
    global queue
    for state in data.states:
        queue.put(state)


def queue_handle():
    global queue, delta_position_pub, actuator_pub, current_actuator_state, current_position
    if not queue.empty():
        state = queue.get()
        rospy.sleep(.1)
        neutral_stamped = PointStamped()
        neutral_stamped.point = neutral
        neutral_stamped.header.frame_id = "robot_base_plate"
        neutral_stamped.header.stamp = rospy.Time.now()
        delta_position_pub.publish(neutral_stamped)
        while l2_distance(current_position,  neutral) > 10:
            rospy.sleep(.1)
        delta_position_pub.publish(state.position)
        goal_point = transformer.transformPoint('robot_base_plate', state.position).point

        while l2_distance(current_position, goal_point) > 10:
            rospy.sleep(.1)
        actuator_pub.publish(state.actuator)
        current_actuator_state = state.actuator.data


def update_current_position(data):
    global current_position
    current_position = data.point


def l2_distance(p1, p2):
    return np.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 +(p1.z-p2.z)**2)


def node():
    global delta_position_pub, actuator_pub, current_actuator_state, current_position, queue
    rospy.init_node('trajectory')
    queue = Queue()
    current_actuator_state = False
    delta_position_pub = rospy.Publisher('desired_position', PointStamped, queue_size=10)
    actuator_pub = rospy.Publisher('actuator', Bool, queue_size=10)
    sub = rospy.Subscriber('state_arrays', DeltaStateArray, trajectory)
    delta_position_subscriber = rospy.Subscriber("delta_position", PointStamped, update_current_position)

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