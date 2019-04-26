import rospy
from remy_me212.msg import DeltaState, DeltaStateArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

# We want to define a state message, which contains a point (the location of the end effector) and a boolean to
# describe the state of the end effector. The input to this trajectory node will then be an array of states. To
# transition from one state to another, the node will generate a series of interpolated points from the current position
# to the next position. It will send move coordinates to the odrive node, and after moving through that subtrajectory
# this node will tell the gripper to change state (if applicable). So the delta robot will move, then grip, then move,
# then drop, then move, etc.

def plan_trajectory(data):
    global delta_position_pub, actuator_pub, current_actuator_state, current_position
    states = list(data.data)
    #TODO finish this

def update_current_position(data)
    current_position = (data.x, data.y, data.z)


def node():
    global delta_position_pub, actuator_pub, current_actuator_state, current_position
    current_actuator_state = False
    delta_position_pub = rospy.publisher('desired_position', Point)
    actuator_pub = rospy.publisher('actuator', Point)
    sub = rospy.subscriber('state_arrays', DeltaStateArray, plan_trajectory)
    delta_position_subscriber = rospy.subscriber("delta_position", Point, update_current_position)
    rospy.init_node()
    rospy,spin()


def main():
    pass


if __name__ == "__main__":
    main()