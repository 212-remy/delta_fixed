import rospy


# We want to define a state message, which contains a point (the location of the end effector) and a boolean to
# describe the state of the end effector. The input to this trajectory node will then be an array of states. To
# transition from one state to another, the node will generate a series of interpolated points from the current position
# to the next position. It will send move coordinates to the odrive node, and after moving through that subtrajectory
# this node will tell the gripper to change state (if applicable). So the delta robot will move, then grip, then move,
# then drop, then move, etc.

def node():
    pass


def main():
    pass


if __name__ == "__main__":
    main()