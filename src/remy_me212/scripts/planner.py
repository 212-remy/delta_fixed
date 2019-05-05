#!/usr/bin/env python

import rospy
from remy_me212.msg import Topping, ToppingArray, DeltaState, DeltaStateArray
from remy_me212.msg.Topping import *
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from itertools import chain
import numpy as np
from Queue import Queue


"""
#############################
Stage 0
-------------
Locate toppings, store each topping in a separate set, sets of locations
Locate pizza, set of holes on pizza
Locate pizza centroid/radius
Locate Salt Shaker
#############################
Stage 1
-------------
Place toppings on pizza
#############################
Stage 2
-------------
Shake salt on pizza 
#############################
Stage 3
-------------
Communicate "READY" to robot; wait till "READY" received
#############################
Stage 4
-------------
Move pizza onto robot
#############################
Stage 5
-------------
Communicate "DONE" to robot; wait till "DELIVERING" received
#############################
Stage 6
-------------
Wait until dough placed on table
#############################
Stage 7
-------------
Smash that dough
"""

PIZZA_RADIUS = 225  # mm

pepperoni = set()
pineapple = set()
ham = set()
olive = set()
anchovy = set()
topping_hole = set()
pizza = None
salt_shaker = None
stage0_complete_flag = False
TRUE = Bool(True)
FALSE = Bool(False)

topping_const_to_data_structure_dict = {
    ANCHOVY: anchovy,
    PEPPERONI: pepperoni,
    HAM: ham,
    OLIVE: olive,
    PINEAPPLE: pineapple,
    TOPPING_HOLE: topping_hole,
    PIZZA: pizza,
    SALT_SHAKER: salt_shaker
}


def topping_callback(data):
    global pepperoni, pineapple, ham, olive, anchovy, topping_hole, pizza, salt_shaker, topping_const_to_data_structure_dict
    for topping in data.toppings:
        if topping.type in {PIZZA, SALT_SHAKER}:
            topping_const_to_data_structure_dict[topping.type] = topping.position
        else:
            topping_const_to_data_structure_dict[topping.type].add(topping.position)


def stage0_callback(data):
    stage0_complete_flag = data.data


def stage1(pub):
    global pepperoni, pineapple, ham, olive, anchovy, topping_hole
    toppings = list(sorted(x) for x in [pepperoni, pineapple, ham, olive, anchovy])

    stage1_delta_state_array = []
    for topping in toppings:
        topping.reverse()
        stage1_delta_state_array.append(DeltaState(position=topping.pop(), actuator=TRUE))
        stage1_delta_state_array.append(DeltaState(position=topping_hole.pop(), actuator=FALSE))

    remaining_toppings = sorted(set(chain.from_iterable(toppings)))
    remaining_toppings.reverse()

    while topping_hole:
        stage1_delta_state_array.append(DeltaState(position=remaining_toppings.pop(), actuator=TRUE))
        stage1_delta_state_array.append(DeltaState(position=topping_hole.pop(), actuator=FALSE))

    pub.publish(DeltaStateArray(stage1_delta_state_array))

#TODO Fix Stage 2
def stage2(pub):
    global salt_shaker, pizza
    pizza_up = Point(pizza.x, pizza.y, pizza.z + 200)
    pizza_down = Point(pizza.x, pizza.y, pizza.z + 50)
    stage2_delta_state_array = []
    stage2_delta_state_array.append(DeltaState(position=salt_shaker, actuator=TRUE))
    for i in range(5):
        stage2_delta_state_array.append(DeltaState(position=pizza_down, actuator=TRUE))
    stage2_delta_state_array.append(DeltaState(position=salt_shaker, actuator=FALSE))
    pub.publish(DeltaStateArray(stage2_delta_state_array))


def node():
    global pepperoni, pineapple, ham, olive, anchovy, topping_hole, pizza, salt_shaker, toppings_processed
    rospy.Subscriber('toppings', ToppingArray, topping_callback)
    rospy.Subscriber('stage0_complete_flag', Bool, stage0_callback)
    trajectory_pub = rospy.Publisher('state_arrays', DeltaStateArray, queue_size=10)
    while not stage0_complete_flag:
        rospy.sleep(.5)

    stage1(trajectory_pub)



def main():
    try:
        node()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()