#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64
from odrive_ros.odrive_interface import ODriveInterfaceAPI as API
import odrive
from fibre.utils import Logger
import logging
from odrive.enums import *
import kinematicsSolver as kin
import time

import time
import math
import fibre
import serial
import struct
import signal
import sys
import numpy as np
from Queue import Queue


#####################################
realBot = True
#####################################

# if not realBot:
#    import robot212_virtual as bot

pi = np.pi  # 3.1415927


class Bot:

    def __init__(self):
        # !/usr/bin/python
        """
        Interface from Python to ODrive

        Daniel J. Gonzalez - dgonz@mit.edu
        Edited by R.E.M.Y.
        2.12 Intro to Robotics Spring 2019
        """



        self.pi = 3.1415927
        self.in2mm = 25.4
        self.mm2in = 1 / self.in2mm
        self.in2m = self.in2mm / 1000

        self.Nm2A = 0.00000604  # N*m/radian to A/count
        # https://www.wolframalpha.com/input/?i=(1+N*m%2Fradian)*(2*pi+radians%2F400000)*(1%2F(2.6+N*m%2FA))

        self.zeroVec = [[[0, 0], [0, 0]]]
        self.offsets = [[[-8.59, -6.11], [-3.61, 5.89]]]
        self.thtDesired = [[[0, 0], [0, 0]]]
        self.velDesired = [[[0, 0], [0, 0]]]
        self.kP = [[[0, 0], [0, 0]]]
        self.kD = [[[0, 0], [0, 0]]]
        self.home_kp = [[[0, 0], [0, 0]]]
        self.home_kd = [[[0, 0], [0, 0]]]
        self.kPd = [[[0, 0], [0, 0]]]
        self.kDd = [[[0, 0], [0, 0]]]

        self.CPR2RAD = (2 * math.pi / 400000)
        self.QUARTER_TURN_COUNTS = 100000

        # odrvs = [None, None]
        # '''[[ODrive 0, ODrive 1]]'''
        self.usb_serials = ['2085377D3548', '205337853548']#, '208637853548']
        # axes = [None, None, None]
        # axis0 = None
        # axis1 = None
        # axis2 = None
        self.odrvs = [None, None]
        # self.usb_serials = ['2087378B3548']

        self.num_odrvs = len(self.odrvs)
        self.axes = [None, None, None]
        self.axis0 = None
        self.axis1 = None
        self.axis2 = None

        self.num_axes = len(self.axes)
        self.zero_encoder = [0 for _ in self.axes]
        self.connect_all()
        self.printPos()
        self.queue = Queue()



    def rad2Count(self, angle, axis=None):
        try:
            return [self.zero_encoder[i] - ang / self.CPR2RAD for i, ang in enumerate(angle)]
        except TypeError:
            return self.zero_encoder[axis] - angle / self.CPR2RAD

    def r2c(self, angle, axis=None):
        return self.rad2Count(angle, axis=axis)

    def count2Rad(self, count, axis=None):
        try:
            return [(self.zero_encoder[i] - cnt) * self.CPR2RAD for i, cnt in enumerate(count)]
        except TypeError:
            return (self.zero_encoder[axis] - count) * self.CPR2RAD

    def c2r(self, count, axis=None):
        return self.count2Rad(count, axis=axis)

    # No offset for velocity/acceleration
    def rad2Count_no_offset(self, angle):
        try:
            return [-ang / self.CPR2RAD for i, ang in enumerate(angle)]
        except TypeError:
            return -angle / self.CPR2RAD

    def r2c_no(self, angle):
        return self.rad2Count_no_offset(angle)

    def count2Rad_no_offset(self, count):
        try:
            return [(-cnt) * self.CPR2RAD for i, cnt in enumerate(count)]
        except TypeError:
            return (-count) * self.CPR2RAD

    def c2r_no(self, count):
        return self.count2Rad_no_offset(count)

    def print_controllers(self):
        for axis in self.axes:
            print(axis.controller)

    def print_encoders(self):
        for axis in self.axes:
            print(axis.encoder)

    def printErrorStates(self):
        ii = 0
        for axis in self.axes:
            print('axis', ii, ' axis error:', hex(axis.error))
            print('axis', ii, ' motor error:', hex(axis.motor.error))
            print('axis', ii, ' encoder error:', hex(axis.encoder.error))
            ii += 1

    def printPos(self):
        ii = 0
        for axis in self.axes:
            print(ii, ' pos_estimate: ', axis.encoder.pos_estimate)
            print(ii, ' count_in_cpr: ', axis.encoder.count_in_cpr)
            print(ii, ' shadow_count: ', axis.encoder.shadow_count)
            ii += 1

    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()

    def connect_all(self):
        for i in range(self.num_odrvs):
            self.odrvs[i] = API()
            self.odrvs[i].connect(serial_number=self.usb_serials[i], timeout=15)
            self.odrvs[i] = self.odrvs[i].driver
        self.axes = [None, None, None]

        self.axis0 = self.odrvs[0].axis0
        self.axis1 = self.odrvs[1].axis0
        self.axis2 = self.odrvs[1].axis1
        self.axes[0] = self.axis0
        self.axes[1] = self.axis1
        self.axes[2] = self.axis2

    def reboot(self, ii):
        try:
            self.odrvs[ii].reboot()
        except:
            print('Rebooted ', ii)
        time.sleep(5)

    def reboot_all(self):
        try:
            self.odrvs[0].reboot()
        except:
            print('Rebooted 0')
        try:
            self.odrvs[1].reboot()
        except:
            print('Rebooted 1')
        time.sleep(5)
        print("Done initializing! Reconnecting...")
        self.connect_all()

    def stop_one(self, ii=0):
        axis = self.axes[ii]
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        axis.controller.vel_setpoint = 0

    def stop_all(self):
        for ii in range(self.num_axes):
            self.stop_one(ii)

    def vel_test_one(self, ii=0, amt=10000, mytime=2):
        axis = self.axes[ii]
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        axis.controller.vel_setpoint = 0
        time.sleep(mytime)
        print(0)
        self.print_all()
        time.sleep(0.25)

        axis.controller.vel_setpoint = amt
        time.sleep(mytime)
        print(1)
        self.print_all()
        time.sleep(0.25)

        axis.controller.vel_setpoint = 0
        time.sleep(mytime)
        print(2)
        self.print_all()
        time.sleep(0.25)

        axis.controller.vel_setpoint = -amt
        time.sleep(mytime)
        print(3)
        self.print_all()
        time.sleep(0.25)

        axis.controller.vel_setpoint = 0
        time.sleep(mytime)
        print(4)
        self.print_all()

    def vel_test_all(self, amt=30000, mytime=2):
        count = 0
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            axis.controller.vel_setpoint = 0
            count += 1
            time.sleep(2)
            print(str(count) + " " + str(axis.encoder.vel_estimate))
        time.sleep(mytime)
        print(0)
        # print_all()
        time.sleep(2)
        count = 0

        for axis in self.axes:
            axis.controller.vel_setpoint = -amt
            print("setpoint " + str(axis.controller.vel_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            count += 1
            time.sleep(2)
            print(str(count) + " " + str(axis.encoder.vel_estimate))
        time.sleep(mytime)
        print(1)
        # print_all()
        time.sleep(2)
        count = 0

        for axis in self.axes:
            axis.controller.vel_setpoint = 0
            print("setpoint " + str(axis.controller.vel_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            count += 1
            time.sleep(2)
            print(str(count) + " " + str(axis.encoder.vel_estimate))
        time.sleep(mytime)
        print(2)
        # print_all()
        time.sleep(2)
        count = 0

        for axis in self.axes:
            axis.controller.vel_setpoint = amt
            print("setpoint " + str(axis.controller.vel_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            count += 1
            time.sleep(2)
            print(str(count) + " " + str(axis.encoder.vel_estimate))
        time.sleep(mytime)
        print(3)
        # print_all()
        time.sleep(2)
        count = 0

        for axis in self.axes:
            axis.controller.vel_setpoint = 0
            print("setpoint " + str(axis.controller.vel_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            count += 1
            time.sleep(2)
            print(str(count) + " " + str(axis.encoder.vel_estimate))
        time.sleep(mytime)
        print(4)
        # print_all()

    def queueHandle(self):
        if not (self.queue.empty() or self.moving()):
            #print("Queue length: %d" % self.queue.qsize())
            args = self.queue.get()
            #print("Going to encoder counts %s" % str(args.__dict__['posDesired']))
            for ii in range(self.num_axes):
                axis = self.axes[ii]
                axis.trap_traj.config.vel_limit = abs(args.velDesired)  # 600000 max, 50000 is 1/8 rev per second
                axis.trap_traj.config.accel_limit = abs(args.accDesired)  # 50000 is 1/8 rev per second per second
                axis.trap_traj.config.decel_limit = abs(args.accDesired)
                axis.controller.move_to_pos(args.posDesired[ii])


    def trajMoveCnt(self, posDesired=(0, 0, 0), velDesired=5000, accDesired=50000):
        args = type("ArgumentPasser", (object,), {})()
        args.posDesired = posDesired
        args.velDesired = velDesired
        args.accDesired = accDesired
        self.queue.put(args)

    def trajMoveRad(self, posDesired=(0, 0, 0), velDesired= 4 * pi / 8, accDesired= 4 * pi / 8):
        self.trajMoveCnt(self.rad2Count(posDesired),
                         self.r2c_no(velDesired),
                         self.r2c_no(accDesired))

    def test_one(self, ii=0, amt=10000, mytime=.5):
        axis = self.axes[ii]
        # IF WE ARE USING INDEX, START AT IDLE, THEN CHANGE TO CLOSED LOOP!!!!
        axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.pos_setpoint = amt
        time.sleep(mytime)
        print(0)
        self.print_all()
        time.sleep(0.25)

        axis.controller.pos_setpoint = 0
        time.sleep(mytime)
        print(1)
        self.print_all()
        time.sleep(0.25)

        axis.controller.pos_setpoint = amt
        time.sleep(mytime)
        print(2)
        self.print_all()
        time.sleep(0.25)

        axis.controller.pos_setpoint = 0
        time.sleep(mytime)
        print(3)
        self.print_all()

    def test_all(self, amt=50000, mytime=2):
        for axis in self.axes:
            # IF WE ARE USING INDEX, START AT IDLE, THEN CHANGE TO CLOSED LOOP!!!!
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            axis.controller.pos_setpoint = amt
        time.sleep(mytime)
        print(0)
        # print_all()
        time.sleep(0.25)

        for axis in self.axes:
            axis.controller.pos_setpoint = 0
            print("setpoint " + str(axis.controller.pos_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            print("detect current " + str(axis.motor.current_control.Iq_measured))
        time.sleep(mytime)
        print(1)
        # print_all()
        time.sleep(0.25)

        for axis in self.axes:
            axis.controller.pos_setpoint = amt
            print("setpoint " + str(axis.controller.pos_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            print("detect current " + str(axis.motor.current_control.Iq_measured))
        time.sleep(mytime)
        print(2)
        # print_all()
        time.sleep(0.25)

        for axis in self.axes:
            axis.controller.pos_setpoint = 0
            print("setpoint " + str(axis.controller.pos_setpoint))
            print("command current " + str(axis.motor.current_control.Iq_setpoint))
            print("detect current " + str(axis.motor.current_control.Iq_measured))
        time.sleep(mytime)
        print(3)
        # for axis in axes:
        #    axis.requested_state = AXIS_STATE_IDLE
        # print_all()
        time.sleep(0.25)
        # axis.requested_state = AXIS_STATE_IDLE

    # test_all()

    def set_gains(self, k_p, k_d, perm=True):
        for axis in self.axes:
            if (k_d != 0):
                axis.controller.config.pos_gain = self.Nm2A * k_p / k_d
                axis.controller.config.vel_gain = k_d * self.Nm2A
        rospy.sleep(.2)
        if (perm):
            self.odrvs[0].save_configuration()
            self.odrvs[1].save_configuration()
            time.sleep(2)

    def set_gainsCounts(self, k_p, k_d, perm=True):
        for axis in self.axes:
            if (k_d != 0):
                axis.controller.config.pos_gain = k_p
                axis.controller.config.vel_gain = k_d
        if (perm):
            self.odrvs[0].save_configuration()
            self.odrvs[1].save_configuration()
            time.sleep(2)

    def full_init(self, reset=False, k_p=200, k_d=50):
        # brake resistance
        self.odrvs[0].config.brake_resistance = 0
        self.odrvs[1].config.brake_resistance = 0 #TODO

        for axis in self.axes:
            if (reset):
                axis.motor.config.pre_calibrated = False
                axis.encoder.config.pre_calibrated = False

            # motor current limit
            axis.motor.config.current_lim = 5

            # pole pairs
            axis.motor.config.pole_pairs = 4

            axis.controller.config.vel_limit = 600000  # 50000 counts/second is 1/8 revolution per second

            # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
            # Max speed is 1.35 Revolutions/second, or 539000counts/second
            axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            axis.encoder.config.cpr = 4000
            axis.encoder.config.bandwidth = 1000
            axis.encoder.config.use_index = True
            axis.encoder.config.zero_count_on_find_idx = True
            #axis.encoder.config.idx_search_speed = 1
            axis.encoder.config.pre_calibrated = False

            # motor calibration current
            axis.motor.config.calibration_current = 5

            # axis state
            if not axis.motor.config.pre_calibrated:
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(2)
        print("Done doing setup.")
        time.sleep(20 if reset else .2)
        print("Saving Configuration...")
        for axis in self.axes:
            axis.motor.config.pre_calibrated = True
            axis.config.startup_encoder_index_search = True
            axis.config.startup_encoder_offset_calibration = True
            axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

            # motor calibration current FOR INDEX SEARCH
            axis.motor.config.calibration_current = 5

            # Set closed loop gains
            kP_des = self.Nm2A * k_p  # pos_gain 2
            kD_des = self.Nm2A * k_d  # vel_gain 0.0015 / 5

            axis.controller.config.pos_gain = kP_des / kD_des  # Convert to Cascaded Gain Structure
            # https://github.com/madcowswe/ODrive/blob/451e79519637fdcf33f220f7dae9a28b15e014ba/Firmware/MotorControl/controller.cpp#L151
            axis.controller.config.vel_gain = kD_des
            axis.controller.config.vel_integrator_gain = 0
            axis.controller.pos_setpoint = 0

            # axis state
            # odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.config.startup_closed_loop_control = True
        # save configuration
        self.odrvs[0].save_configuration()
        self.odrvs[1].save_configuration() #TODO
        time.sleep(.2)
        self.printErrorStates()
        try:
            self.odrvs[0].reboot()
        except:
            print('Rebooted 0')
        try:
            self.odrvs[1].reboot()
        except:
            print('Rebooted 1')
        time.sleep(.5)
        print("Done initializing! Reconnecting...")
        self.connect_all()

    def make_perm(self, ii):
        self.odrvs[ii].save_configuration()

    def make_perm_all(self):
        for ii in range(self.num_odrvs):
            if self.odrvs[ii] == None:
                continue
            self.odrvs[ii].save_configuration()

    def closed_loop_state_all(self):
        for axis in self.axes:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def get_cnt_one(self, axis):
        return self.axes[axis].encoder.pos_estimate

    def get_cnt_all(self):
        return [self.get_cnt_one(i) for i in range(self.num_axes)]

    def get_rad_all(self):
        return self.count2Rad(self.get_cnt_all())

    def encoder_count_difference(self, axis):
        return axis.controller.pos_setpoint-axis.encoder.pos_estimate

    def get_enc_cnt_dif_all(self):
        return tuple(self.encoder_count_difference(axis) for axis in self.axes)

    def get_enc_deg_dif_all(self):
        return tuple(self.encoder_count_difference(axis) * self.CPR2RAD * 180 / pi for axis in self.axes)

    def moving(self):
        return any(abs(dif) > 2 for dif in self.get_enc_deg_dif_all())

# def callback(data, bot, deltaKin):
#
#     x = data.x
#     y = data.y
#     z = data.z
#     endpos = (x, y, z)
#     thtDes = deltaKin.IK((x, y, z))
#
#     assert (deltaKin.FK(thtDes) - endpos < .1).all()
#     if all([deltaKin.check_constraints(i + 1, endpos, thtDes[i]) for i in range(3)]):
#         print("x:%3.5f    y:%3.5f    z:%3.5f" % (x, y, z))
#         print(u"\u03b8\u2081:%3.5f    \u03b8\u2082:%3.5f    \u03b8\u2083:%3.5f" % (thtDes[0], thtDes[1], thtDes[2]))
#         bot.trajMoveRad(thtDes, 2 * pi / 8,
#                    2 * pi / 8)  # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
#     else:
#         print("Illegal position: x:%3.5f    y:%3.5f    z:%3.5f" % (x, y, z))


def move_to_point(data):
    x = data.x
    y = data.y
    z = data.z
    end_position = (x, y, z)
    try:
        tht_des = delta_kin.IK((x, y, z))
        if any([not delta_kin.check_constraints(i + 1, end_position, tht_des[i]) for i in range(3)]):
            raise ValueError('Angles out of workspace')
        # print("x:%3.5f   y:%3.5f   z:%3.5f" % (x, y, z))
        # print(u"\u03b8\u2081:%3.5f   \u03b8\u2082:%3.5f   \u03b8\u2083:%3.5f" % (tht_des[0], tht_des[1], tht_des[2]))
        bot.trajMoveRad(tht_des, 4 * pi / 8, 4 * pi / 8)
    except ValueError:
        print("Illegal position: x:%3.5f    y:%3.5f    z:%3.5f" % (x, y, z))


def callback_lim_init(axis):
    def ret(data):
        pushed = data.data
        if pushed:
            bot.stop_one(axis)
            bot.zero_encoder[axis] = bot.get_cnt_one(axis)
            lim_subs[axis].unregister()
            bot.axes[axis].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            bot.axes[axis].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            limit_flags[axis] = True
            print("Limit switch %d pushed" % axis)
            return
    return ret


def node():
    LIMIT_SWITCH_INIT_VEL = -15000 # counts per second

    print("Node starting...")
    global bot, delta_kin, lim_subs, limit_flags
    rospy.init_node('inverse_kinematics', anonymous=False)

    bot = Bot()
    #bot.full_init(reset=False, k_p=400, k_d=60)
    #bot.set_gains(400, 60, perm=True)
    for axis in bot.axes:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        axis.controller.vel_setpoint = 0
    # bot.full_init()
    # for i in range(3):
    #    bot.test_one(i, mytime=.1)

    delta_kin = kin.deltaSolver(realBot=realBot)

    # Limit switch init begins here
    limit_flags = [False for _ in range(3)]
    lim_subs = [rospy.Subscriber('lim%d' % i, Bool, callback_lim_init(i)) for i in range(3)]

    print("Beginning limit switch initialization...")
    for axis in bot.axes:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        axis.controller.vel_setpoint = LIMIT_SWITCH_INIT_VEL
    while not all(limit_flags):
        rospy.sleep(.001)
    print("Limit switch initialization complete.")

    for axis in bot.axes:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        axis.controller.vel_setpoint = 0
    for axis in bot.axes:
        axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

    pub = rospy.Publisher('delta_position', Point, queue_size=10)
    moving_pub = rospy.Publisher('delta_moving', Bool, queue_size=10)
    degrees_off_pub = rospy.Publisher('degrees_off', Point, queue_size=10)
    rospy.Subscriber('desired_position', Point, move_to_point)
    current_pubs = [rospy.Publisher('current%d' % i, Float64, queue_size=100) for i, _ in enumerate(bot.axes)]
    move_to_point(Point(165, 75, -810))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        bot.queueHandle()
        thetas = bot.get_rad_all()
        pos = delta_kin.FK(thetas)
        pub.publish(Point(x=pos[0], y=pos[1], z=pos[2]))
        moving_pub.publish(Bool(bot.moving()))
        degrees_off_pub.publish(Point(*bot.get_enc_deg_dif_all()))
        for i, p in enumerate(current_pubs):
            p.publish(Float64(bot.axes[i].motor.current_control.Iq_measured))
        rate.sleep()
        #bot.printErrorStates()



def main():
    # Bot().full_init(reset=True ,k_p=400, k_d=60); return  # ONLY UNCOMMENT WHEN ARMS AREN'T ATTACHED EXCLAMATION POINT
    try:
        node()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

