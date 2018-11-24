#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
import constants
import time

def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
        else: 
            angles = raw_input("Enter comma separated seven angles")
            angles = angles.split(",")
            names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
            joint_command = {joint_name: float(angle) for joint_name, angle in zip(names, angles)}
            left.set_joint_positions(joint_command)
            #elif c in bindings:
            #    cmd = bindings[c]
            #    #expand binding to something like "set_j(right, 's0', 0.1)"
            #    cmd[0](*cmd[1])
            #    print("command: %s" % (cmd[2],))
            #else:
            #    print("key bindings: ")
            #    print("  Esc: Quit")
            #    print("  ?: Help")
            #    for key, val in sorted(bindings.items(),
            #                           key=lambda x: x[1][2]):
            #        print("  %s: %s" % (key, val[2]))

class ActionManager():
    def __init__(self):
        from tuck_arms import Tuck
        self.tucker = Tuck(True)
        rospy.on_shutdown(self.tucker.clean_shutdown)
        self.untucker = Tuck(False)
        rospy.on_shutdown(self.untucker.clean_shutdown)

    def make_callback(self):
        def callback(msg, *args):
            if msg == constants.TUCK_ARMS:
                self.tucker.supervised_tuck()
            elif msg == constants.UNTUCK_ARMS:
                self.untucker.supervised_tuck()
        return callback


def main():
    print("Initializing node... ")
    rospy.init_node("actions")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    action_manager = ActionManager()
    action_callback = action_manager.make_callback()
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()
    
    while not rospy.is_shutdown():
        #action_callback(constants.UNTUCK_ARMS)
        action_callback(constants.UNTUCK_ARMS)
        return
    
    
if __name__ == '__main__':
    main()
