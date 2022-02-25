#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([179.05, -132.94, 128.44, -87.84, -88.67, 38.38])

# Hanoi tower location 1
Q13 = np.radians([170.28, -65.04, 111.82, -137.41, -90.33, 41.44])
Q12 = np.radians([170.44, -59.14, 112.17, -140.80, -90.35, 41.44])
Q11 = np.radians([170.30, -52.67, 112.17, -146.79, -90.34, 41.44])
Q23 = np.radians([186.77, -58.77, 100.29, -132.40, -93.06, 40.65])
Q22 = np.radians([186.48, -53.33, 102.86, -140.97, -92.99, 40.65])
Q21 = np.radians([186.15, -47.13, 102.42, -145.95, -92.03, 40.65])
Q33 = np.radians([200.42, -47.80, 80.31, -123.75, -93.32, 40.65])
Q32 = np.radians([200.58, -43.07, 80.20, -127.15, -93.33, 40.65])
Q31 = np.radians([200.09, -37.64, 79.92, -131.50, -91.56, 40.65])
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
gripper_on = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    global gripper_on
    gripper_on = msg.DIGIN



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global gripper_on
    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    time.sleep(1.0) # Delay to safe at home/ clear path
    
    rospy.loginfo("Sending Goals ...")
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0) # Delay for the suction
    if gripper_on == False:
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(1.0) # Delay for the suction
        move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        time.sleep(1.0) # Delay to safe at home/ clear path
        error = 1
        return error

    rospy.loginfo("Sending Goals ...")
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    time.sleep(1.0) # Delay to safe at home/ clear path

    rospy.loginfo("Sending Goals ...")
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0) #Delay for the block to drop

    rospy.loginfo("Sending Goals ...")
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    time.sleep(1.0) # Delay to safe at home/ clear path

    error = 0

    return error



############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)
    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    loop_count = 1
    third_pin = 0
    starting_postion = int(raw_input("Enter number for start location <Either 1 2 3 or 0 to quit> ")) - 1
    destination = int(raw_input("Enter number for destination <Either 1 2 3 or 0 to quit> ")) - 1
    third_pin = 3 - starting_postion - destination
    print("Starting position : " + str(starting_postion) + "\n")
    print("Final position " + str(destination) + "\n")
    






    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    while(loop_count > 0):

        if (starting_postion == destination):
            break

        if move_block(pub_command, loop_rate, starting_postion, 2, destination, 0):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, starting_postion, 1, third_pin, 0):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, destination, 0, third_pin, 1):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, starting_postion, 0, destination, 0):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, third_pin, 1, starting_postion, 0):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, third_pin, 0, destination, 1):
            rospy.loginfo("Error: Block not found")
            break
        if move_block(pub_command, loop_rate, starting_postion, 0, destination, 2):
            rospy.loginfo("Error: Block not found")
            break
        loop_count = loop_count -1

    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
