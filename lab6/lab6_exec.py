#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
xw_yw_G = []
xw_yw_B = [] 
xw_yw_O = [] #middle block

# Any other global variable you want to define
# Hints: where to put the blocks?


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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


"""
Function to control the suction cup on/off
"""
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

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
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
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel, theta):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    # global variable1
    # global variable2

    error = 0
    #move to the blob--10cm above
    move_arm(pub_cmd, loop_rate, lab_invk(float(start_xw_yw_zw[0]), float(start_xw_yw_zw[1]),  .1, 0), vel, accel)

    #move down to the blob
    move_arm(pub_cmd, loop_rate, lab_invk(float(start_xw_yw_zw[0]), float(start_xw_yw_zw[1]), 0.03, 0), vel, accel)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(.5)

    #checks if there's a blob
    if (digital_in_0 == False):
        print("NO BLOCK FOUND ")
        gripper(pub_cmd, loop_rate, suction_off)
        #move to go_away position
        move_arm(pub_cmd, loop_rate, go_away, vel, accel)
        #set error to 1 and return error if block is not found
        error = 1
        return error
 
    #move to the blob--10cm above
    move_arm(pub_cmd, loop_rate, lab_invk(float(start_xw_yw_zw[0]), float(start_xw_yw_zw[1]),  .1, theta), vel, accel)

    #move to the target position 10cm above
    move_arm(pub_cmd, loop_rate, lab_invk(float(target_xw_yw_zw[0]), float(target_xw_yw_zw[1]), .1, theta), vel, accel)

    #move to the target height
    move_arm(pub_cmd, loop_rate, lab_invk(float(target_xw_yw_zw[0]), float(target_xw_yw_zw[1]), 0.03, theta), vel, accel)
    #turn off the gripper
    gripper(pub_cmd, loop_rate, suction_off)
    #move to the target position 10cm above
    move_arm(pub_cmd, loop_rate, lab_invk(float(target_xw_yw_zw[0]), float(target_xw_yw_zw[1]), .1, 0), vel, accel)
    # ========================= Student's code ends here ===========================

    return error

def group_block(xw_yw_O, xw_yw_B, xw_yw_G):
    first_set = [xw_yw_B[0], xw_yw_O[0], xw_yw_G[0]]
    second_set = [xw_yw_B[1], xw_yw_O[1], xw_yw_G[1]]
    
    #distance between blue and green ~5cm
    dist_B0G0 = np.sqrt(abs(first_set[0][0]-first_set[2][0])**2+abs(first_set[0][1]-first_set[2][1])**2)
    #distance between blue and orange ~2.5cm
    dist_B0O0 = np.sqrt(abs(first_set[0][0]-first_set[1][0])**2+abs(first_set[0][1]-first_set[1][1])**2)
    #distance between green and orange ~2.5cm
    dist_G0O0 = np.sqrt(abs(first_set[2][0]-first_set[1][0])**2+abs(first_set[2][1]-first_set[1][1])**2)   
    
    if dist_B0G0 > .06 or dist_B0G0 < .04:
        first_set = [xw_yw_B[0], xw_yw_O[0], xw_yw_G[1]]
        second_set = [xw_yw_B[1], xw_yw_O[1], xw_yw_G[0]]
        dist_G0O0 = np.sqrt(abs(first_set[2][0]-first_set[1][0])**2+abs(first_set[2][1]-first_set[1][1])**2)    
         
    elif dist_B0O0 < .02 or dist_B0O0 > .03 or dist_G0O0 < .02 or dist_G0O0 > .03:
        first_set = [xw_yw_B[0], xw_yw_O[1], xw_yw_G[1]]
        second_set = [xw_yw_B[1], xw_yw_O[0], xw_yw_G[0]]
        
    return first_set, second_set

def find_theta(xw_yw_O, xw_yw_B, xw_yw_G):

    first_set, second_set = group_block(xw_yw_O, xw_yw_B, xw_yw_G)
    #order: Blue Orange Green
    #Blue facing 5 o'clock
    if first_set[0][0] > first_set[2][0] and first_set[0][1] > first_set[2][1]:
        theta1 = -np.arctan((first_set[0][1] - first_set[2][1])/(first_set[0][0] - first_set[2][0]))
    #Blue facing 7 o'clock
    elif first_set[0][0] > first_set[2][0] and first_set[0][1] < first_set[2][1]:
        theta1 = np.arctan((first_set[2][1] - first_set[0][1])/(first_set[0][0] - first_set[2][0]))
    #Blue facing 2 o'clock
    elif first_set[0][0] < first_set[2][0] and first_set[0][1] > first_set[2][1]:
        theta1 = np.arctan((first_set[0][1] - first_set[2][1])/(first_set[2][0])-first_set[0][0])
    # Blue facing 10 o'clock
    elif first_set[0][0] < first_set[2][0] and first_set[0][1] < first_set[2][1]:
        print("here")
        theta1 = -np.arctan((first_set[2][1] - first_set[0][1])/(first_set[2][0])-first_set[0][0])    
    else:
        theta1 = 0 
    
    #Blue facing 5 o'clock
    if second_set[0][0] > second_set[2][0] and second_set[0][1] > second_set[2][1]:
        theta2 = -np.arctan((second_set[0][1] - second_set[2][1])/(second_set[0][0] - second_set[2][0]))
    #Blue facing 7 o'clock
    elif second_set[0][0] > second_set[2][0] and second_set[0][1] < second_set[2][1]:
        theta2 = np.arctan((second_set[2][1] - second_set[0][1])/(second_set[0][0] - second_set[2][0]))
    #Blue facing 2 o'clock
    elif second_set[0][0] < second_set[2][0] and second_set[0][1] > second_set[2][1]:
        theta2 = np.arctan((second_set[0][1] - second_set[2][1])/(second_set[2][0])-second_set[0][0])
    # Blue facing 10 o'clock
    elif second_set[0][0] < second_set[2][0] and second_set[0][1] < second_set[2][1]:
        print("here")
        theta2 = -np.arctan((second_set[2][1] - second_set[0][1])/(second_set[2][0])-second_set[0][0])    
    else:
        theta2 = 0  
    
    first_set.append(np.degrees(theta1))
    second_set.append(np.degrees(theta2))
    print(np.degrees(theta1))
    print(np.degrees(theta2))
    # else:
    #     first_set = [[0,0],[0,0],[0,0],0]
    #     second_set = [[0,0],[0,0],[0,0],0]
    #     print("invalid length")
    return [first_set, second_set]

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_B # store found yellow blocks in this list
        global xw_yw_O
        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_B respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_B.

        # Remember, xw_yw_G & xw_yw_B are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_G = blob_search(cv_image, "green")
        xw_yw_B = blob_search(cv_image, "blue")
        xw_yw_O = blob_search(cv_image, "orange")
        
       
        
        
       

"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_B
    global xw_yw_G
    global xw_yw_O

    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab6node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_B to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    #Group Blocks and find theta--[Green, Blue, Blue, Theta]
    print("orange")
    print(xw_yw_O)
    group_blocks = find_theta(xw_yw_O, xw_yw_B, xw_yw_G)
    #Target 
    TARGET = np.array([[.30541, -.25952, 0.03], [.42870, -.26510, 0.03]])
    #Change coordinates to the world  frame
    TARGET = TARGET + np.array([-0.15,+0.15,0.0])
    print(TARGET)
    

    #uses inverse kinematic fn to find joint angles
    #start_xw_yw_zw are the world coordinates of the block to pick up
    # #target_xw_yw_zw are the world coordinates of the goal location

    if(len(group_blocks) > 1) : 
        move_block(pub_command, loop_rate, [group_blocks[1][1][0], group_blocks[1][1][1], .025], TARGET[1], vel, accel, group_blocks[1][3])
    if(len(group_blocks) > 0) : 
        move_block(pub_command, loop_rate, [group_blocks[0][1][0], group_blocks[0][1][1], .025], TARGET[0], vel, accel, group_blocks[0][3])


    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
