The following codes are from ECE 470 Introduction to Robotics Lab

The lab utilized the 6 DOF Universal Robot's UR3 robot. 

Lab2

The objective of lab2 is to solve the Tower of Hanoi. The setup is limited to three pegs. 
The code is capable to solve the tower of Hanoi when the blocks are placed randomly within the three peg locations.

Lab3

The objective of lab3 is to calculate the forward kinematics of the UR3 robot and its end-effector. 
To calculate the rotational matrix, the dimensions of the UR3 robot is utilized.
The product of exponential formula is calculated using the screw axis.

Lab4

The objective of lab4 is to calculate the inverse kinematics of the UR3 robot with a given input end-effector destination.
Geometric analysis is used to calculate the joint angle for each joint.
One of the joint is fixed to 90 degrees to constrain the output.

Lab5

The objective of lab5 is to move colored blocks to a destine location using OpenCV.
Two sets of colored blocks are recognized using a webcam from the top view. Blocks locations are transformed to the robot's world frame.
Then the robot moves each block to a destine location using inverse kinematics.

Lab6

The objective of lab6 is to move two identical long blocks, consist of three different color blocks, to a destine location with the same orientation.
On top of lab5, a function to group the two set of blocks are added. Then another function calculates the angle necessary to rotate to accomplish the same end orientation.
