#!/usr/bin/env python

import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *
import math
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and v1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	M = np.array([[0, -1, 0, .390],
				  [0,  0,-1, 0.401],
				  [1, 0, 0, 0.2155],
				  [0, 0, 0, 1]])

	q1 = np.array([-150,150,10])
	q2 = q1 + np.array([0,120,152])
	q3 = q2 + np.array([244,0,0])
	q4 = q3 + np.array([213,-93,0])
	q5 = q4 + np.array([0,83,0])
	q6 = q5 + np.array([83,0,0])
	
	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	v1 = np.cross(-w1, q1)
	v2 = np.cross(-w2, q2)
	v3 = np.cross(-w3, q3)
	v4 = np.cross(-w4, q4)
	v5 = np.cross(-w5, q5)
	v6 = np.cross(-w6, q6)
	M = np.array([[0,-1,0,390],\
				[0,0,-1,401],\
				[1,0,0,215.5], \
				[0,0,0,1]])
	
	
	
	S1 = [[0,-w1[2],w1[1], v1[0]],[w1[2],0,-w1[0],v1[1]],[-w1[1],w1[0],0,v1[2]],[0,0,0,0]]
	S2 = [[0,-w2[2],w2[1], v2[0]],[w2[2],0,-w2[0],v2[1]],[-w2[1],w2[0],0,v2[2]],[0,0,0,0]]
	S3 = [[0,-w3[2],w3[1], v3[0]],[w3[2],0,-w3[0],v3[1]],[-w3[1],w3[0],0,v3[2]],[0,0,0,0]]
	S4 = [[0,-w4[2],w4[1], v4[0]],[w4[2],0,-w4[0],v4[1]],[-w4[1],w4[0],0,v4[2]],[0,0,0,0]]
	S5 = [[0,-w5[2],w5[1], v5[0]],[w5[2],0,-w5[0],v5[1]],[-w5[1],w5[0],0,v5[2]],[0,0,0,0]]
	S6 = [[0,-w6[2],w6[1], v6[0]],[w6[2],0,-w6[0],v6[1]],[-w6[1],w6[0],0,v6[2]],[0,0,0,0]]

	S = np.array([S1, S2, S3, S4,S5,S6])



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    
	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	T = np.identity(4)
	for i in range(6):
    		T = np.matmul(T,expm(S[i]*theta[i]))
	T = np.matmul(T,M)
	print(str(T) + "\n")


	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""

def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059

	yaw_WgripDegree = np.radians(yaw_WgripDegree)

	xCenter = xWgrip + 0.15 - L9*np.cos(yaw_WgripDegree)
	yCenter = yWgrip - 0.15 - L9*np.sin(yaw_WgripDegree)
	zCenter = zWgrip - 0.01

	l_to_base = np.sqrt(xCenter**2+yCenter**2)
	l_virtual = 0.11
	theta_virtual = np.arcsin(l_virtual/l_to_base)
	theta1 = np.arcsin(yCenter/l_to_base) - theta_virtual

	x_3end = xCenter-L6*np.cos(theta1)+0.11*np.sin(theta1)
	y_3end = yCenter-L7*np.sin(theta1)-0.11*np.cos(theta1)
	z_3end  = zCenter + L8 + L10

	l_3end_xy = np.sqrt(x_3end**2+y_3end**2)
	l_3height = z_3end-L1
	
	bottom_theta2 = np.arctan(l_3height/l_3end_xy)
	l_m = np.sqrt((l_3end_xy)**2+(l_3height**2))
	
	top_theta2 = np.arccos((L5**2-L3**2-l_m**2)/(-2*L3*l_m))
	theta2 = -(top_theta2 + bottom_theta2)
	
	theta3 = np.pi - np.arccos((l_m**2-L3**2-L5**2)/(-2*L3*L5))
	theta4 = -theta2-theta3
	theta5 = -np.pi/2
	theta6 = (np.pi/2)+theta1-yaw_WgripDegree

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

	


