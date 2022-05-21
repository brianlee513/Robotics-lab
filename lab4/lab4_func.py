#!/usr/bin/env python
import numpy as np
import math
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
    	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	
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
	yaw_WgripDegree = math.radians(yaw_WgripDegree)
	xCenter = xWgrip + 0.15 - 0.0535*math.cos(yaw_WgripDegree)
	yCenter = yWgrip - 0.15 - 0.0535*math.sin(yaw_WgripDegree)
	zCenter = zWgrip - 0.01

	l_to_base = math.sqrt(xCenter**2+yCenter**2)
	l_virtual = 0.11
	theta_virtual = math.asin(l_virtual/l_to_base)
	theta1 = math.asin(yCenter/l_to_base) - theta_virtual

	x_3end = xCenter-0.083*math.cos(theta1)+0.11*math.sin(theta1)
	y_3end = yCenter-0.083*math.sin(theta1)-0.11*math.cos(theta1)
	z_3end  = zCenter + 0.082 + 0.059

	l_3end_xy = math.sqrt(x_3end**2+y_3end**2)
	l_3height = z_3end-0.152
	
	bottom_theta2 = math.atan(l_3height/l_3end_xy)
	l_m = math.sqrt((x_3end**2)+(y_3end**2)+(l_3height**2))
	theta2 = -(math.acos((0.213**2-0.244**2-l_m**2)/(-2*0.244*l_m)) + bottom_theta2)
	
	theta3 = np.pi - np.arccos((l_m**2-0.244**2-0.213**2)/(-2*0.244*0.213))
	theta4 = -theta2-theta3
	theta5 = -np.pi/2
	theta6 = (np.pi/2)+theta1-yaw_WgripDegree
	# ==============================================================#
	print(theta1, theta2, theta3, theta4, theta5, theta6)
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
