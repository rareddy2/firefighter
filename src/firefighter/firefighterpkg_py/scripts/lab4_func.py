#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	s1 = [0,0,1,0.15,0.15,0]
	s2 = [0,1,0,-0.162,0,-0.15]
	s3 = [0,1,0,-0.162,0,0.094]
	s4 = [0,1,0,-0.162,0,0.307]
	s5 = [1,0,0,0,0.162,-0.26]
	s6 = [0,1,0,-0.162,0,0.39]

	S = [s1,s2,s3,s4,s5,s6]

	M = [[0,-1,0,0.39],[0,0,-1,0.401],[1,0,0,0.2155],[0,0,0,1]]


	
	# ==============================================================#
	return M, S

def get_bracket_S(S):

	w3 = S[2]

	w2 = S[1]

	w1 = S[0]




	v3 = S[5]

	v2 = S[4]

	v1 = S[3]




	bracket_S = np.array([

	[0,-w3,w2,v1], 

	[w3,0,-w1,v2], 

	[-w2,w1,0,v3], 

	[0,0,0,0]

	])




	return bracket_S

"""
Function that calculates encoder numbers for each motor
"""

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M,S = Get_MS()


	bracket_s = []

	for i in range(len(S)):

		bracket_s.append( get_bracket_S(S[i]) )

	bracket_s = np.array(bracket_s)

	T = np.matmul( expm(bracket_s[0]*theta1) , expm(bracket_s[1]*theta2) )
	T = np.matmul( T , expm(bracket_s[2]*theta3) ) 
	T = np.matmul( T, expm(bracket_s[3]*theta4) )
	T = np.matmul( T, expm(bracket_s[4]*theta5) )
	T = np.matmul( T, expm(bracket_s[5]*theta6) )

	T = np.matmul(T , M)




	# ==============================================================#

	print(str(T) + "\n")

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
	A = 0.0535
	L1 = 152.0/1000
	L2 = 120.0/1000
	L3 = 244.0/1000
	L4 = 93.0/1000
	L5 = 213.0/1000
	L6 = 83.0/1000
	L7 = 83.0/1000
	L8 = 82.0/1000
	L9 = 53.5/1000
	L10 = 59.0/1000

	x_grip = xWgrip + (150.0/1000)
	y_grip = yWgrip - (150.0/1000)
	z_grip = zWgrip - (10.0/1000)

	yaw_WgripDegree = (yaw_WgripDegree/180) * np.pi

	x_cen = x_grip - np.cos(yaw_WgripDegree)*A
	y_cen = y_grip - np.sin(yaw_WgripDegree)*A
	z_cen = z_grip

	theta_1 = np.arctan2( y_cen,x_cen ) - np.arcsin( (L2 - L4 + L6) / np.sqrt( x_cen**2+ y_cen**2 ) )
	theta_6 = ( np.pi/2 ) + theta_1 - yaw_WgripDegree 

	#x3end = x_cen - L6 * np.cos(theta_1)
	#y3end = y_cen - (L2 - L4 + L6) * np.sin(theta_1)
	#z3end = z_cen + L8 + L10

	x3end = x_cen - L7*np.cos(theta_1) + (L6+27/1000.0)*np.sin(theta_1)
	y3end = y_cen - L7*np.sin(theta_1) - (L6+27/1000.0)*np.cos(theta_1)
	z3end = z_cen + L8 + L10

	length_A = np.sqrt(x3end**2 + y3end**2)
	length_B = np.sqrt(x3end**2 + y3end**2 + (z3end - L1)**2 )

	print(L3,length_B,L5)

	theta_a = np.arccos((L3**2 + length_B**2 - L5**2) / (2*L3*length_B))
	theta_b = np.arcsin((z3end - L1)/length_B)
	#theta_b = np.arccos((length_A**2 + length_B**2 - (z3end-L1)**2) / (2*length_A*length_B))
	theta_2 = -(theta_a + theta_b)

	theta_3 = np.pi - (np.arccos((L3**2 + L5**2 - length_B**2) / (2*L3*L5)))
	theta_4 = -(theta_3 + theta_2)

	theta_5 = -np.pi/2

	new_dest = np.array([theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6])
	print(new_dest) 

	return lab_fk(theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6)


	# ==============================================================#
	
   


