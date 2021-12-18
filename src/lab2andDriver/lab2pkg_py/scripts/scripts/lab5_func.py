#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	w1 = np.array([ [0],
					[0],
					[1]])
	w2 = np.array([ [0],
					[1],
					[0]])
	w3= np.array([  [0],
					[1],
					[0]])
	w4 = np.array([  [0],
					[1],
					[0]])
	w5 = np.array( [ [1],
					[0],
					[0]])
	w6 = np.array( [ [0],
					[1],
					[0]])

	v1 =np.cross([0, 0, -1], [-.15,.15,.10])
	v2 =np.cross([0, -1, 0], [-.15,.26,.162])
	v3 =np.cross([0, -1, 0], [.094,.26,.162])
	v4 =np.cross([0, -1, 0], [.307,.243,.162])
	v5 =np.cross([-1, 0, 0], [.307,.26,.162])
	v6 =np.cross([0, -1, 0], [.39,.26,.162])

	S1 =np.vstack((w1,v1.reshape(3,1)))
	S2 =np.vstack((w2,v2.reshape(3,1)))
	S3 =np.vstack((w3,v3.reshape(3,1)))
	S4 =np.vstack((w4,v4.reshape(3,1)))
	S5 =np.vstack((w5,v5.reshape(3,1)))
	S6 =np.vstack((w6,v6.reshape(3,1)))

	S = np.concatenate((S1,S2,S3,S4,S5,S6), axis=-1)

	M =np.array( [[0,-1, 0,0.39],
				[0, 0, -1, 0.411],
				[1, 0, 0, 0.2155],
				[0, 0, 0, 1 ] ])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta=np.array([theta1, theta2, theta3, theta4, theta5, theta6])
	x = np.identity(4)
	M,S = Get_MS()
	T=M
	for i in range(6):
		s_skew = np.array([[0, -S[2][i], S[1][i], S[3][i]],
						[S[2][i], 0, -S[0][i], S[4][i]],
						[-S[1][i], S[0][i], 0, S[5][i]],
						[0, 0, 0, 0]])
		x =np.matmul(x, expm(s_skew*theta[i]))

	T = np.matmul(x, T)
	# print(T)

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
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	yaw = np.radians(yaw_WgripDegree)

	l1 = 0.152
	l2 = 0.120
	l3 = 0.244
	l4 = 0.093
	l5 = 0.213
	l6 = 0.083
	l7 = 0.083
	l8 = 0.082
	l9 = 0.0535
	l10 = 0.059
	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15
	zgrip = zWgrip

	xcen = xgrip - l9*np.cos(yaw)
	ycen = ygrip - l9*np.sin(yaw)
	zcen = zgrip

	thetas[0] = np.arctan2(ycen, xcen) - np.arcsin(1.0*(l2 - l4 + l6)/ (np.sqrt(xcen**2 + ycen**2)))
	thetas[5] = PI/2 + thetas[0] - yaw

	x3end = xcen - l7*np.cos(thetas[0]) + (l6 + 0.027)*np.sin(thetas[0])
	y3end = ycen - l7*np.sin(thetas[0]) - (l6 + 0.027)*np.cos(thetas[0])
	z3end = zcen + l10 + l8

	d = z3end - l1
	R = np.sqrt(x3end**2 + y3end**2 + d**2)

	alpha = np.arcsin(d / R)
	beta = np.arccos((R**2 + l3**2 - l5**2) / (2*l3*R))
	gamma = np.arccos((l3**2 +l5**2 - R**2) / (2*l3*l5))

	thetas[1]= -alpha - beta
	thetas[2]= PI - gamma
	thetas[3]= -(PI - alpha - beta - gamma)
	thetas[4]=  -PI/2

	# print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
    float(thetas[3]), float(thetas[4]), float(thetas[5]) )

	# ==============================================================#
	pass
