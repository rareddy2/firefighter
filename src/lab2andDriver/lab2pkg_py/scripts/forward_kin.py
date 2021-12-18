#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from proj_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""




def Get_MS():
	# =================== Your code starts here ====================#


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

	print(S)

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

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

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





	# ==============================================================#

	print(str(T) + "\n")
	distance = ((T[0][3]-.15)**2 +(T[1][3]+.15)**2+T[2][3]**2)**.5
	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
