#!/usr/bin/env python
import copy
import time
import rospy
import sys
import numpy as np
from random import randrange
from lab2_spawn import *
from lab4_header import *
from lab5_header import *
from lab4_func import *
from blob_search import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 130*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

xw_yw_ice = []
xw_yw_Y = []

"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0
	global analog_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
	analog_in_0 = msg.AIN0

	digital_in_0 = False
	current_io_0 = True

	

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
        global xw_yw_ice # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list

        try:
          # Convert ROS image to OpenCV image
        	raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        xw_yw_ice = blob_search(cv_image, "ice")
        xw_yw_Y = blob_search(cv_image, "yellow")


"""
Program run from here
"""
def main():

	global home
	global xw_yw_ice
	global xw_yw_Y

	# Initialize ROS node
	rospy.init_node('lab4node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

	new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	# if(len(sys.argv) != 5):
	# 	print("\n")
	# 	print("Invalid number of input!\n")
	# 	print("rosrun lab4pkg_py lab4_exec.py xWgrip yWgrip zWgrip yaw_WgripDegree \n")
	# else:
	# 	print("\nxWgrip: " + sys.argv[1] + ", yWgrip: " + sys.argv[2] + \
	# 		  ", zWgrip: " + sys.argv[3] + ", yaw_WgripDegree: " + sys.argv[4] + "\n")

	# print(sys.argv)
	vel = 3.0
	accel = 3.0

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	move_arm(pub_command, loop_rate, home, vel, accel)
	time.sleep(2)

	x = randrange(2)
	spawn_block(x,False)
	print(x)
	ic = ImageConverter(SPIN_RATE)
	time.sleep(5)

	block_x = -0.2
	block_y = -0.15
	block_z = 0.03
	block_yaw = -45
	
	fire_x = -0.20
	fire_y = 0.57
	fire_z = 0.07
	fire_yaw = -45

	block_start_xw_yw_zw_1 = [xw_yw_ice[0][0],xw_yw_ice[0][1],block_z]
	fire_xw_yw_zw_1 = [xw_yw_Y[0][0],xw_yw_Y[0][1],fire_z]

	block_dest_high = lab_invk(block_start_xw_yw_zw_1[0], block_start_xw_yw_zw_1[1], 0.13, block_yaw)
	time.sleep(0.5)
	block_dest = lab_invk(block_start_xw_yw_zw_1[0], block_start_xw_yw_zw_1[1], block_z, block_yaw)
	time.sleep(0.5)
	block_dest_high = lab_invk(block_start_xw_yw_zw_1[0], block_start_xw_yw_zw_1[1], block_z + 0.1, block_yaw)
	time.sleep(0.5)
	fire_dest_high = lab_invk(fire_xw_yw_zw_1[0], fire_xw_yw_zw_1[1], 0.13, fire_yaw)
	time.sleep(0.5)
	fire_dest = lab_invk(fire_xw_yw_zw_1[0], fire_xw_yw_zw_1[1], fire_z, fire_yaw)
	
	move_arm(pub_command, loop_rate, block_dest_high, vel, accel)
	time.sleep(0.5)
	move_arm(pub_command, loop_rate, block_dest, vel, accel)
	time.sleep(1.0)
	gripper(pub_command, loop_rate, True)
	time.sleep(0.5)
	#if digital_in_0 != 0:'
	move_arm(pub_command, loop_rate, block_dest_high, vel, accel)
	time.sleep(0.5)
	move_arm(pub_command, loop_rate, fire_dest_high, vel, accel)
	time.sleep(0.5)
	move_arm(pub_command, loop_rate, fire_dest, vel, accel)
	time.sleep(1.0)
	gripper(pub_command, loop_rate, False)
	time.sleep(0.5)
	
	# move_arm(pub_command, loop_rate, fire_dest, vel, accel)

	rospy.loginfo("Destination is reached!")



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass