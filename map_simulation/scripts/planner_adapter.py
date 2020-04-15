#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Pose

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_about_axis, quaternion_multiply
from math import atan2, sqrt, pi, cos, sin, log

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pr2_controllers_msgs.msg import Pr2GripperCommand
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import time as time

# global variable of grippable world objects
objects = ['unit_box','unit_box_clone']
grasped = []

# global variables with position of robot
x = 0.0
y = 0.0
theta = 0.0

# global variable of gazebo world model
world = ModelStates()

# set gripper opened
def open_gripper(gripper, rate, wait = 3):
	opened = Pr2GripperCommand()
	opened.position = 1.0
	opened.max_effort = 10.0
	i = 0
	while i < 30:
		gripper.publish(opened)
		rate.sleep()
		i += 1
	rospy.sleep(wait)

# set gripper closed
def close_gripper(gripper, rate, wait = 4):
	closed = Pr2GripperCommand()
	closed.position = 0.0
	closed.max_effort = 10.0
	i = 0
	while i < 30:
		gripper.publish(closed)
		rate.sleep()
		i += 1
	rospy.sleep(wait)

# move manipulator group
def move_group(group, pose_goal, wait = 0.1):
	group.set_pose_target(pose_goal)
	plan = group.go(wait=True)
	group.stop()
	group.clear_pose_targets()
	rospy.sleep(wait)

# calculate gripper position relative to robot
def adjust_pose(x_pos, y_pos, z_pos):
	global theta
	global x
	global y

	rospy.sleep(0.1)

	x_new = x_pos*cos(theta) - y_pos*sin(theta)
	y_new = x_pos*sin(theta) + y_pos*cos(theta)

	x_intercept = -0.05
	x_new_i = (x_pos + x_intercept)*cos(theta) - y_pos*sin(theta)
	y_new_i = (x_pos + x_intercept)*sin(theta) + y_pos*cos(theta)

	x_pos_i = x_new_i + x
	y_pos_i = y_new_i + y 
	
	x_pos = x_new + x
	y_pos = y_new + y

	rospy.sleep(0.1)
	pose_goal_up = geometry_msgs.msg.Pose()
	pose_goal_up.orientation.w = 0.5
	pose_goal_up.orientation.x = 0.5
	pose_goal_up.orientation.y = 0.5
	pose_goal_up.orientation.z = -0.5

	w = pose_goal_up.orientation.w
	x = pose_goal_up.orientation.x
	y = pose_goal_up.orientation.y
	z = pose_goal_up.orientation.z

	gripper_angle = theta
	q_rotation = quaternion_about_axis(gripper_angle, (0,0,1))

	q_pose = [x, y, z, w]
	q_res = quaternion_multiply(q_rotation, q_pose)
	x, y, z, w = q_res

	pose_goal_up.orientation.w = w
	pose_goal_up.orientation.x = x
	pose_goal_up.orientation.y = y
	pose_goal_up.orientation.z = z

	pose_goal_up.position.x = x_pos_i
	pose_goal_up.position.y = y_pos_i
	pose_goal_up.position.z = z_pos + 0.3


	pose_goal_down = geometry_msgs.msg.Pose()
	pose_goal_down.orientation.w = pose_goal_up.orientation.w
	pose_goal_down.orientation.x = pose_goal_up.orientation.x
	pose_goal_down.orientation.y = pose_goal_up.orientation.y
	pose_goal_down.orientation.z = pose_goal_up.orientation.z
	pose_goal_down.position.x = x_pos
	pose_goal_down.position.y = y_pos
	pose_goal_down.position.z = z_pos

	return pose_goal_up, pose_goal_down

# grasp object on given coordinates relatively to robot
def pick(rate, gripper, group, x_pos = 0.0, y_pos = -0.5, z_pos = 0.21, wait = 3):
	pose_goal_up, pose_goal_down = adjust_pose(x_pos, y_pos, z_pos)

	open_gripper(gripper, rate)
	move_group(group, pose_goal_up, 1)

	move_group(group, pose_goal_down, 1)
	close_gripper(gripper, rate, wait)

	move_group(group, pose_goal_up, 1)

# release object on given coordinates relatively to robot
def place(rate, gripper, group, x_pos = 0.0, y_pos = -0.5, z_pos = 0.21, wait = 3):
	pose_goal_up, pose_goal_down = adjust_pose(x_pos, y_pos, z_pos)
	
	move_group(group, pose_goal_down, 1)
	open_gripper(gripper, rate, wait)

	move_group(group, pose_goal_up, 1)

# updates global variables 'x','y' and 'theta'
def new_pos(msg):
	global x
	global y
	global theta
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	roll, pitch, theta = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

# updates 'world' global variable
def new_state(msg):
	global world
	world = msg

# move robot to given coordinates
def move_diag(x_coord, y_coord, rate, pub, eps=0.001):
	point = Point()
	point.x = x_coord
	point.y = y_coord

	twist = Twist()
	rate.sleep()
	while not rospy.is_shutdown():
		inc_x = point.x - x
		inc_y = point.y - y
		angle = theta
		dist = sqrt(inc_x**2 + inc_y**2)

		xsp = inc_x/abs(dist) if abs(dist) > 1 else inc_x
		ysp = inc_y/abs(dist) if abs(dist) > 1 else inc_y

		rospy.loginfo('Distance remained: ' + str(dist))

		twist.linear.x = xsp*cos(angle) + ysp*sin(angle)
		twist.linear.y = -xsp*sin(angle) + ysp*cos(angle)
		
		pub.publish(twist)

		if ((abs(inc_y) <= eps) and (abs(inc_x) <= eps)):
			rate.sleep()
			twist.linear.x = 0
			twist.linear.y = 0
			pub.publish(twist)
			break

		pub.publish(twist)
		rate.sleep()

# rotate robot on angle
def rotate(angle, rate, pub, eps=1.e-2):
	global theta

	twist = Twist()
	rate.sleep()
	while not rospy.is_shutdown():
		theta = (theta) % (2 * pi)
		angle = (angle) % (2 * pi)
		
		a = (theta - angle) % (2 * pi)
		rospy.loginfo("Angle remained: " + str(min(abs(a), 2 * pi - abs(a))))

		if (min(abs(a), 2 * pi - abs(a)) >= eps):
			twist.angular.z = min(abs(a), 2 * pi - abs(a))
			if ((abs(a) < pi)):
				twist.angular.z *= - 1
			pub.publish(twist)
		else:			
			twist.angular.z = 0.0
			pub.publish(twist)
			rate.sleep()
			break
		rate.sleep()

# rotate robot on named angle
def set_direction(direction,rate, pub, eps=1.e-2):
	if (direction == 'above'):
		angle = 0
	elif (direction == 'left'):
		angle = pi/2
	elif (direction == 'right'):
		angle = -pi/2
	elif (direction == 'below'):
		angle = pi
	elif (direction == 'above-left'):
		angle = pi/4
	elif (direction == 'above-right'):
		angle = -pi/4
	elif (direction == 'below-left'):
		angle = 3 * pi / 4
	elif (direction == 'below-right'):
		angle = -3 * pi / 4
	else:
		angle = 0

	rotate(angle, rate, pub, eps)

# return objects from global 'objects' list 
# ordered by distance in given radius
def object_near(rate, radius=1.0):
	global world
	global objects
	global x
	global y
	global grasped

	rate.sleep()

	indexes = []
	distances = []

	# finds objects near
	for obj in objects:
		if (obj in world.name) and (not obj in grasped):
			i = world.name.index(obj)

			cur_x = world.pose[i].position.x
			cur_y = world.pose[i].position.y

			dist = sqrt((x - cur_x)**2 + (y - cur_y)**2)

			if (dist <= radius):
				indexes.append(i)
				distances.append(dist)

	distances, indexes = zip(*sorted(zip(distances, indexes)))
	return distances, indexes

def top_object_near(rate, radius=1.0):
	global world
	global objects
	global x
	global y

	rate.sleep()

	indexes = []
	height = []

	# finds not grasped objects near
	for obj in objects:
		if (obj in world.name) and (not obj in grasped):
			i = world.name.index(obj)

			cur_x = world.pose[i].position.x
			cur_y = world.pose[i].position.y
			cur_z = world.pose[i].position.z

			dist = sqrt((x - cur_x)**2 + (y - cur_y)**2)

			if (dist <= radius):
				indexes.append(i)
				height.append(cur_z)

	height, indexes = zip(*sorted(zip(height, indexes)))
	return height, indexes


# grasp closest known object in given radius 
def pick_near(rate, pub, gripper, group, x_pos = 0.0, y_pos = -0.5, z_pos = 0.21, radius=1.0, eps=1.e-4):
	global world
	global objects
	global theta
	global x
	global y

	rate.sleep()

	# save initial state
	init_x = x
	init_y = y
	init_theta = theta

	# check for objects near
	distances, indexes = object_near(rate, radius)

	if (len(indexes) > 0):
		# closest object
		distances, indexes = zip(*sorted(zip(distances, indexes)))
		i = indexes[0]

		# rotation and bias calculation
		position = world.pose[i].position
		grasped.append(world.name[i])

		obj_rel_x = world.pose[i].position.x - x
		obj_rel_y = world.pose[i].position.y - y

		obj_rel_angle = atan2(obj_rel_y, obj_rel_x)

		theta %= 2 * pi
		obj_rel_angle %= 2 * pi

		# box's orientation
		orientation = world.pose[i].orientation
		orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
		_, _, orientation = euler_from_quaternion(orientation)

		obj_z_angle1 = orientation
		obj_z_angle1 %= 2 * pi

		obj_z_angle2 = orientation + (pi / 2)
		obj_z_angle2 %= 2 * pi

		obj_z_angle3 = orientation + pi
		obj_z_angle3 %= 2 * pi

		obj_z_angle4 = orientation + ((3 * pi) / 2)
		obj_z_angle4 %= 2 * pi

		z_angles = [obj_z_angle1, obj_z_angle2, obj_z_angle3, obj_z_angle4]

		# set proper angle for grasping
		min_bound = (obj_rel_angle) % (2 * pi)
		max_bound = (obj_rel_angle + 3 * pi / 4) % (2 * pi)

		rotation_angles = []
		for angle in z_angles:
			if ((angle >= min_bound) and (angle <= max_bound)) or ((max_bound <= min_bound) and (0 <= angle) and (angle <= max_bound)) or ((max_bound <= min_bound) and (angle <= 2 * pi) and (angle >= min_bound)):
				rotation_angles.append(angle)

		qwert,rotation_angles = zip(*sorted(zip([(angle - theta) % 2 * pi for angle in rotation_angles], rotation_angles)))
		rotation_angle = rotation_angles[0]

		# set proper position for grasping
		dist = sqrt((x - world.pose[i].position.x)**2 + (y - world.pose[i].position.y)**2)
		obj_rel_angle = atan2(world.pose[i].position.y - y, world.pose[i].position.x - x)
		obj_abs_angle = atan2(world.pose[i].position.y, world.pose[i].position.x)
		
		a = -x_pos
		b = -y_pos

		a1 = a*cos(rotation_angle) - b*sin(rotation_angle)
		b1 = a*sin(rotation_angle) + b*cos(rotation_angle)

		a = a1
		b = b1

		next_x = world.pose[i].position.x + a
		next_y = world.pose[i].position.y + b

		rate.sleep()

		# rotate and move towards object
		rotate(rotation_angle, rate, pub, eps)
		move_diag(next_x, next_y, rate, pub, eps)
		pick(rate, gripper, group, x_pos, y_pos, z_pos)
		
		# restore initial pose
		move_diag(init_x, init_y, rate, pub, eps)
		rotate(init_theta, rate, pub, eps)

def unstack_near(rate, pub, gripper, group, x_pos = 0.0, y_pos = -0.5, radius=1.0, eps=1.e-4):
	global world
	global objects
	global theta
	global x
	global y

	rate.sleep()

	# save initial state
	init_x = x
	init_y = y
	init_theta = theta

	# check for objects near
	height, indexes = top_object_near(rate, radius)

	if (len(indexes) > 0):
		# highest object
		# height, indexes = zip(*sorted(zip(height, indexes)))
		i = indexes[len(indexes) - 1]

		# rotation and bias calculation
		position = world.pose[i].position
		grasped.append(world.name[i])
		
		obj_rel_x = world.pose[i].position.x - x
		obj_rel_y = world.pose[i].position.y - y

		obj_rel_angle = atan2(obj_rel_y, obj_rel_x)

		theta %= 2 * pi
		obj_rel_angle %= 2 * pi

		# box's orientation
		orientation = world.pose[i].orientation
		orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
		_, _, orientation = euler_from_quaternion(orientation)

		obj_z_angle1 = orientation
		obj_z_angle1 %= 2 * pi

		obj_z_angle2 = orientation + (pi / 2)
		obj_z_angle2 %= 2 * pi

		obj_z_angle3 = orientation + pi
		obj_z_angle3 %= 2 * pi

		obj_z_angle4 = orientation + ((3 * pi) / 2)
		obj_z_angle4 %= 2 * pi

		z_angles = [obj_z_angle1, obj_z_angle2, obj_z_angle3, obj_z_angle4]

		# set proper angle for grasping
		min_bound = (obj_rel_angle) % (2 * pi)
		max_bound = (obj_rel_angle + 3 * pi / 4) % (2 * pi)

		rotation_angles = []
		for angle in z_angles:
			if ((angle >= min_bound) and (angle <= max_bound)) or ((max_bound <= min_bound) and (0 <= angle) and (angle <= max_bound)) or ((max_bound <= min_bound) and (angle <= 2 * pi) and (angle >= min_bound)):
				rotation_angles.append(angle)

		qwert,rotation_angles = zip(*sorted(zip([(angle - theta) % 2 * pi for angle in rotation_angles], rotation_angles)))
		rotation_angle = rotation_angles[0]

		# set proper position for grasping
		dist = sqrt((x - world.pose[i].position.x)**2 + (y - world.pose[i].position.y)**2)
		obj_rel_angle = atan2(world.pose[i].position.y - y, world.pose[i].position.x - x)
		obj_abs_angle = atan2(world.pose[i].position.y, world.pose[i].position.x)
		
		a = -x_pos
		b = -y_pos

		a1 = a*cos(rotation_angle) - b*sin(rotation_angle)
		b1 = a*sin(rotation_angle) + b*cos(rotation_angle)

		a = a1
		b = b1

		next_x = world.pose[i].position.x + a
		next_y = world.pose[i].position.y + b

		rate.sleep()

		# rotate and move towards object
		rotate(rotation_angle, rate, pub, eps)
		move_diag(next_x, next_y, rate, pub, eps)
		pick(rate, gripper, group, x_pos, y_pos, height[0] + 0.25)
		
		# restore initial pose
		move_diag(init_x, init_y, rate, pub, eps)
		rotate(init_theta, rate, pub, eps)

def main():
	global theta
	global x
	global y
	rospy.init_node('planner_adapter', anonymous=True)
	
	# sub = rospy.Subscriber("/base_odometry/odom", Odometry, new_pos)
	sub = rospy.Subscriber("/ground_truth/state", Odometry, new_pos)

	pub = rospy.Publisher('base_controller/command', Twist, queue_size=1)
	
	#relative position of block to robot
	block_size = 0.12

	x_pos = 0.2
	y_pos = -0.5
	z_pos = 0.23 + block_size/2

	block_angle = 0

	arm_name = 'right_arm'
	# gripper_name = 'right_gripper'

	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	# scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander(arm_name)
	# eef_link = group.get_end_effector_link()
	gripper = rospy.Publisher('r_gripper_controller/command', Pr2GripperCommand, queue_size=1)
	model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, new_state)

	rate = rospy.Rate(30)
	
	# place(rate, gripper, group)
	set_direction('above',rate, pub)
	# pick_near(rate, pub, gripper, group, x_pos, y_pos, z_pos)
	unstack_near(rate, pub, gripper, group, x_pos, y_pos)

	
	# gripper = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


	# set_direction('above',rate, pub)

	# move_diag(2, 3, rate, pub)
	# set_direction('above',rate, pub)
	# move_diag(0, 0, rate, pub)
	# pick(rate, gripper, group)

	rate.sleep()

	# move_diag(1, 1, rate, pub)

	# set_direction('above-right',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('right',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('below-right',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('below',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('below-left',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('left',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('above-left',rate, pub)
	# pick(rate, gripper, group)
	# rate.sleep()

	# set_direction('above',rate, pub)


	# rospy.sleep(0.1)
	# set_direction('below',rate, pub)
	# set_direction('below',rate, pub)
	# set_direction('right',rate, pub)

	
	# set_direction('left',rate, pub)
	# pick(rate, gripper, group)

	# set_direction('above',rate, pub)
	# pick(rate, gripper, group)
	# place(rate, gripper, group)

	# set_direction('above-left',rate, pub)
	# set_direction('below-left',rate, pub)
	# move_diag(0, 0, rate, pub)

	# set_direction('above-right',rate, pub)
	# set_direction('below-right',rate, pub)
	# pick(rate, gripper, group)
	# place(rate, gripper, group)
	# set_direction('above',rate, pub)
	rate.sleep()

	# theta = (theta * 180) / pi
	# if (theta < 0):
	# 	theta = 360 + theta


	# theta = 360 - theta
	# theta += 2*pi

	# x2 = x*cos(theta) + y*sin(theta)
	# y2 = -x*sin(theta) + y*cos(theta)
	# x_pos += x
	# y_pos += y

	# x_pos = x_pos*cos(theta) - y_pos*sin(theta)
	# y_pos = x_pos*sin(theta) + y_pos*cos(theta)
	# rospy.loginfo(str(theta))
	# rospy.loginfo(str(x_pos))
	# rospy.loginfo(str(y_pos))

	# pick(rate, gripper, group)
	# pick(rate, gripper, group, x_pos, y_pos, z_pos)


	# place(rate, gripper, group, x_pos, y_pos, z_pos)

	moveit_commander.roscpp_shutdown()

	rospy.loginfo("kind of shouldn't...")


if __name__ == "__main__":
	main()
