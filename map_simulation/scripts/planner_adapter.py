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
import json
import argparse

block_size = 0.065
gripper_size = 0.144
plain_heigth = -0.015
height_adjustment = 0.05

# global variable of grippable world objects
objects = ['unit_box','unit_box_copy']
agents = {}

rate = None

# global variable of gazebo world model
world = ModelStates()

# set gripper opened
def open_gripper(ag_name, wait = 1):
	global agents
	global rate
	gripper = agents[ag_name]['gripper']
	opened = Pr2GripperCommand()
	opened.position = 1.0
	opened.max_effort = -10.0
	i = 0
	while i < 30:
		gripper.publish(opened)
		rate.sleep()
		i += 1
	rate.sleep()
	rospy.sleep(wait)

# set gripper closed
def close_gripper(ag_name, wait = 1):
	global agents
	global rate

	gripper = agents[ag_name]['gripper']
	closed = Pr2GripperCommand()
	closed.position = 0.0
	closed.max_effort = -10.0
	i = 0
	while i < 30:
		gripper.publish(closed)
		rate.sleep()
		i += 1
	rospy.sleep(wait)

# move manipulator group
def move_group(ag_name, pose_goal, wait = 0.1):
	global agents
	group = agents[ag_name]['group']
	
	group.set_pose_target(pose_goal)
	group.go(wait=True)
	group.stop()
	group.clear_pose_targets()

	rospy.sleep(wait)

# calculate gripper position relative to robot
def adjust_pose(ag_name, x_pos = 0.2, y_pos = -0.5, z_pos = 0.032):
	global block_size
	global gripper_size
	global plain_heigth
	global height_adjustment

	global agents
	global rate

	rate.sleep()

	theta = agents[ag_name]['theta']
	x = agents[ag_name]['x']
	y = agents[ag_name]['y']

	gripper_angle = theta
	q_rotation = quaternion_about_axis(gripper_angle, (0,0,1))

	# safe position calculation
	# x_safe = 0.75
	# y_safe = -0.19
	# z_safe = 0.88
	
	x_safe = 0.42
	y_safe = -0.38
	z_safe = 0.5

	# x_safe = -0.22
	# y_safe = -0.13
	# z_safe = 0.44

	x_new_s = x_safe*cos(theta) - y_safe*sin(theta)
	y_new_s = x_safe*sin(theta) + y_safe*cos(theta)

	x_pos_s = x_new_s + x
	y_pos_s = y_new_s + y 


	pose_goal_safe = geometry_msgs.msg.Pose()

	pose_goal_safe.position.x = x_pos_s
	pose_goal_safe.position.y = y_pos_s
	pose_goal_safe.position.z = z_safe

	q_pose = [0.5, 0.5, -0.5, 0.5]
	q_res = quaternion_multiply(q_rotation, q_pose)

	pose_goal_safe.orientation.x = q_res[0]
	pose_goal_safe.orientation.y = q_res[1]
	pose_goal_safe.orientation.z = q_res[2]
	pose_goal_safe.orientation.w = q_res[3]

	#__________________________________
	# rotated gripper position calculation
	x_new = x_pos*cos(theta) - y_pos*sin(theta)
	y_new = x_pos*sin(theta) + y_pos*cos(theta)

	x_intercept = -0.05
	x_new_i = (x_pos + x_intercept)*cos(theta) - y_pos*sin(theta)
	y_new_i = (x_pos + x_intercept)*sin(theta) + y_pos*cos(theta)

	x_pos_i = x_new_i + x
	y_pos_i = y_new_i + y 
	
	x_pos = x_new + x
	y_pos = y_new + y

	#__________________________________
	z_level = z_pos + gripper_size + height_adjustment

	w = 0.5
	x = 0.5
	y = 0.5
	z = -0.5

	q_pose = [x, y, z, w]
	q_res = quaternion_multiply(q_rotation, q_pose)
	x, y, z, w = q_res

	#__________________________________
	pose_goal_up = geometry_msgs.msg.Pose()

	pose_goal_up.orientation.w = w
	pose_goal_up.orientation.x = x
	pose_goal_up.orientation.y = y
	pose_goal_up.orientation.z = z

	pose_goal_up.position.x = x_pos_i
	pose_goal_up.position.y = y_pos_i
	pose_goal_up.position.z = z_level + 0.1

	#__________________________________
	pose_goal_down = geometry_msgs.msg.Pose()

	pose_goal_down.orientation.w = w
	pose_goal_down.orientation.x = x
	pose_goal_down.orientation.y = y
	pose_goal_down.orientation.z = z

	pose_goal_down.position.x = x_pos
	pose_goal_down.position.y = y_pos
	pose_goal_down.position.z = z_level

	return pose_goal_up, pose_goal_down, pose_goal_safe# pose_goal_up

# grasp object on given coordinates relatively to robot
def pick(ag_name, z_pos = 0.032, wait = 1):
	global rate

	pose_goal_up, pose_goal_down, pose_goal_safe = adjust_pose(ag_name, z_pos=z_pos)

	open_gripper(ag_name)
	move_group(ag_name, pose_goal_up, 1)

	move_group(ag_name, pose_goal_down, 1)
	close_gripper(ag_name)

	move_group(ag_name, pose_goal_up, 1)
	move_group(ag_name, pose_goal_safe, 1)
	rate.sleep()
	# move_group(ag_name, pose_goal_up, 1)

# release object on given coordinates relatively to robot
def place(ag_name, z_pos = 0.032, wait = 1):
	global agents
	global objects
	global rate

	grasped = agents[ag_name]['grasped']
	pose_goal_up, pose_goal_down, pose_goal_safe = adjust_pose(ag_name, z_pos=z_pos)
	
	move_group(ag_name, pose_goal_up, 1)

	move_group(ag_name, pose_goal_down, 1)
	open_gripper(ag_name, wait)

	move_group(ag_name, pose_goal_up, 1)
	move_group(ag_name, pose_goal_safe, 1)
	obj = grasped.pop()
	objects.append(obj)

	agents[ag_name]['grasped'] = grasped
	rate.sleep()


# move robot to given coordinates
def move_diag(ag_name, x_coord, y_coord, eps=1.e-2, percise=False):
	global agents
	global rate

	point = Point()
	point.x = x_coord
	point.y = y_coord
	prev_dist = None

	base = agents[ag_name]['base']

	twist = Twist()
	rate.sleep()
	while not rospy.is_shutdown():
		x = agents[ag_name]['x']
		y = agents[ag_name]['y']
		inc_x = point.x - x
		inc_y = point.y - y
		# angle = theta !!!
		angle = agents[ag_name]['theta']
		dist = sqrt(inc_x**2 + inc_y**2)

		xsp = inc_x/abs(dist) if abs(dist) > 1 else inc_x
		ysp = inc_y/abs(dist) if abs(dist) > 1 else inc_y

		twist.linear.x = xsp*cos(angle) + ysp*sin(angle)
		twist.linear.y = -xsp*sin(angle) + ysp*cos(angle)
		
		# rospy.loginfo("Distance remained: " + str(dist))

		if ((abs(inc_y) <= eps) and (abs(inc_x) <= eps)):
			rate.sleep()
			twist.linear.x = 0
			twist.linear.y = 0
			base.publish(twist)
			break

		if not percise:
			if prev_dist:
				last_step = abs(prev_dist - dist) 
				if (last_step < eps**2) and (dist < eps*2):
					rate.sleep()
					twist.linear.x = 0
					twist.linear.y = 0
					base.publish(twist)
					break

		# else:
		# 	rospy.sleep(0.1)
		prev_dist = dist

		base.publish(twist)
		rate.sleep()

# rotate robot on angle
def rotate(ag_name, angle, eps=1.e-2, percise=False):
	global agents
	global rate

	base = agents[ag_name]['base']

	twist = Twist()
	rate.sleep()
	prev_angle = None
	while not rospy.is_shutdown():
		theta = agents[ag_name]['theta']

		theta = (theta) % (2 * pi)
		angle = (angle) % (2 * pi)
		
		a = (theta - angle) % (2 * pi)
		ang_dist = min(abs(a), 2 * pi - abs(a))
		# rospy.loginfo("Angle remained: " + str(ang_dist))
		if (ang_dist >= eps):
			twist.angular.z = ang_dist
			if ((abs(a) < pi)):
				twist.angular.z *= - 1
			base.publish(twist)
		else:			
			twist.angular.z = 0.0
			base.publish(twist)
			rate.sleep()
			break

		# if rotation got to slow
		if not percise:
			if prev_angle:
				last_step = min(prev_angle - theta % 2 * pi, theta - prev_angle % 2 * pi)
				if (last_step < eps**2) and (ang_dist < eps*2):
					twist.angular.z = 0.0
					base.publish(twist)
					rate.sleep()
					break
		# else:
		# 	rospy.sleep(0.1)
		prev_angle = theta
		rate.sleep()

# rotate robot on named angle
def set_direction(ag_name, direction):
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

	rotate(ag_name, angle)

# return objects from global 'objects' list 
# ordered by distance in given radius
def object_near(ag_name, radius=1.0):
	global world
	global objects
	global agents
	global rate

	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	grasped = agents[ag_name]['grasped']

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
	if (len(height) == 0):
		return [],[]
	distances, indexes = zip(*sorted(zip(distances, indexes)))
	return distances, indexes

# return objects from global 'objects' list 
# ordered by height in given radius
def top_object_near(ag_name, radius=1.0):
	global world
	global objects
	global agents
	global rate


	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	grasped = agents[ag_name]['grasped']

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

	if (len(height) == 0):
		return [],[]

	height, indexes = zip(*sorted(zip(height, indexes)))

	height = list(height)
	indexes = list(indexes)

	height.reverse()
	indexes.reverse()
	return height, indexes


def object_interaction_preparation(ag_name, world_obj_id, x_pos = 0.2, y_pos = -0.5):
	global world
	global agents
	global rate
	global objects

	rate.sleep()
	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	theta = agents[ag_name]['theta']
	grasped = agents[ag_name]['grasped']

	# rotation and bias calculation
	i = world_obj_id
	position = world.pose[i].position
	grasped.append(world.name[i])
	objects.pop(objects.index(world.name[i]))

	agents[ag_name]['grasped'] = grasped

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
	min_bound = (obj_rel_angle + pi / 4) % (2 * pi)
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
	rotate(ag_name, rotation_angle)
	move_diag(ag_name, next_x, next_y)


# grasp closest known object in given radius
# x_pos, y_pos, z_pos - desired position of block, relatively to robot
def pick_near(ag_name, radius=1.0):
	global agents
	global rate

	rate.sleep()

	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	theta = agents[ag_name]['theta']

	# save initial state
	init_x = x
	init_y = y
	init_theta = theta
	rospy.loginfo('theta: ' + str(init_theta))

	# check for objects near
	distances, indexes = object_near(ag_name, radius)

	if (len(indexes) > 0):
		# closest object
		i = indexes[0]

		object_interaction_preparation(ag_name, i)
		
		pick(ag_name, world.pose[i].position.z)
		
		# restore initial pose
		move_diag(ag_name, init_x, init_y)
		rotate(ag_name, init_theta)

def unstack_near(ag_name, radius=1.0):
	global agents
	global rate

	rate.sleep()
	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	theta = agents[ag_name]['theta']


	# save initial state
	init_x = x
	init_y = y
	init_theta = theta
	rospy.loginfo('theta: ' + str(init_theta))


	# check for objects near
	height, indexes = top_object_near(ag_name, radius)

	if (len(indexes) > 0):
		i = indexes[0]

		object_interaction_preparation(ag_name, i)

		pick(ag_name, height[0])
		
		# restore initial pose
		move_diag(ag_name, init_x, init_y)
		rotate(ag_name, init_theta)

def stack_near(ag_name, radius=1.0):
	global rate
	global agents
	global block_size

	rate.sleep()
	x = agents[ag_name]['x']
	y = agents[ag_name]['y']
	theta = agents[ag_name]['theta']

	rate.sleep()

	# save initial state
	init_x = x
	init_y = y
	init_theta = theta
	rospy.loginfo('theta: ' + str(init_theta))

	# check for objects near
	height, indexes = top_object_near(ag_name, radius)

	if (len(indexes) > 0):
		i = indexes[0]

		object_interaction_preparation(ag_name, i)

		place(ag_name, height[0] + block_size)
		
		# restore initial pose
		move_diag(ag_name, init_x, init_y)
		rotate(ag_name, init_theta)

def read_solution(filename='solution.txt'):
	file = open(filename, 'r')
	plan = []
	for action in file.readlines():
		for symb in "(),\'":
			action = action.replace(symb,"")
		action = action.split()
		action[2] = float(action[2]) / 20
		action[3] = float(action[3]) / 20
		plan.append(action)
	return plan

# def world_as_json():
# 	global world
# 	rospy.sleep(0.1)

# 	data = {}
# 	objects = []

# 	pass

# updates global variables 'x','y' and 'theta'
class NodeListner(object):
	def __init__(self, ag_name):
		self.ag_name = ag_name
		
	def new_pos(self, msg):
		agents[self.ag_name]['x'] = msg.pose.pose.position.x
		agents[self.ag_name]['y'] = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		_, _, angle = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
		agents[self.ag_name]['theta'] = angle

# updates 'world' global variable
def new_state(msg):
	global world
	world = msg

def main(args):
	global objects
	global rate
	global agents

	objects = args.objects
	rospy.init_node('planner_adapter', anonymous=True)
	model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, new_state)
	rate = rospy.Rate(30)
	commanders = []

	# setting agents
	for name in args.agents:
		ag = {}
		m_c = moveit_commander
		m_c.roscpp_initialize(name)
		commanders.append(m_c)

		ag['x'] = 0
		ag['y'] = 0
		ag['theta'] = 0
		ag['grasped'] = []
		ag['base'] = rospy.Publisher('/'+name+'/base_controller/command', Twist, queue_size=1)
		robot = moveit_commander.RobotCommander()
		print(robot.get_group_names())
		ag['group'] = moveit_commander.MoveGroupCommander('right_arm', ns=name)
		ag['gripper'] = rospy.Publisher('/'+name+'/r_gripper_controller/command', Pr2GripperCommand, queue_size=1)

		nl_ag = NodeListner(name)
		rospy.Subscriber("/"+name+"/ground_truth/state", Odometry, nl_ag.new_pos)

		agents[name] = ag

	rospy.sleep(1)
	agents['I'] = agents['ag1']

	for action in read_solution(args.plan):
		rospy.loginfo(action)
		if (action[0] == 'move'):
			move_diag(action[1], action[2], action[3])
		elif (action[0] == 'pick-up'):
			unstack_near(action[1])
		elif (action[0] == 'stack'):
			stack_near(action[1])
		elif (action[0] == 'rotate'):
			set_direction(action[1], action[4])
		elif (action[0] == 'Clarify'):
			pass
		elif (action[0] == 'Abstract'):
			pass

	for m_c in commanders:
		m_c.roscpp_shutdown()

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='ROS Kinetic node for sign based spatial planner\'s plans simulation. Manages PR2 robots in fully observable Gazebo environment.')
	parser.add_argument('--plan', type=str, default='solution.txt', nargs='?', help='relative path to prepared plan')
	parser.add_argument('--objects', type=str, nargs='+', default=['block-a','block-b','block-c'], help='names of all objects, which agents can grab')
	parser.add_argument('--agents', type=str, nargs='+', default=['ag1'], help='names of all agents (same for plan and Gazebo environment)')
	main(parser.parse_args())