#!/usr/bin/env python

import rospy

def main():
	rospy.init_node('planner_adapter', anonymous=True)
	
	rospy.loginfo("planner_adapter works!")
	
	rospy.spin()

if __name__ == "__main__":
	main()
