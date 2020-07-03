#! /usr/bin/env python
#coding=utf-8
import rospy
from pick_place_bridge.srv import PickPlacePose, PickPlacePoseRequest, PickPlacePoseResponse

rospy.init_node("client_test")
client = rospy.ServiceProxy("place", PickPlacePose)
req = PickPlacePoseRequest()
req.Pose.pose.position.x = 0.845525
req.Pose.pose.position.y = 0.0713995
req.Pose.pose.position.z = 1.35711
req.Pose.pose.orientation.x = -0.0171715
req.Pose.pose.orientation.y = 0.015509
req.Pose.pose.orientation.z = 0.0865081
req.Pose.pose.orientation.w = 0.995982
p = PickPlacePose()

client.call(req)

