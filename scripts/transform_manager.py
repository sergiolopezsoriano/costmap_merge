#!/usr/bin/env python

import rospy
import traceback
from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import RobotName
from helpers import TransformHelper
import tf2_ros
from random import random
from frame_tree import FrameTree


class TransformManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service receiving poses and angles for transforms computation
        rospy.Service('handshake2_service', Handshake2, self.cb_handshake2)
        # Publishes all known robot poses in the detector frame.
        self.poses = dict()
        self.robot_list_publisher = rospy.Publisher('/detected_robots_topic', RobotName, queue_size=10)
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # representation of the tf_tree created by this detector
        self.frame_tree = FrameTree(self.namespace)
        # List of known robots
        self.detected_robots = [self.namespace]

    def cb_handshake2(self, msg):
        if self.tree.find_node_in_tree(msg.robot_ns):
            pass
        if msg.robot_ns not in self.detected_robots:
            self.detected_robots.append(msg.robot_ns)
        # If there are two or more detectors
        if self.namespace[0:8] == msg.robot_ns[0:8]:
            pass
        return Handshake2Response()

    def cb_build_tree(self, msg):
        pass


    def publish_transforms(self):
        rospy.sleep(random())

        #     if self.namespace[9:] < msg.robot_ns[9:]:
        #         t = TransformHelper.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)
        #     else:
        #         t = TransformHelper.get_frame_transform(msg.pose_R_R, msg.pose_R_D, msg.beta, msg.alpha)
        # else:
        #     t = TransformHelper.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)
        # self.br.sendTransform(t)

    def publish_robots_list(self):
        for robot in self.detected_robots:
            self.robot_list_publisher.publish(robot)


if __name__ == "__main__":

    try:
        rospy.init_node('transform_manager', log_level=rospy.INFO)
        rospy.loginfo('[transform_manager]: Node started')
        manager = TransformManager()
        while not rospy.is_shutdown():
            if manager.detected_robots:
                manager.publish_robots_list()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[transform_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
