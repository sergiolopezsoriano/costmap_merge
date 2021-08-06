#!/usr/bin/env python

import rospy
import traceback
from nodes import OdomNode as On
import math
from costmap_merge.srv import Handshake
from helpers import TransformHelper, PoseHelper
import numpy as np


class HandshakeNode:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.robots_names = rospy.get_param('~robots_names')
        # OdomNode dictionary of all the simulated robots
        self.robots = dict()
        for namespace in self.robots_names:
            self.robots[namespace] = On(namespace)
            self.robots[namespace].pose = TransformHelper.get_map_to_odom_transform(namespace)
        # Service to send the pose and frame transforms
        self.detector_finder_service = rospy.Service('handshake_service', Handshake, self.find_detector)

    def find_detector(self, msg):
        for robot in [self.namespace, msg.namespace]:
            yaw = TransformHelper.get_yaw_from_orientation(self.robots[robot].pose.pose.orientation)
            self.robots[robot].x = self.robots[robot].pose.pose.position.x + self.robots[
                robot].odom.pose.pose.position.x * np.cos(yaw) - self.robots[
                                       robot].odom.pose.pose.position.y * np.sin(yaw)
            self.robots[robot].y = self.robots[robot].pose.pose.position.y + self.robots[
                robot].odom.pose.pose.position.y * np.cos(yaw) + self.robots[
                                       robot].odom.pose.pose.position.x * np.sin(yaw)
        pose_D_R = self.find_detector_in_costmap(msg.namespace)
        pose_R_R = PoseHelper.get_pose_from_odom(self.odom)
        beta = math.atan2((self.robots[msg.namespace].y - self.robots[self.namespace].y) / (
                    self.robots[msg.namespace].x - self.robots[self.namespace].x))
        return pose_D_D, pose_R_D

    def find_detector_in_costmap(self, namespace):
        pose_D_R = PoseHelper.get_pose_from_odom(self.robots[namespace].odom)
        return pose_D_R


if __name__ == "__main__":

    try:
        rospy.init_node('handshake_node', log_level=rospy.INFO)
        rospy.loginfo('[handshake_node]: Node started')
        hn = HandshakeNode()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[handshake_node]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)