#!/usr/bin/env python

import rospy
import traceback
from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import Frame, FrameList
from helpers import TransformHelper as th
import tf2_ros
from random import random
from frame_tree import *


class TFTreeManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service receiving poses and angles for transforms computation
        rospy.Service('handshake2_service', Handshake2, self.cb_handshake2)
        # Publishes all known robot poses in the detector frame.
        self.poses = dict()
        self.robot_list_publisher = rospy.Publisher('/detected_robots_topic', FrameList, queue_size=10)
        rospy.Subscriber('/detected_robots_topic', FrameList, self.cb_update_tree_network, queue_size=1)
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # representation of the tf_tree created by this detector
        self.tree = Node(self.namespace)
        # FrameTrees managers
        self.published_trees = dict()
        self.builder = FrameListMsgBuilder()
        self.parser = FrameListMsgParser()
        # List of known robots
        self.robots = [self.namespace]

    def cb_handshake2(self, msg):
        if msg.robot_ns is not self.namespace:
            for tree in self.published_trees:
                self.robots = list(set(self.published_trees[tree].get_frame_ids(self.robots)))
                if self.published_trees[tree].get_node(msg.robot_ns):
                    return
        else:
            if self.tree.get_node(msg.robot_ns):
                pass
            else:
                if msg.robot_ns not in self.robots:
                    self.robots.append(msg.robot_ns)
            self.update_tree(msg.robot_ns)
            self.update_tf_tree(msg)

        return Handshake2Response()

    def update_tf_tree(self, msg):
        t = th.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)
        self.br.sendTransform(t)

    def add_node(self, frame_id, parent_frame):

    def update_tree(self, frame_id):
        self.tree.create_child(frame_id)

    def cb_update_tree_network(self, msg):
        if msg.detector_ns is not self.namespace:
            self.published_trees[msg.detector_ns] = self.parser.frame_list_to_node(msg.frames)

    def publish_robots_list(self):
        for robot in self.robots:
            self.robot_list_publisher.publish(robot)


if __name__ == "__main__":

    try:
        rospy.init_node('tf_tree_manager', log_level=rospy.INFO)
        rospy.loginfo('[tf_tree_manager]: Node started')
        manager = TFTreeManager()
        while not rospy.is_shutdown():
            if manager.detected_robots:
                manager.publish_robots_list()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[tf_tree_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
