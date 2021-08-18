#!/usr/bin/env python

import traceback

from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import RobotName, RobotList
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
        # TransformBroadcaster to send transformations to the tf_tree
        self.br = tf2_ros.TransformBroadcaster()
        # representation of the tf_tree created by this detector
        self.tree = Node(self.namespace)
        # Publishes this detector frame tree
        self.tree_publisher = rospy.Publisher('/frame_tree', FrameList, queue_size=10)
        # Subscribes to other detectors frame trees
        rospy.Subscriber('/frame_tree', FrameList, self.cb_update_world, queue_size=1)
        # FrameTrees managers
        self.world = dict()
        self.world[self.namespace] = self.tree
        self.builder = FrameListMsgBuilder()
        self.parser = FrameListMsgParser()
        # List of known robots
        self.robots = [self.namespace]
        # Publishes robots detected by any detector
        self.robot_names_publisher = rospy.Publisher('/robots', RobotList, queue_size=10)

    def cb_handshake2(self, msg):
        # TODO: there is still the open issue of two detectors detecting each other at exactly the same time.
        rospy.sleep(random())
        for tree in self.world:
            # rospy.loginfo('[tf_tree_manager]: ' + self.namespace + ' - ' + str(tree))
            if tree is not self.namespace and self.world[tree]:
                self.robots = list(set(self.world[tree].get_frame_ids(self.robots)))
                if self.world[tree].get_node(msg.robot_ns):
                    return Handshake2Response()
        if not self.tree.get_node(msg.robot_ns):
            self.update_tree(msg.robot_ns)
            self.publish_tree()
            rospy.loginfo('[tf_tree_manager]: ' + self.namespace + ' - ' + str(self.tree.frame_id)
                          + ' - ' + str(self.tree.parent_frame))
            rospy.loginfo('[tf_tree_manager]: ' + self.namespace + ' - ' + msg.robot_ns)
            self.world[self.namespace] = self.tree
            if msg.robot_ns not in self.robots:
                self.robots.append(msg.robot_ns)
                self.publish_robots_list()
        rospy.loginfo('[tf_tree_manager]: ' + self.namespace + ' - ' + str(self.tree.frame_id)
                      + ' - ' + str(self.tree.parent_frame))
        rospy.loginfo('[tf_tree_manager]: ' + self.namespace + ' - ' + msg.robot_ns)
        self.update_tf_tree(msg)
        return Handshake2Response()

    def update_tf_tree(self, msg):
        t = th.get_frame_transform(msg.pose_R_D, msg.pose_R_R, msg.alpha, msg.beta)
        self.br.sendTransform(t)

    def update_tree(self, frame_id):
        self.tree.create_child(frame_id)

    def cb_update_world(self, msg):
        if msg.detector_ns is not self.namespace:
            self.world[msg.detector_ns] = self.parser.frame_list_to_node(msg)

    def publish_robots_list(self):
        robot_list = RobotList()
        for robot in self.robots:
            _robot = RobotName()
            _robot.robot_ns = robot
            robot_list.robot_list.append(_robot)
        self.robot_names_publisher.publish(robot_list)

    def publish_tree(self):
        frame_list = self.builder.build_frame_list_msg(self.tree)
        self.tree_publisher.publish(frame_list)

    def add_node(self, frame_id, parent_frame):
        node = Node(frame_id, parent_frame)
        if not node.parent_frame:
            self.tree.add_child(node)
        if self.tree.get_node(node.parent_frame):
            self.tree.get_node(node.parent_frame).add_child(node)


if __name__ == "__main__":

    try:
        rospy.init_node('tf_tree_manager', log_level=rospy.INFO)
        rospy.loginfo('[tf_tree_manager]: Node started')
        manager = TFTreeManager()
        while not rospy.is_shutdown():
            manager.publish_tree()
            rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[tf_tree_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
