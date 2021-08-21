#!/usr/bin/env python

import traceback
from costmap_merge.srv import Handshake2, Handshake2Response
from costmap_merge.msg import RobotList, TreeQueue
from random import random
from frame_tree import *
from queue_manager import QueueManager
from threading import Lock


class DetectionManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service receiving poses and angles for transforms computation
        rospy.Service('handshake2_service', Handshake2, self.cb_handshake2)

        # FRAME TREE MANAGEMENT
        self.tree = Node(self.namespace)
        self.tree_publisher = rospy.Publisher('/frame_tree', FrameList, queue_size=10)
        self.world = dict()
        self.world[self.namespace] = self.tree
        rospy.Subscriber('/frame_tree', FrameList, self.cb_update_world, queue_size=1)
        self.builder = FrameListMsgBuilder()
        self.parser = FrameListMsgParser()

        # List of known robots
        self.robots = [self.namespace]
        self.robot_names_publisher = rospy.Publisher('/robots', RobotList, queue_size=10)

        # QUEUE ACCESS MANAGEMENT
        self.queue_manager = QueueManager()
        self.lock = Lock()
        self.timeout = rospy.Time(10).to_sec()

    def cb_handshake2(self, msg):
        self.lock.acquire()
        self.queue_manager.modify_queue(TreeQueue.PUSH)
        while not self.queue_manager.received:  # waiting for its own callback
            rospy.sleep(random())
        while self.queue_manager.detection_queue.queue[0].keys()[0] != self.namespace:
            rospy.sleep(random())
        tree = self.check_world(msg.robot_ns)
        if tree:
            if rospy.Time.now().to_sec() - self.world[tree].get_stamp() < self.timeout:
                return Handshake2Response()
            else:
                self.transform_manager.stop_transformer_node(tree, msg.robot_ns)
        else:
            node = self.tree.get_node(msg.robot_ns)
            if not node:
                self.tree.create_child(msg.robot_ns)
                frame_list = self.builder.build_frame_list_msg(self.tree)
                self.tree_publisher.publish(frame_list)
                self.world[self.namespace] = self.tree
                self.transform_manager.start_transformer_node(msg)
                print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
                if msg.robot_ns not in self.robots:
                    self.robots.append(msg.robot_ns)
            else:
                node.update_stamp(rospy.Time.now().to_sec())
                self.transform_manager.update_transform(msg)
        self.publish_robots_list()
        self.queue_manager.modify_queue(TreeQueue.POP)
        self.lock.release()
        return Handshake2Response()

    def check_world(self, robot):
        for tree in self.world:
            if tree is not self.namespace and self.world[tree]:  # TODO: delete the second condition
                self.robots = list(set(self.world[tree].get_frame_ids(self.robots)))
                if self.world[tree].get_node(robot):
                    return tree
        return None

    def cb_update_world(self, msg):
        if msg.detector_ns is not self.namespace:
            self.world[msg.detector_ns] = self.parser.frame_list_to_node(msg)

    def publish_robots_list(self):
        msg = RobotList()
        for robot in self.robots:
            msg.robot_list.append(robot)
        self.robot_names_publisher.publish(msg)


if __name__ == "__main__":

    try:
        rospy.init_node('detection_manager', log_level=rospy.INFO)
        rospy.loginfo('[detection_manager]: Node started')
        manager = DetectionManager()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[detection_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
