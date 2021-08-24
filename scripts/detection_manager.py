#!/usr/bin/env python

import traceback
from costmap_merge.srv import Handshake2, Handshake2Response, Transform, TransformRequest
from costmap_merge.msg import RobotList, TreeQueue
from random import random
from frame_tree import *
from queue_manager import QueueManager
from threading import Lock
from time import time


def translate_message(action, msg, detector_ns=''):
    msg2 = TransformRequest()
    msg2.action = action
    msg2.detector_ns = detector_ns
    msg2.robot_ns = msg.robot_ns
    msg2.pose_R_D = msg.pose_R_D
    msg2.pose_R_R = msg.pose_R_R
    msg2.alpha = msg.alpha
    msg2.beta = msg.beta
    return msg2


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

        # List of known robots for the costmap network
        self.robots = [self.namespace]
        self.robot_names_publisher = rospy.Publisher('/robots', RobotList, queue_size=10)

        # QUEUE ACCESS MANAGEMENT
        self.queue_manager = QueueManager()
        self.lock = Lock()
        self.timeout = rospy.Time(10).to_sec()

        # TRANSFORM MANAGER
        rospy.wait_for_service('transform_manager_service')
        self.transform_manager_proxy = rospy.ServiceProxy('transform_manager_service', Transform)

    def cb_handshake2(self, msg):
        rospy.sleep(random())
        self.lock.acquire()
        self.queue_manager.modify_queue(TreeQueue.PUSH)
        while not self.queue_manager.is_my_turn:  # waiting for its own callback
            print(self.namespace + ' dice: no es mi turno')
            rospy.sleep(random())
        while self.queue_manager.detection_queue.queue[0] != self.namespace:
            rospy.sleep(random())
        tree = self.check_world(msg.robot_ns)
        print('detector ' + self.namespace)
        print('tree: ')
        print(tree)
        if tree:
            rospy.loginfo('[detection_manager ' + self.namespace + ']: time difference ' + str(
                time() - self.world[tree].get_node(msg.robot_ns).get_stamp()))
            if time() - self.world[tree].get_node(msg.robot_ns).get_stamp() < self.timeout:
                print('detector ' + self.namespace + 'saliendo del handshake2')
                self.queue_manager.modify_queue(TreeQueue.POP)
                self.lock.release()
                return Handshake2Response()
            else:
                rospy.loginfo('STOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOP')
                self.call_transform_manager('STOP', msg, tree)
        else:
            if not self.tree.get_node(msg.robot_ns):
                self.tree.create_child(msg.robot_ns)
                frame_list = self.builder.build_frame_list_msg(self.tree)
                print(self.namespace)
                print(frame_list.frames)
                self.tree_publisher.publish(frame_list)
                self.world[self.namespace] = self.tree
                self.call_transform_manager('START', msg, self.namespace)
                if msg.robot_ns not in self.robots:
                    self.robots.append(msg.robot_ns)
            else:
                self.tree.get_node(msg.robot_ns).update_stamp(time())
                frame_list = self.builder.build_frame_list_msg(self.tree)
                print(self.namespace)
                print(frame_list.frames)
                self.tree_publisher.publish(frame_list)
                self.world[self.namespace] = self.tree
                self.call_transform_manager('UPDATE', msg, self.namespace)
        self.publish_robots_list()
        self.queue_manager.modify_queue(TreeQueue.POP)
        self.lock.release()
        print('END OF THE HANDSHAKE 2')
        return Handshake2Response()

    def check_world(self, robot):
        for tree in self.world:
            if tree is not self.namespace:
                self.robots = list(set(self.world[tree].get_frame_ids(self.robots)))
                print('self.robots')
                print(self.robots)
                if self.world[tree].get_node(robot):
                    return tree
        return None

    def cb_update_world(self, msg):
        if msg.detector_ns is not self.namespace:
            self.world[msg.detector_ns] = self.parser.frame_list_to_node(msg)
            print(self.namespace)
            print(msg.detector_ns)
            print(self.world[msg.detector_ns].child_frames)

    def publish_robots_list(self):
        msg = RobotList()
        for robot in self.robots:
            msg.robot_list.append(robot)
        self.robot_names_publisher.publish(msg)

    def call_transform_manager(self, action, msg, detector_ns):
        msg2 = translate_message(action, msg, detector_ns)
        self.transform_manager_proxy(msg2)


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
