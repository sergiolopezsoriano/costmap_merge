#!/usr/bin/env python

import traceback
from costmap_merge.srv import Handshake2, Handshake2Response, Transform, TransformRequest, RobotName, RobotNameResponse
from costmap_merge.msg import RobotList, TreeQueue
from random import random
from frame_tree import *
from queue_manager import QueueManager
from threading import Lock


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
        self.tree = Node(self.namespace, rospy.Time.now())
        self.tree_publisher = rospy.Publisher('/tree', FrameList, queue_size=10)
        self.world = dict()
        self.world[self.namespace] = self.tree
        rospy.Subscriber('/tree', FrameList, self.cb_update_world, queue_size=1)
        self.builder = FrameListMsgBuilder()
        self.parser = FrameListMsgParser()
        rospy.Service('remove_node_service', RobotName, self.cb_remove_node)
        self.remove_node_proxies = dict()

        # List of known robots for the costmap network
        self.robots = [self.namespace]
        self.robot_names_publisher = rospy.Publisher('/robots', RobotList, queue_size=10)

        # QUEUE ACCESS MANAGEMENT
        self.queue_manager = QueueManager()
        self.lock = Lock()
        self.timeout = rospy.get_param('~timeout')

        # TRANSFORM MANAGER
        rospy.wait_for_service('transform_manager_service')
        self.transform_manager_proxy = rospy.ServiceProxy('transform_manager_service', Transform)

    def cb_handshake2(self, msg):
        rospy.sleep(random())
        self.lock.acquire()
        self.queue_manager.modify_queue(TreeQueue.PUSH)
        while not self.queue_manager.is_my_turn:  # waiting for its own callback
            rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: is_my_turn check!')
            rospy.sleep(random())
        my_detector = self.check_world(self.namespace)
        my_detector_frame_id = None
        if my_detector:
            my_detector_frame_id = self.world[my_detector].frame_id
        tree = self.check_world(msg.robot_ns)
        rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: ' + str(tree) + ' has detected ' + msg.robot_ns + ' first.')
        rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: frame ids = ' + str(self.tree.get_frame_ids(list())))
        if tree and tree != msg.robot_ns and my_detector_frame_id != msg.robot_ns:
            rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: time difference = ' + str(rospy.Time.now().to_sec() - self.world[tree].get_node(msg.robot_ns).get_stamp().to_sec()))
            if rospy.Time.now().to_sec() - self.world[tree].get_node(msg.robot_ns).get_stamp().to_sec() < self.timeout:
                rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: exiting handshake2. ' + msg.robot_ns + ' detected by ' + tree + ', less than ' + str(self.timeout) + ' seconds ago.')
                self.queue_manager.modify_queue(TreeQueue.POP)
                self.lock.release()
                return Handshake2Response()
            else:
                rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: Stopping transform broadcaster and removing node ' + msg.robot_ns + ' from tree ' + tree)
                self.call_transform_manager('STOP', msg, tree)
                self.remove_node_from_tree(msg.robot_ns, tree)
        else:  # If the robot is not in a tree or the robot in its own tree, therefore a detector
            if not self.tree.get_node(msg.robot_ns):
                self.tree.create_child(msg.robot_ns, rospy.Time.now())
                self.update_world()
                self.call_transform_manager('START', msg, self.namespace)
                if msg.robot_ns not in self.robots:
                    self.robots.append(msg.robot_ns)
            else:
                self.tree.get_node(msg.robot_ns).update_stamp(rospy.Time.now())
                self.update_world()
                self.call_transform_manager('UPDATE', msg, self.namespace)
        self.publish_robots_list()
        self.queue_manager.modify_queue(TreeQueue.POP)
        self.lock.release()
        rospy.logdebug('[' + self.namespace + '][detection_manager.cb_handshake2]: END OF THE HANDSHAKE 2')
        return Handshake2Response()

    def check_world(self, robot):
        for tree in self.world:
            if tree is not self.namespace:
                self.robots = list(set(self.world[tree].get_frame_ids(self.robots)))
                rospy.logdebug('[' + self.namespace + '][detection_manager.check_world]: robots = ' + str(self.robots))
                if self.world[tree].get_node(robot):
                    return tree
        return None

    def cb_update_world(self, msg):
        if msg.detector_ns != self.namespace:
            self.world[msg.detector_ns] = self.parser.frame_list_to_node(msg)
            rospy.logdebug('[' + self.namespace + '][detection_manager.cb_update_world]: ' + msg.detector_ns + ' has detected ' + str(self.world[msg.detector_ns].child_frames))

    def publish_robots_list(self):
        msg = RobotList()
        for robot in self.robots:
            msg.robot_list.append(robot)
        self.robot_names_publisher.publish(msg)

    def call_transform_manager(self, action, msg, detector_ns):
        msg2 = translate_message(action, msg, detector_ns)
        self.transform_manager_proxy(msg2)

    def remove_node_from_tree(self, frame_id, tree):
        rospy.logdebug('[' + self.namespace + '][detection_manager.remove_node]: ' + frame_id + ' from ' + tree + ' world[tree]')
        if tree not in self.remove_node_proxies:
            rospy.wait_for_service('/' + tree + '/remove_node_service')
            self.remove_node_proxies[tree] = rospy.ServiceProxy('/' + tree + '/remove_node_service', RobotName)
        self.remove_node_proxies[tree](frame_id)

    def cb_remove_node(self, msg):
        rospy.logdebug('[' + self.namespace + '][detection_manager.cb_remove_node]: ' + msg.robot_ns + ' from self.tree')
        self.tree.delete_node(msg.robot_ns)
        self.update_world()
        return RobotNameResponse()

    def update_world(self):
        rospy.logdebug('[' + self.namespace + '][detection_manager.update_world]: Publishing self.tree')
        frame_list = self.builder.build_frame_list_msg(self.tree)
        # rospy.logdebug('[' + self.namespace + ' detection_manager]: frame list: ' + str(frame_list.frames))
        self.tree_publisher.publish(frame_list)
        self.world[self.namespace] = self.tree


if __name__ == "__main__":

    try:
        rospy.init_node('detection_manager', log_level=rospy.DEBUG)
        rospy.logdebug('[detection_manager]: Node started')
        manager = DetectionManager()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('[detection_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
