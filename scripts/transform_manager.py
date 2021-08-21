#!/usr/bin/env python

import traceback
import rospy
import roslaunch
from costmap_merge.srv import RobotName, RobotNameResponse
from costmap_merge.srv import TransformerData, TransformerDataResponse
from queue import Queue


class TransformManager:
    def __init__(self):
        # Service to communicate with the detection manager
        rospy.Service('/transform_manager_service', TransformerData, self.cb_select_action)
        # Service to communicate with other transform managers
        rospy.Service('/stop_node_service', RobotName, self.cb_stop_transform)
        self.manager_proxies = dict()
        # Dictionary of alive transformer nodes
        self.transformer_nodes = dict()
        # Proxies to send transforms to the transformer nodes
        self.node_proxies = dict()
        # ROS node launcher
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        # ROS node handlers
        self.nodes = dict()
        # Operation mode
        self.mode = 'IDLE'
        # Transformer data
        self.transformer_data_queue = Queue()

    def cb_select_action(self, msg):
        if msg.action == 'START':
            self.mode = 'START'
        elif msg.action == 'STOP':
            self.mode = 'STOP'
        elif msg.action == 'UPDATE':
            self.mode = 'UPDATE'
        self.transformer_data_queue.put(msg)

    def start_transformer_node(self, msg):
        self.nodes[msg.robot_ns] = roslaunch.core.Node('costmap_merge', 'transformer_node.py')
        if msg.robot_ns not in self.transformer_nodes:
            self.transformer_nodes[msg.robot_ns] = self.launch.launch(self.nodes[msg.robot_ns])
        if msg.robot_ns not in self.node_proxies:
            rospy.wait_for_service(self.nodes[msg.robot_ns].namespace + '/transform_node_service')
            self.node_proxies[msg.robot_ns] = rospy.ServiceProxy(self.nodes[msg.robot_ns].namespace +
                                                                 '/transformer_node_service', TransformerData)
        msg = self.translate_message(msg, 'START')
        self.node_proxies[msg.robot_ns](msg)

    @staticmethod
    def translate_message(msg, action):
        msg2 = TransformerData()
        msg2.action = action
        msg2.robot_ns = msg.robot_ns
        msg2.pose_R_D = msg.pose_R_D
        msg2.pose_R_R = msg.pose_R_R
        msg2.alpha = msg.alpha
        msg2.beta = msg.beta
        return msg2

    def update_transform(self, msg):
        msg = self.translate_message(msg, 'UPDATE')
        self.node_proxies[msg.robot_ns](msg)

    def stop_transformer_node(self, detector_ns, robot_ns):
        if detector_ns not in self.manager_proxies:
            rospy.wait_for_service(detector_ns + '/stop_node_service')
            self.manager_proxies[detector_ns] = rospy.ServiceProxy(detector_ns + '/stop_node_service', RobotName)
        self.manager_proxies[detector_ns](robot_ns)

    def cb_stop_transform(self, msg):
        self.transformer_nodes[msg.robot_ns].stop()
        return RobotNameResponse()


if __name__ == "__main__":

    try:
        rospy.init_node('transform_manager', log_level=rospy.INFO)
        rospy.loginfo('[transform_manager]: Node started')
        manager = TransformManager()
        while not rospy.is_shutdown():
            transformer_data = manager.transformer_data_queue.get()
            if manager.mode == 'START' and transformer_data.action == 'START':
                manager.start_transformer_node(transformer_data)
                manager.mode = 'IDLE'
            elif manager.mode == 'UPDATE' and transformer_data.action == 'UPDATE':
                manager.update_transform(transformer_data)
                manager.mode = 'IDLE'
            elif manager.mode == 'STOP' and transformer_data.action == 'IDLE':
                manager.stop_transformer_node()  
                manager.mode = 'IDLE'
            else:
                rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[transform_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)