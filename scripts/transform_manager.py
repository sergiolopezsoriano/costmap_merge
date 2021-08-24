#!/usr/bin/env python

import traceback
import rospy
import roslaunch
from costmap_merge.srv import RobotName, RobotNameResponse
from costmap_merge.srv import Transform, TransformResponse
from queue import Queue


class TransformManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service to communicate with the detection manager
        rospy.Service('transform_manager_service', Transform, self.cb_select_action)
        # Service to communicate with other transform managers
        rospy.Service('stop_node_service', RobotName, self.cb_stop_transform)
        self.manager_proxies = dict()
        # Dictionary of alive transformer nodes
        self.transformer_nodes = dict()
        # Proxies to send transforms to the transformer nodes
        self.node_proxies = dict()
        # ROS node launcher
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        # ROS node handles
        self.nodes = dict()
        # Operation mode
        self.mode = 'IDLE'
        # Transformer data
        self.transform_msg_queue = Queue()

    def cb_select_action(self, msg):
        self.mode = msg.action
        self.transform_msg_queue.put(msg)
        return TransformResponse()

    def start_transformer_node(self, msg):
        print(self.nodes[msg.robot_ns].namespace + 'transform_node_service')
        if msg.robot_ns not in self.node_proxies:
            rospy.wait_for_service(self.nodes[msg.robot_ns].namespace + 'transformer_node_service')

            self.node_proxies[msg.robot_ns] = rospy.ServiceProxy(self.nodes[msg.robot_ns].namespace +
                                                                 'transformer_node_service', Transform)
        self.node_proxies[msg.robot_ns](msg)

    def update_transform(self, msg):
        self.node_proxies[msg.robot_ns](msg)

    def stop_transformer_node(self, detector_ns, robot_ns):
        if detector_ns not in self.manager_proxies:
            rospy.wait_for_service('/' + detector_ns + '/stop_node_service')
            self.manager_proxies[detector_ns] = rospy.ServiceProxy('/' + detector_ns + '/stop_node_service', RobotName)
        self.manager_proxies[detector_ns](robot_ns)
        print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC')

    def cb_stop_transform(self, msg):
        self.transformer_nodes[msg.robot_ns].stop()
        return RobotNameResponse()


if __name__ == "__main__":

    try:
        rospy.init_node('transform_manager', log_level=rospy.INFO)
        rospy.loginfo('[transform_manager]: Node started')
        manager = TransformManager()
        while not rospy.is_shutdown():
            rospy.loginfo('[transform_manager ' + manager.namespace + ']: Mode = ' + manager.mode)
            if manager.mode == 'IDLE':
                rospy.sleep(1)
            else:
                transform_msg = manager.transform_msg_queue.get()
                if transform_msg.action == 'START':
                    manager.nodes[transform_msg.robot_ns] = roslaunch.core.Node('costmap_merge', 'transformer_node.py', namespace='transformer_' + manager.namespace + '_' + transform_msg.robot_ns, output='screen')
                    if transform_msg.robot_ns not in manager.transformer_nodes:
                        manager.transformer_nodes[transform_msg.robot_ns] = manager.launch.launch(
                            manager.nodes[transform_msg.robot_ns])
                    manager.start_transformer_node(transform_msg)
                    manager.mode = 'IDLE'
                elif transform_msg.action == 'UPDATE':
                    manager.update_transform(transform_msg)
                    manager.mode = 'IDLE'
                elif transform_msg.action == 'IDLE':
                    manager.stop_transformer_node(transform_msg.detector_ns, transform_msg.robot_ns)
                    manager.mode = 'IDLE'
                else:
                    print('[Transform_manager]: WRONG ACTION')

    except Exception as e:
        rospy.logfatal('[transform_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
