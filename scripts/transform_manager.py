#!/usr/bin/env python

import traceback
import rospy
import roslaunch
from costmap_merge.srv import RobotName, RobotNameResponse
from costmap_merge.srv import Transform, TransformResponse
from Queue import Queue
import signal


class TransformManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Service to communicate with the detection manager
        rospy.Service('transform_manager_service', Transform, self.cb_select_action)
        # Service to communicate with other transform managers
        rospy.Service('stop_node_service', RobotName, self.cb_stop_transform)
        self.stop_node_proxies = dict()
        # Dictionary of alive transformer nodes
        self.transformer_nodes = dict()
        # Proxies to send transforms to the transformer nodes
        self.node_proxies = dict()
        # ROS node launcher
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        # ROS node handles
        self.nodes = dict()
        # Transformer data
        self.transform_msg_queue = Queue()
        # manage keyboard interrupt due to open processes after exiting
        signal.signal(signal.SIGINT, self.signal_handler)

    def cb_select_action(self, msg):
        print('[' + self.namespace + '][transform_manager.cb_select_action]:  ' + msg.action + ' broadcasting transform from ' + msg.detector_ns + ' to ' + msg.robot_ns)
        self.transform_msg_queue.put(msg)
        return TransformResponse()

    def start_broadcasting_transforms(self, msg):
        print('[' + self.namespace + '][transform_manager.start_broadcasting_transforms]: Starting ' + self.namespace + ' ' + self.nodes[msg.robot_ns].namespace + 'transform_node_service')
        if msg.robot_ns not in self.node_proxies:
            rospy.wait_for_service(self.nodes[msg.robot_ns].namespace + 'transformer_node_service')

            self.node_proxies[msg.robot_ns] = rospy.ServiceProxy(self.nodes[msg.robot_ns].namespace +
                                                                 'transformer_node_service', Transform)
        self.node_proxies[msg.robot_ns](msg)

    def update_transform(self, msg):
        print('[' + self.namespace + '][transform_manager.update_transform]: Updating ' + self.namespace + ' ' + self.nodes[msg.robot_ns].namespace + 'transform_node_service')
        self.node_proxies[msg.robot_ns](msg)

    def stop_transformer_node(self, detector_ns, robot_ns):
        print('[' + self.namespace + '][transform_manager.stop_transformer_node]: ' + detector_ns + ' to ' + robot_ns)
        if detector_ns not in self.stop_node_proxies:
            rospy.wait_for_service('/' + detector_ns + '/stop_node_service')
            self.stop_node_proxies[detector_ns] = rospy.ServiceProxy('/' + detector_ns + '/stop_node_service', RobotName)
        self.stop_node_proxies[detector_ns](robot_ns)
        print('[' + self.namespace + '][transform_manager.stop_transformer_node]: Node ' + robot_ns + ' stopped.')

    def cb_stop_transform(self, msg):
        print('[' + self.namespace + '][transform_manager.cb_stop_transform]: received ' + msg.robot_ns)
        self.transformer_nodes[msg.robot_ns].stop()
        self.node_proxies.pop(msg.robot_ns)
        self.transformer_nodes.pop(msg.robot_ns)
        return RobotNameResponse()

    @staticmethod
    def signal_handler():
        rospy.signal_shutdown('')


if __name__ == "__main__":

    try:
        rospy.init_node('transform_manager', log_level=rospy.DEBUG)
        manager = TransformManager()
        while not rospy.is_shutdown():
            try:
                transform_msg = manager.transform_msg_queue.get()
                print('[' + manager.namespace + '][transform_manager.main]: Performing action = ' + transform_msg.action)
                if transform_msg.action == 'START':
                    manager.nodes[transform_msg.robot_ns] = roslaunch.core.Node('costmap_merge', 'transformer_node.py', namespace='transformer_' + manager.namespace + '_' + transform_msg.robot_ns, output='screen')
                    if transform_msg.robot_ns not in manager.transformer_nodes:
                        manager.transformer_nodes[transform_msg.robot_ns] = manager.launch.launch(
                            manager.nodes[transform_msg.robot_ns])
                    manager.start_broadcasting_transforms(transform_msg)
                elif transform_msg.action == 'UPDATE':
                    manager.update_transform(transform_msg)
                elif transform_msg.action == 'STOP':
                    manager.stop_transformer_node(transform_msg.detector_ns, transform_msg.robot_ns)
                else:
                    print('[' + manager.namespace + '][Transform_manager.main]: WRONG ACTION')
            except Exception as e:
                rospy.logfatal('[transform_manager]: first try exception %s', str(e.message) + str(e.args))
                e = traceback.format_exc()
                rospy.logfatal(e)
                rospy.sleep(1)

    except Exception as e:
        rospy.logfatal('[transform_manager]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
