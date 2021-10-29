#!/usr/bin/env python

import rospy
from costmap_merge.msg import AccessList, QueueAccess, QueueElement
# from queue import Queue
from Queue import Queue
import copy


def queue_2_list(_queue):
    _list = list()
    while not _queue.empty():
        _list.append(_queue.get())
    return _list


def list_2_queue(_list):
    _queue = Queue()
    _list.reverse()
    while _list:
        _queue.put(_list.pop())
    return _queue


def queue_2_dict(_queue):
    _dict = dict()
    while not _queue.empty():
        item = _queue.get()
        _dict[item.keys()[0]] = item.values()[0]
    return _dict


def dict_2_list(_dict):
    _list = list()
    for key, value in _dict.iteritems():
        temp = [key, value]
        _list.append(temp)
    return _list


class QueueManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Detection synchronization
        self.access_queue = Queue()
        self.queue_action_publisher = rospy.Publisher('/access_queue', QueueAccess, queue_size=10)
        rospy.Subscriber('/access_queue', QueueAccess, self.cb_access_queue, queue_size=10)
        # Next turn decision
        self.access_list_publisher = rospy.Publisher('/access_list', AccessList, queue_size=10)
        rospy.Subscriber('/access_list', AccessList, self.cb_update_queue, queue_size=10)

    def modify_queue(self, mode):
        msg = QueueAccess()
        msg.mode = mode
        msg.item.detector_ns = self.namespace
        msg.item.stamp = rospy.Time.now()
        self.queue_action_publisher.publish(msg)

    def cb_access_queue(self, msg):
        if msg.mode == QueueAccess.PUSH:
            self.access_queue.put([msg.item.detector_ns, msg.item.stamp])
        elif msg.mode == QueueAccess.POP:
            if msg.item.detector_ns == self.access_queue.queue[0][0]:
                self.access_queue.get()
            else:
                rospy.logwarn('[' + self.namespace + '][queue manager.cb_detection]: IRREGULAR ACCESS!')
                rospy.logwarn('[' + self.namespace + '][queue manager.cb_detection]: QUEUE SEQUENCE HAS BEEN CORRUPTED!!!')
                # FIXME: Fix the concurrent access to the queue. Probably starting from the Handshake2 callback.

    def cb_update_queue(self, msg):
        _access_list = list()
        for item in msg.access_list:
            _access_list.append([item.detector_ns, item.stamp])
        _access_list.sort(key=lambda x: x[1])
        if self.access_queue.empty():
            self.access_queue = list_2_queue(_access_list)
        elif not _access_list:
            pass
        else:
            if self.access_queue.queue[0][1] <= _access_list[0][1]:
                pass
            else:
                self.access_queue = list_2_queue(_access_list)

    def publish_queue(self):
        msg = AccessList()
        _list = copy.deepcopy(self.access_queue.queue)
        for item in _list:
            element = QueueElement()
            element.detector_ns = item[0]
            element.stamp = item[1]
            msg.access_list.append(element)
        self.access_list_publisher.publish(msg)
