#!/usr/bin/env python

import rospy
from costmap_merge.msg import AccessList, QueueAccess, QueueElement
from Queue import Queue


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
        self.queue_action_publisher = rospy.Publisher('/access_queue', QueueAccess)
        rospy.Subscriber('/access_queue', QueueAccess, self.cb_access_queue, queue_size=10)

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
            print('[' + self.namespace + '] 1: ' + str(self.access_queue.queue))
            _list = queue_2_list(self.access_queue)
            _list.sort(key=lambda x: x[1])
            for index, element in enumerate(_list):
                if element[0] == msg.item.detector_ns:
                    _list.pop(index)
                    self.access_queue = list_2_queue(_list)
                    print('[' + self.namespace + '] 2: ' + str(self.access_queue.queue))
                    break
            print('[' + self.namespace + '] 3: ' + str(self.access_queue.queue))
