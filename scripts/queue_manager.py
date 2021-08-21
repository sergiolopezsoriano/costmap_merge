#!/usr/bin/env python

import rospy
from costmap_merge.msg import TreeQueue
from queue import Queue


class QueueManager:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Detection synchronization
        self.detection_queue = Queue()
        self.detection_publisher = rospy.Publisher('/queue_access', TreeQueue, queue_size=10)
        rospy.Subscriber('/queue_access', TreeQueue, self.cb_detection, queue_size=10)
        self.received = bool()

    def modify_queue(self, mode):
        self.received = False
        msg = TreeQueue()
        msg.detector_ns = self.namespace
        msg.mode = mode
        msg.stamp = rospy.Time.now()
        self.detection_publisher.publish(msg)

    def cb_detection(self, msg):
        if msg.mode == TreeQueue.PUSH:
            self.detection_queue.put({msg.detector_ns: msg.stamp})
        elif msg.mode == TreeQueue.POP:
            if msg.detector_ns == self.detection_queue.queue[0].keys()[0]:
                self.detection_queue.get()
            else:
                print('QUEUE MANAGER PROBLEM')
                print('QUEUE MANAGER PROBLEM')
                print('QUEUE MANAGER PROBLEM')
                print('QUEUE MANAGER PROBLEM')
                print('QUEUE MANAGER PROBLEM')
        if msg.detector_ns == self.namespace:
            self.received = True
