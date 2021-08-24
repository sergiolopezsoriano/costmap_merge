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
        self.is_my_turn = bool()

    def modify_queue(self, mode):
        self.is_my_turn = False
        msg = TreeQueue()
        msg.detector_ns = self.namespace
        msg.mode = mode
        self.detection_publisher.publish(msg)

    def cb_detection(self, msg):
        if msg.mode == TreeQueue.PUSH:
            self.detection_queue.put(msg.detector_ns)
        elif msg.mode == TreeQueue.POP:
            if msg.detector_ns == self.detection_queue.queue[0]:
                self.detection_queue.get()
            else:
                rospy.logwarn('[Queue manager]: IRREGULAR ACCESS!')
                rospy.logwarn('[Queue manager]: QUEUE SEQUENCE HAS BEEN CORRUPTED!!!')
        try:
            print('self.queue = ' + str(self.detection_queue.queue))
            print('self.queue = ' + str(self.detection_queue.queue[0]))
            if self.detection_queue.queue[0] == self.namespace:
                self.is_my_turn = True
        except IndexError:
            rospy.logwarn('[Queue manager ' + self.namespace + ']: Exception IndexError')
