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
                rospy.logwarn('[' + self.namespace + '][queue manager.cb_detection]: IRREGULAR ACCESS!')
                rospy.logwarn('[' + self.namespace + '][queue manager.cb_detection]: QUEUE SEQUENCE HAS BEEN CORRUPTED!!!')
                # FIXME: Fix the concurrent access to the queue. Probably starting from the Handshake2 callback.
                aux_list = list()
                while self.detection_queue.queue:
                    aux_list.append(self.detection_queue.get())
                while aux_list:
                    self.detection_queue.put(aux_list.pop())

        if self.detection_queue.queue:
            rospy.logdebug('[' + self.namespace + '][queue manager.cb_detection]: queue = ' + str(self.detection_queue.queue) + '. Next = ' + str(self.detection_queue.queue[0]))
            if self.detection_queue.queue[0] == self.namespace:
                self.is_my_turn = True
