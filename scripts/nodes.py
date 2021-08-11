#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from helpers import PoseHelper
from geometry_msgs.msg import PoseStamped


class OdomNode(object):
    def __init__(self, namespace):
        self.namespace = namespace
        # Starting relative pose to the map TODO: apply changes for real operation
        self.start = PoseStamped()
        # Flags to prevent publishing before the odometry is received
        self.odom_ready = False
        # Odometry pose
        self.odom = Odometry()
        # Odom pose in the map frame
        self.transformed_odom = PoseStamped()
        # Odometry subscriber
        rospy.Subscriber('/' + namespace + '/odom', Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        self.odom = msg
        if not self.odom_ready:
            self.odom_ready = True

    def set_start_from_transform(self, t):
        self.start.header.stamp = t.header.stamp
        self.start.header.frame_id = t.child_frame_id
        self.start.pose.position = t.transform.translation
        self.start.pose.orientation = t.transform.rotation

    def set_start_pose(self, frame_id, time_stamp, coordinates):
        self.start = PoseHelper.set_2D_pose(frame_id, time_stamp, coordinates)

    def set_transformed_odom(self, frame_id, time_stamp, coordinates):
        self.transformed_odom = PoseHelper.set_2D_pose(frame_id, time_stamp, coordinates)


class CostmapNode(OdomNode):
    def __init__(self, namespace):
        super(CostmapNode, self).__init__(namespace)
        # temporal coordinates
        self.x = float()
        self.y = float()
        # Costmap
        self.local_costmap = OccupancyGrid()
        # Rotated local costmap dimensions
        self.roloco_width = int()
        self.roloco_height = int()
        # Flag to prevent publishing before both costmaps are received
        self.local_ready = False
        # Costmap subscriber
        rospy.Subscriber('/' + self.namespace + '/local_costmap', OccupancyGrid, self.cb_local_costmap, queue_size=1)

    def cb_local_costmap(self, msg):
        self.local_costmap = msg
        self.local_ready = True
