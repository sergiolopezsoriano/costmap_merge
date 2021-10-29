#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from helpers import PoseHelper
from geometry_msgs.msg import PoseStamped


class OdomRobot(object):
    def __init__(self, namespace):
        self.namespace = namespace
        # Starting relative pose to the map
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

    def set_start_from_coordinates(self, frame_id, time_stamp, coordinates):
        self.start = PoseHelper.set_2D_pose(frame_id, time_stamp, coordinates)

    def set_start_at_origin(self, frame_id, time_stamp):
        self.start = PoseHelper.set_2D_pose(frame_id, time_stamp, [0, 0, 0, 0, 0, 0])

    def set_transformed_odom(self, frame_id, time_stamp, coordinates):
        self.transformed_odom = PoseHelper.set_2D_pose(frame_id, time_stamp, coordinates)


class CostmapRobot(OdomRobot):
    def __init__(self, namespace):
        super(CostmapRobot, self).__init__(namespace)
        # coordinates made constant during a costmap building iteration
        self.x = float()
        self.y = float()
        self.yaw = float()
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
