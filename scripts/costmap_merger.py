#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
# from robot_location.srv import RobotLocation
import time


# class Robot:
#     def __init__(self, ID, robot_type, robot_ns):
#         self.ID = ID
#         self.robot_type = robot_type
#         self.robot_ns = robot_ns
#
#     def cb_robot_location(self):


class CostmapMerger:
    def __init__(self):
        # def __init__(self, ID, robot_type, robot_ns):
        #     # self definition
        #     self.robot = Robot(ID, robot_type, robot_ns)
        #     # Pawns are informed by the Locator through this service
        #     rospy.Service('robot_location', RobotLocation, self.cb_robot_location)
        #     # Locator receives Pawns response through this proxy
        #     send_robot_location = rospy.ServiceProxy('robot_location', RobotLocation)

        self.robot1 = 'robot1'
        self.robot2 = 'robot2'

        self.occupancy_range = 100
        self.local1_ready = False
        self.global1_ready = False
        self.local2_ready = False
        self.global2_ready = False

        self.tl = tf.TransformListener()
        self.pose_tf1 = PoseStamped()

        # Costmap communication
        rospy.Subscriber('/' + self.robot1 + '/global_costmap', OccupancyGrid, self.cb_global_costmap1, queue_size=1)
        rospy.Subscriber('/' + self.robot2 + '/global_costmap', OccupancyGrid, self.cb_global_costmap2, queue_size=1)
        rospy.Subscriber('/' + self.robot1 + '/local_costmap', OccupancyGrid, self.cb_local_costmap1, queue_size=1)
        rospy.Subscriber('/' + self.robot2 + '/local_costmap', OccupancyGrid, self.cb_local_costmap2, queue_size=1)
        self.local1_publisher = rospy.Publisher('/' + self.robot1 + '/move_base/local_costmap/costmap', OccupancyGrid, queue_size=1)
        self.global1_publisher = rospy.Publisher('/' + self.robot1 + '/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
        self.local2_publisher = rospy.Publisher('/' + self.robot2 + '/move_base/local_costmap/costmap', OccupancyGrid, queue_size=1)
        self.global2_publisher = rospy.Publisher('/' + self.robot2 + '/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
        self.local1 = OccupancyGrid()
        self.local2 = OccupancyGrid()
        self.global1 = OccupancyGrid()
        self.global2 = OccupancyGrid()

        # Odometry
        rospy.Subscriber('/' + self.robot1 + '/odom', Odometry, self.cb_odom1, queue_size=1)
        rospy.Subscriber('/' + self.robot2 + '/odom', Odometry, self.cb_odom2, queue_size=1)
        self.odom1 = Odometry()
        self.odom2 = Odometry()

    def cb_local_costmap1(self, msg):
        self.local1_ready = True
        self.local1 = msg

    def cb_local_costmap2(self, msg):
        self.local2_ready = True
        self.local2 = msg

    def cb_global_costmap1(self, msg):
        self.global1_ready = True
        self.global1 = msg

    def cb_global_costmap2(self, msg):
        self.global2_ready = True
        self.global2 = msg

    def cb_odom1(self, msg):
        self.odom1 = msg

    def cb_odom2(self, msg):
        self.odom2 = msg

    def build_global_costmap(self):
        #TODO: add the case where robot1 and robot2 costmaps have different size
        global_costmap = OccupancyGrid()
        global_costmap.info.resolution = self.global1.info.resolution
        global_costmap.info.width = self.global1.info.width / 2 + self.global2.info.width / 2 + \
                                    int(abs(self.pose_tf1.pose.position[0] / self.global1.info.resolution)) + \
                                    int(abs(self.odom1.pose.pose.position.x / self.global1.info.resolution)) + \
                                    int(abs(self.odom2.pose.pose.position.x / self.global2.info.resolution))
        global_costmap.info.height = self.global1.info.height / 2 + self.global1.info.height / 2 + \
                                     int(abs(self.pose_tf1.pose.position[1] / self.global1.info.resolution)) + \
                                     int(abs(self.odom1.pose.pose.position.y / self.global1.info.resolution)) + \
                                     int(abs(self.odom2.pose.pose.position.y / self.global2.info.resolution))
        # angles = tf.transformations.euler_from_quaternion([self.pose_tf1.pose.orientation[0], self.pose_tf1.pose.orientation.x,
        #                                                   self.pose_tf1.pose.orientation.y, self.pose_tf1.pose.orientation.z])

        # Filling the global global costmap with random numbers
        # global_costmap.data = np.random.random_integers(self.occupancy_range, size=(global_costmap.info.width * global_costmap.info.height)).tolist()
        # zero padding
        global_costmap.data = np.zeros(int(global_costmap.info.width * global_costmap.info.height)).tolist()

        for row in range(self.global1.info.height):  # putting global costmap of robot 1 in the global global costmap
            length = row * self.global1.info.width
            global_costmap.data[(row + 1) * global_costmap.info.width - self.global1.info.width: (row + 1) * global_costmap.info.width] = \
                self.global1.data[::-1][length:length + self.global1.info.width]

        for row in range(self.global2.info.height):  # putting global costmap of robot 2 in the global global costmap
            length = row * self.global2.info.width
            iteration_start = global_costmap.info.height * global_costmap.info.width - global_costmap.info.width * self.global2.info.height + row * global_costmap.info.width
            global_costmap.data[iteration_start: iteration_start + self.global2.info.width] = self.global2.data[length: length + self.global2.info.width]

        global_costmap.info.origin.position.x = - self.global1.info.width * self.global1.info.resolution / 2 + self.odom2.pose.pose.position.x
        global_costmap.info.origin.position.y = self.pose_tf1.pose.position[1] - self.global2.info.height * self.global2.info.resolution / 2 + self.odom2.pose.pose.position.y
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, 90)
        # global_costmap.info.origin.orientation.x = quaternion[0]
        # global_costmap.info.origin.orientation.y = quaternion[1]
        # global_costmap.info.origin.orientation.z = quaternion[2]
        # global_costmap.info.origin.orientation.w = quaternion[3]

        self.global1_publisher.publish(self.global1)
        self.global2_publisher.publish(self.global2)
        self.local1_publisher.publish(self.local1)
        self.local2_publisher.publish(self.local2)

    def get_transform_from_tf_tree(self):
        try:
            self.tl.waitForTransform('/' + self.robot1 + '_tf/odom', '/' + self.robot2 + '_tf/odom', rospy.Time(0), rospy.Duration(1.0))
            (self.pose_tf1.pose.position, self.pose_tf1.pose.orientation) = self.tl.lookupTransform('/' + self.robot1 + '_tf/odom', '/' + self.robot2 + '_tf/odom', rospy.Time(0))
            rospy.logdebug('[Costmap merger]: ' + str(self.pose_tf1.pose.position) + '-' + str(self.pose_tf1.pose.orientation))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal('Error in the transform listener')


if __name__ == "__main__":

    try:
        rospy.init_node('costmap_merger', log_level=rospy.INFO)
        rospy.loginfo('[costmap_merger]: Node started')
        cm = CostmapMerger()
        while not rospy.is_shutdown():
            if cm.local1_ready and cm.global1_ready and cm.local2_ready and cm.global2_ready:
                cm.get_transform_from_tf_tree()
                cm.build_global_costmap()

    except Exception as e:
        rospy.logfatal('[costmap_merger]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
