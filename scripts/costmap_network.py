#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from costmap_merge.msg import RobotName
from robots import CostmapRobot
from helpers import TransformHelper, PoseHelper
import math
import tf2_ros


class CostmapNetwork:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        # Transform managers
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Timeout to discard the robot transform
        self.location_timeout = rospy.get_param('~transform_timeout')
        # Costmap information
        self.occupancy_range = 100
        # Subscriber to the detected robots topic
        rospy.Subscriber('/detected_robots_topic', RobotName, self.cb_get_robot_transform, queue_size=10)
        # Costmap
        self.merged_global_costmap = OccupancyGrid()
        # Costmap Publishers
        self.local_publisher = rospy.Publisher('/' + self.namespace + '/move_base/local_costmap/costmap',
                                               OccupancyGrid, queue_size=1)
        self.global_publisher = rospy.Publisher('/' + self.namespace + '/move_base/global_costmap/costmap',
                                                OccupancyGrid, queue_size=1)
        # Dictionary for the costmap_network nodes
        self.robots = dict()
        # Creating the costmap network with this costmap node
        self.robots[self.namespace] = CostmapRobot(self.namespace)
        self.robots[self.namespace].set_start_at_origin('/' + str(self.namespace) + '/odom', rospy.Time.now())

    def cb_get_robot_transform(self, msg):
        if msg.robot_ns not in self.robots:
            self.robots[msg.robot_ns] = CostmapRobot(msg.robot_ns)
        rospy.sleep(1)  # waiting for the detection manager to broadcast the transform
        try:
            t = self.tfBuffer.lookup_transform(self.namespace + '/odom', msg.robot_ns + '/odom',  rospy.Time())
            self.robots[msg.robot_ns].set_start_from_transform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exception:
            rospy.logfatal('[costmap_network]: Exception %s', str(exception.message) + str(exception.args))

    def publish_costmap(self):
        if self.robots[self.namespace].local_ready:
            self.global_publisher.publish(self.merged_global_costmap)
            self.local_publisher.publish(self.robots[self.namespace].local_costmap)

    def build_global_costmap(self):
        try:
            # Building the merged_global_costmap area
            global_costmap = OccupancyGrid()
            while not self.robots[self.namespace].local_ready:
                rospy.sleep(1)
            global_costmap.info.resolution = self.robots[self.namespace].local_costmap.info.resolution
            x_min_list = []
            x_max_list = []
            y_min_list = []
            y_max_list = []
            # Calculating the position of all robots
            for robot in self.robots:
                while not self.robots[robot].local_ready:
                    rospy.sleep(1)
                self.robots[robot].yaw = PoseHelper.get_yaw_from_orientation(self.robots[robot].start.pose.orientation)
                self.robots[robot].x = self.robots[robot].start.pose.position.x + self.robots[
                    robot].odom.pose.pose.position.x * np.cos(self.robots[robot].yaw) - self.robots[
                    robot].odom.pose.pose.position.y * np.sin(self.robots[robot].yaw)
                self.robots[robot].y = self.robots[robot].start.pose.position.y + self.robots[
                    robot].odom.pose.pose.position.y * np.cos(self.robots[robot].yaw) + self.robots[
                    robot].odom.pose.pose.position.x * np.sin(self.robots[robot].yaw)
                self.calculate_costmap_size(robot)
                x_min_list.append(
                    self.robots[robot].x - self.robots[robot].roloco_width * global_costmap.info.resolution / 2)
                x_max_list.append(
                    self.robots[robot].x + self.robots[robot].roloco_width * global_costmap.info.resolution / 2)
                y_min_list.append(
                    self.robots[robot].y - self.robots[robot].roloco_height * global_costmap.info.resolution / 2)
                y_max_list.append(
                    self.robots[robot].y + self.robots[robot].roloco_height * global_costmap.info.resolution / 2)

            for robot in self.robots:
                if min(x_min_list) == self.robots[robot].x - self.robots[robot].roloco_width * global_costmap.info.resolution / 2:
                    robot_xmin = robot
                if max(x_max_list) == self.robots[robot].x + self.robots[robot].roloco_width * global_costmap.info.resolution / 2:
                    robot_xmax = robot
                if min(y_min_list) == self.robots[robot].y - self.robots[robot].roloco_height * global_costmap.info.resolution / 2:
                    robot_ymin = robot
                if max(y_max_list) == self.robots[robot].y + self.robots[robot].roloco_height * global_costmap.info.resolution / 2:
                    robot_ymax = robot
            global_costmap.info.width = int(
                (self.robots[robot_xmax].x - self.robots[robot_xmin].x) / global_costmap.info.resolution) + self.robots[
                                            robot_xmax].roloco_width / 2 + self.robots[robot_xmin].roloco_width / 2
            global_costmap.info.height = int(
                (self.robots[robot_ymax].y - self.robots[robot_ymin].y) / global_costmap.info.resolution) + self.robots[
                                             robot_ymax].roloco_height / 2 + self.robots[robot_ymin].roloco_height / 2
            # Filling the global global costmap with random numbers
            # global_costmap.data = np.random.random_integers(self.occupancy_range, size=(
            #             global_costmap.info.width * global_costmap.info.height)).tolist()

            # zero padding
            global_costmap.data = np.zeros(int(global_costmap.info.width * global_costmap.info.height)).tolist()

            # Adding all robots local_costmaps to the merged_global_costmap
            for robot in self.robots:
                if robot != self.namespace:
                    self.add_costmap(robot, global_costmap, robot_xmin, robot_ymin)
            # Self costmap over the other ones
            self.add_costmap(self.namespace, global_costmap, robot_xmin, robot_ymin)

            global_costmap.info.origin.position.x = self.robots[robot_xmin].x - self.robots[
                robot_xmin].roloco_width / 2 * global_costmap.info.resolution
            global_costmap.info.origin.position.y = self.robots[robot_ymin].y - self.robots[
                robot_ymin].roloco_height / 2 * global_costmap.info.resolution
            global_costmap.info.origin.orientation = self.robots[self.namespace].start.pose.orientation
            global_costmap.header.frame_id = str(self.namespace) + '/odom'
            return global_costmap
        except Exception as exception:
            rospy.logfatal('Build global costmap error: ' + str(exception))
            return OccupancyGrid()

    def add_costmap(self, robot, global_costmap, robot_xmin, robot_ymin):
        # Taking into account the starting angle
        data = self.rotate_costmap(robot)
        pos_x = int((self.robots[robot].start.pose.position.x + self.robots[robot].odom.pose.pose.position.x * np.cos(
            self.robots[robot].yaw) - self.robots[robot].odom.pose.pose.position.y * np.sin(self.robots[robot].yaw) -
                     self.robots[robot_xmin].x) / global_costmap.info.resolution) + self.robots[
                    robot_xmin].roloco_width / 2 - self.robots[robot].roloco_width / 2
        pos_y = int((self.robots[robot].start.pose.position.y + self.robots[robot].odom.pose.pose.position.y * np.cos(
            self.robots[robot].yaw) + self.robots[robot].odom.pose.pose.position.x * np.sin(self.robots[robot].yaw) -
                     self.robots[robot_ymin].y) / global_costmap.info.resolution) + self.robots[
                    robot_ymin].roloco_height / 2 - self.robots[robot].roloco_height / 2

        for row in range(self.robots[robot].roloco_height):
            row_start = row * self.robots[robot].roloco_width
            global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x:
                                (row + pos_y) * global_costmap.info.width + pos_x +
                                self.robots[robot].roloco_width] = \
                data[row_start:row_start + self.robots[robot].roloco_width]

        ####################### overlapping costmaps ########################
        # for row in range(self.robots[robot].local_costmap.info.height):
        #     for col in range(self.robots[robot].local_costmap.info.width):
        #         if data[row * self.robots[robot].local_costmap.info.width + col] > \
        #                 global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x + col]:
        #             global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x + col] = \
        #                 data[row * self.robots[robot].local_costmap.info.width + col]

        return global_costmap

    def calculate_costmap_size(self, robot):
        resol = self.robots[robot].local_costmap.info.resolution
        width1 = self.robots[robot].local_costmap.info.width
        height1 = self.robots[robot].local_costmap.info.height
        d = math.sqrt((resol * width1) ** 2 + (resol * height1) ** 2)
        theta = math.atan2(height1, width1)
        height2 = abs(d * math.sin(abs(self.robots[robot].yaw) + theta))
        width2 = abs(d * math.cos(abs(self.robots[robot].yaw) - theta))
        # if height2 % resol == 0:
        #     height2 = int(height2 / resol)
        # else:
        #     height2 = int(height2 / resol) + 1
        # if width2 % resol == 0:
        #     width2 = int(width2 / resol)
        # else:
        #     width2 = int(width2 / resol) + 1
        height2 = int(height2 / resol)
        width2 = int(width2 / resol)
        self.robots[robot].roloco_width = max(width1, width2)
        self.robots[robot].roloco_height = max(height1, height2)

    def rotate_costmap(self, robot):
        # a = rospy.get_rostime()
        resol = self.robots[robot].local_costmap.info.resolution
        width1 = self.robots[robot].local_costmap.info.width
        height1 = self.robots[robot].local_costmap.info.height
        data = np.reshape(self.robots[robot].local_costmap.data, (width1, height1))
        # center of the initial costmap matrix in cartesian coordinates
        xo1 = self.robots[robot].local_costmap.info.origin.position.x
        yo1 = self.robots[robot].local_costmap.info.origin.position.y
        origin = [self.robots[robot].local_costmap.info.origin.position.x + width1 / 2 * resol,
                  self.robots[robot].local_costmap.info.origin.position.y + height1 / 2 * resol]
        height2 = self.robots[robot].roloco_height
        width2 = self.robots[robot].roloco_width
        x1 = list()
        y1 = list()
        x2 = list()
        y2 = list()
        # data2 = np.random.random_integers(self.occupancy_range, size=(width2 * height2))
        # data2 = np.reshape(data2, (width2, height2)).tolist()
        data2 = np.zeros([width2, height2])
        # positions corresponding with the cell grid
        x = xo1 - (float(width2) - float(width1)) / 2 * resol
        y = yo1 - (float(height2) - float(height1)) / 2 * resol
        for row in range(width2):
            x1.append(x + row * resol)
            x2.append(x + row * resol)
        for col in range(height2):
            y1.append(y + col * resol)
            y2.append(y + col * resol)
        index_x = 0
        index_y = 0
        # b = rospy.get_rostime()
        # rospy.loginfo('[' + str(self.namespace) + '-' + str(robot) + '] b: ' + str(b.secs - a.secs))
        # rospy.loginfo('[' + str(self.namespace) + '-' + str(robot) + '] yaw: ' + str(self.robots[robot].yaw))
        for row in range(height1):
            for col in range(width1):
                point1 = [xo1 + row * resol, yo1 + col * resol]
                point2 = PoseHelper.rotate(origin, point1, self.robots[robot].yaw)
                # index_x = list(filter(lambda i: i > point2[0], x1))[0]
                # index_y = list(filter(lambda i: i > point2[1], y1))[0]
                for value in x1:
                    if value > point2[0]:
                        index_x = x1.index(value)
                        break
                for value in y1:
                    if value > point2[1]:
                        index_y = y1.index(value)
                        break
                data2[index_x][index_y] = data[row][col]
        # c = rospy.get_rostime()
        # rospy.loginfo('[' + str(self.namespace) + '-' + str(robot) + '] c: ' + str(c.secs - a.secs))
        for row in range(height2):
            for col in range(width2):
                try:
                    if data2[row][col] == 0 and (data2[row - 1][col] != 0) and (data2[row + 1][col] != 0) and (data2[row][col - 1] != 0) and (data2[row][col + 1] != 0):
                        data2[row][col] = (data2[row - 1][col] + data2[row + 1][col] + data2[row][col - 1] + data2[row][
                            col + 1]) / 4
                except Exception as exception:
                    # rospy.logfatal('rotate costmap error: ' + str(exception))
                    pass
        # d = rospy.get_rostime()
        # rospy.loginfo('[' + str(self.namespace) + '-' + str(robot) + '] d: ' + str(d.secs - a.secs))
        data2 = np.reshape(data2, (width2 * height2)).tolist()
        return data2


if __name__ == "__main__":

    try:
        rospy.init_node('costmap_network', log_level=rospy.INFO)
        rospy.loginfo('[costmap_network]: Node started')
        costmap_network = CostmapNetwork()
        # preventing building the map before receiving any costmap
        while not costmap_network.robots[costmap_network.namespace].local_ready:
            rospy.sleep(1)
        while not rospy.is_shutdown():
            costmap_network.merged_global_costmap = costmap_network.build_global_costmap()
            costmap_network.publish_costmap()

    except Exception as e:
        rospy.logfatal('[costmap_network]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
