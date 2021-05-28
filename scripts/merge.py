#!/usr/bin/env python

import traceback
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from costmap_merge.srv import RobotLocation, RobotLocationResponse
import thread


class Robot:
    def __init__(self, robot_ns, robot_type='', robot_pose=PoseStamped()):
        # Initialization
        self.robot_type = robot_type
        self.robot_ns = robot_ns
        self.pose = robot_pose  # in the frame of the Robot running the costmap_merger node
        # Variables
        self.odom = Odometry()  # in this Robot frame, of course!
        self.global_costmap = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        # Flags to wait until both costmaps are received
        self.global_ready = False
        self.local_ready = False
        # Odometry subscriber
        rospy.Subscriber('/' + self.robot_ns + '/odom', Odometry, self.cb_odom, queue_size=1)
        # Costmaps subscribers
        rospy.Subscriber('/' + self.robot_ns + '/global_costmap', OccupancyGrid, self.cb_global_costmap, queue_size=1)
        rospy.Subscriber('/' + self.robot_ns + '/local_costmap', OccupancyGrid, self.cb_local_costmap, queue_size=1)
        # Proxy for the Locator-Pawn handshake
        # rospy.wait_for_service('/' + self.robot_ns + '/location_handshake')
        self.location_handshake_proxy = rospy.ServiceProxy('/' + self.robot_ns + '/location_handshake', RobotLocation)

    def cb_global_costmap(self, msg):
        self.global_ready = True
        self.global_costmap = msg

    def cb_local_costmap(self, msg):
        self.local_ready = True
        self.local_costmap = msg

    def cb_odom(self, msg):
        self.odom = msg

    def set_pose(self, ps):
        self.pose = ps


class Merger:

    def __init__(self):
        # getting ros params
        self.namespace = rospy.get_namespace().strip('/')
        self.robot_type = rospy.get_param('~robot_type')
        # Pawns are informed by the Locator and send back the response
        self.s = rospy.Service('location_handshake', RobotLocation, self.cb_location_handshake)
        # Dictionary containing all the robots found so far
        self.robots = dict()
        # robot definition
        self.robots[self.namespace] = Robot(self.namespace, self.robot_type)
        # Costmap information
        self.occupancy_range = 100
        # Global costmap
        self.merged_global_costmap = OccupancyGrid()
        # Costmap publishers
        self.local_publisher = rospy.Publisher('/' + self.namespace + '/move_base/local_costmap/costmap',
                                               OccupancyGrid, queue_size=1)
        self.global_publisher = rospy.Publisher('/' + self.namespace + '/move_base/global_costmap/costmap',
                                                OccupancyGrid, queue_size=1)
        # Transformations
        self.tl = tf.TransformListener()
        self.quaternion = [] * 4
        # Flag for simulation
        self.simulation = True

    def cb_location_handshake(self, msg):
        rospy.logdebug('[costmap_merger-' + str(self.namespace) + ']: Locator calling')
        if msg.robot_ns in self.robots:
            self.update_robots(msg)
        else:
            self.add_robot(msg)
        if not self.simulation:
            try:
                thread.start_new_thread(self.build_global_costmap())
            except:
                rospy.logfatal("[costmap_merger]: Unable to start thread")
        return RobotLocationResponse(self.namespace, self.robot_type,
                                     self.robots[self.namespace].pose.pose.position.x,
                                     self.robots[self.namespace].pose.pose.position.y,
                                     self.robots[self.namespace].pose.header.stamp)

    def update_robots(self, msg):
        ps = PoseStamped()
        ps.header.frame_id = str(msg.robot_ns) + '/base_link'
        ps.header.stamp = msg.ts
        ps.pose.position.x = msg.x
        ps.pose.position.y = msg.y
        self.robots[msg.robot_ns].set_pose(ps)

    def add_robot(self, msg):
        ps = PoseStamped()
        ps.header.frame_id = str(msg.robot_ns) + '/base_link'
        ps.header.stamp = msg.ts
        ps.pose.position.x = msg.x
        ps.pose.position.y = msg.y
        self.robots[msg.robot_ns] = Robot(msg.robot_ns, msg.robot_type, ps)

    def talk_to_robot(self, robot_ns):
        rospy.logdebug('[costmap_merger-' + str(self.namespace) + ']: Talking to Pawn ' + str(robot_ns))
        try:
            # pose_tf = self.get_transform_between_robots_odoms(robot_ns)
            response = self.robots[robot_ns].location_handshake_proxy(self.namespace, self.robot_type,
                                                                      self.robots[self.namespace].pose.pose.position.x,
                                                                      self.robots[self.namespace].pose.pose.position.y,
                                                                      self.robots[self.namespace].pose.header.stamp)
            if self.robots[response.robot_ns]:
                self.update_robots(response)
            else:
                self.add_robot(response)
        except rospy.ServiceException:
            rospy.logfatal("[costmap_merger]: Service proxy failed")

    def get_transform_between_robots_odoms(self, robot_ns):
        pose_tf = PoseStamped()
        time = rospy.Time(0)
        try:
            self.tl.waitForTransform('/' + self.namespace + '_tf/odom', '/' + robot_ns + '_tf/odom', time,
                                     rospy.Duration(1.0))
            (position, orientation) = \
                self.tl.lookupTransform('/' + self.namespace + '_tf/odom', '/' + robot_ns + '_tf/odom', time)
            pose_tf.pose.position.x = position[0]
            pose_tf.pose.position.y = position[1]
            pose_tf.pose.position.z = position[2]
            pose_tf.pose.orientation.x = orientation[0]
            pose_tf.pose.orientation.y = orientation[1]
            pose_tf.pose.orientation.z = orientation[2]
            pose_tf.pose.orientation.w = orientation[3]
            pose_tf.header.stamp = time
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal('Error in get_transform_between_robots_odoms')
        return pose_tf

    def get_transform_from_map(self):
        pose_tf = PoseStamped()
        time = rospy.Time(0)
        try:
            self.tl.waitForTransform('/map', '/' + self.namespace + '_tf/odom', time, rospy.Duration(1.0))
            (position, orientation) = \
                self.tl.lookupTransform('/map', '/' + self.namespace + '_tf/odom', time)
            pose_tf.pose.position.x = position[0]
            pose_tf.pose.position.y = position[1]
            pose_tf.pose.position.z = position[2]
            pose_tf.pose.orientation.x = orientation[0]
            pose_tf.pose.orientation.y = orientation[1]
            pose_tf.pose.orientation.z = orientation[2]
            pose_tf.pose.orientation.w = orientation[3]
            pose_tf.header.stamp = time
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logfatal('Error in get_transform_from_map')
        return pose_tf

    def build_global_costmap(self):
        # TODO: The transform must be received from the AI and the Find_robot_in_costmap scripts
        self.robots[self.namespace].pose = self.get_transform_from_map()
        # print(self.robots[self.namespace].pose)
        global_costmap = OccupancyGrid()
        global_costmap.info.resolution = self.robots[self.namespace].global_costmap.info.resolution
        x = []
        y = []
        for robot in self.robots:
            x.append(self.robots[robot].pose.pose.position.x + self.robots[robot].odom.pose.pose.position.x)
            y.append(self.robots[robot].pose.pose.position.y + self.robots[robot].odom.pose.pose.position.y)
        x_min = min(x)
        x_max = max(x)
        y_min = min(y)
        y_max = max(y)
        rospy.logdebug('[costmap_merger-' + str(self.namespace) + '] x_min: ' + str(x_min))
        rospy.logdebug('[costmap_merger-' + str(self.namespace) + '] y_min: ' + str(y_min))
        global_costmap.info.origin.position.x = sum(x) / len(self.robots)
        global_costmap.info.origin.position.y = sum(y) / len(self.robots)
        global_costmap.info.width = int(
            (x_max - x_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.width
        global_costmap.info.height = int(
            (y_max - y_min) / global_costmap.info.resolution) + self.robots[self.namespace].global_costmap.info.height

        # Filling the global global costmap with random numbers
        # global_costmap.data = np.random.random_integers(self.occupancy_range, size=(
        #             global_costmap.info.width * global_costmap.info.height)).tolist()

        # zero padding
        global_costmap.data = np.zeros(int(global_costmap.info.width * global_costmap.info.height)).tolist()

        # Adding all robots global_costmaps to the merged_global_costmap
        for robot in self.robots:
            pos_x = int((self.robots[robot].pose.pose.position.x + self.robots[
                robot].odom.pose.pose.position.x - x_min) / global_costmap.info.resolution)
            pos_y = int((self.robots[robot].pose.pose.position.y + self.robots[
                robot].odom.pose.pose.position.y - y_min) / global_costmap.info.resolution)
            for row in range(self.robots[robot].global_costmap.info.height):
                row_start = row * self.robots[robot].global_costmap.info.width
                global_costmap.data[(row + pos_y) * global_costmap.info.width + pos_x:
                                    (row + pos_y) * global_costmap.info.width + pos_x +
                                    self.robots[robot].global_costmap.info.width] = \
                    self.robots[robot].global_costmap.data[
                    row_start:row_start + self.robots[robot].global_costmap.info.width]

        if self.simulation:
            # global_costmap.info.origin.orientation = self.robots[self.namespace].pose.pose.orientation
            global_costmap.info.origin.position.x = x_min - self.robots[
                self.namespace].global_costmap.info.width / 2 * global_costmap.info.resolution
            global_costmap.info.origin.position.y = y_min - self.robots[
                self.namespace].global_costmap.info.height / 2 * global_costmap.info.resolution
            # self.quaternion = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(-90))
            # global_costmap.info.origin.orientation.x = self.quaternion[0]
            # global_costmap.info.origin.orientation.y = self.quaternion[1]
            # global_costmap.info.origin.orientation.z = self.quaternion[2]
            # global_costmap.info.origin.orientation.w = self.quaternion[3]
        else:
            global_costmap.info.origin.position.x = x_min + (
                        x_max - x_min) / 2 - global_costmap.info.width / 2 * global_costmap.info.resolution
            global_costmap.info.origin.position.y = y_min + (
                        y_max - y_min) / 2 - global_costmap.info.height / 2 * global_costmap.info.resolution
        self.merged_global_costmap = global_costmap

    def publish_robot_costmaps(self):
        self.global_publisher.publish(self.merged_global_costmap)
        self.local_publisher.publish(self.robots[self.namespace].local_costmap)


if __name__ == "__main__":

    try:
        robot_names = ['robot1', 'robot2', 'robot3']  # TODO: The robot_ns must be given by the AI node!!!
        rospy.init_node('costmap_merger', log_level=rospy.INFO)
        rospy.loginfo('[costmap_merger]: Node started')
        cm = Merger()
        rospy.sleep(5)  # Waiting for the ROS subscribers, publishers and servers to initialize
        if cm.robot_type == 'locator':
            for name in robot_names:
                if name != cm.namespace:
                    cm.robots[name] = Robot(name)
        while not rospy.is_shutdown():
            if cm.robots[cm.namespace].local_ready and cm.robots[cm.namespace].global_ready:
                if cm.robot_type == 'locator':
                    for name in robot_names:
                        if name != cm.namespace:
                            cm.talk_to_robot(name)
                rospy.sleep(1)
                cm.build_global_costmap()
                cm.publish_robot_costmaps()

    except Exception as e:
        rospy.logfatal('[costmap_merger]: Exception %s', str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
