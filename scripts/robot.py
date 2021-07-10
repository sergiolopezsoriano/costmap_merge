#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from cn_lib import get_yaw_from_orientation, get_map_to_odom_transform
from geometry_msgs.msg import PoseStamped
from costmap_merge.srv import RobotLocation, RobotLocationResponse, RobotDetected, RobotDetectedResponse
from costmap_merge.msg import RobotPub
from threading import Thread, Lock


class OdomNode(object):
    def __init__(self, namespace):
        self.namespace = namespace
        # Initial pose
        self.pose = PoseStamped()
        # Rotated coordinates
        self.x = int()
        self.y = int()
        # Flags to prevent publishing before the odometry is received
        self.odom_ready = False
        # Odometry subscriber
        self.odom = Odometry()
        rospy.Subscriber('/' + namespace + '/odom', Odometry, self.cb_odom, queue_size=1)

    def cb_odom(self, msg):
        self.odom = msg
        self.odom_ready = True

    def set_pose(self, ps):
        self.pose = ps


class CostmapNode(OdomNode):
    def __init__(self, namespace, robot_type):
        super(CostmapNode, self).__init__(namespace)
        self.type = robot_type
        # Costmaps
        self.local_costmap = OccupancyGrid()
        self.merged_global_costmap = OccupancyGrid()
        # Rotated local costmap dimensions
        self.roloco_width = int()
        self.roloco_height = int()
        # Flags to prevent publishing before both costmaps are received
        self.local_ready = False
        # Costmaps subscriber
        rospy.Subscriber('/' + self.namespace + '/local_costmap', OccupancyGrid, self.cb_local_costmap, queue_size=1)

    def cb_local_costmap(self, msg):
        self.local_ready = True
        self.local_costmap = msg


class Robot(CostmapNode, Thread):
    def __init__(self, namespace, robot_type):
        Thread.__init__(self)
        super(Robot, self).__init__(namespace, robot_type)
        # Setting the initial pose of the Robot
        self.set_pose(get_map_to_odom_transform(self.namespace))
        # Service for the Detector-Robot communication
        self.robot_communication_service = rospy.Service('robot_communication_service', RobotLocation,
                                                         self.cb_robot_communication)
        # Proxy to update costmap_network robots
        rospy.wait_for_service('/' + self.namespace + '/update_robots')
        self.update_robots_proxy = rospy.ServiceProxy('/' + self.namespace + '/update_robots', RobotLocation)
        # Lock to avoid concurrent calls with the update_robots_proxy
        self.lock_update_robots = Lock()
        # Costmap Publishers
        self.local_publisher = rospy.Publisher('/' + self.namespace + '/move_base/local_costmap/costmap',
                                               OccupancyGrid, queue_size=1)
        self.global_publisher = rospy.Publisher('/' + self.namespace + '/move_base/global_costmap/costmap',
                                                OccupancyGrid, queue_size=1)
        # Namespaces of the known detectors
        self.detectors = []
        self.last_detector = ''
        self.detected_robots_subscribers = dict()

    def run(self):
        while not rospy.is_shutdown():
            self.publish_costmap()

    def publish_costmap(self):
        if self.local_ready:
            self.global_publisher.publish(self.merged_global_costmap)
            self.local_publisher.publish(self.local_costmap)

    def cb_robot_communication(self, msg):
        # rospy.loginfo('[' + str(self.type) + '-' + str(self.namespace) + ']: Detector-' + str(msg.namespace) + ' calling')
        # Creating or updating robots in the costmap_network
        self.update_robots(msg)
        # Adding detector
        self.last_detector = msg.namespace
        self.add_detector()
        # Replying to the locator
        return RobotLocationResponse(self.namespace, self.type, self.pose.pose.position.x, self.pose.pose.position.y,
                                     get_yaw_from_orientation(self.pose.pose.orientation), self.pose.header.stamp)

    def add_detector(self):
        if self.last_detector not in self.detectors and self.last_detector != '':
            self.detectors.append(self.last_detector)
            # Adding subscribers to receive the robots detected by the known detectors
            self.detected_robots_subscribers[self.last_detector] = rospy.Subscriber(
                '/' + self.last_detector + '/detected_robots_topic', RobotPub, self.cb_add_robot_from_detector_topic,
                queue_size=10)

    def cb_add_robot_from_detector_topic(self, msg):
        if msg.namespace != self.namespace:
            self.update_robots(msg)

    def update_robots(self, msg):
        self.lock_update_robots.acquire()
        try:
            self.update_robots_proxy(msg.namespace, msg.type, msg.x, msg.y, msg.yaw, msg.ts)
        finally:
            self.lock_update_robots.release()


class Detector(Robot):
    def __init__(self, namespace, robot_type):
        super(Detector, self).__init__(namespace, robot_type)
        # Iinitializes the proxies dictionary for the robot communication
        self.robot_communication_proxies = dict()
        # Service receiving the detected robot's name
        self.robot_detection_service = rospy.Service('robot_detection_service', RobotDetected, self.cb_robot_detection)
        # Publisher that shares the last detected robot
        self.detected_robots_publisher = rospy.Publisher('/' + self.namespace + '/detected_robots_topic', RobotPub,
                                                         queue_size=10)

    def cb_robot_detection(self, msg):
        """When the AI node detects a robot, it starts the communication"""
        # Adding a proxy for communication with the detected robot
        if msg.namespace not in self.robot_communication_proxies:
            rospy.wait_for_service('/' + msg.namespace + '/robot_communication_service')
            self.robot_communication_proxies[msg.namespace] = rospy.ServiceProxy(
                '/' + msg.namespace + '/robot_communication_service', RobotLocation)
        # rospy.loginfo('[' + str(self.type) + '-' + str(self.namespace) + ']: Talking to ' + str(msg.namespace))
        response = self.robot_communication_proxies[msg.namespace](self.namespace, self.type, self.pose.pose.position.x,
                                                                   self.pose.pose.position.y,
                                                                   get_yaw_from_orientation(self.pose.pose.orientation),
                                                                   self.pose.header.stamp)
        self.update_robots(response)
        self.detected_robots_publisher.publish(response)
        return RobotDetectedResponse(True)
