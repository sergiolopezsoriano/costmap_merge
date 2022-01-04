#!/usr/bin/env python

import rospy
from math import pi
import traceback
from ant_robot.msg import EpcInfo, ReadEpcsAction, ReadEpcsGoal
from costmap_merge.srv import RobotIDs, RobotIDsResponse


class Reading:
    def __init__(self, antenna, rssi, stamp):
        self.stamp = stamp
        self.antenna_port = antenna
        self.rssi = rssi


class RobotTag:

    def __init__(self, ID):
        self.ID = ID
        self.readings = list()

    def add_reading(self, antenna, rssi, stamp):
        self.readings.append(Reading(antenna, rssi, stamp))

    def get_robot_data(self, antenna_map):
        rssi = {'front': 0, 'left': 0, 'right': 0, 'back': 0}
        total_rssi = 0
        max_rssi = 0
        for reading in self.readings:
            # TODO: transform reading
            rssi_lin = 10 ** (reading.rssi / 10)
            total_rssi += 10 ** (reading.rssi / 10)
            max_rssi = max(max_rssi, rssi_lin)
            if reading.antenna_port == antenna_map['front']:
                rssi['front'] += rssi_lin
            elif reading.antenna_port == antenna_map['left']:
                rssi['left'] += rssi_lin
            elif reading.antenna_port == antenna_map['right']:
                rssi['right'] += rssi_lin
            elif reading.antenna_port == antenna_map['back']:
                rssi['back'] += rssi_lin
        angle = (2 * pi * rssi['front'] + pi / 2 * rssi['left'] + pi * rssi['back'] + 3 * pi / 2 * rssi['right']) / total_rssi
        ''' sorted dictionary:       dict(sorted(rssi.items(), key=lambda item: item[1])) '''
        return angle % (2 * pi), max_rssi


class Identification:

    def __init__(self):
        self.namespace = rospy.get_namespace().strip('/')
        self.antenna_map = dict()
        self.antenna_map['front'] = rospy.get_param('/' + self.namespace + '/rfid/antenna_port/front')
        self.antenna_map['back'] = rospy.get_param('/' + self.namespace + '/rfid/antenna_port/back')
        self.antenna_map['left'] = rospy.get_param('/' + self.namespace + '/rfid/antenna_port/left')
        self.antenna_map['right'] = rospy.get_param('/' + self.namespace + '/rfid/antenna_port/right')
        # Server controlling the main operation of discriminating and sorting a list of epcs
        self.robot_epcs = list()
        rospy.Service('/' + self.namespace + '/robot_identification_srv', RobotIDs, self.cb_robot_identification)
        # PART 1: check identification symmetry with other robot
        # self.last_tags_pub = rospy.Publisher('/' + self.namespace + '/robot_ids', RobotList, queue_size=10)
        self.robots = rospy.get_param('/' + self.namespace + '/robots')
        self.robot_ids = dict()
        for robot in self.robots.items():
            self.robot_ids[robot[1]['tag_id']] = robot[0]
            self.robot_epcs.append(robot[1]['tag_id'])
        self.rfid_detections = dict()

    def empty_detections(self):
        self.rfid_detections = dict()

    def cb_robot_identification(self, req):
        # PART 2: check identification symmetry with other robot
        # rospy.Subscriber('/tags_update', String, self.cb_tags_update, queue_size=10)
        self.empty_detections()
        rfid_subs = rospy.Subscriber('/' + self.namespace + '/rfid_publisher/epcs', EpcInfo, self.cb_rfid_controller, queue_size=200)
        while not rfid_subs.get_num_connections():
            rospy.sleep(0.1)
        rospy.sleep(2)
        rfid_subs.unregister()
        rssi = dict()
        angle = dict()
        for tag_id in self.rfid_detections:
            angle[tag_id], rssi[tag_id] = self.rfid_detections[tag_id].get_robot_data(self.antenna_map)

        sorted_detections = list()
        for item in sorted(rssi.items(), key=lambda x: x[1]):
            rospy.logdebug(item)
            rospy.logdebug(angle[item[0]])
            if pi / 3 >= angle[item[0]] or angle[item[0]] >= 5 * pi / 3:
                sorted_detections.append(self.robot_ids[item[0]])

        msg = RobotIDsResponse()
        msg.sorted_detections = sorted_detections[0:req.num_detections]
        msg.sorted_detections.reverse()
        rospy.logdebug('[identification]: ' + str(msg.sorted_detections))
        print(msg)
        return msg

    def cb_rfid_controller(self, msg):
        if msg.epc in self.robot_epcs and msg.epc is not self.robots[self.namespace]['tag_id']:
            tag_id = msg.epc
            antenna = msg.antenna_port
            rssi = msg.RSSI
            stamp = msg.ts
            if tag_id not in self.rfid_detections:
                self.rfid_detections[tag_id] = RobotTag(tag_id)
            self.rfid_detections[tag_id].add_reading(antenna, rssi, stamp)
            rospy.logdebug('[identification]: ' + str(msg.epc) + ' : ' + str(msg.antenna_port) + ' : ' + str(msg.RSSI))


if __name__ == "__main__":

    try:
        rospy.init_node('identification', log_level=rospy.INFO)
        rospy.loginfo('[identification-' + rospy.get_namespace().strip('/') + ']: Node started')
        identification = Identification()
        rospy.spin()
    except Exception as e:
        rospy.logfatal('[identification-' + rospy.get_namespace().strip('/') + ']: Exception %s',
                       str(e.message) + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
