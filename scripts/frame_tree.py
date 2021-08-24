#!/usr/bin/env python

from costmap_merge.msg import Frame, FrameList
from time import time
import rospy


class Node:
    def __init__(self, frame_id, parent_frame='', stamp=time()):
        self.frame_id = frame_id
        self.parent_frame = parent_frame
        self.child_frames = dict()
        self.stamp = stamp

    def update_parent(self, parent_frame):
        self.parent_frame = parent_frame

    def update_stamp(self, stamp):
        self.stamp = stamp

    def get_stamp(self):
        return self.stamp

    def create_child(self, child_id):
        self.child_frames[child_id] = Node(child_id, self.frame_id)

    def print_childs(self):
        for child_id in self.child_frames:
            print(child_id)

    def add_child(self, node):
        node.parent_frame = self.frame_id
        self.child_frames[node.frame_id] = node

    def delete_child(self, frame_id):
        del self.child_frames[frame_id]

    def get_node(self, frame_id):
        node = None
        if frame_id == self.frame_id:
            return self
        if self.child_frames:
            for child_id in self.child_frames:
                if child_id == frame_id:
                    return self.child_frames[child_id]
                else:
                    node = self.child_frames[child_id].get_node(frame_id)
        return node

    def get_node_list(self, frame_id, node_list1):
        if frame_id == self.frame_id:
            node_list1.append(frame_id)
            return node_list1
        if self.child_frames:
            for child_id in self.child_frames:
                node_list2 = self.child_frames[child_id].get_node_list(frame_id, node_list1)
                if node_list2:
                    node_list2.append(self.frame_id)
                    return node_list2

    def delete_node(self, frame_id):
        node_list = list()
        node_list = self.get_node_list(frame_id, node_list)
        node_list.reverse()
        branch = 'self'
        for node in node_list:
            if node == node_list[0]:
                pass
            else:
                branch = branch + ".child_frames['" + str(node) + "']"
        exec 'del ' + branch

    def get_frame_ids(self, frame_ids):
        frame_ids.append(self.frame_id)
        for child_id in self.child_frames:
            self.child_frames[child_id].get_frame_ids(frame_ids)
        return frame_ids


def node_to_frame_msg(node):
    frame_msg = Frame()
    frame_msg.frame_id = node.frame_id
    frame_msg.parent_frame = node.parent_frame
    frame_msg.stamp = node.stamp
    return frame_msg


class FrameListMsgBuilder:
    def __init__(self):
        # Getting ROS parameters
        self.namespace = rospy.get_namespace().strip('/')
        self.frame_ids = list()
        self.frame_list = FrameList()
        self.frame_list.detector_ns = self.namespace

    def clear_frame_list(self):
        self.frame_list = FrameList()
        self.frame_list.detector_ns = self.namespace

    def build_frame_list_msg(self, node):
        self.clear_frame_list()
        self.add_frame_to_frame_list(node)
        self.frame_list.frames = self.frame_ids
        return self.frame_list

    def add_frame_to_frame_list(self, node):
        self.frame_ids.append(node_to_frame_msg(node))
        if node.child_frames:
            for child_id in node.child_frames:
                self.add_frame_to_frame_list(node.child_frames[child_id])


class FrameListMsgParser:
    def __init__(self):
        self.node_dict = dict()

    def frame_list_to_node(self, frame_list):
        for frame in frame_list.frames:
            found = False
            if self.node_dict and frame.parent_frame:
                for node in self.node_dict:
                    if self.search_in_branch(frame, node):
                        found = True
                        break
                if not found:
                    self.node_dict[frame.frame_id] = Node(frame.frame_id, frame.parent_frame, frame.stamp)
            else:
                self.node_dict[frame.frame_id] = Node(frame.frame_id, frame.parent_frame, frame.stamp)

        node_dict_keys = self.node_dict.keys()
        removed = list()
        while self.node_dict.__len__() > 1:
            for frame in node_dict_keys:
                for node in node_dict_keys:
                    if frame is not node and frame not in removed:
                        if self.node_dict[node].get_node(self.node_dict[frame].parent_frame):
                            self.node_dict[node].get_node(
                                self.node_dict[frame].parent_frame).add_child(self.node_dict[frame])
                            del self.node_dict[frame]
                            node_dict_keys.remove(frame)
                            removed.append(frame)
        return self.node_dict.popitem()[1]

    def search_in_branch(self, frame, node):
        if self.node_dict[node].get_node(frame.frame_id):
            print('[FrameListMsgParser]: WARNING! duplicated frame ' + str(frame.frame_id))
            return True
        elif self.node_dict[node].get_node(frame.parent_frame):
            self.node_dict[node].get_node(frame.parent_frame).create_child(frame.frame_id)
            return True
        return False
