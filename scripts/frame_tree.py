#!/usr/bin/env python


from costmap_merge.msg import Frame, FrameList


class FrameTree:
    def __init__(self, frame_id, parent_frame=None):
        self.frame_id = frame_id
        self.parent_frame = parent_frame
        self.child_frames = dict()

    def create_child(self, child_frame):
        self.child_frames[child_frame] = FrameTree(child_frame, self.frame_id)

    def add_child(self, child_frame):
        child_frame.parent_frame = self.frame_id
        self.child_frames[child_frame.frame_id] = child_frame

    def delete_child(self, child_frame):
        del self.child_frames[child_frame]

    def get_frame(self, frame_id):
        if frame_id == self.frame_id:
            return self
        if self.child_frames:
            for key in self.child_frames:
                if key == frame_id:
                    return self.child_frames[key]
                else:
                    return self.child_frames[key].get_frame(frame_id)

    def delete_frame(self, frame_id):
        if frame_id == self.frame_id:
            del self
            return
        if self.child_frames:
            for key in self.child_frames:
                if key == frame_id:
                    del self.child_frames[key]
                else:
                    self.child_frames[key].get_frame(frame_id)


def convert_frame_tree_to_frame_msg(frame):
    frame_msg = Frame()
    frame_msg.frame_id = frame.frame_id
    frame_msg.parent_frame = frame.parent_frame
    return frame_msg


class FrameListMsgBuilder:
    def __init__(self):
        self.all_frames = list()
        self.frame_list = FrameList()

    def build_tree_msg(self, frame):
        self.get_tree(frame)
        self.frame_list.frames = self.all_frames
        return self.frame_list

    def get_tree(self, frame):
        self.all_frames.append(convert_frame_tree_to_frame_msg(frame))
        if frame.child_frames:
            for child_frame in frame.child_frames:
                self.get_tree(frame.child_frames[child_frame])


class FrameListMsgParser:
    def __init__(self):
        self.frame_tree = dict()

    def list_to_tree(self, frame_list):
        for frame in frame_list.frames:
            found = False
            if self.frame_tree and frame.parent_frame:
                for frame_tree in self.frame_tree:
                    if self.search_in_branch(frame, frame_tree):
                        found = True
                        break
                if not found:
                    self.frame_tree[frame.frame_id] = FrameTree(frame.frame_id, frame.parent_frame)
            else:
                self.frame_tree[frame.frame_id] = FrameTree(frame.frame_id, frame.parent_frame)

        while self.frame_tree.__len__() > 1:
            for frame in self.frame_tree:
                for frame_tree in self.frame_tree:
                    if frame is not frame_tree:
                        if self.frame_tree[frame_tree].get_frame(self.frame_tree[frame].parent_frame):
                            self.frame_tree[frame_tree].get_frame(
                                self.frame_tree[frame].parent_frame).add_child(self.frame_tree[frame].frame_id)
                            del self.frame_tree[frame]

    def search_in_branch(self, frame, frame_tree):
        if self.frame_tree[frame_tree].get_frame(frame.frame_id):
            print('[FrameListMsgParser]: WARNING! duplicated frame ' + str(frame.frame_id))
            return True
        elif self.frame_tree[frame_tree].get_frame(frame.parent_frame):
            self.frame_tree[frame_tree].get_frame(frame.parent_frame).create_child(frame.frame_id)
            return True
        return False
