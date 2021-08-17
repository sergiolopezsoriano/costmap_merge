#!/usr/bin/env python


from ..frame_tree import *


a = Node('a')
a.create_child('b')
a.create_child('c')
a.create_child('d')
e = Node('e')
e.create_child('f')
e.create_child('g')
e.create_child('h')
i = Node('i')
i.create_child('j')
i.create_child('k')
i.create_child('l')
m = Node('m')
m.create_child('n')
m.create_child('o')
m.create_child('p')
if not a.get_node('e'):
    a.add_child(e)
    a.child_frames['e'].parent = 'a'
    del e
if not a.get_node('m'):
    a.add_child(m)
    a.child_frames['m'].parent = 'a'
    del m
try:
    if a.get_node('e'):
        i.add_child(a.get_node('e'))
        a.delete_node('e')
except Exception as e:
    print(e)
if not i.get_node('a'):
    i.add_child(a)
    i.child_frames['a'].parent = 'i'
    del a
tb = FrameListMsgBuilder()
frame_list_msg = tb.build_frame_list_msg(i)
print(frame_list_msg)
tmp = FrameListMsgParser()
tmp.frame_list_to_node(frame_list_msg)
# print(tmp.frame_tree)
robot_list = list()
tmp.node_dict[tmp.node_dict.keys()[0]].get_frame_ids(robot_list)
print(robot_list)
