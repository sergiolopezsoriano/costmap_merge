#!/usr/bin/env python


from costmap_merge.scripts.frame_tree import *


a = FrameTree('a')
a.create_child('b')
a.create_child('c')
a.create_child('d')
e = FrameTree('e')
e.create_child('f')
e.create_child('g')
e.create_child('h')
i = FrameTree('i')
i.create_child('j')
i.create_child('k')
i.create_child('l')
m = FrameTree('m')
m.create_child('n')
m.create_child('o')
m.create_child('p')
if not a.get_frame('e'):
    a.add_child(e)
    a.child_frames['e'].parent = 'a'
    del e
if not a.get_frame('m'):
    a.add_child(m)
    a.child_frames['m'].parent = 'a'
    del m
try:
    if a.get_frame('e'):
        i.add_child(a.get_frame('e'))
        a.delete_frame('e')
except Exception as e:
    print(e)
if not i.get_frame('a'):
    i.add_child(a)
    i.child_frames['a'].parent = 'i'
    del a
tb = FrameListMsgBuilder()
frame_list_msg = tb.build_tree_msg(i)
print(frame_list_msg)
tmp = FrameListMsgParser()
tmp.list_to_tree(frame_list_msg)
print('d')
