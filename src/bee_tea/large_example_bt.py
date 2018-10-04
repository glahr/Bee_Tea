#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-09-25
# A un-ros-ified copy of the porto paper's BT implementation for testing purposes

from bt_pieces import Fallback, Seq, Negate, InstantLeaf, ActionNodeLeaf
from bt_states import *


class porto_lolo:
    def __init__(self, namespace='lolo_auv_1'):
        """
        a just-enough container for lolo for the paper
        """

        self._ns = namespace

        self._mission_aborted = False

        self._max_depth = 20
        self._min_depth = 10
        # TODO set with data from sim
        self._depth = 11
        self._target_depth = 15

        self._continue_command_received = False
        self._away_from_ship = False
        self._go_command_received = False
        self._compass_calibrated = False
        self._payload_on = False

        self._mission_finalized = False

    def mission_not_aborted(self):
        return SUCCESS

    def able_to_descend_or_ascend(self):
        return SUCCESS

    def actuator_operational(self):
        return SUCCESS

    def prop_operational(self):
        return SUCCESS

    def no_leaks(self):
        return SUCCESS

    def depth_ok(self):
        if self._depth < self._max_depth and self._depth >= self._min_depth:
            return SUCCESS
        else:
            return FAILURE

    def set_mission_abort(self):
        self._mission_aborted = True
        return SUCCESS

    # TODO actionnode
    def drop_weight(self):
        return SUCCESS

    # TODO waypoints or weight change
    def go_to_surface(self):
        return SUCCESS

    def path_obstacle_free(self):
        return SUCCESS

    # TODO waypoints
    def avoid_obstacle_in_direction_of_next_WP(self):
        return SUCCESS

    def continue_command_received(self):
        if self._continue_command_received:
            return SUCCESS
        return FAILURE

    def away_from_ship(self):
        if self._away_from_ship:
            return SUCCESS
        return FAILURE

    # TODO waypoints
    def keep_distance(self):
        return SUCCESS

    def go_command_received(self):
        if self._go_command_received:
            return SUCCESS
        return FAILURE

    #TODO implement as action node
    def wait(self):
        return SUCCESS

    def compass_calibrated(self):
        if self._compass_calibrated:
            return SUCCESS
        return FAILURE

    def calibrate_compass(self):
        print('Calibrated compass')
        self._compass_calibrated = True
        return SUCCESS

    def payload_on(self):
        if self._payload_on:
            return SUCCESS
        return FAILURE

    def turn_on_payload(self):
        print('Payload turned on')
        self._payload_on = True
        return SUCCESS

    #TODO implement with actual data?
    def at_target_depth(self):
        if abs(self._depth - self._target_depth) <= 0.5:
            return SUCCESS
        else:
            return FAILURE

    # TODO implement, weight probly
    def adjust_depth(self):
        return SUCCESS

    def mission_synchronized(self):
        return SUCCESS

    def no_goto_surface_command(self):
        return SUCCESS

    # TODO implement, weight or wps?
    def surface(self):
        return SUCCESS

    def no_user_commanded_waypoints(self):
        return SUCCESS

    def update_waypoints(self):
        print('Updated waypoints')
        return SUCCESS

    # TODO vague af
    def no_autonomy_comanded_way_points(self):
        return SUCCESS

    # TODO implement simply
    def mission_complete(self):
        return SUCCESS

    # TODO 
    def at_target_waypoint(self):
        return SUCCESS

    # TODO action node
    def go_to_target_waypoint(self):
        return SUCCESS

    # TODO
    def set_target_waypoint_to_next_waypoint(self):
        return SUCCESS

    # TODO
    def mission_finalized(self):
        if self._mission_finalized:
            return SUCCESS
        return FAILURE

    def at_surface(self):
        if self._depth < 0.5:
            return SUCCESS
        return FAILURE

    def payload_off(self):
        if self._payload_on:
            return FAILURE
        return SUCCESS

    def shutdown_payload(self):
        print('Shutdown payload')
        self._payload_on = False
        return SUCCESS



def make_instant_post_condition_subtree(subtree_name,
                                        condition_name, condition_func,
                                        action_name, action_func):
    fb = Fallback(subtree_name)
    fb.add_child(InstantLeaf(condition_name, condition_func))
    fb.add_child(InstantLeaf(action_name, action_func))
    return fb


# the Lolo object that has all the instant functions and such
L = porto_lolo()

# root sequence
root = Seq('root')

# safety 1 subtree
safety1 = Fallback('Safety 1')
root.add_child(safety1)

s1_seq1 = Seq('safety1_seq_1')
safety1.add_child(s1_seq1)

s1_seq1.add_child(InstantLeaf('Mission not aborted?', L.mission_not_aborted))
s1_seq1.add_child(InstantLeaf('Able to ascend/descend?', L.able_to_descend_or_ascend))
s1_seq1.add_child(InstantLeaf('Actuator operational?', L.actuator_operational))
s1_seq1.add_child(InstantLeaf('Prop operational?', L.prop_operational))
s1_seq1.add_child(InstantLeaf('No leaks?', L.no_leaks))
s1_seq1.add_child(InstantLeaf('Depth ok?', L.depth_ok))

s1_seq2 = Seq('safety1_seq_2')
safety1.add_child(s1_seq2)

s1_seq2.add_child(InstantLeaf('Set Mission Abort', L.set_mission_abort))
s1_seq2.add_child(InstantLeaf('Drop weight', L.drop_weight))
s1_seq2.add_child(InstantLeaf('Go to surface', L.go_to_surface))


# safety 2 subtree
safety2 = Fallback('Safety 2')
root.add_child(safety2)

safety2.add_child(InstantLeaf('Path obstacle free', L.path_obstacle_free))
safety2.add_child(InstantLeaf('Avoid obstacle in dorection of next WP', L.avoid_obstacle_in_direction_of_next_WP))


# system prep subtree
sysprep = Fallback('System Preparation')
root.add_child(sysprep)

sysprep.add_child(InstantLeaf('Continue command received', L.continue_command_received))

sysprep_seq = Seq('sysprep_seq')
sysprep.add_child(sysprep_seq)
sysprep_seq.add_child(make_instant_post_condition_subtree('away_from_ship',
                                                          'Away from ship?', L.away_from_ship,
                                                          'Keep distance', L.keep_distance))
sysprep_seq.add_child(make_instant_post_condition_subtree('go_command_received',
                                                          'Go command received', L.go_command_received,
                                                          'Wait', L.wait))
sysprep_seq.add_child(make_instant_post_condition_subtree('compass_calibrated',
                                                          'Compass calibrated?', L.compass_calibrated,
                                                          'Calibrate compass', L.calibrate_compass))
sysprep_seq.add_child(make_instant_post_condition_subtree('payload_on',
                                                          'Payload on?', L.payload_on,
                                                          'Turn on payload', L.turn_on_payload))
sysprep_seq.add_child(make_instant_post_condition_subtree('at_target_depth',
                                                          'At target depth?', L.at_target_depth,
                                                          'Adjust depth', L.adjust_depth))
sysprep_seq.add_child(make_instant_post_condition_subtree('continue_command_received',
                                                          'Continue command received', L.continue_command_received,
                                                          'Wait', L.wait))

# mission synch subtree
mission_sync = Fallback('Mission Synchornisation')
root.add_child(mission_sync)

mission_sync.add_child(InstantLeaf('Mission synchronized', L.mission_synchronized))
msync_seq = Seq('msync_seq')
mission_sync.add_child(msync_seq)

msync_seq.add_child(make_instant_post_condition_subtree('no_goto_surface_command',
                                                        'No "goto surface" command',L.no_goto_surface_command,
                                                        'Surface', L.surface))
msync_seq.add_child(make_instant_post_condition_subtree('no_user_commanded_waypoints',
                                                        'No user commanded waypoints', L.no_user_commanded_waypoints,
                                                        'Update waypoints', L.update_waypoints))
msync_seq.add_child(make_instant_post_condition_subtree('no_autonomy_comanded_way_points',
                                                        'No autonomy commanded waypoints',L.no_autonomy_comanded_way_points,
                                                        'Update waypoints', L.update_waypoints))

# mission exec. subtree
mission_exec = Fallback('Mission Execution')
root.add_child(mission_exec)

mission_exec.add_child(InstantLeaf('Mission complete(all waypoints visited)?',L.mission_complete))
mexec_seq = Seq('mexec_seq')
mission_exec.add_child(mexec_seq)
mexec_seq.add_child(make_instant_post_condition_subtree('at_target_waypoint',
                                                        'At target waypoint', L.at_target_waypoint,
                                                        'Go to target waypoint', L.go_to_target_waypoint))

mexec_seq.add_child(InstantLeaf('Target waypoint <-- next waypoint', L.set_target_waypoint_to_next_waypoint))

# mission finalization subtree
mission_final = Fallback('Mission Finalisation')
root.add_child(mission_final)
mission_final.add_child(InstantLeaf('Mission finalized (at surface with payload turned off)?',L.mission_finalized))
mfinal_seq = Seq('mfinal_seq')
mission_final.add_child(mfinal_seq)
mfinal_seq.add_child(make_instant_post_condition_subtree('at_surface',
                                                         'At surface?', L.at_surface,
                                                         'Go to surface', L.go_to_surface))
mfinal_seq.add_child(make_instant_post_condition_subtree('payload_off',
                                                         'Payload off?', L.payload_off,
                                                         'Shutdown payload', L.shutdown_payload))



print(root.display(0))
root.tick()
print(root.display(0))
print(root.traverse())

import networkx as nx
import matplotlib.pyplot as plt
plt.ion()
G = nx.OrderedGraph()
root.traverse_graph(G)
pos_dict = nx.drawing.nx_pydot.graphviz_layout(G, prog='dot')
# do not use together with pyqtgraph, somehow blocks eachother
#  nx.draw(G, pos=pos, with_labels=True)


import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

# enable antialiasing
pg.setConfigOptions(antialias=True)

# createa a window
window = pg.GraphicsWindow()
window.setWindowTitle('Bee Tea')
# a view box, literally where stuff is drawn
view = window.addViewBox()
# no wonkiness please
view.setAspectLocked()
# the visual graph, separate from the networkx graph up above
viz_graph = pg.GraphItem()
# add to view
view.addItem(viz_graph)

root_node = root.traverse()
# this will define the order of all nodes going forward
ordered_node_unames = []
# node positions, same order as names
ordered_node_pos = []
# adjacency list of index pairs
adj = []
# (rgba width) list of colors
lines = []
node_fills = []
node_lines = []
# status of each node in the same order as the unames
ordered_node_status = []

# we will put all children into a queue, this creates
# a natural ordering of all nodes
node_queue = [root_node]

# emulate a do-while loop to also process the root node in one chunk
queue_empty = False
while not queue_empty:
    # dequeue
    node = node_queue[0]
    node_queue = node_queue[1:]

    # child is a dict of stuff
    uname = node['unique_name']
    ordered_node_unames.append(uname)
    ordered_node_pos.append(pos_dict[uname])
    ordered_node_status.append(node['status'])

    # color the nodes according to their status
    status = node['status']
    if status == FAILURE:
        # red
        c = pg.mkBrush((255,0,0,255))
        l = pg.mkPen((255,0,0,255))
    elif status == SUCCESS:
        # green
        c = pg.mkBrush((0, 255, 0, 255))
        l = pg.mkPen((0, 255, 0, 255))
    elif status == RUNNING:
        # thick blue
        c = pg.mkBrush((0, 0, 255, 255))
        l = pg.mkPen((0, 0, 255, 255))
    else:
        # unticked, thin grey
        c = pg.mkBrush((150, 150, 150, 255))
        l = pg.mkPen((150, 150, 150, 255))
    node_fills.append(c)
    node_lines.append(l)

    try:
        new_nodes = node['children']
        node_queue.extend(new_nodes)
    except KeyError:
        # no children for this node
        pass

    queue_empty = len(node_queue) <= 0

# we can find the edge between a child and parent from their unique names easily
# R0 is R's child, R000 is R00's child
# XYZ is XY's child
for i,uname in enumerate(ordered_node_unames):
    if i == 0:
        # skip the root, we know it has no parents
        continue

    # unames are constructed so that uname = parent_uname+'-'+child_index
    parent_uname = '-'.join(uname.split('-')[:-1])
    parent_i = ordered_node_unames.index(parent_uname)
    adj.append( (parent_i, i) )

    # color the lines according to the status of the child
    status = ordered_node_status[i]
    if status == FAILURE:
        # red
        line = (255, 0, 0, 255, 2)
    elif status == SUCCESS:
        # green
        line = (0, 255, 0, 255, 2)
    elif status == RUNNING:
        # thick blue
        line = (0, 0, 255, 255, 5)
    else:
        # unticked, thin grey
        line = (150, 150, 150, 255, 1)
    lines.append(line)


ordered_node_pos = np.array(ordered_node_pos)
adj = np.array(adj)
lines = np.array(lines, dtype=[('red',np.ubyte),
                               ('green',np.ubyte),
                               ('blue',np.ubyte),
                               ('alpha',np.ubyte),
                               ('width',float)])
viz_graph.setData(pos=ordered_node_pos,
                  adj=adj,
                  pen=lines,
                  symbolBrush = node_fills,
                  symbolPen = node_lines)


