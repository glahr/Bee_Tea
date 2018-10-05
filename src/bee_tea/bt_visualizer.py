#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-10-02

from __future__ import print_function
import rospy
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING


import networkx as nx
import matplotlib.pyplot as plt
plt.ion()
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui



from large_example_bt import make_large_tree


class LabeledDraggableGraph(pg.GraphItem):
    def __init__(self):
        """
        lifted from the pyqtgraph examples
        """

        self.dragPoint = None
        self.dragOffset = None
        self.textItems = []
        pg.GraphItem.__init__(self)
        self.scatter.sigClicked.connect(self.clicked)

    def setData(self, **kwds):
        self.text = kwds.pop('text', [])
        self.textbox = kwds.pop('textbox', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = np.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = np.arange(npts)
        self.setTexts(self.text, self.textbox)
        self.updateGraph()

    def setTexts(self, text, textbox):
        for i in self.textItems:
            i.scene().removeItem(i)
        self.textItems = []
        for t,tb in zip(text, textbox):
            item = pg.TextItem(t, anchor=(0.5, 0.5), color='k', border=tb['border'], fill=tb['fill'])
            self.textItems.append(item)
            item.setParentItem(self)

    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        for i,item in enumerate(self.textItems):
            item.setPos(*self.data['pos'][i])


    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return

        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return

        ind = self.dragPoint.data()[0]
        self.data['pos'][ind] = ev.pos() + self.dragOffset
        self.updateGraph()
        ev.accept()

    def clicked(self, pts):
        print("clicked: %s" % pts)


clicked = None

root = make_large_tree()

G = nx.OrderedGraph()
root.traverse_graph(G)
pos_dict = nx.drawing.nx_pydot.graphviz_layout(G, prog='dot')

# enable antialiasing
pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')

# createa a window
window = pg.GraphicsWindow()
window.setWindowTitle('Bee Tea')
# a view box, literally where stuff is drawn
view = window.addViewBox()
# no wonkiness please
view.setAspectLocked()
# the visual graph, separate from the networkx graph up above
#  viz_graph = pg.GraphItem()
viz_graph = LabeledDraggableGraph()
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
# textboxes should contain ['border':pen, 'fill':brush]
# to create a box around the label
textboxes = []
node_labels = []
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
    textboxes.append({'border':l, 'fill':c})

    node_type = node['type']
    if node_type == '-->' or node_type==' ? ':
        node_labels.append(node_type)
    else:
        node_labels.append(node['name'])

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
                  size=50,
                  symbol='s',
                  adj=adj,
                  pen=lines,
                  symbolBrush = None,
                  symbolPen = None,
                  text = node_labels,
                  textbox = textboxes)


