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


class BT_Visual(pg.GraphItem):
    def __init__(self, root):
        """
        A complete visualiser for a behaviour tree.
        root should be the root node of the tree.
        """

        # needed when initting the superclass
        self.textItems = []
        self.ordered_node_pos = []
        self.ordered_node_unames = []
        self.adj = []
        self.node_labels = []

        # create the underlying graph object from the tree
        G = nx.OrderedGraph()
        # first things first, the tree needs to have been traversed at least once 
        # so that the unique names are generated
        root_node = root.traverse()
        # this traversal lets each node add itself to the graph
        root.traverse_graph(G)
        # create the visual layout of the nodes
        self.pos_dict = nx.drawing.nx_pydot.graphviz_layout(G, prog='dot')
        ###########################################
        # graphics, windows etc
        ###########################################

        # createa a window
        window = pg.GraphicsWindow()
        # init the superclass
        pg.GraphItem.__init__(self)
        # enable antialiasing
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', 'w')
        window.setWindowTitle('Bee Tea')
        # a view box, literally where stuff is drawn
        view = window.addViewBox()
        # no wonkiness please
        view.setAspectLocked()
        # add this to the view
        # we will set the data later, at the end
        view.addItem(self)
        # tell the window that we want to listen to clicks
        window.scene().sigMouseClicked.connect(self.clicked)


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
            ordered_node_pos.append(self.pos_dict[uname])
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

        lines = np.array(lines, dtype=[('red',np.ubyte),
                                       ('green',np.ubyte),
                                       ('blue',np.ubyte),
                                       ('alpha',np.ubyte),
                                       ('width',float)])
        ##################################################################
        # self settings
        ##################################################################

        # after all that stuff, we want some of them to be accessible
        # when clicking etc.
        self.ordered_node_pos = np.array(ordered_node_pos)
        self.ordered_node_unames = ordered_node_unames
        self.adj = np.array(adj)
        self.node_labels = node_labels

        # comes from the superclass
        self.setData(pos=self.ordered_node_pos,
                     size=50,
                     symbol='s',
                     adj=self.adj,
                     pen=lines,
                     symbolBrush = None,
                     symbolPen = None,
                     text = self.node_labels,
                     textbox = textboxes)



    def setData(self, **kwds):
        # overwrite the superclass because of texts
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


    def clicked(self, ev):
        """
        click listener
        """

        if ev.button() == QtCore.Qt.LeftButton:
            # this is aleft button click on a graph piece, minimize/maximize it
            pos = ev.pos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) != 1:
                # ignore clicks that 'touches' too many things
                ev.ignore()
                return

            item_index = pts[0].data()[0]
            text_of_item = self.textItems[item_index].textItem.toPlainText()
            print('clicked:', item_index, text_of_item)
            uname_of_item = self.ordered_node_unames[item_index]
            print('clicked:', uname_of_item)
        else:
            ev.ignore()



if __name__=='__main__':
    import json
    with open('large_example_bt.json', 'r') as fin:
        root = json.load(fin)

    viz = BT_Visual(root)



