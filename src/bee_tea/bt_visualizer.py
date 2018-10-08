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


class VisualNode:
    def __init__(self, node, visual_tree, nx_graph):
        str_id = '-'.join(str(nid) for nid in node['id'])
        nx_graph.add_node(str_id)

        self._visual_tree = visual_tree
        self._nx_graph = nx_graph

        self.label = node['label']
        self.id = node['id']
        self.str_id = str_id
        self.status = node['status']
        self.type = node['type']
        self.children = []

        visual_tree.add_node(self)

        for child in node['children']:
            visual_child = VisualNode(child, visual_tree, nx_graph)
            self.children.append(visual_child)
            nx_graph.add_edge(str_id, visual_child.str_id)

    def refresh(self, node):
        """
        refresh the entire tree by re-constructing from the given dict node.
        if all nodes are unchanged except their status's, the tree is kept as it
        and the status are modified.
        """

        self_changed = any([node['label'] != self.label,
                            node['id'] != self.id,
                            node['type'] != self.type])

        # no need to check children if self is changed completely, we need to
        # re-make the whole tree
        # since the parent will be using 'any' to asses if children are
        # changed, we need to return True to signal the change
        if self_changed:
            return self_changed

        children_changed = []
        # number of children the same?
        if len(self.children) == len(node['children']):
            # are the children unchanged?
            for dict_child, child in zip(node['children'], self.children):
                children_changed.append(child.refresh(dict_child))

            # any one could be changed to trigger a re-build
            children_changed = any(children_changed)
        else:
            # not the same num of children!
            children_changed = True

        # if children changed, gotta signal that up
        if children_changed:
            return children_changed

        # nothing changed, good.
        # just update the status then
        self.status = node['status']

        # no change!
        return False



class VisualTree:
    def __init__(self, bt_dict):
        """
        creates an object that mimics the given dictionary
        and adds a few extra fields and such

        bt_dict is a dictionary of dictionaries

        self._nodes and _nx_grap.nodes() have the same order by construction
        """

        self._create_new(bt_dict)


    def _create_new(self, bt_dict):
        # a list of VisualNode instances
        self._nodes = []
        # networkx graph, can do layouts and such
        self._nx_graph = nx.OrderedGraph()
        # construct the graph
        self.root_node = VisualNode(bt_dict, self, self._nx_graph)


    def add_node(self, visual_node):
        self._nodes.append(visual_node)


    def refresh(self, bt_dict):
        bt_changed = self.root_node.refresh(bt_dict)
        # root node will have checked all the children if there is any
        # breaking changes in the structure of the tree
        if bt_changed:
            # if there is a structural change, re-make the whole thing
            self._create_new(bt_dict)
            return True

        return False






def traverse_graph(node, graph):
    """
    node should be a dict of dicts, where children are sub-dicts.
    expected fields are: id, label, status, children, type
    graph should be a networkx graph object which will be modified
    """
    str_nid = '-'.join(str(nid) for nid in node['id'])
    graph.add_node(str_nid)

    for child in node['children']:
        child_str_nid = traverse_graph(child, graph)
        graph.add_edge(str_nid, child_str_nid)

    return str_nid


class BT_Visual(pg.GraphItem):
    def __init__(self, root):
        """
        A complete visualiser for a behaviour tree.
        root should be the root node of the tree.
        """

        # needed when initting the superclass
        self.textItems = []
        self.ordered_node_pos = []
        self.ordered_node_ids= []
        self.adj = []
        self.node_labels = []

        # create the underlying graph object from the tree
        G = nx.OrderedGraph()
        # this traversal lets each node add itself to the graph
        traverse_graph(root, G)
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
        ordered_node_ids = []
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
        # status of each node in the same order as the ids
        ordered_node_status = []

        # we will put all children into a queue, this creates
        # a natural ordering of all nodes
        node_queue = [root]

        # emulate a do-while loop to also process the root node in one chunk
        queue_empty = False
        while not queue_empty:
            # dequeue
            node = node_queue[0]
            node_queue = node_queue[1:]

            # child is a dict of stuff
            nid = node['id']
            # nid is a list of ints, we need a hashable thing, a string!
            strnid = '-'.join(str(i) for i in nid)
            ordered_node_ids.append(strnid)
            ordered_node_pos.append(self.pos_dict[strnid])
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
            if node_type == 'SEQ':
                node_labels.append('-->')
            elif node_type == 'FB':
                node_labels.append('?')
            else:
                node_labels.append(node['label'])

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
        for i,nid in enumerate(ordered_node_ids):
            if i == 0:
                # skip the root, we know it has no parents
                continue

            # ids are constructed so that id = parent_id + '-' + child_index
            parent_nid = '-'.join(nid.split('-')[:-1])
            parent_i = ordered_node_ids.index(parent_nid)
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
        self.ordered_node_ids = ordered_node_ids
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
            id_of_item = self.ordered_node_ids[item_index]
            print('clicked:', id_of_item)
        else:
            ev.ignore()



if __name__=='__main__':
    import json
    with open('large_example_bt.json', 'r') as fin:
        root = json.load(fin)

    #  viz = BT_Visual(root)
    vt = VisualTree(root)



