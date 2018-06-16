#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-06-14

from __future__ import print_function

import rospy
from bt_states import SUCCESS, FAILURE, RUNNING
from bt_pieces import ActionNodeLeaf, InstantLeaf, Seq, Fallback, Negate

import pickle
import os

class Pepper:
    def __init__(self):
        self.punch_goal = ''


    def goal_to_the_right(self):
        #TODO
        print("GOAL TO THE RIGHT")
        return SUCCESS


    def set_punch_goal(self):
        try:
            with open('/home/ozer/sorohack/punch_goal.txt', 'r') as fin:
                goal = pickle.load(fin)
                goal = pickle.dumps(goal['coord'])
                self.punch_goal = goal
                print("PUNCH GOAL SET")
                return SUCCESS
        except:
            print("PUNCH GOAL FILE NOT FOUND")
            return FAILURE

        print("PUNCH GOAL NOT SET")
        return FAILURE

    def check_password(self):
        pw = ''
        try:
            with open('/home/ozer/sorohack/password.txt', 'r') as fin:
                pw = fin.read()
                if pw == 'almonds\n':
                    print("PW CORRECT")
                    return SUCCESS
        except:
            print('NO PW FILE FOUND')
            return FAILURE

        print("PW INCORRECT",pw)
        return FAILURE

    def can_reach_punch_goal(self):
        #TODO
        print('CAN REACH PUNCH GOAL')
        return SUCCESS

    def shutdown(self):
        #TODO
        print('SHUTTING DOWN')
        return SUCCESS

    def wait(self):
        #TODO
        print('WAITING')
        return RUNNING

    def delete_goal_file(self):
        try:
            os.remove('/home/ozer/sorohack/punch_goal.txt')
            return SUCCESS
        except:
            return FAILURE


    def get_punch_goal(self):
        return self.punch_goal




if __name__=='__main__':
    rospy.init_node('BT_MAIN')

    P = Pepper()

    # long action nodes
    move_right = ActionNodeLeaf('move', goal='right')
    move_left = ActionNodeLeaf('move', goal='left')
    punch_object = ActionNodeLeaf('punch', goal_fn=P.get_punch_goal)

    set_punch_goal = InstantLeaf('set_punch_goal', P.set_punch_goal)
    delete_goal_file = InstantLeaf('delete_goal_file', P.delete_goal_file)
    # movement
    can_reach_punch_goal = InstantLeaf('can_reach_punch_goal',P.can_reach_punch_goal)
    goal_to_the_right = InstantLeaf('goal_to_the_right',P.goal_to_the_right)

    mover = Fallback('mover',[
        can_reach_punch_goal,
        Seq('mover_right',[
            goal_to_the_right,
            move_right
        ]),
        move_left
    ])

    # check if we need to shutdown and finalize
    check_password = InstantLeaf('check_password', P.check_password)
    shutdown = InstantLeaf('shutdown', P.shutdown)

    finalize = Seq('finalize',[
        check_password,
        shutdown
    ])

    wait = InstantLeaf('wait', P.wait)

    root = Fallback('root',[
        finalize,
        Fallback('move or wait',[
            Seq('can we punch yet',[
                set_punch_goal,
                mover,
                punch_object,
                delete_goal_file
            ]),
            wait
        ])
    ])





    prev_s = ''
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        r = root.tick()
        s = root.display(0)
        #  if s != prev_s:
        print(s)
        prev_s = s
        #  if r == SUCCESS:
            #  break
        rate.sleep()


