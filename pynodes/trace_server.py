#!/usr/bin/env python

import rospy


class TraceServer(object):
    '''
    TraceServer is a ros node that runs as a server for avida dorg (digital organism) traces.
        Available services:
            - Blah
            - Blah
    '''
    def __init__(self):
        '''
        TraceServer ROS node constructor
        '''
        # Initialize as ROS node
        rospy.init_node("TraceServer")
        # Load relevant parameters from ROS parameter server

        # Decide what to do based on parameters

if __name__ == "__main__":
    tserve = TraceServer()
