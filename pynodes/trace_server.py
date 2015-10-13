#!/usr/bin/env python

import rospy, os

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
        ##########################################
        # Instance variables for this class
        self.avida_processed_loc = None
        self.dataspace_loc = None
        self.trace_dump = None
        self.trace_info_path = None
        ##########################################
        # Initialize as ROS node
        rospy.init_node("TraceServer")
        ##########################################
        # Load relevant parameters from ROS parameter server
        # Get avida processed directory
        self.avida_processed_loc = rospy.get_param("avida_processed", None)
        # Get avida ros dataspace base location
        self.dataspace_loc = rospy.get_param("avida_ros_dataspace/base_loc", None)
        # Get trace dump we'll use to load/save traces in dataspace
        self.trace_dump = rospy.get_param("avida_ros_dataspace/trace_dump", None)
        # Force extraction?
        force_extraction = rospy.get_param("trace_server/force_extraction", False)
        # Verify everything
        if not self._clean_params():
            rospy.logerr("Failed to clean parameters.")
            exit(-1)
        ##########################################
        # Load/Extract traces
        self._load_traces(force_extract = force_extraction)


    def _clean_params(self):
        '''
        This function verifies and cleans current parameters
          -- I don't really like the way this is currently implemented.  I feel like I could clean it up some.
        '''
        # Check avida_processed_loc
        if not self.avida_processed_loc:
            rospy.logerr("Failed to load avida processed location parameter.")
            return False
        # Expand ~
        self.avida_processed_loc = os.path.expanduser(self.avida_processed_loc)
        # See if directory exists
        if not os.path.isdir(self.avida_processed_loc):
            rospy.logerr("Failed to find avida processed location.")
            return False

        # Check avida dataspace directory
        if not self.dataspace_loc:
            rospy.logerr("Failed to load dataspace location parameter.")
            return False
        # Expand ~
        self.dataspace_loc = os.path.expanduser(self.dataspace_loc)
        # See if directory exists
        if not os.path.isdir(self.dataspace_loc):
            rospy.loginfo("Failed to find dataspace location. Creating new one..")
            # Create dataspace directory (todo: turn into utility)
            try:
                os.makedirs(self.dataspace_loc)
            except:
                rospy.logerr("Failed to make dataspace directory.")
                return False
            else:
                rospy.loginfo("Successfully created new dataspace directory.")

        # Check avida trace dump directory
        if not self.trace_dump:
            rospy.logerr("Failed to load trace dump location parameter.")
            return False
        # Join with dataspace directory
        self.trace_dump = os.path.join(self.dataspace_loc, self.trace_dump)
        # See if already exists
        if not os.path.isdir(self.trace_dump):
            rospy.loginfo("Failed to find trace dump location.  Creating new one...")
            try:
                os.makedirs(self.trace_dump)
            except:
                rospy.logerr("Failed to make trace dump directory from params.")
                return False
            else:
                rospy.loginfo("Successfully created new trace dump location directory")

        return True

    def _load_traces(self, force_extract = False):
        pass


if __name__ == "__main__":
    tserve = TraceServer()
