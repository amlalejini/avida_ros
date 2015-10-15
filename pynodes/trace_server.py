#!/usr/bin/env python

import rospy, os, re
import cPickle as pickle
from avida_ros.msg import StandardDorgCPU, Trace, Stack
from avida_ros.srv import GetAllTraces, GetAllTracesResponse, GetAllTracesRequest

class TraceServer(object):
    '''
    TraceServer is a ros node that runs as a server for avida dorg (digital organism) traces.
        Available services:
            - Blah
            - Blah

    IMPOSED RESTRICTION ON WAY TRACES ARE STORED:
      {trial_id: {
        "env_id":
            ("trace_id": trace_msg_obj),
            ...
        ,
        "env_id":
            ("trace_id": trace_msg_obj),
            ...
      },
      trial_id: {
        ...
      }
        ...
      }
    In terms of how this works out with the file structure: /...avida processed.../<trial_id>/<env_id>/<trace_id>...
    '''
    def __init__(self):
        '''
        TraceServer ROS node constructor
        '''
        ##########################################
        # Instance variables for this class
        # TODO: right now, I'm storing traces in a bunch of different ways (mostly for different potential ways I would want to acess it)
        #       I need to settle on a storage method.
        self.avida_processed_loc = None     # Location of processed avida data (from avida analyze)
        self.dataspace_loc = None           # Location of ros avida dataspace
        self.trace_dump = None              # Location of trace dump (in dataspace)
        self.trace_dict = None
        self.trace_objs = None
        self.get_all_traces_srv = None
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
        # Load/Extract traces (populate trace list and trace dict)
        self._load_traces(force_extract = force_extraction)
        ##########################################
        # Setup services available to other ROS nodes
        self.get_all_traces_srv = rospy.Service("get_all_traces", GetAllTraces, self.get_all_traces_handler)

    def run(self):
        '''
        Infinite run loop.
        '''
        rospy.loginfo("Traces loaded successfully.  Listening for requests.")
        rospy.spin()

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
        '''
        This function first dives into avida_processed looking for .trace files and builds a list of all found .trace files (trace_list)
        if not force_extract:
            check to see if pickle of trace already exists
            if a pickle already exists, <load it up?>
            otherwise, generate new trace object, save as pickle
        else
            generate new trace object, save as pickle
        '''
        trace_dict = {}
        trace_objs = []
        # Get trace locations from avida processed directory
        trace_list = self._find_all_traces(self.avida_processed_loc)
        # Load them up and pickle 'em out
        for trace in trace_list:
            # Collect some information about our trace
            try:
                tname_split = trace.split("/")
                trial_id = tname_split[0]               # Grab the trial id
                env_id = tname_split[1]                 # Grab the env id
                trace_id = "/".join(tname_split[2:])    # Grab the trace id
            except:
                rospy.logerr("Failed to collect trace ID, env ID, and trial ID information about: %s" % trace)
                continue
            if trial_id not in trace_dict: trace_dict[trial_id] = {}
            if env_id not in trace_dict[trial_id]: trace_dict[trial_id][env_id] = {}
            if trace_id not in trace_dict[trial_id][env_id]: trace_dict[trial_id][env_id][trace_id] = None

            # Check to see if we already have a pickle of the trace object
            pickled = os.path.exists(os.path.join(self.trace_dump, trace + ".pickle"))
            # If pickle exists and not force_extract: load from pickle
            if pickled and not force_extract:
                # Load from pickle!
                with open(os.path.join(self.trace_dump, trace + ".pickle")) as fp:
                    trace_obj = pickle.load(fp)
                # Store it in trace dict
                trace_dict[trial_id][env_id][trace_id] = trace_obj
                trace_objs.append(trace_obj)
            else:
                # Load from file!
                with open(os.path.join(self.avida_processed_loc, trace)) as fp:
                    trace_obj = self._extract_trace(fp)
                    trace_obj.trial_id = trial_id
                    trace_obj.env_id = env_id
                    trace_obj.trace_id = trace_id
                # store it (not sure which of these structures I'll eventually settle on)
                trace_dict[trial_id][env_id][trace_id] = trace_obj
                trace_objs.append(trace_obj)
                # save it out!
                if not os.path.exists(os.path.dirname(os.path.join(self.trace_dump, trace))): os.makedirs(os.path.dirname(os.path.join(self.trace_dump, trace)))
                pickle.dump(trace_obj, open(os.path.join(self.trace_dump, trace) + ".pickle", "wb"))
        # Point trace dict instance variable to trace_dict we just built
        self.trace_dict = trace_dict
        self.trace_objs = trace_objs

    def _find_all_traces(self, targ_dir):
        '''
        This function returns a list of all traces found in search of target directory
        traces are stored as file paths ([path_to_trace1, path_trace_2, ...])
        '''
        trace_list = []
        for root, dirnames, filenames in os.walk(targ_dir):
            # Any trace files?
            tnames = [t for t in filenames if ".trace" in t]
            for trace in tnames:
                # Found a trace, append path (relative to target directory) to trace to trace list
                trace_list.append(os.path.join(root[len(targ_dir) + 1:], trace))
        # trace_list now contains all traces found in target directory (recursive)
        return trace_list

    def _extract_trace(self, fp):
        '''
        Given file pointer to trace file, extract and return trace.
        '''
        execution_states = []  # Stores chunks of file content.  Each chunk corresponds to one execution state.
        # Extract digital organism hardware states string chunks from trace file
        current_state = -1
        for line in fp:
            if "---------------------------" in line:
                current_state += 1
                execution_states.append("")
            else:
                execution_states[current_state] += line
        # trace will store list of execution states as DorgCPU messages in the order in which they occurred
        trace = Trace()
        trace.trial_id = ""
        trace.env_id = ""
        trace.trace_id = ""
        for state in execution_states:
            # These are the things I need to extract:
            #  - instr_head, read_head, write_head, stacks, active_stack, reg_AX, reg_BX, reg_CX, memory, environment buffer, input buffer, output, current_instruction
            # check to see if we're done extracting trace
            m = re.search(pattern = "#\sFinal\sMemory:", string = state)
            n = re.search(pattern = "#\sTIMEOUT:", string = state)
            if m != None or n != None: break
            # Get merit base (currently don't use)
            m = re.search(pattern = "MeritBase:(-?[0-9]+\.?\d*)", string = state)
            merit_base = float(m.group(1))
            # get merit bonus (currently don't use)
            m = re.search(pattern = "Bonus:([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)", string = state)
            bonus = float(m.group(1))
            # get current update (currently don't use)
            m = re.search(pattern = "U:(-?[0-9]+)\n", string = state)
            current_update = int(m.group(1))
            ## parse out current state of avidian hardware ##
            # get instruction head location
            m = re.search(pattern = "IP:(\d+)", string = state)
            instr_head = int(m.group(1))
            # get current instruction
            m = re.search(pattern = "IP:\d+\s\((.*)\)\n", string = state)
            current_instruction = str(m.group(1))
            # get read head location
            m = re.search(pattern = "R-Head:(\d+)", string = state)
            read_head = int(m.group(1))
            # get write head location
            m = re.search(pattern = "W-Head:(\d+)", string = state)
            write_head = int(m.group(1))
            # get flow head location
            m = re.search(pattern = "F-Head:(\d+)", string = state)
            flow_head = int(m.group(1))
            # get stacks
            m = re.findall(pattern = "Stack\s\d+:\s.*\n", string = state)
            #  - allocate space for found stacks
            stacks = [Stack() for i in xrange(0, len(m))]
            #  - for each stack matched in the chunk,
            for stk in m:
                # get and set the stack id number
                stk_m = re.search(pattern = "Stack\s(\d+):", string = stk)
                stk_id = int(stk_m.group(1))
                # populate the correct stack with the data extracted from the trace
                stacks[stk_id].stack = stk.split(":")[-1].strip().split(" ")
            # get active stack
            m = re.search(pattern = "\*\sStack\s(\d+):", string = state)
            active_stack = int(m.group(1))
            # get register content
            registers = {}
            m = re.findall(pattern = "(\w+X):-?\d+\s\[(0x[a-f0-9]+)\]", string = state)
            for reg in m:
                # AX, BX, CX
                registers[reg[0]] = reg[1]
            # get memory content
            m = re.search(pattern = "Mem\s\(\d+\):\s*(\w*)\n", string = state)
            memory = list(m.group(1))
            # get env contents
            m = re.search(pattern = "Input\s\(env\):\s?([\w\s]*)\n", string = state)
            buff = m.group(1).strip()
            #  if buffer is empty, we want an empty list
            env_buffer = buff.split(" ") if buff != "" else []
            # get input buffer
            m = re.search(pattern = "Input\s\(buf\):\s?([\w\s]*)\n", string = state)
            buff = m.group(1).strip()
            #  if buffer is empty, we want an empty list
            input_buffer = buff.split(" ") if buff != "" else []
            # get output
            m = re.search(pattern = "Output:\s*(\w*)\n", string = state)
            buff = m.group(1).strip()
            output = buff if buff != "" else str(None)
            #########################################
            # Create ROS message for DORG CPU
            dorg_cpu = StandardDorgCPU()
            dorg_cpu.instr_head = instr_head
            dorg_cpu.read_head = read_head
            dorg_cpu.write_head = write_head
            dorg_cpu.stacks = stacks
            dorg_cpu.active_stack = active_stack
            dorg_cpu.reg_AX = registers["AX"]
            dorg_cpu.reg_BX = registers["BX"]
            dorg_cpu.reg_CX = registers["CX"]
            dorg_cpu.memory = memory
            dorg_cpu.environment_buffer = env_buffer
            dorg_cpu.input_buffer = input_buffer
            dorg_cpu.output = output
            dorg_cpu.current_instruction = current_instruction
            # append current avidian state to trace
            trace.dorgs.append(dorg_cpu)
        return trace

    def get_all_traces_handler(self, request):
        '''
        get_all_traces service handler
        '''
        response = GetAllTracesResponse()
        response.traces = self.trace_objs
        return response


if __name__ == "__main__":
    tserve = TraceServer()
    tserve.run()
