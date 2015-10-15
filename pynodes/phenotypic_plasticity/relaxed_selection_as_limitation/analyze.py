#!/usr/bin/env python

import rospy

'''
This module implements the analysis for the PhenotypicPlasticity/RelaxedSelectionAsLimitation Project
'''
class Experiment(object):
    '''
    This class describes the experiment.
    '''
    def __init__(self):
        # Descriptive information
        self.exp_id = None
        self.exp_desc = None
        # Treatments
        self.treatments = {} # Nand+Not-/Nand-Not+, Nand+Not~/Nand~Not~, Nand+Not+

class Treatment(object):
    '''
    This class describes a treatment analysis.
    '''
    def __init__(self):
        # Descriptive information
        self.treatment_id = None
        self.treatment_desc = None
        # Stats about trails
        self.total_plastic = None
        self.total_dynamic = None
        self.total_static = None
        # Trials of treatment
        self.trials = []

class Trial(object):
    '''
    This class describes a trial analysis.
    '''
    def __init__(self):
        # Descriptive information
        self.trial_id = None
        # Trial characteristics (relative to dominant)
        self.environments = []
        self.is_plastic = None  # Different tasks in different environments? (T/F)
        self.plasticity_type = None
        self.fitness = {}       # Fitness by environment
        self.avg_fitness = None # Fitness avged across environments
        self.generation_length = {} # Generation length by environment



class analysis_node(object):

    def __init__(self):
        ##########################################
        # Instance variables for this class
        # TODO: at some point, I want analysis scripts to be totally ignorant of file paths; they should just be working with datastructures
        #   - something upstream should handle all of the actual data parsing/extraction
        self.t1_avida_processed_loc = None  # Location of data from avida analyze mode
        self.t2_avida_processed_loc = None
        self.t3_avida_processed_loc = None
        self.dataspace_loc = None           # Location of dataspace
        self.analysis_dump = None           # Location of experiment structure (save file) relative to dataspace_loc
        self.treatments = None              # List of treatments
        self.experiment = None
        ##########################################
        # Initialize as ROS node
        rospy.init_node("RSaL_analysis")
        ##########################################
        # Load (most) parameters from ROS parameter server
        # TODO: implement verification of parameters?
        # Load up the avida processed locations
        self.t1_avida_processed_loc = rospy.get_param("treatment_1/avida_processed", None)
        self.t2_avida_processed_loc = rospy.get_param("treatment_2/avida_processed", None)
        self.t3_avida_processed_loc = rospy.get_param("treatment_2/avida_processed", None)
        # Load up dataspace location
        self.dataspace_loc = rospy.get_param("analysis/dataspace/base_loc", None)
        # Load up analysis dump location relative to dataspace
        self.analysis_dump = rospy.get_param("analysis/dataspace/analysis_dump", None)
        # Force experiment rebuild?
        force_build = rospy.get_param("force_exp_rebuild", False)
        # Load up experiment parameters
        # General experiment description
        exp_desc = rospy.get_param("experiment/desc", None)
        # Treatment list
        self.treatments = rospy.get_param("experiment/treatments", [])
        ##########################################
        # Load/build experiment
        self._load_experiment(force_build = force_build)

    def _load_experiment(self, force_build = False):
        '''
        Load experiment
        '''
        # if not force build and experiment pickle already exists:
        #  Load in that experiment pickle
        # else:
        #  Rebuild experiment from avida processed/trace server information







if __name__ == "__main__":
    anode = analysis_node()
