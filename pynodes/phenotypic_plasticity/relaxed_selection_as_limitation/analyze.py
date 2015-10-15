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
        rospy.init("RSaL_analysis")

if __name__ == "__main__":
    anode = analysis_node()
