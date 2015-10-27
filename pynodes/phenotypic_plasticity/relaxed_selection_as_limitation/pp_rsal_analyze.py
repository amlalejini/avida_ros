#!/usr/bin/env python

import rospy, os, re, math
import cPickle as pickle
from avida_ros.srv import GetAllGenotypes, GetAllTraces
from avida_ros.srv import ShutdownRequest, ShutdownRequestResponse, ShutdownRequestRequest

'''
This module implements the analysis for the PhenotypicPlasticity/RelaxedSelectionAsLimitation Project
'''

# Some global constants (TURN THESE INTO ROS PARAMS)
ENV_BY_TREATMENT = {"treatment_1": ["nand+not-", "nand-not+"],
                    "treatment_2": ["nand+not~", "nand~not+"],
                    "treatment_3": ["nand+not+"]}
AVAILABLE_TASKS = ["not", "nand"]


class Experiment(object):
    '''
    This class describes the experiment.
    '''
    def __init__(self):
        # Descriptive information
        self.id = None
        self.desc = None
        # Treatments
        self.treatments = {} # Nand+Not-/Nand-Not+, Nand+Not~/Nand~Not~, Nand+Not+

class Treatment(object):
    '''
    This class describes a treatment analysis.
    '''
    def __init__(self):
        # Descriptive information
        self.id = None
        self.desc = None
        # Stats about trails
        self.total_plastic = None
        self.total_dynamic = None
        self.total_static = None
        # Trials of treatment
        self.trials = {}        # Trials by trial ID
        self.environments = []  # List of environments in treatment

    def __str__(self):
        stringy = ""
        stringy += "ID: %s\n" % self.id
        stringy += "Description: %s\n" % self.desc
        stringy += "Trials: %s\n" % str(self.trials)
        return stringy

class Trial(object):
    '''
    This class describes a trial analysis.
    '''
    def __init__(self):
        # Descriptive information
        self.id = None
        # Loaded trial characteristics
        self.traces = {}    # Traces by environment
        self.genotypes = {} # Genotypes by environment
        self.environments = [] # List of environments in trial
        # Found trial characteristics (relative to dominant)
        self.is_plastic = None  # Different tasks in different environments? (T/F)
        self.plasticity_type = None
        self.trace_deviation = None
        self.fitness = {}       # Fitness by environment
        self.log_fitness = {}   # Log Fitness by environment
        self.avg_incubator_log_fitness = None # Fitness avged across environments that the organism evolved in
        self.generation_length = {} # Generation length by environment
        self.task_distribution = {} # Task distribution by environment


class analysis_node(object):

    def __init__(self):
        ##########################################
        # Instance variables for this class
        self.dataspace_loc = None           # Location of dataspace
        self.analysis_dump = None           # Location of experiment structure (save file) relative to dataspace_loc
        self.experiment = None
        self.get_all_genotypes_services = {}
        self.get_all_traces_services = {}
        ##########################################
        # Initialize as ROS node
        rospy.init_node("RSaL_analysis")
        ##########################################
        # Load (most) parameters from ROS parameter server
        # TODO: implement verification of parameters?
        # Load up dataspace location
        self.dataspace_loc = rospy.get_param("analysis/dataspace/base_loc", None)
        # Load up analysis dump location relative to dataspace
        self.analysis_dump = rospy.get_param("analysis/dataspace/analysis_dump", None)
        # Force experiment rebuild?
        force_build = rospy.get_param("force_exp_rebuild", False)
        # Load up experiment parameters
        # General experiment description
        exp_desc = rospy.get_param("experiment/desc", None)
        # experiment ID
        exp_id = rospy.get_param("experiment/id", None)
        # Treatment list
        treatments = rospy.get_param("experiment/treatments", [])
        # Loop through treatments getting relevant information
        treatment_info = {}
        for treatment in treatments:
            treatment_id = rospy.get_param("experiment/%s/id" % treatment, None)
            treatment_desc = rospy.get_param("experiment/%s/desc" % treatment, None)
            treatment_info[treatment] = {"id": treatment_id, "desc": treatment_desc}
        ##########################################
        # Clean/verify some parameters
        if not self._clean_params():
            rospy.logerr("Failed to verify/clean parameters.")
            exit(-1)
        ##########################################
        # Setup services for each treatment
        pickled = os.path.exists(os.path.join(self.analysis_dump, "experiment.pickle"))
        for treatment in treatments:
            srv_name = "%s/get_all_traces" % treatment
            if force_build or not pickled: rospy.wait_for_service(srv_name)
            self.get_all_traces_services[treatment] = rospy.ServiceProxy(srv_name, GetAllTraces)
            srv_name = "%s/get_all_genotypes" % treatment
            if force_build or not pickled: rospy.wait_for_service(srv_name)
            self.get_all_genotypes_services[treatment] = rospy.ServiceProxy(srv_name, GetAllGenotypes)
        # Load/build experiment
        self._load_experiment(force_build, exp_desc, exp_id, treatments, treatment_info)

    def run(self):
        '''
        This is the analysis run function.
        Here is where we should output visualizations, etc. using the experiment data structure.
        '''


        summary = "" # This will keep track of human readable summary of data
        print(self.experiment.treatments.keys())
        for treatment_key in self.experiment.treatments:
            treatment = self.experiment.treatments[treatment_key]
            # Build our treatment overview output summary!
            summary += "###########################################################################\n"
            summary += "Treatment: %s\n" % treatment.id
            summary += "Treatment Description: %s\n" % treatment.desc
            summary += "Total Plastic: %d/%d\n" % (treatment.total_plastic, len(treatment.trials))
            summary += "Total Dynamic: %d/%d\n" % (treatment.total_dynamic, len(treatment.trials))
            summary += "Total Static: %d/%d\n" % (treatment.total_static, len(treatment.trials))
            for trial_key in treatment.trials:
                trial = treatment.trials[trial_key]
                # Build trial output summary
                summary += "=======================================================\n"
                summary += "Trial: %s\n" % trial.id
                summary += "Task Distribution: %s\n" % str(trial.task_distribution)
                summary += "Fitness Distribution: %s\n" % str(trial.fitness)
                summary += "Log Fitness Distribution: %s\n" % str(trial.log_fitness)
                summary += "Average Incubator Log Fitness: %f\n" % trial.avg_incubator_log_fitness
                summary += "Generation Length: %s\n" % str(trial.generation_length)
                summary += "Is Plastic: %s\n" % str(trial.is_plastic)
                summary += "Plasticity Type: %s\n" % trial.plasticity_type
        # Write summary out to file
        summary_loc = os.path.join(self.analysis_dump, "exp_summary.txt")
        with open(summary_loc, "w") as fp:
            fp.write(summary)
        print("Done.")

    def _clean_params(self):
        '''
        Calling this cleans/processes params loaded from parameter server.
        '''
        # Check avida dataspace directory
        if not self.dataspace_loc:
            rospy.logerr("Failed to load dataspace location parameter.")
            return False
        # Expand ~
        self.dataspace_loc = os.path.expanduser(self.dataspace_loc)
        # See if directory exists
        if not os.path.isdir(self.dataspace_loc):
            rospy.logerr("Failed to find dataspace directory.")
            return False

        # Check dataspace/analysis dump location
        if not self.analysis_dump:
            rospy.logerr("Failed to load analysis dump location parameter.")
            return False
        # Join with dataspace directory
        self.analysis_dump = os.path.join(self.dataspace_loc, self.analysis_dump)
        # See if already exists
        if not os.path.isdir(self.analysis_dump):
            rospy.loginfo("Failed to find analysis dump location. Creating new one.")
            try:
                os.makedirs(self.analysis_dump)
            except:
                rospy.logerr("Failed to make analysis dump.")
                return False
            else:
                rospy.loginfo("Successfully created new analysis dump location.")
        return True

    def _load_experiment(self, force_build = False, experiment_description = "", experiment_id = "", treatments = [], treatment_info = {}):
        '''
        Load experiment
        # if not force build and experiment pickle already exists:
        #  Load in that experiment pickle
        # else:
        #  Rebuild experiment from avida processed/trace server information
        '''
        # Check to see if a pickle already exists
        pickled = os.path.exists(os.path.join(self.analysis_dump, "experiment.pickle"))
        if not force_build and pickled:
            print("Load from pickle")
            # Load from pickle
            with open(os.path.join(self.analysis_dump, "experiment.pickle")) as fp:
                self.experiment = pickle.load(fp)
        else:
            # Extract from servers
            print("Load from servers.")

            ##########################################
            # Made new experiment struct
            experiment = Experiment()
            # Populate descriptive fields
            experiment.id = experiment_id
            experiment.desc = experiment_description
            # Populate treatments with treatments
            # Also, get all traces and genotypes (by treatment) from servers
            for treatment in treatments:
                # trace_shutdown_srv = rospy.ServiceProxy("%s/shutdown_trace_server" % treatment, ShutdownRequest)
                # genotype_shutdown_srv = rospy.ServiceProxy("%s/shutdown_genotype_server" % treatment, ShutdownRequest)

                treatment_id = treatment_info[treatment]["id"]
                treatment_desc = treatment_info[treatment]["desc"]
                # Ask servers for all traces/genotypes for this treatment
                traces = self.get_all_traces_services[treatment]().traces
                #trace_shutdown_srv("pp rsal")
                genotypes = self.get_all_genotypes_services[treatment]().genotypes
                #genotype_shutdown_srv("pp rsal")

                # Build treatment
                experiment.treatments[treatment] = self._build_treatment(treatment_id, treatment_desc, traces, genotypes)
                #pickle.dump(self._build_treatment(treatment_id, treatment_desc, traces, genotypes), open(os.path.join(self.analysis_dump, "%s.pickle" % treatment_id), "wb"))
            # Save out experiment as pickle
            self.experiment = experiment
            #pickle.dump(experiment, open(os.path.join(self.analysis_dump, "experiment.pickle"), "wb"))

    def _build_treatment(self, id, description, traces, genotypes):
        '''
        Given treatment ID, description, list of traces, and list of genotypes, build and return a treatment struct
        '''
        treatment = Treatment()
        # Populate ID field
        treatment.id = id
        # Populate description field
        treatment_desc = description
        # Populate available environments
        treatment.environments = ENV_BY_TREATMENT[id]
        # Separate traces by trial
        traces_by_trial = {}
        for trace in traces:
            trial_id = trace.trial_id
            if trial_id not in traces_by_trial: traces_by_trial[trial_id] = []
            traces_by_trial[trial_id].append(trace)
        # Separate genotypes by trial
        genotypes_by_trial = {}
        for genotype in genotypes:
            trial_id = genotype.trial_id
            if trial_id not in genotypes_by_trial: genotypes_by_trial[trial_id] = []
            genotypes_by_trial[trial_id].append(genotype)
        # for each trial...build trial (where to get trial list)
        # Make sure genotype trials and trace trials match (not actually doing any verification here...)
        trials = list(set(genotypes_by_trial.keys() + traces_by_trial.keys()))
        plasticity_cnt = 0
        static_cnt = 0
        dynamic_cnt = 0
        for trial in trials:
            treatment.trials[trial] = self._build_trial(trial_id = trial, treatment_id = treatment.id, environments = treatment.environments, genotypes = genotypes_by_trial[trial], traces = traces_by_trial[trial])
            # Update plasticity counts
            if treatment.trials[trial].is_plastic:
                 plasticity_cnt += 1
                 if treatment.trials[trial].plasticity_type == "STATIC": static_cnt += 1
                 if treatment.trials[trial].plasticity_type == "DYNAMIC": dynamic_cnt += 1
        # Populate treatment stats
        treatment.total_plastic = plasticity_cnt
        treatment.total_static = static_cnt
        treatment.total_dynamic = dynamic_cnt

        return treatment

    def _build_trial(self, trial_id, treatment_id, environments, genotypes, traces):
        '''
        Given list of genotypes and list of traces, this function builds and returns a trial struct
        '''
        # FOR THIS, I'm restricting it to one genotype/trace per environment
        trial = Trial()
        # Populate trial id
        trial.id = trial_id
        # Populate environments
        trial.environments = environments
        # Populate traces by environment
        traces_by_env = {}
        for trace in traces:
            # Environment?
            env = trace.env_id.split("_")[1]
            if not env in traces_by_env: traces_by_env[env] = []
            traces_by_env[env] = trace
        trial.traces = traces_by_env
        # Populate genotypes by environment
        genotypes_by_env = {}
        for genotype in genotypes:
            # Environment?
            env = genotype.genotype_id.split("_")[-1].split(".")[0]
            genotypes_by_env[env] = genotype
        trial.genotypes = genotypes_by_env
        # Populate trial metrics
        # - is_plastic, plasticity_type, fitness by env, avg fitness across environments dorg evolved in
        # - generation length
        # is plastic? and if so, where is the deviation?
        if treatment_id == "treatment_1":
            # treatment 1
            trial.is_plastic = self.is_plastic(genotypes_by_env["nand+not-"], genotypes_by_env["nand-not+"])
        elif treatment_id == "treatment_2":
            # treatment 2
            trial.is_plastic = self.is_plastic(genotypes_by_env["nand+not~"], genotypes_by_env["nand~not+"])
        else:
            # if treatment 3, no plasticity
            trial.is_plastic = None
        # plasticity type?
        if trial.is_plastic and treatment_id == "treatment_1":
            plasticity_type = self.get_plasticity_type(traces_by_env["nand+not-"], traces_by_env["nand-not+"])
        elif trial.is_plastic and treatment_id == "treatment_2":
            plasticity_type = self.get_plasticity_type(traces_by_env["nand+not~"], traces_by_env["nand~not+"])
        else:
            plasticity_type = (None, None)
        trial.plasticity_type = plasticity_type[0]
        trial.trace_deviation = plasticity_type[1]
        # Fitness by environment & generation length by environment & task distribution by environment
        for env in genotypes_by_env:
            trial.fitness[env] = genotypes_by_env[env].fitness
            trial.log_fitness[env] = 0 if trial.fitness[env] == 0 else math.log(trial.fitness[env], 2)
            trial.generation_length[env] = genotypes_by_env[env].generation_length
            trial.task_distribution[env] = {genotypes_by_env[env].tasks[i]: genotypes_by_env[env].task_cnts[i] for i in xrange(0, len(genotypes_by_env[env].tasks))}
        # Avg fitness across incubator environments
        if treatment_id == "treatment_1":
            trial.avg_incubator_log_fitness = 0.5 * (trial.log_fitness["nand+not-"] + trial.log_fitness["nand-not+"])
        elif treatment_id == "treatment_2":
            trial.avg_incubator_log_fitness = 0.5 * (trial.log_fitness["nand+not~"] + trial.log_fitness["nand~not+"])
        elif treatment_id == "treatment_3":
            trial.avg_incubator_log_fitness = trial.log_fitness["nand+not+"]
        # print("=============================")
        # print("BUILDING TRIAL")
        # print("Trial ID: " + str(trial.id))
        # print("Trial Envs: " + str(trial.environments))
        # print("Task distribution: " + str(trial.task_distribution))
        # print("Plastic? " + str(trial.is_plastic))
        # print("Plasticity Type: " + str(trial.plasticity_type))
        # print("Trace Deviation: " + str(trial.trace_deviation))
        # print("Fitness by env: " + str(trial.fitness))
        # print("Avg Incubator fitness: " + str(trial.avg_incubator_fitness))
        # print("Generation Length by env: " + str(trial.generation_length))
        return trial

    def is_plastic(self, genotype1, genotype2):
        '''
        Given two genotypes (expected that the are from the same genome executed in different environments), returns true if plastic, false if not
        Phenotypic Plasticity -- Dorg performs different distribution of tasks in different environments
        '''
        g1_task_dist = {genotype1.tasks[i]: genotype1.task_cnts[i] for i in xrange(0, len(genotype1.tasks))}
        g2_task_dist = {genotype2.tasks[i]: genotype2.task_cnts[i] for i in xrange(0, len(genotype2.tasks))}
        for task in AVAILABLE_TASKS:
            if g1_task_dist[task] != g2_task_dist[task]:
                return True
        return False

    def get_plasticity_type(self, trace1, trace2):
        '''
        Given two traces, determine if they are statically plastic, or dynamically plastic
        '''
        #trace.dorgs[trace_loc].instr_head
        end_signal = "!END!"
        trace_loc = 0
        while True:
            current_ips = []
            if trace_loc >= len(trace1.dorgs):
                # Trace is over
                current_ips.append(end_signal)
            else:
                current_ips.append(trace1.dorgs[trace_loc].instr_head)

            if trace_loc >= len(trace2.dorgs):
                # Trace is over
                current_ips.append(end_signal)
            else:
                current_ips.append(trace2.dorgs[trace_loc].instr_head)

            # Check current ips for deviation
            num_ip_locs = len(set(current_ips))
            if num_ip_locs > 1:
                return ("DYNAMIC", trace_loc)
            elif "!END!" in current_ips:
                return ("STATIC", trace_loc)
            else:
                trace_loc += 1


if __name__ == "__main__":
    anode = analysis_node()
    anode.run()
