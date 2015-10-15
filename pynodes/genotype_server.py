#!/usr/bin/env python
import rospy, os, re
import cPickle as pickle
from avida_ros.msg import Genotype
from avida_ros.srv import GetAllGenotypes, GetAllGenotypesResponse, GetAllGenotypesRequest

class GenotypeServer(object):
    '''
    GenotypeServer is a ROS node that runs as a server for Avida .gen files produced from avida analyze mode.
    '''
    def __init__(self):
        '''
        '''
        ##########################################
        # Instance variables for this class
        self.avida_processed_loc = None
        self.dataspace_loc = None
        self.genotype_dump = None
        self.genotype_dict = None           # Organized dictionary of genotype message objects extracted
        self.genotype_objs = None           # List of genotype message objects extracted
        self.get_all_genotypes_srv = None
        ##########################################
        # Initialize as ROS node
        rospy.init_node("GenotypeServer")
        ##########################################
        # Load relevant parameters
        # Get avida processed location
        self.avida_processed_loc = rospy.get_param("avida_processed", None)
        # Get avida ros dataspace location
        self.dataspace_loc = rospy.get_param("avida_ros_dataspace/base_loc", None)
        # Get genotype dump that we'll use to load/save genotypes in dataspace
        self.genotype_dump = rospy.get_param("avida_ros_dataspace/genotype_dump", None)
        # Force extraction?
        force_extraction = rospy.get_param("genotype_server/force_extraction", False)
        # Verify everything
        if not self._clean_params():
            rospy.logerr("Failed to clean parameters.")
            exit(-1)
        ##########################################
        # Load/Extract genotypes (populate genotype list, genotype dictionary, genotype_objs list)
        self._load_genotypes(force_extract = force_extraction)
        ##########################################
        # Setup services available to other ROS nodes
        self.get_all_genotypes_srv = rospy.Service("get_all_genotypes", GetAllGenotypes, self.get_all_genotypes_handler)

    def get_all_genotypes_handler(self, request):
        '''
        get_all_genotypes service handler
        '''
        response = GetAllGenotypesResponse()
        response.genotypes = self.genotype_objs
        return response

    def run(self):
        '''
        Genotype server run loop
        '''
        rospy.loginfo("Genotypes loaded successfully. Listening for requests.")
        rospy.spin()

    def _clean_params(self):
        '''
        Cleans parameters
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
            rospy.logerr("Failed to find dataspace location.")
            return False

        # Check dataspace/genotype dump directory
        if not self.genotype_dump:
            rospy.logerr("Failed to load genotype dump location parameter.")
            return False
        # Join with dataspace directory
        self.genotype_dump = os.path.join(self.dataspace_loc, self.genotype_dump)
        # See if already exists
        if not os.path.isdir(self.genotype_dump):
            rospy.loginfo("Failed to find genotype dump location.  Creating new one...")
            try:
                os.makedirs(self.genotype_dump)
            except:
                rospy.logerr("Failed to make genotype dump directory from params.")
                return False
            else:
                rospy.loginfo("Successfully created new genotype dump location directory.")

        return True

    def _load_genotypes(self, force_extract = False):
        '''
        This function first looks (recursively) for all .gen files in a given root directory.
        if not force_extract:
            check to see if pickle of genotype already exists
            if pickle exists: load it
            otherwise, generate new genotype object, save as pickle
        else
            generate new genotype object, save as pickle
        '''
        genotype_dict = {}
        genotype_objs = []
        # Get .gen locations from avida processed directory
        genotype_list = self._find_all_genotypes(self.avida_processed_loc)
        # Load them up and pickle 'em out
        for genotype in genotype_list:
            # Collect some information about our genotype
            try:
                gname_split = genotype.split("/")
                trial_id = gname_split[0]
                genotype_id = "/".join(gname_split[1:])
            except:
                rospy.logerr("Failed to collect trial id and genotype id info about: %s" % genotype)
                continue

            if trial_id not in genotype_dict: genotype_dict[trial_id] = {}
            if genotype_id not in genotype_dict[trial_id]: genotype_dict[trial_id][genotype_id] = None
            # Check to see if we already have a pickle of the genotype object
            pickled = os.path.exists(os.path.join(self.genotype_dump, genotype + ".pickle"))
            # If pickle exists and not force extract: load from pickle
            if pickled and not force_extract:
                # Load from pickle!
                with open(os.path.join(self.genotype_dump, genotype + ".pickle")) as fp:
                    genotype_obj = pickle.load(fp)
                # Store it in genotype dict
                genotype_dict[trial_id][genotype_id] = genotype_obj
                genotype_objs.append(genotype_obj)
            else:
                # Load from file!
                with open(os.path.join(self.avida_processed_loc, genotype)) as fp:
                    genotype_obj = self._extract_genotype(fp)
                    genotype_obj.trial_id = trial_id
                    genotype_obj.genotype_id = genotype_id
                # Store it
                genotype_dict[trial_id][genotype_id] = genotype_obj
                genotype_objs.append(genotype_obj)
                # Save it out
                if not os.path.exists(os.path.dirname(os.path.join(self.genotype_dump, genotype))): os.makedirs(os.path.dirname(os.path.join(self.genotype_dump, genotype)))
                pickle.dump(genotype_obj, open(os.path.join(self.genotype_dump, genotype) + ".pickle", "wb"))
            # Point instance variables to things we just made
            self.genotype_dict = genotype_dict
            self.genotype_objs = genotype_objs

    def _find_all_genotypes(self, targ_dir):
        '''
        This function returns a list of all genotypes
        '''
        gen_list = []
        for root, dirnames, filenames in os.walk(targ_dir):
            # Any genotype files?
            gnames = [g for g in filenames if ".gen" in g]
            for gname in gnames:
                # Found a genotype, append path (relative to target directory)
                gen_list.append(os.path.join(root[len(targ_dir) + 1:], gname))
        # gen list now contains all genotypes found in target directory (recursive)
        return gen_list

    def _extract_genotype(self, fp):
        '''
        Given file pointer: extract and return genotype message object
        '''
        msg = Genotype()
        ################################
        # Chunk file data for parsing
        content_chunk = ""
        task_chunk = ""
        genome_chunk = ""
        state = "content"
        for line in fp:
            if state == "content":
                # Transition?
                if "# Tasks Performed:" in line:
                    state = "tasks"
                    continue
                # No transition: add line to content chunk
                content_chunk += line
            elif state == "tasks":
                # Transition?
                if line.strip() == "":
                    state = "genome"
                    continue
                # No transition: add line to tasks chunk
                task_chunk += line
            elif state == "genome":
                if line.strip() == "": continue
                genome_chunk += line
        ################################
        # Parse Genome
        msg.genome = genome_chunk.strip().split("\n")
        ################################
        # Parse tasks and task counts
        m = re.findall(pattern = "#\s([a-zA-Z]+)\s([0-9]+)\s.*\n", string = task_chunk)
        msg.tasks = []
        msg.task_cnts = []
        for task in m:
            # Get task name
            msg.tasks.append(task[0])
            msg.task_cnts.append(int(task[1]))
        ################################
        # Parse Content Chunk (random crap at top of file)
        # Find generation
        m = re.search(pattern = "#\sGeneration\.*:\s(-?[0-9]+)\n", string = content_chunk)
        msg.generation = int(m.group(1))
        # Find merit
        m = re.search(pattern = "#\sMerit\.*:\s([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)", string = content_chunk)
        msg.merit = float(m.group(1))
        # Find Generation Length (Gestation Time)
        m = re.search(pattern = "#\sGestation\sTime\.*:\s([0-9]+)", string = content_chunk)
        msg.generation_length = int(m.group(1))
        # Find Fitness
        m = re.search(pattern = "#\sFitness\.*:\s([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)", string = content_chunk)
        msg.fitness = float(m.group(1))
        # Find errors
        m = re.search(pattern = "#\sErrors\.*:\s([0-9]+)", string = content_chunk)
        msg.errors = int(m.group(1))
        # Find genome size
        m = re.search(pattern = "#\sGenome\sSize\.*:\s([0-9]+)", string = content_chunk)
        msg.genome_size = int(m.group(1))
        # Find copied size
        m = re.search(pattern = "#\sCopied\sSize\.*:\s([0-9]+)", string = content_chunk)
        msg.copied_size = int(m.group(1))
        # Find executed size
        m = re.search(pattern = "#\sExecuted\sSize\.*:\s([0-9]+)", string = content_chunk)
        msg.executed_size = int(m.group(1))
        # Find offspring
        m = re.search(pattern = "#\sOffspring\.*:\s(.*)", string = content_chunk)
        msg.offspring = str(m.group(1))
        # Return the msg that we built
        return msg

if __name__ == "__main__":
    gserver = GenotypeServer()
    gserver.run()
