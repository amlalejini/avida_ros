#!/usr/bin/env python
import rospy, os, re
import cPickle as pickle
from avida_ros.msg import Genotype

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
        print self.genotype_dump
        print genotype_list
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
                print("G: Load from pickle")
                # Load from pickle!
                with open(os.path.join(self.genotype_dump, genotype + ".pickle")) as fp:
                    genotype_obj = pickle.load(fp)
                # Store it in genotype dict
                genotype_dict[trial_id][genotype_id] = genotype_obj
                genotype_objs.append(genotype_obj)
            else:
                print("G: Load from file!")
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
        return Genotype()

if __name__ == "__main__":
    gserver = GenotypeServer()
