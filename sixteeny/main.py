import numpy as np
# import cupy as cp
import matplotlib.pyplot as plt

from sixteeny.utils.tracker import ArenaTracker

import sys
import os
import json
import time

if __name__ == '__main__':
    # Get project directory and name from command line arguments
    if len(sys.argv) > 2:
        project_directory = sys.argv[1]
        experiment_name = sys.argv[2]
    else:
        # send error message
        print('Please specify experiment directory and name as command line arguments.')
        sys.exit(1)
    
    # ensure that project directory exists
    if project_directory[-1] != '/':
        project_directory += '/'
    if not os.path.isdir(project_directory):
        print('Project directory does not exist.')
        sys.exit(1)

    # ensure that experiment name directory exists
    if not os.path.isdir(project_directory + experiment_name):
        print('Experiment directory does not exist.')
        sys.exit(1)

    
    
    


    

    
    