#!/usr/bin/python2
'''
Generate a pickle of nxm parameters, of n scenarios for m users. 
The key is that individual users will have different orderings of parameter values for the attention and workload tasks
This pickle is loaded by the master control node, which calls services in the UI to manipulate the parameters as needed
This is in contrast to the scenarios.csv file, which is the same for every subject

'''

import pickle
from taskEnums import *
import numpy as np
import sys

import pdb

blockSize = 5 #how many assignment events define a parameter block
pickleFile = 'subject_params.pkl'

def makeParamList(numTasks):
    #Return a randomized difficulty level for the number of tasks
    #Given the size of the block

    paramList = list()
    paramList.append(TaskDifficulty.Easy) #start with easy
    
    for index in range(1, int(numTasks / blockSize)):    
        paramList.append(np.random.choice(tuple(TaskDifficulty)))
    return paramList
        
    
def main():
    if len(sys.argv) < 3:
        print 'Usage: ', sys.argv[0], ' <number of tasks> <number of subjects>'
        sys.exit(0)

    numTasks = int(sys.argv[1])
    numSubjects = int(sys.argv[2])
    
    output = dict()
    #First, define how many tasks form a block
    output['tasksPerBlock'] = blockSize
    output['numTasks'] = numTasks
    
    attentionParams = list()
    workloadParams = list()
    
    for index in range(0, numSubjects):
        attentionParams.append(makeParamList(numTasks))
        workloadParams.append(makeParamList(numTasks))

    output['attentionParams'] = attentionParams
    output['workloadParams'] = workloadParams

    outFile = open(pickleFile, 'wb')
    pickle.dump(output, outFile)
    outFile.close()
    print 'Param file generated in: ', pickleFile 
    
if __name__ == "__main__":
    """If this script is run as stand alone then call main() function."""
    main();
