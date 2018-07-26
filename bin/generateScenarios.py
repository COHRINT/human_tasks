#!/usr/bin/python2
'''
Generate scenarios by sampling uniformly over TaskEnums.TaskTypes

'''
import sys
import pickle
import csv
import numpy as np
import pdb

from taskEnums import *

outFileName = 'scenarios.csv'

def lineCount(filename):
    lines = 0
    for line in open(filename):
        lines += 1
    return lines

def getTaskType():
    #Draw three samples to define this task:
    #Choose a difficulty:
    taskType = np.random.choice([TaskType.Navigation,
                                 TaskType.Grasping,
                                 TaskType.Handling])
    return taskType

def main():
    if len(sys.argv) < 5:
        print 'Usage: ', sys.argv[0], ' <#samples> <navtasks.csv> <grasptasks.csv> <handlingtasks.csv>'
        print 'Writes to:', outFileName
        sys.exit(0)

    numSamples = int(sys.argv[1])
    numNav = lineCount(sys.argv[2])
    numGrasp = lineCount(sys.argv[3])
    numHandling = lineCount(sys.argv[4])
    
    outFile = open(outFileName, 'wb')
    writer = csv.writer(outFile, delimiter=',')

    navTasks = list(range(0, numNav))

    graspTasks = list(range(0, numGrasp))

    handlingTasks = list(range(0, numHandling))
    availableTasks = [navTasks, graspTasks, handlingTasks] #order by the enum value


    for index in range(0, numSamples):

        #Assemble the potential tasks we have left:
        
        availableTypes = []
        for task in TaskType:
            if len(availableTasks[task.value]) > 0:
                availableTypes.append(task)

        #Randomly choose what's next
        if len(availableTypes) == 0:
            print  'After ', index, ', we ran out of tasks....'
            break
        
        task = np.random.choice(availableTypes)

        #Proceed with the scenarios - makeno assumptions about whether
        #there is randomness already built into the scenario definitions...
        
        selIndex = np.random.choice(availableTasks[task.value])
        #index when you just want linear iteration
        
        
        availableTasks[task.value].remove(selIndex) #sample without replacement

        #pdb.set_trace()
            
        sampRow = (task.value, selIndex)
        writer.writerow(sampRow)

    outFile.close()
    
    
if __name__ == '__main__':
        main()
