#!/usr/bin/python2
'''
Generate sample handling tasks:
Pull wind samples uniformly from 
[0-5]: Easy
[5-15]: Medium
[15-25]: Hard

Directions get sampled uniformly from [0-360]

'''
import sys
import pickle
import csv
import numpy as np

from taskEnums import *

outFileName = 'handling_tasks.csv'

def getSample():
    #Draw three samples to define this task:
    #Choose a difficulty:
    taskDiff = np.random.choice([TaskDifficulty.Easy,
                                 TaskDifficulty.Medium,
                                 TaskDifficulty.Hard])

    #Choose a velocity:
    vel = 0.0
    if taskDiff == TaskDifficulty.Easy:
        vel = np.random.uniform(low=0.0, high=5.0)
    elif taskDiff == TaskDifficulty.Medium:
        vel = np.random.uniform(low=5.0, high=15.0)
    elif taskDiff == TaskDifficulty.Hard:
        vel = np.random.uniform(low=15.0, high=25.0)
    else:
        print 'Unknown difficulty:', taskDiff.value
        sys.exit(0)

    #Choose a direction:
    windDir = np.random.uniform(low=0.0, high=360.0)

    return (taskDiff.value, windDir, vel)

def main():
    if len(sys.argv) < 2:
        print 'Usage: ', sys.argv[0], ' <#samples>'
        print 'Writes to:', outFileName
        sys.exit(0)

    numSamples = int(sys.argv[1])
    outFile = open(outFileName, 'wb')
    writer = csv.writer(outFile, delimiter=',')
    
    for index in range(0, numSamples):
        sampRow = getSample()
        writer.writerow(sampRow)

    outFile.close()
    
    
if __name__ == '__main__':
        main()
