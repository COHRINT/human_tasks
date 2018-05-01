#!/usr/bin/python2
'''
Fix coordinate system - translate scenegraph coordinates (image coordinates) to gazebo coords (cartesian)
'''
import sys
import csv
import numpy as np
from copy import deepcopy
import pdb


outFileName = 'navtasks_full2.csv'
def makeGazeboCoords(demWidth, inRow):
        outRow = deepcopy(inRow)

        #Translate the traverse origin (elements 1,2)
        outRow[1] = float(inRow[1]) - demWidth / 2.0 
        outRow[2] = demWidth / 2.0 - float(inRow[2])

        #Translate the goal (elements 4,5)
        outRow[4] = float(inRow[4]) - demWidth / 2.0
        outRow[5] = demWidth / 2.0 - float(inRow[5])
        return outRow

def main():
        if len(sys.argv) < 2:
                print 'Usage:', sys.argv[0], ' <dem width in pixels> <infile.csv>'
                sys.exit(0)

        demWidth = int(sys.argv[1])
        inFileName = sys.argv[2]
        inFilePieces = inFileName.split('.')
        print inFilePieces
        
        outFileNameBase = ''.join(inFilePieces[0:-1])
        outFile = open(outFileNameBase + '_gazebo.csv', 'wb')
 
        inFile = open(inFileName, 'rb')
        reader = csv.reader(inFile)
        writer = csv.writer(outFile, delimiter=',')
        
        for row in reader:
                outRow = makeGazeboCoords(demWidth, row)
                #pdb.set_trace()
                writer.writerow(outRow)

        inFile.close()
        outFile.close()
        
    	
if __name__ == '__main__':
        main()
