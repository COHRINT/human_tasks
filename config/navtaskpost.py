#!/usr/bin/python2
'''
Fix coordinate system
'''
import sys
import csv
import numpy as np


outFileName = 'navtasks_full2.csv'

def main():
	out = file('navtasks_all2.csv', 'w')
	for line in file('navtasks_all.csv', 'r'):
    		current = line.split(",")
		float(current[0])
    	
if __name__ == '__main__':
        main()