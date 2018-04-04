#!/usr/bin/env python2

import math
import numpy as np
from scipy.spatial import ConvexHull


from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class QPolygonGenerator(object):
    def __init__(self):
        pass
    def getStandardPolygon(self, index):
        pass
    def getRandomPolygons(self, num, corners):
        #Generate num polygons with no less than corners per
        polys = []
        for index in range(0, num):
            curX = 0.0
            curY = 0.0
            thePoly = QPolygonF()

            #Draw random points, then let SciPy figure out the convex hull

            hullPts = np.random.normal(0, 10, (corners, 2))
            hull = ConvexHull(hullPts)

            #print 'Hull', hull.vertices
            for index in hull.vertices:
                thePoly.append(QPointF(hullPts[index,0], hullPts[index,1])) 

            polys.append(thePoly)

        return polys
    
if __name__ == '__main__':
    gen = QPolygonGenerator()
    polys = gen.getRandomPolygons(1, 4)
    for index, poly in enumerate(polys):
        print 'Polygon ', index, ':'
        for j in range(0, polys[index].count()):
            print polys[index].value(j).x(), ',', polys[index].value(j).y()

