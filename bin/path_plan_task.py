#!/usr/bin/env python2
'''
Path planning task 
Setup: The user is presented with a gaussian mixture representing p(getting stuck), as well as a start and destination.
User traces a path between start and destination using mouse. 

User clicks and drags mouse to draw a path - no straight lines, etc
First click draws a straight line from start point to click point
On mouse up, path is complete

UI elements:
Button: Clear
Button: Done

Heat map: Inferno (or similar) color scaling

Steps: 
1. Given list of 2d gaussian params (x,y, sig_x, sig_y), compose a gaussian mixture
2. Normalize so that it's a probability map
3. Score by summing cells along user's path
4. Strictly, one shouldn't sum the p(haz) - since there's not necessarily an independence in a hazard map (there's correlations that aren't being considered)

Development:
1. Given a gaussian spec, draw into a GraphicsScene with a given discretization
a. It's a mesh evaluation - sample every Gaussian at each point and then sum to compose a heatmap

Scoring:
1. Count number of cells marked in the path (measure of length)

2. Sum the p(hazards)
3. Timing data



'''
import pdb

import sys
import csv
import math
import rospy
import signal
import time
import json

import numpy as np
from scipy import stats
from matplotlib.cm import inferno

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


from human_tasks.msg import *

#Main window area: Contains a QGraphicsView and some buttons
class PathPlanningWidget(QWidget):
    scoringComplete = pyqtSignal()
    reinitialize = pyqtSignal(float, float)
    onTelemetry = pyqtSignal(str, str)
    
    def __init__(self):
        super(QWidget, self).__init__()
        self.reinitialize.connect(self.initialize)
        #Create the layout once
        self.initUI()
        
    def initialize(self, means, covs, startPos, endPos):
        print 'Initializing UI'

        self.setVisible(True)
        self.planView.setHazards(means, covs)
        self.planView.setStart(startPos)
        self.planView.setEnd(endPos)
        self.planView.setDisabled(False)
        
        self.score = 0.0
        self.finalTime = None
        self.startTime = rospy.Time.now()
        self.update()
        self.telemetryTimer.start(100)
        self.planView.grabKeyboard()
        
    def initUI(self):

        self.setWindowTitle('Path Planning')
        mainLayout = QVBoxLayout()
        self.planView = PathPlanView(parent = self)
        self.w = 500
        self.h = 500
        
        mainLayout.addWidget(self.planView)
        self.btnDone = QPushButton('Get Score')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        self.btnDone.keyPressEvent = self.btnDone_keyPress
        
        mainLayout.addWidget(self.btnDone)

        self.setLayout(mainLayout)

        self.telemetryTimer = QTimer()
        self.telemetryTimer.timeout.connect(self.pushTelemetry)
        
    def pushTelemetry(self):
        self.onTelemetry.emit('PathPlanningWidget', self.makeStateJSON())
        
    def makeStateJSON(self):
        #Produce a Python dict, then encode to JSON

        state = dict()
        #state['crosshairPosition'] = (self.sampleView.crossHair.pos().x(),
        #                              self.sampleView.crossHair.pos().y())
        return json.dumps(state)
    
    def btnDone_keyPress(self, evt):
        if evt.key() == Qt.Key_Return:
            self.btnDone_onclick()
        
    def btnDone_onclick(self):
        if self.finalTime is None:
            self.planView.releaseKeyboard()
            self.getScore()
            elapsedTime = self.finalTime - self.startTime
            self.btnDone.setText('Score: %1.2f Time: %1.1f sec - Get Next Task' %
                                 (self.score, elapsedTime.to_sec()))
            self.telemetryTimer.stop()
        else:
            self.scoringComplete.emit()
            
            #Reset the button for next time
            self.btnDone.setText('Get Score')
            #self.parent().close()

    def getScore(self):
        print 'Calculating score'
        self.score = float(funnelCount) / sampleCount
        self.finalTime = rospy.Time.now()
        
        #print 'User earned a score of:', self.score
        
        self.sampleView.setDisabled(True)
        self.btnDone.setFocus(Qt.TabFocusReason)

        
class PathPlanView(QGraphicsView):
    def __init__(self, parent=None):
        super(PathPlanView, self).__init__(parent=parent)
        self._parent = parent

        self.setScene(QGraphicsScene())
        self._scene = self.scene()
        self.w = 100
        self.h = 100
        self.hazmapItems = None
        self.colors = inferno(np.linspace(0,1,255))
        self.pathPoints = np.zeros((self.h, self.w), dtype=bool)
        self.pathList = []
        
        #Draw some directions:
        directions = QGraphicsSimpleTextItem('Use arrow keys to move\nPress spacebar to dump')
        directions.setFont(QFont("SansSerif", max(self.h / 70.0,3), QFont.Bold))
        self._scene.addItem(directions)
        dirBounds = directions.boundingRect()
        directions.setPos(QPointF(self.w - dirBounds.width(), 0.0))

        #Define a discretization grid
        #Indexed by row, col
        self.p_haz = np.zeros((self.h, self.w))

    def setHazards(self, means, covs):
        #Eval the mesh for every gaussian to obtain a numpy array
        #Then, add graphics elements to the scene for each grid cell, appropriately colorized
        # https://stackoverflow.com/questions/47683415/evaluate-the-probability-density-of-gaussian-distribution-on-a-square-grid
       
        #lims = (int(-self.w/2), int(self.w/2)) # support of the PDF
        #pdb.set_trace()
        
        xx, yy = np.meshgrid(np.linspace(int(-self.w/2), int(self.w/2), self.w), np.linspace(int(-self.h/2), int(self.h/2), self.h))
        points = np.stack((xx, yy), axis=-1)

        self.hazmap = np.zeros((self.h, self.w))
        
        for i in range(0, len(means)):
            pdf = stats.multivariate_normal.pdf(points, means[i], covs[i])
            #pdb.set_trace()
            self.hazmap += pdf

        #Normalize the hazmap:
        total = np.sum(np.sum(self.hazmap))

        print 'Total of the hazmap:', total
        self.hazmap /= total
        self.drawHazmap()

    def drawHazmap(self):
        #Draw the final hazmap in the graphicsview:
        if not self.hazmapItems is None:
            for row in range(0, len(self.hazmapItems)):
                for col in range(0,len(self.hazmapItems[0])):
                    self._scene.removeItem(self.hazmapItems[row][col])
                    
        self.hazmapItems = []

        maxHaz = np.max(np.max(self.hazmap))

        for row in range(0, self.hazmap.shape[1]):
            self.hazmapItems.append([])
            for col in range(0, self.hazmap.shape[0]):
                
                cmapColor = self.colors[int(self.hazmap[row][col] / maxHaz * 254.0)]
                theBrush = QBrush(QColor(cmapColor[0] * 254.0,
                                         cmapColor[1] * 254.0,
                                         cmapColor[2] * 254.0))
                thePen = QPen(theBrush, 0.0)
                
                thisCell = QGraphicsRectItem(0, 0, 1, 1)
                thisCell.setBrush(theBrush)
                thisCell.setPen(thePen)
               
                self.hazmapItems[row].append(thisCell)
                self._scene.addItem(thisCell)

                #bounds = thisDir.boundingRect()
                #scale_w = scaledWidth / bounds.width()
                #scale_h = scaledHeight / bounds.height()
                #thisDir.setTransformOriginPoint(QPointF(bounds.width() / 2, bounds.height() / 2))

                
                #thisDir.setScale(scale_w)
                #bounds = thisDir.boundingRect()
                thisCell.setPos(QPointF(col, row))
       
        
        
    def setStart(self, startPos):
        #Draw something to indicate the start position
        pass

    def setEnd(self, endPos):
        #Draw something to indicate the end position
        pass
        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        scale = 1
        bounds = self._scene.sceneRect()
        if bounds:
            self._scene.setSceneRect(0, 0, self.w, self.h)
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            self.show()
            
    def keyPressEvent(self, event):
        currentPos = self.crossHair.pos()
        keyMove = 5
        if event.key() == Qt.Key_Up:
            self.crossHair.setPos(QPointF(currentPos.x(), currentPos.y() - keyMove))
        elif event.key() == Qt.Key_Down:
            self.crossHair.setPos(QPointF(currentPos.x(), currentPos.y() + keyMove))
        elif event.key() == Qt.Key_Left:
            self.crossHair.setPos(QPointF(currentPos.x()-keyMove, currentPos.y()))
        elif event.key() == Qt.Key_Right:
            self.crossHair.setPos(QPointF(currentPos.x()+keyMove, currentPos.y()))
        elif event.key() == Qt.Key_Space:
            self.parent().btnDone_onclick()

            
    #Ignore other keys...
    #Capture mousedown to start drawing the user's path
    def mouseMoveEvent(self, event):
        #print 'Mouse move:', self.mapToScene(event.x(),event.y())
        if self.drawPath:
            sceneCoords = self.mapToScene(event.x(),event.y())
            sceneX = int(sceneCoords.x()) 
            sceneY = int(sceneCoords.y())
            #print 'index:', sceneX, ',', sceneY
            #Bounds check
            if sceneX >= self.w or sceneY >= self.h:
                return
            if sceneX < 0 or sceneY < 0:
                return

            if len(self.pathList) == 0:
                #This is the first point  
                self.pathPoints[sceneY][sceneX] = True
                newCell = self.makePathItem(sceneX, sceneY)
                print 'Added primary point:', sceneX, ',', sceneY
                self._scene.addItem(newCell)
                self.pathList.append(newCell)
                
            #Check to make sure the path is smooth:
            if len(self.pathList) > 0:
                #We have at least one prior element that we need to link to
                lastPoint = self.pathList[-1]
                #print 'Last point X,Y:', lastPoint.pos().x(), ',', lastPoint.pos().y()

                #The problem is that a mouse movement may be too fast to have an 8-connected path
                deltaY = float(sceneY - lastPoint.pos().y())
                deltaX = float(sceneX - lastPoint.pos().x())

                lastX = int(lastPoint.pos().x())
                lastY = int(lastPoint.pos().y())
                
                #Number of iterations is the l2 norm
                fillDist = np.linalg.norm((deltaX, deltaY))
                if fillDist > np.sqrt(2):

                    print 'Distance was:', fillDist
                    print 'from :', lastX, ',', lastY, 'to ', sceneX, ',', sceneY 
                    incX = deltaX / fillDist
                    incY = deltaY / fillDist

                    for i in range(1, int(fillDist)+1):
                        #pdb.set_trace()
                        #Should start from the last point, and go to the current point:
                        
                        candX = int(lastX + i*incX)
                        candY = int(lastY + i*incY)
                        print 'Considering ', candX, ',', candY
                        #Is it already in the path?
                        if self.pathPoints[candY][candX]:
                            continue
                        
                        self.pathPoints[candY][candX] = True
                        print 'Added gap point:', candX, ',', candY

                        newCell = self.makePathItem(candX, candY)
                        self._scene.addItem(newCell)
                        self.pathList.append(newCell)
                else:
                    #There wasn't a gap to be filled
                    if not self.pathPoints[sceneY][sceneX]:
                        self.pathPoints[sceneY][sceneX] = True
                        newCell = self.makePathItem(sceneX, sceneY)
                        print 'Added primary point:', sceneX, ',', sceneY
                        self._scene.addItem(newCell)
                        self.pathList.append(newCell)
            
   
    def makePathItem(self, sceneX, sceneY):
        cmapColor = (0.0, 0.0, 1.0, 0.75)
        theBrush = QBrush(QColor(cmapColor[0] * 254.0,
                                 cmapColor[1] * 254.0,
                                 cmapColor[2] * 254.0,
                                 cmapColor[3] * 254.0))
        thePen = QPen(theBrush, 0.0)
        #         thePen.setBrush(theBrush)
        
        thisCell = QGraphicsRectItem(0, 0, 1, 1)
        thisCell.setBrush(theBrush)
        thisCell.setPen(thePen)
        thisCell.setPos(QPointF(sceneX, sceneY))
        thisCell.setZValue(10)
        return thisCell
    
    def mousePressEvent(self, event):
        print 'Mouse press'
        print event

        for theCell in self.pathList:
            self._scene.removeItem(theCell)
        self.pathList = []
        
        self.pathPoints = np.zeros((self.h, self.w), dtype=bool)
        self.drawPath = True
        
    def mouseReleaseEvent(self, event):
        print 'Mouse release'
        self.drawPath = False

    #def mousePressEvent(self, event):
    #    sceneCoords = self.mapToScene(event.pos())
    #    bounds = self.crossHair.boundingRect()
    #    self.crossHair.setPos(QPointF(sceneCoords.x()-bounds.width()/2, sceneCoords.y()-bounds.height()/2))

                
            
def main():
        if len(sys.argv) < 6:
                print 'Usage:', sys.argv[0], ' <x> <y> <start_x> <start_y> <end_x> <end_y>'
                print 'Please provide parameters'
                sys.exit(-1)
                
	app = QApplication(sys.argv)
        rospy.init_node('path_planning')

        #Compose the tuples for the widget from the cmd line args
        means = list()
        means.append((float(sys.argv[1]), float(sys.argv[2])))
        means.append((-float(sys.argv[1]), float(sys.argv[2])))

        covs = list()
        scale = 5.0
        covs.append(stats.wishart.rvs(df=2,scale=(scale,scale)))
        covs.append(stats.wishart.rvs(df=2,scale=(scale,scale)))
                  
        startPos = (int(sys.argv[3]), int(sys.argv[4]))
        endPos = (int(sys.argv[5]), int(sys.argv[6]))
        
	planApp = PathPlanningWidget()
        planApp.initialize(means, covs, startPos, endPos)
        mainWindow = QMainWindow()
        mainWindow.setWindowTitle('Path Planning')
        mainWindow.setCentralWidget(planApp)
        mainWindow.resize(planApp.w, planApp.h)
        mainWindow.setStatusBar(QStatusBar())
        planPub = rospy.Publisher('path_planning', Navigation, queue_size=10)
        
        startTime = rospy.Time.now()
        mainWindow.show()
        app.exec_()

        if not planApp.finalTime is None:
            totalTime = planApp.finalTime - startTime #a Duration
            print 'Publishing score:', planApp.score, ' duration:', totalTime.to_sec(), ' sec'

            #The Nav message is more or less what I want for this task
            msg = Navigation()
            msg.header.stamp = rospy.Time.now()
            
            msg.score = planApp.score
            msg.duration = totalTime.to_sec()

            planPub.publish(msg)
        
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
