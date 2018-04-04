#!/usr/bin/env python2

import sys
import csv
import math
import rospy
import signal
import time

import numpy as np


from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

from QDirection import *
from QCrosshair import *

from human_tasks.msg import *

#Main window area: Contains a QGraphicsView and some buttons
class SampleHandlingWidget(QWidget):
    def __init__(self, windDir, windVel):
        super(QWidget, self).__init__()
        self.windDir = windDir
        self.windVel = windVel
        self.initUI()

    def initUI(self):
        rospy.init_node('sample_handling')
        self.setWindowTitle('Sample Handling')
        mainLayout = QVBoxLayout()
        self.sampleView = SampleView(parent = self)
        self.w = 500
        self.h = 500
        
        mainLayout.addWidget(self.sampleView)
        self.btnDone = QPushButton('Done')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        self.btnDone.keyPressEvent = self.btnDone_keyPress
        
        mainLayout.addWidget(self.btnDone)

        self.setLayout(mainLayout)
        self.sampleView.setWindParams(self.windDir+90, self.windVel)
        self.score = 0.0
        self.finalTime = None
        
    def btnDone_keyPress(self, evt):
        if evt.key() == Qt.Key_Return:
            self.btnDone_onclick()
        
    def btnDone_onclick(self):
        if self.finalTime is None:
            self.drawSamples()
        else:
            self.parent().close()

    def drawSamples(self):
        print 'Calculating score'

        #Corrupt the wind direction and velocity:
        windNoise = np.random.normal(0.0, 1.0, 2)
        windDirNoisy = self.windDir + windNoise[0]
        windVelNoisy  = max(self.windVel + windNoise[1],0.1)

        self.windDirNoisy = windDirNoisy
        self.windVelNoisy = windVelNoisy
        
        baseVar = 10.0
        sampleCount = 2000
        onAxis = np.random.normal(windVelNoisy*10.0, windVelNoisy*4.0 + baseVar, sampleCount)
        lateralAxis = np.random.normal(0.0, baseVar, sampleCount)

        #Rotate first
        rotatedX = np.cos(self.windDir/180.0*math.pi) * onAxis - np.sin(self.windDir/180.0*math.pi)* lateralAxis 
        rotatedY = np.cos(self.windDir/180.0*math.pi) * lateralAxis + np.sin(self.windDir/180.0*math.pi)* onAxis 

        #Translate according to the crosshair pos
        sampleCoords = zip(rotatedX, rotatedY)
        self.sampleView.removeSamples()
        
        self.sampleView.addSamples(sampleCoords)

        #Score the samples, then color good samples green
        funnelCount = self.sampleView.scoreSamples()
        self.score = float(funnelCount) / sampleCount
        self.finalTime = rospy.Time.now()
        
        print 'User earned a score of:', self.score
        self.parent().statusBar().showMessage('Earned score: %d%%' % (self.score*100))

        '''
        mbox = QMessageBox()
        mbox.setWindowTitle('Sample Handling')
        mbox.setText('Task complete!')
        mbox.setInformativeText('Earned score: %d%%' % (self.score*100))
        mbox.setStandardButtons(QMessageBox.Ok)
        mbox.setIcon(QMessageBox.Information)
        mbox.exec_()
        '''
        
        self.sampleView.setDisabled(True)
        self.btnDone.setFocus(Qt.TabFocusReason)
        
class SampleView(QGraphicsView):
    def __init__(self, parent=None):
        super(SampleView, self).__init__(parent=parent)
        self._parent = parent

        self.setScene(QGraphicsScene())
        self._scene = self.scene()
        self.w = 500
        self.h = 500
        self.windDir = QDirection(color=Qt.red, lineColor=Qt.darkRed)
        self._scene.addItem(self.windDir)
        dirBounds = self.windDir.boundingRect()
        self.windDir.setScale(1.0)
        self.windDir.setPos(QPointF(0,0))
        #self.windDir.setTransformOriginPoint(QPointF(dirBounds.width() / 2, dirBounds.height() / 2))

        #Draw a box around the wind direction:
        self.windBox = self._scene.addRect(0,0,100,100, QPen(Qt.black), QBrush(Qt.transparent))
        

        self.windVel = QGraphicsSimpleTextItem('Speed')
        
        self.windVel.setFont(QFont("SansSerif", max(self.h / 60.0,3), QFont.Bold))
        self._scene.addItem(self.windVel)
        velBounds = self.windVel.boundingRect()

        #center the label
        self.windVel.setPos(QPointF(dirBounds.left() + dirBounds.width()/2 - velBounds.width()/2, dirBounds.top()+dirBounds.height()))

        #Draw some directions:
        directions = QGraphicsSimpleTextItem('Use arrow keys to move\nPress spacebar to dump')
        directions.setFont(QFont("SansSerif", max(self.h / 70.0,3), QFont.Bold))
        self._scene.addItem(directions)
        dirBounds = directions.boundingRect()
        
        directions.setPos(QPointF(self.w - dirBounds.width(), 0.0))
        
        #Draw the funnel
        theColor = QColor(Qt.gray)
        theColor.setAlpha(80)
        thePen = QPen(theColor)
        thePen.setWidthF(5)
        theBrush = QBrush(theColor)
        theBrush.setStyle(Qt.SolidPattern)

        self.funnel = self._scene.addEllipse(0.0, 0.0, 100.0, 100.0, thePen, theBrush)
        bounds = self.funnel.boundingRect()
        self.funnel.setPos(QPointF(self.w/2-bounds.width()/2, self.h/2-bounds.height()/2))


        #Make a crosshair
        self.crossHair = QCrosshair(color=Qt.red, thickness=5)
        self._scene.addItem(self.crossHair)
        bounds = self.crossHair.boundingRect()
        self.crossHair.setPos(QPointF(self.w/2-bounds.width()/2, self.h/2-bounds.height()/2))

        self.samples = []
        
    def removeSamples(self):
        for pnt in self.samples:
            curScene = pnt.scene()
            if not curScene is None:
                curScene.removeItem(pnt)
            
    def addSamples(self, sampleCoords):
        theColor = QColor(Qt.blue)
        theColor.setAlpha(80)
        thePen = QPen(theColor)
        thePen.setWidthF(2)
        theBrush = QBrush(theColor)
        theBrush.setStyle(Qt.SolidPattern)

        crossPos = self.crossHair.pos()
        crossBounds = self.crossHair.boundingRect()
        
        for theSample in sampleCoords:
            pnt = self._scene.addEllipse(-1.0, -1.0, 2.0, 2.0, thePen, theBrush)

            #bake in the translation from the crosshair
            pnt.setPos(QPointF(theSample[0]+crossPos.x()+crossBounds.width()/2, theSample[1]+crossPos.y()+crossBounds.height()/2))
            self.samples.append(pnt)

        print 'Samples added:', len(sampleCoords)

    def scoreSamples(self):
        #Figure out how many samples intersect the funnel:
        funnelSamples = self._scene.collidingItems(self.funnel) #, mode=Qt.ContainsItemShape)

        print 'Intersecting samples:', len(funnelSamples)
        sampleCount = 0
        for pnt in funnelSamples:
            if type(pnt) is QGraphicsEllipseItem:
                thePen = QPen(Qt.green)
                thePen.setWidthF(2)
                pnt.setPen(thePen)
                theBrush = QBrush(Qt.green)
                theBrush.setStyle(Qt.SolidPattern)
                pnt.setBrush(theBrush)
                sampleCount += 1
                
        return sampleCount
    
    def setWindParams(self, windDir, windVel):
        self.windDir.setRotation(windDir)
        self.windVel.setText('Speed: %d' % windVel)
        velBounds = self.windVel.boundingRect()
        dirBounds = self.windDir.boundingRect()
        #center the label
        self.windVel.setPos(QPointF(dirBounds.left() + dirBounds.width()/2 - velBounds.width()/2, dirBounds.top()+dirBounds.height()))
        
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
            self.parent().drawSamples()

            
        #Ignore other keys...
            
    def mousePressEvent(self, event):
        sceneCoords = self.mapToScene(event.pos())
        bounds = self.crossHair.boundingRect()
        self.crossHair.setPos(QPointF(sceneCoords.x()-bounds.width()/2, sceneCoords.y()-bounds.height()/2))

                
            
def main():
        if len(sys.argv) < 3:
                print 'Usage:', sys.argv[0], ' <wind dir> <wind vel>'
                print 'Please provide wind direction and speed'
                sys.exit(-1)
                
	app = QApplication(sys.argv)
	sampleApp = SampleHandlingWidget(int(sys.argv[1]), float(sys.argv[2]))

        mainWindow = QMainWindow()
        mainWindow.setWindowTitle('Sample Handling')
        mainWindow.setCentralWidget(sampleApp)
        mainWindow.resize(sampleApp.w, sampleApp.h)
        mainWindow.setStatusBar(QStatusBar())
        samplePub = rospy.Publisher('handling', SampleHandling, queue_size=10)
        
        startTime = rospy.Time.now()
        mainWindow.show()
        app.exec_()

        if not sampleApp.finalTime is None:
            totalTime = sampleApp.finalTime - startTime #a Duration
            print 'Publishing score:', sampleApp.score, ' duration:', totalTime.to_sec(), ' sec'
            
            msg = SampleHandling()
            msg.header.stamp = rospy.Time.now()
            msg.windDir = sampleApp.windDirNoisy
            msg.windVel = sampleApp.windVelNoisy
            msg.score = sampleApp.score
            msg.duration = totalTime.to_sec()

            samplePub.publish(msg)
        
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
