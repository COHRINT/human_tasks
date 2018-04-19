#!/usr/bin/python2
import sys

import csv
import math
import rospy
import signal
import time

import numpy as np
from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *

from human_tasks.msg import *
from human_tasks.srv import *

from QFuelGauge import *
from QArrow import *
                
class TankWidget(QWidget):
    def __init__(self, parent=None):
        super(TankWidget, self).__init__(parent)
        mainLayout = QVBoxLayout()
        self.tankView = TankView(parent=self)
        mainLayout.addWidget(self.tankView)
        self.setLayout(mainLayout)

        #Difficulty setting: change the probability of the valve closing on its own
        self.pClose = 0.05

        self.ticksHigh = 0
        self.ticksLow = 0
        self.tickCount = 0
        self.tankView.fuelLow.connect(self.onFuelLow)
        self.tankView.fuelHigh.connect(self.onFuelHigh)

        self.paramsSrv = rospy.Service('~SetWorkloadParams', SetWorkloadParams, self.setWorkloadParams)
        
        self.timer = QTimer(self)
        map(self.timer.timeout.connect, [self.tankView.updateFuel, self.updateTicks])
        self.start()

    def setWorkloadParams(self, msg):
        #Set the prob of closing the valve to the given value
        self.pClose = msg.pClose

        #Produce a report for the return
        resp = SetWorkloadParamsResponse()
        resp.totalTicks = self.tickCount
        resp.lowTicks = self.ticksLow
        resp.highTicks = self.ticksHigh

        #Reset for the next block
        self.tickCount = 0
        self.ticksLow = 0
        self.ticksHigh = 0

        return resp
    def onFuelLow(self):
        self.ticksLow+=1
        
    def onFuelHigh(self):
        self.ticksHigh+=1
    
    def updateTicks(self):
        self.tickCount+=1
        #Sample from a uniform and see if we should randomly close the valve...
        if self.tankView.valveState: #valve is open
            if np.random.uniform() < self.pClose:
                print 'Closing valve randomly'
                self.tankView.closeValve()
                
    def stop(self):
        self.timer.stop()

    def start(self):
        self.timer.start(100)
        
    def printReport(self):
        print 'Total ticks:', self.tickCount
        print 'Exceeded ticks:', self.ticksHigh
        print 'Underrun ticks:', self.ticksLow
        
class TankView(QGraphicsView):
    fuelLow = pyqtSignal()
    fuelHigh = pyqtSignal()
    
    def __init__(self, parent=None):
        super(TankView, self).__init__(parent=parent)
        self.w = 400.0
        self.h = 300.0
        
        self.setScene(QGraphicsScene())
        self.scene().setBackgroundBrush(parent.palette().brush(QPalette.Window))

        #Gauge
        self.fuelGauge = QFuelGaugeItem(color=Qt.blue, lineColor=Qt.darkBlue)
        self.scene().addItem(self.fuelGauge)
        self.fuelGauge.setPos(0,0)
        boundsGauge = self.fuelGauge.boundingRect()
        gaugePos = QPointF(self.w/8, self.h/2)
        #The fuelGauge's transform origin is at its 0,0 - and the thing is drawn with that origin coords
        self.fuelGauge.setPos(gaugePos)

        self.fuelLowHigh = (0.25, 0.75)
        self.fuelGauge.setLowHigh(self.fuelLowHigh[0], self.fuelLowHigh[1])
        
        #Draw the source
        self.fuelSource = QGraphicsEllipseItem(0, 0, 75, 75)
        fuelColor = QColor(Qt.blue)
        fuelColor.setAlpha(75)
        self.scene().addItem(self.fuelSource)
        self.fuelSource.setPen(QPen(Qt.darkBlue))
        self.fuelSource.setBrush(QBrush(fuelColor))
        boundsSource = self.fuelSource.boundingRect()
        self.fuelSource.setPos(3*self.w/4, self.h/2 - boundsSource.height()/2)
        
               
        #Valve arrow:
        self.fuelValve = QArrow()
        self.scene().addItem(self.fuelValve)
        
        boundsValve = self.fuelValve.boundingRect()
        #self.fuelValve.setTransformOriginPoint(bounds.width()/2, self.height()/2)
        self.fuelValve.setRotation(-90)
        self.closeValve()
        self.fuelValve.setPos(self.w/2, self.h/2)

        #Draw some lines to connect things:

        fuelPen = QPen(Qt.darkBlue)
        fuelWidth = 3.0
        fuelPen.setWidth(fuelWidth)
        
        
        self.fuelLineToTank = self.scene().addLine(boundsGauge.width()/2+gaugePos.x()+fuelWidth/2, self.h/2, self.fuelValve.pos().x()-boundsValve.width()/2-fuelWidth/2, self.h/2)
        self.fuelLineToTank.setPen(fuelPen)
        self.fuelLineToValve = self.scene().addLine(self.fuelValve.pos().x() + boundsValve.width()/2 + fuelWidth/2, self.h/2, self.fuelSource.pos().x()-fuelWidth/2, self.h/2)
        self.fuelLineToValve.setPen(fuelPen)
        

        #And some directions:
        directions = QGraphicsSimpleTextItem('Click to open/close the valve')
        directions.setFont(QFont("SansSerif", max(self.h / 60.0,12), QFont.Bold))
        self.scene().addItem(directions)
        dirBounds = directions.boundingRect()
        directions.setPos(QPointF(self.w/2-dirBounds.width()/2, self.h - dirBounds.height()))
        
        self.setSceneRect(0.0,0.0,self.w, self.h)

        self.addFuelIncrement = 0.003
        self.subFuelIncrement = 0.001

    def updateFuel(self):
        self.fuelGauge.setFuelLevel(self.fuelGauge.fuelLevel + (self.addFuelIncrement if self.valveState else -self.subFuelIncrement))

        if self.fuelGauge.fuelLevel < self.fuelLowHigh[0]:
            self.fuelLow.emit()
        elif self.fuelGauge.fuelLevel > self.fuelLowHigh[1]:
            self.fuelHigh.emit()

        
    def closeValve(self):
        self.valveState = False
        self.fuelValve.setColor(QColor(Qt.red))
        
    def openValve(self):
        #Set the valve to open, change the increment to fill the tank up
        self.fuelValve.setColor(QColor(Qt.green))
        self.valveState = True
        
    def mousePressEvent(self, event):
        #implement a mousing interface - translate and rotate
        #sceneCoords = self.mapToScene(event.pos())
        #print sceneCoords
        if event.button() == Qt.LeftButton:
            if self.valveState:
                self.closeValve()
            else:
                self.openValve()
                
    def keyPressEvent(self, event):
        '''
        if event.key() == Qt.Key_Space:
            if self.valveState:
                self.closeValve()
            else:
                self.openValve()
        '''
        if event.key() == Qt.Key_R:
            self.parent().printReport()
            
    def resizeEvent(self, evt=None):
        #Resize to fill window
        scale = 1
        bounds = self.scene().sceneRect()
        if bounds:
            self.scene().setSceneRect(0, 0, self.w, self.h)
            self.fitInView(self.scene().sceneRect(), Qt.KeepAspectRatio)
            self.show()
            
def main():
        app = QApplication(sys.argv)
        rospy.init_node('tank_task')
        
        tankTask = TankWidget()
        
        mainWindow = QMainWindow()
        mainWindow.setWindowTitle('Tank Task')
        mainWindow.setCentralWidget(tankTask)
        #mainWindow.resize(tankTask.w, tankTask.h)
        mainWindow.setStatusBar(QStatusBar())
        mainWindow.show()
        
        sys.exit(app.exec_())

if __name__ == '__main__':
        main()
