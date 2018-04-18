#!/usr/bin/python2

import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *

import pdb

class QFuelGauge(QWidget):
    def __init__(self, color=Qt.green, lineColor=Qt.darkGreen,parent=None):
        super(QFuelGauge, self).__init__(parent)
        self._margins = 10
        
        #Fuel is expressed in percent of max
        self.fuelLevel = 0.5
        self.brush = QBrush(color)
        self.lineColor = QColor(lineColor)
        #self.color.setAlpha(90)
        #self.lineColor.setAlpha(100)
        
        self.lineWidth = 1
        self.pen = QPen(self.lineColor, self.lineWidth, Qt.SolidLine,
                        Qt.RoundCap, Qt.RoundJoin)
        self.resize(100,230)
        
    def setFuelLevel(self, fuelLevel):
        
        self.fuelLevel = max(min(fuelLevel, 1.0), 0.0)
        #print 'Setting fuel to ', self.fuelLevel
        self.repaint()
        
    def sizeHint(self):
        return QSize(100,230)
    
    def paintEvent(self, evt):
        painter= QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width()/2, self.height()/2)
        scale = min((self.width() - self._margins)/100.0,
                    (self.height() - self._margins)/230.0)
        #Keep the same aspect ratio
        painter.scale(scale, scale)

        #We can now draw in fixed coords and it will be automatically sized to fit
        self.drawGauge(painter)
        self.drawFuel(painter)
        self.drawStaff(painter)
        self.drawReadout(painter)
        
        painter.end()
        
    def drawGauge(self, qp):

        self.pen.setColor(Qt.black)
        self.brush.setColor(Qt.transparent)
        qp.setBrush(self.brush)
        qp.setPen(self.pen)
        #Border:
        qp.drawRect(QRectF(-50, -100, 100, 200))

    def drawFuel(self,qp):
        #Fuel inside:
        fuelColor = QColor(Qt.green)
        fuelColor.setAlpha(75)
        self.brush.setColor(fuelColor)
        self.pen.setColor(Qt.darkGreen)
        
        qp.setPen(self.pen)
        qp.setBrush(self.brush)
        qp.drawRect(QRectF(self.lineWidth - 50, 100 - 200*self.fuelLevel, 100-self.lineWidth*2, 200*self.fuelLevel-self.lineWidth))
        
    def drawStaff(self, qp):
        self.brush.setColor(Qt.black)
        self.pen.setColor(Qt.black)
        qp.setPen(self.pen)
        qp.setBrush(self.brush)

        for major in range(-50, 100,50):
            qp.drawLine(-25, major, 25, major)
        for minor in range(-75,100,25):
            qp.drawLine(-10, minor, 10, minor)

        #qp.drawLine(0,-100,0,100)
            
    def drawReadout(self, qp):
        self.pen.setColor(Qt.black)
        self.brush.setColor(Qt.black)
        qp.setPen(self.pen)
        qp.setBrush(self.brush)
        
        qp.drawText(-50, 100, 100, 20, Qt.AlignHCenter | Qt.AlignTop, '%d%%' % (self.fuelLevel*100.0))

class QFuelGaugeItem(QGraphicsItem):
    def __init__(self,  color=Qt.green, lineColor=Qt.darkGreen, parent=None):        
        super(QFuelGaugeItem, self).__init__(parent)
        self._margins = 10
        self.width = 100
        self.height = 230
        #Fuel is expressed in percent of max
        self.fuelLevel = 0.5

        self.brush = QBrush(color)
        self.color = QColor(color)
        self.lineColor = QColor(lineColor)
        #self.color.setAlpha(90)
        #self.lineColor.setAlpha(100)
        
        self.lineWidth = 1
        self.pen = QPen(self.lineColor, self.lineWidth, Qt.SolidLine,
                        Qt.RoundCap, Qt.RoundJoin)
        #self.setTransformOriginPoint(QPointF(0,0))
        self.lowHigh = None
        
    def boundingRect(self):
        bounds = QRectF(-50,-100, 100, 230)
        return bounds
    
    def setLowHigh(self, low, high):
        self.lowHigh = (low, high)
        self.update()
        
    def setFuelLevel(self, fuelLevel):
        self.fuelLevel = max(min(fuelLevel, 1.0), 0.0)
        #print 'Setting fuel to ', self.fuelLevel
        self.update(self.boundingRect())
        
    def paint(self, painter, options, widget):
        painter.setRenderHint(QPainter.Antialiasing)
        #painter.translate(self.width/2, self.height/2)
        #scale = min((self.width - self._margins)/100.0,
        #            (self.height - self._margins)/230.0)
        #Keep the same aspect ratio
        #painter.scale(scale, scale)

        #We can now draw in fixed coords and it will be automatically sized to fit
        self.drawGauge(painter)
        self.drawFuel(painter)
        self.drawStaff(painter)
        self.drawReadout(painter)
        
    def drawGauge(self, qp):
        self.pen.setColor(Qt.black)
        self.brush.setColor(Qt.transparent)
        qp.setBrush(self.brush)
        qp.setPen(self.pen)
        #Border:
        qp.drawRect(QRectF(-50, -100, 100, 200))

    def drawFuel(self,qp):
        #Fuel inside:
        fuelColor = self.color
        fuelColor.setAlpha(75)
        self.brush.setColor(fuelColor)
        self.pen.setColor(self.lineColor)
        
        qp.setPen(self.pen)
        qp.setBrush(self.brush)
        qp.drawRect(QRectF(self.lineWidth - 50, 100 - 200*self.fuelLevel, 100-self.lineWidth*2, 200*self.fuelLevel-self.lineWidth))
        
    def drawStaff(self, qp):
        self.brush.setColor(Qt.black)
        self.pen.setColor(Qt.black)
        qp.setPen(self.pen)
        qp.setBrush(self.brush)

        for major in range(-50, 100,50):
            qp.drawLine(-25, major, 25, major)
        for minor in range(-75,100,25):
            qp.drawLine(-10, minor, 10, minor)

        if not self.lowHigh is None:
            #lowhigh is a float tuple expressed in percents
            
            lowHighPen = QPen(Qt.red)
            lowHighPen.setStyle(Qt.DotLine)
            lowHighPen.setWidth(3.0)
            qp.setPen(lowHighPen)
            qp.drawLine(-50, self.lowHigh[0]*200-100, 50, self.lowHigh[0]*200-100)
            qp.drawLine(-50, self.lowHigh[1]*200-100, 50, self.lowHigh[1]*200-100)
                
    def drawReadout(self, qp):
        self.pen.setColor(Qt.black)
        self.brush.setColor(Qt.black)
        qp.setPen(self.pen)
        qp.setBrush(self.brush)
        
        qp.drawText(-50, 100, 100, 20, Qt.AlignHCenter | Qt.AlignTop, '%d%%' % (self.fuelLevel*100.0))
