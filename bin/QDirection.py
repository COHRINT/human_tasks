#!/usr/bin/python2

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


class QDirection(QGraphicsItem):
    def __init__(self, width=100, height=100, color=Qt.green, lineColor=Qt.darkGreen, parent=None):
        
        super(QDirection, self).__init__(parent)

        #We now have a 100x100 canvas to draw on...
        #self.pixmap().fill(Qt.white)
        
        self._color = QColor(color)
        self.lineColor = QColor(lineColor)
        self._color.setAlpha(90)
        #self.lineColor.setAlpha(100)
        
        self.lineWidth = 1
        self._pen = QPen(self.lineColor, self.lineWidth, Qt.SolidLine,
                         Qt.RoundCap, Qt.RoundJoin)

        self._brush = QBrush(self._color)
        
        self.arrow = QPolygonF()
        self.arrow.clear()
        self.arrow.append(QPointF(50, 0))
        self.arrow.append(QPointF(40, 60))
        self.arrow.append(QPointF(47, 40))
        self.arrow.append(QPointF(47, 100))
        self.arrow.append(QPointF(53, 100))
        self.arrow.append(QPointF(53, 40))
        self.arrow.append(QPointF(60, 60))
        self.arrow.append(QPointF(50, 0))
        self.path = QPainterPath()
        self.path.addPolygon(self.arrow)
        self.setTransformOriginPoint(QPointF(50,50))
        
    def boundingRect(self):
        bounds = QRectF(40 - self.lineWidth / 2, 0 - self.lineWidth/2, 60 + self.lineWidth/2, 100+self.lineWidth/2)
        return bounds

    def shape(self):
        return self.path
    
    def paint(self, qp, options, widget):
        #QGraphicsPixmapItem.paint(self,qp, options, widget)
        qp.setBrush(self._brush)
        qp.setPen(self._pen)
        qp.drawPath(self.path)

        #qp.drawRect(0,0,100,100)

       
        
        
