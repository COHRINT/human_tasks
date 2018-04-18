#!/usr/bin/python2

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class QArrow(QGraphicsPixmapItem):
    def __init__(self, width=50, height=50, color=Qt.green, parent=None):
        
        super(QArrow, self).__init__(QPixmap(width, height), parent)

        #We now have a 100x100 canvas to draw on...
        self.pixmap().fill(Qt.white)
        
        self._color = QColor(color)
        self._color.setAlpha(90)
        
        #self._pen = QPen(self._color, 2, Qt.SolidLine,
        #        Qt.RoundCap, Qt.RoundJoin))
        self.arrowHead = QPolygonF()
        self.arrowHead.clear()

        
        self.arrowHead.append(QPointF(0, -25))
        self.arrowHead.append(QPointF(-25, 0))
        self.arrowHead.append(QPointF(25, 0))
        
        self.setTransformOriginPoint(QPointF(0,0))
    def boundingRect(self):
        return QRectF(-25,-25,50,50)
    
    def setColor(self, color):
        self._color = color
        self._color.setAlpha(90)
        self.update(self.boundingRect())
        
    def paint(self, qp, options, widget):
        #QGraphicsPixmapItem.paint(self,qp, options, widget)
        qp.setBrush(QBrush(self._color))
        qp.setPen(QColor(self._color))
        
        qp.drawPolygon(self.arrowHead)
        qp.drawRect(-10, 0, 20, 25)

