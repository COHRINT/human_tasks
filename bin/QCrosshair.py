#!/usr/bin/python2

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class QCrosshair(QGraphicsPixmapItem):
    def __init__(self, width=100, height=100, color=Qt.green, thickness=5, parent=None):
        
        super(QCrosshair, self).__init__(QPixmap(width, height), parent)

        #We now have a 100x100 canvas to draw on...
        self.pixmap().fill(Qt.white)
        
        self._color = QColor(color)
        self._color.setAlpha(100)
        
        self._pen = QPen(self._color, thickness, Qt.DotLine,
                         Qt.FlatCap, Qt.RoundJoin)
        self.arrowHead = QPolygonF()
        self.arrowHead.clear()

        self.arrowHead.append(QPointF(50, 40))
        self.arrowHead.append(QPointF(40, 50))
        self.arrowHead.append(QPointF(50, 0))
        self.arrowHead.append(QPointF(60, 50))
        self.arrowHead.append(QPointF(50, 40))
        
         
    def paint(self, qp, options, widget):
        #QGraphicsPixmapItem.paint(self,qp, options, widget)
        qp.setBrush(QBrush(self._color))
        qp.setPen(self._pen)
        qp.drawLine(50,0,50,100)
        qp.drawLine(0,50,100,50)
        
