#!/usr/bin/python2

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


class QGripper(QGraphicsItem):
    def __init__(self, parent=None):
        
        super(QGripper, self).__init__(parent)

        self.color = QColor(Qt.gray)
        self.lineColor = QColor(Qt.darkGray)
        #self.color.setAlpha(90)
        #self.lineColor.setAlpha(100)
        
        self.lineWidth = 1
        self.pen = QPen(self.lineColor, self.lineWidth, Qt.SolidLine,
                         Qt.RoundCap, Qt.RoundJoin)

        self.brush = QBrush(self.color)

        self.path = QPainterPath()
  
        self.wrist = QPolygonF()
        self.wrist.append(QPointF(-20,-30))
        self.wrist.append(QPointF(-40,-10))
        self.wrist.append(QPointF(-40, 10))
        self.wrist.append(QPointF(-20, 30))
        self.wrist.append(QPointF(20, 30))
        self.wrist.append(QPointF(40, 10))
        self.wrist.append(QPointF(40,-10))
        self.wrist.append(QPointF(20,-30))
        self.wrist.append(QPointF(-20,-30))

        
        self.crossBar = QPolygonF()
        self.crossBar.append(QPointF(-20, 30))
        self.crossBar.append(QPointF(-75, 65))
        self.crossBar.append(QPointF(-75, 90))
        self.crossBar.append(QPointF(-20, 70))
        self.crossBar.append(QPointF(20, 70))
        self.crossBar.append(QPointF(75, 90))
        self.crossBar.append(QPointF(75, 65))
        self.crossBar.append(QPointF(20, 30))
        self.crossBar.append(QPointF(-20, 30))

        self.leftGrip = QPolygonF()
        self.leftGrip.append(QPointF(-80, 70))
        self.leftGrip.append(QPointF(-80, 130))
        self.leftGrip.append(QPointF(-75, 130))
        self.leftGrip.append(QPointF(-75, 65))
        self.leftGrip.append(QPointF(-80, 70))

        self.rightGrip = QPolygonF()
        self.rightGrip.append(QPointF(80, 70))
        self.rightGrip.append(QPointF(80, 130))
        self.rightGrip.append(QPointF(75, 130))
        self.rightGrip.append(QPointF(75, 65))
        self.rightGrip.append(QPointF(80, 70))
        
        self.path.addPolygon(self.wrist)
        self.path.addPolygon(self.crossBar)
        self.path.addPolygon(self.leftGrip)
        self.path.addPolygon(self.rightGrip)

        self.setTransformOriginPoint(QPointF(0,0))
        
    def getLeftJaw(self):
        return (QPoint(-75, 90), QPoint(-75,130))

    def getRightJaw(self):
        return (QPoint(75, 90), QPoint(75, 130))
    
    def getPivot(self):
        return self.scene().mapToScene(QPointF(0,0))

    def getPivotOffset(self):
        return QPointF(80, 20)
    
    def boundingRect(self):
        bounds = QRectF(-80 - self.lineWidth / 2, -30 - self.lineWidth/2, 160 + self.lineWidth/2, 160+self.lineWidth/2)
        return bounds

    def shape(self):
        return self.path
    
    def paint(self, qp, options, widget):
        #QGraphicsPixmapItem.paint(self,qp, options, widget)
        
        path = QPainterPath()

        self.pen.setColor(Qt.darkGray)
        self.brush.setColor(Qt.gray)
        qp.setBrush(self.brush)
        qp.setPen(self.pen)
        
        path.addPolygon(self.wrist)
        path.addPolygon(self.leftGrip)
        path.addPolygon(self.rightGrip)
        qp.drawPath(path)
        self.pen.setColor(Qt.black)
        self.brush.setColor(Qt.lightGray)
        qp.setBrush(self.brush)
        qp.setPen(self.pen)
        qp.drawPolygon(self.crossBar)




       
        
        
