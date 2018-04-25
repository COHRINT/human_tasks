from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class RobotIcon(QLabel):
 
    def __init__(self, text, parent):
        QLabel.__init__(self, text, parent)
        
    def __init__(self, text):
        QLabel.__init__(self, text)
        
           
    def mouseReleaseEvent(self, ev):
        self.emit(SIGNAL('clicked()'))

class RobotWidget(QGraphicsSimpleTextItem):
    def __init__(self, text, color):
        QGraphicsSimpleTextItem.__init__(self, text)
        self._selected = False
        self._color = color
        self._color.setAlpha(100)
        
    def selected(self, m_sel):
        self._selected = m_sel
        
    def mouseReleaseEvent(self, ev):
        self.emit(SIGNAL('clicked()'))
        
    def paint(self, qp, options, widget):
        if self._selected:
            qp.setBrush(QBrush(self._color)) #light transparent version of the text color
            qp.drawRect(self.boundingRect())
            
        QGraphicsSimpleTextItem.paint(self,qp, options, widget)
