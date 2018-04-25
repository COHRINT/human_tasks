#!/usr/bin/python2

'''
Class to have a QLabel with a sunken QSlider next to it so that I can have labelled data values

'''
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class QLabeledSlider(QWidget):
    valueChanged = pyqtSignal(float)
    
    def __init__(self, labelText, minVal, maxVal, initVal):
        super(QLabeledSlider, self).__init__()
        self.label = QLabel(labelText)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet('font-weight:bold')
        
        #self.label.setWidth

        self.value = QSlider(Qt.Horizontal)
        #self.value.setFrameShape(QFrame.Panel)
        #self.value.setFrameShadow(QFrame.Sunken)
        #self.value.setLineWidth(1)
        self.value.setMinimum(minVal)
        self.value.setMaximum(maxVal)
        self.value.setValue(initVal)
        self.value.setTickInterval(1)
        self.value.setTickPosition(QSlider.TicksBelow)
        self.value.setPageStep(1)
        self.value.setSingleStep(1)
        self.value.keyPressEvent = self.keyPressEvent
        
        self.lblValue = QLabel("0.0")
        self.lblValue.setText('%1.1f' % initVal)
        self.lblValue.setAlignment(Qt.AlignCenter)
        self.lblValue.setStyleSheet('font-weight:bold')
        
        self.layout = QVBoxLayout()
        #self.label.setContentsMargins(-1, 0, -1, 0)
        self.layout.setContentsMargins(-1, 0, -1, 0)

        self.btnZero = QPushButton('Set to 0.0')
        self.btnZero.clicked.connect(self.btnZero_onclick)
        self.btnZero.setMaximumWidth(100)
        

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.value)
        self.layout.addWidget(self.lblValue)
        self.layout.addWidget(self.btnZero)

        self.layout.setAlignment(self.btnZero, Qt.AlignCenter)
        self.setLayout(self.layout)

        self.value.valueChanged.connect(self.onValueChanged)

    def btnZero_onclick(self):
        self.lblValue.setText('0.0')
        self.valueChanged.emit(0.0)
        self.value.setValue(0)
        
    def onValueChanged(self, newValue):
        self.lblValue.setText('%1.1f' % (newValue / 10.0))
        self.valueChanged.emit(newValue / 10.0)
        
    def getValue(self):
        return float(self.lblValue.text())
    
    def updateValue(self, value):
        print 'Updating slider to:', value
        self.value.setValue(value)
        
        '''
        if type(value) is str:
            self.value.setText(value)
        elif type(value) is float:
            self.value.setText('%1.2f' % value)
        else:
            self.value.setText(str(value))
        '''
        
    def keyPressEvent(self, evt):
        evt.ignore()
        
    def triggerAction(self, action):
        self.value.triggerAction(action)
        
