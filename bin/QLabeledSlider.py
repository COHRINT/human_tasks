#!/usr/bin/python2

'''
Class to have a QLabel with a sunken QSlider next to it so that I can have labelled data values

'''
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class QLabeledSlider(QWidget):
    def __init__(self, labelText):
        super(QLabeledSlider, self).__init__()
        self.label = QLabel(labelText)
        self.label.setAlignment(Qt.AlignRight)
        #self.label.setWidth

        self.value = QSlider(Qt.Horizontal)
        #self.value.setFrameShape(QFrame.Panel)
        #self.value.setFrameShadow(QFrame.Sunken)
        #self.value.setLineWidth(1)
       
        self.layout = QHBoxLayout()
        #self.label.setContentsMargins(-1, 0, -1, 0)
        self.layout.setContentsMargins(-1, 0, -1, 0)
        
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.value)
        self.setLayout(self.layout)

    def updateValue(self, value):
        print 'Updating slider to:', value
        '''
        if type(value) is str:
            self.value.setText(value)
        elif type(value) is float:
            self.value.setText('%1.2f' % value)
        else:
            self.value.setText(str(value))
        '''
