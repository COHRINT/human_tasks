#!/usr/bin/python2

#Attention task, based on J. McGinley's attentionToast.py
import sys
import numpy as np

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from QLabeledValue import *
from human_tasks.srv import *
from human_tasks.msg import *
import rospy
from enum import Enum

class CommCategory(Enum):
    Low = 0
    High = 1

class AttentionWidget(QWidget):
    timer_expired = pyqtSignal()
    box_closed = pyqtSignal()
    duration = pyqtSignal()
    messageSources = ['EVA 1', 'EVA 2', 'Hab: Infirmary', 'Hab: Greenhouse',
                      'Hab: Science Lab', 'Earth: CAPCOM', 'Earth: Physio',
                      'Earth: Science']
    
    def __init__(self):
	super(AttentionWidget,self).__init__()

        self.getLambda = rospy.Service('~GetAttentionLambda', GetCurrentLambda, self.get_lambda)
        self.changeLambda = rospy.Service('~SetAttentionLambda', ChangeLambda, self.change_lambda)
        self.attentionPub = rospy.Publisher('attention', AttentionTask, queue_size=10)
        
        self.exp_parameter = 10.0
        self.timer_expired.connect(self.showNewMessage)
        
        #self.duration.connect(self.open_duration)
        #self.box_closed.connect(self.publish_message)
        
        self.commsGroup = QGroupBox('Communications')
        commsLayout = QVBoxLayout()
        self.commSource = QLabeledValue('Message from:')
        commsLayout.addWidget(self.commSource)

        self.lcdTime = QLabeledValue('Elapsed Time:')
        self.lcdTime.updateValue(0)
        self.lcdTimer = QTimer()
        self.lcdTimer.timeout.connect(self.incrementLCD)

        commsLayout.addWidget(self.lcdTime)
        
        self.btnCommLow = QPushButton('Low')
        self.btnCommLow.clicked.connect(self.btnCommLow_onclick)
        self.btnCommLow.setEnabled(False)
        
        self.btnCommHigh = QPushButton('High')
        self.btnCommHigh.clicked.connect(self.btnCommHigh_onclick)
        self.btnCommHigh.setEnabled(False)
        


        commCatLayout = QHBoxLayout()
        commCatLayout.addWidget(self.btnCommLow)
        commCatLayout.addWidget(self.btnCommHigh)

        commsLayout.addLayout(commCatLayout)

        self.commsGroup.setLayout(commsLayout)

        masterLayout = QVBoxLayout()
        masterLayout.addWidget(self.commsGroup)
        self.setLayout(masterLayout)
        self.setTimer()
        
    def setTimer(self):
        #set the next timer
        sleepTime = np.random.exponential(self.exp_parameter)
        #print 'Sleeping for:', sleepTime
        self.sleepTimer = QTimer.singleShot(sleepTime*1000, self.timer_expired)

    def incrementLCD(self):
        baseTime = int(self.lcdTime.getValue())
        if baseTime >= 9:
            self.lcdTime.setStylesheet('color:red; font-weight: bold')
        self.lcdTime.updateValue(baseTime + 1)
        
        
    def showNewMessage(self):
        src = np.random.choice(self.messageSources)
        self.commSource.updateValue(src)
        self.btnCommLow.setEnabled(True)
        self.btnCommHigh.setEnabled(True)
        self.startTime = rospy.Time.now()
        
        self.lcdTimer.start(1000)

    def commReport(self, category):
        msg = AttentionTask()
        msg.spent =  rospy.Time.now() - self.startTime
        msg.lambda_value = self.exp_parameter
        msg.msgSrc = self.commSource.getValue()
        msg.category = category.value
        
        self.attentionPub.publish(msg)
        
        self.commSource.updateValue('')
        self.lcdTime.updateValue(0)
        self.lcdTime.setStylesheet('')
        self.lcdTimer.stop()
        
        self.btnCommLow.setEnabled(False)
        self.btnCommHigh.setEnabled(False)
        self.setTimer()
        
    def btnCommLow_onclick(self):
        self.commReport(CommCategory.Low)
        
    def btnCommHigh_onclick(self):
        self.commReport(CommCategory.High)

        
    def get_lambda(self,req):
        resp = GetCurrentLambdaResponse()
        resp.exp_param = self.exp_parameter
        return resp

    def change_lambda(self,req):
        self.exp_parameter = req.change
        #print(req.change)
        return []

def main():     
	app = QApplication(sys.argv)
        rospy.init_node('attention_task')
                
	attentionWidget = AttentionWidget()
        attentionWidget.show()
	app.startTimer(200)
        
        app.exec_()
            
        print 'App exec returned!'
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
