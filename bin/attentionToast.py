#!/usr/bin/python2

import sys
import math
import rospy
import signal

from human_tasks.msg import *
from human_tasks.srv import *

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import numpy as np

def signal_handler(signal, frame):
		print 'You pressed Ctrl+C!'
		sys.exit(0)
		
signal.signal(signal.SIGINT, signal_handler)
class MBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        self.setSizeGripEnabled(True)

    def event(self, e):
        result = QMessageBox.event(self, e)

        self.setMinimumHeight(150)
        self.setMinimumWidth(250)

        return result

class ParameterWindow(QWidget):
    timer_expired = pyqtSignal()
    box_closed = pyqtSignal()
    duration = pyqtSignal()

    def __init__(self):
	super(ParameterWindow,self).__init__()
        rospy.init_node('attention_task')
        
        self.getLambda = rospy.Service('~GetLambda', GetCurrentLambda, self.get_lambda)
        self.changeLambda = rospy.Service('~ChangeLambda', ChangeLambda, self.change_lambda)

        self.exp_parameter = 2.0

        self.timer_expired.connect(self.showToast)
        self.duration.connect(self.open_duration)
        self.box_closed.connect(self.publish_message)

        self.setTimer()
        
    def showToast(self):
        box = MBox()
        box.setText('Congratulations! You are the 1000th visitor!')

        #Start timing
        box.exec_()
        self.duration.emit()
        
        #User closed the box, publish the time
        self.box_closed.emit()
        self.setTimer()
        
    def setTimer(self):
        
        #set the next timer
        sleepTime = np.random.exponential(1/self.exp_parameter)
        print 'Sleeping for:', sleepTime
        self.sleepTimer = rospy.Timer(rospy.Duration(sleepTime), self.timerTick, oneshot = True)

    def timerTick(self, msg):
        self.timer_expired.emit()
        self.sleepTimer = None

    def open_duration(self):
        self.start = rospy.Time.now()

    def publish_message(self):
        self.pub = rospy.Publisher('time_taken', AttentionTask, queue_size=10)
        self.data_msg = AttentionTask()


        self.data_msg.spent =  rospy.Time.now()-self.start
        self.data_msg.header.stamp = rospy.Time.now()
        self.data_msg.lambda_value = self.exp_parameter

        self.pub.publish(self.data_msg)

    def get_lambda(self,req):
        resp = GetCurrentLambdaResponse()
        resp.exp_param = self.exp_parameter

        return resp

    def change_lambda(self,req):
        self.exp_parameter = req.change
        print(req.change)
        
class Application(QApplication):
	def event(self, e):
		return QApplication.event(self, e)
app = None
shouldQuit = False

def onSigint(signum, param):
    sys.exit(0)
    
def main():     
	app = Application(sys.argv)
	coretools_app = ParameterWindow()
        shouldQuit = False
	signal.signal(signal.SIGINT, onSigint)
	app.startTimer(200)
        while not shouldQuit:
            app.exec_()
            
        print 'App exec returned!'
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
