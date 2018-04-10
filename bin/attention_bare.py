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

class ParameterWindow(QWidget):
    timer_expired = pyqtSignal()
    
    def __init__(self):
	super(ParameterWindow,self).__init__()
        rospy.init_node('attention_task')
        
        self.beta = 1.0
        self.timer_expired.connect(self.showToast)
        self.setTimer()
        
    def showToast(self):
        box = QMessageBox()
        box.setText('Message!')

        #Start timing
        box.exec_()
        
        #User closed the box, publish the time
        self.setTimer()
        
    def setTimer(self):
        
        #set the next timer
        sleepTime = np.random.exponential(self.beta)
        print 'Sleeping for:', sleepTime
        self.sleepTimer = rospy.Timer(rospy.Duration(sleepTime), self.timerTick, oneshot = True)

    def timerTick(self, msg):
        #print 'Msg:', msg
        self.timer_expired.emit()
        self.sleepTimer = None
        
class Application(QApplication):
	def event(self, e):
		return QApplication.event(self, e)
app = None
shouldQuit = False

def onSigint(signum, param):
    app.quit()
    
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
