#!/usr/bin/env python2

import sys
import math
import rospy
import signal
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *


def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)
        
signal.signal(signal.SIGINT, signal_handler)
class box(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        self.setSizeGripEnabled(True)

    def event(self, e):
        result = QMessageBox.event(self, e)

        self.setMinimumHeight(150)
        self.setMaximumHeight(16777215)
        self.setMinimumWidth(250)
        self.setMaximumWidth(16777215)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        textEdit = self.findChild(QTextEdit)
        if textEdit != None :
            textEdit.setMinimumHeight(150)
            textEdit.setMaximumHeight(16777215)
            textEdit.setMinimumWidth(250)
            textEdit.setMaximumWidth(16777215)
            textEdit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        return result
class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('attentionToast')

  		self.msg = box()

  	
   		self.msg.setText("Click to dismiss")
   		self.msg.setWindowTitle("Be Attentive")

   		self.msg.setStandardButtons(QMessageBox.Ok)

   		self.submit = self.msg.exec_()
   		if self.submit == QMessageBox.Ok:
   			self.submit_data()

	def submit_data(self):
		print(6)
		rospy.sleep(3)
		self.msg.exec_()
   		if self.submit == QMessageBox.Ok:
   			self.submit_data()
class Application(QApplication):
    def event(self, e):
        return QApplication.event(self, e)

def main():     
	app = Application(sys.argv)
	coretools_app = ParameterWindow()
        signal.signal(signal.SIGINT, lambda *a: app.quit())
        app.startTimer(200)

	sys.exit(app.exec_())

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


