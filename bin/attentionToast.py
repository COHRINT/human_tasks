#!/usr/bin/env python2

import sys
import math
import rospy
import signal
from human_tasks.msg import *
from human_tasks.srv import *

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
		self.pub = rospy.Publisher('time_taken', AttentionTask, queue_size=10)
		self.data_msg = AttentionTask()

		self.getLambda = rospy.Service('~GetLambda', GetCurrentLambda, self.get_lambda)
		#self.changeLambda = rospy.Service('~ChangeLambda', ChangeLambda, self.change_lambda)

		self.msg = box()  	
		self.msg.setText("Press OK to dismiss this message")
		self.msg.setWindowTitle("Be Attentive")

		self.msg.setStandardButtons(QMessageBox.Ok)

		self.submit = self.msg.exec_()
		self.start = rospy.Time.now()
		if self.submit == QMessageBox.Ok:
			self.submit_data()

	def submit_data(self):

		self.data_msg.lambda_value = 6
		#self.data_msg.duration = rospy.Time.now()-self.start
		self.data_msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.data_msg)


		rospy.sleep(3) #to be exponential, potentially implement a Timer here
		self.msg.exec_()
		self.start = rospy.Time.now()
		if self.submit == QMessageBox.Ok:
			self.submit_data()

	def get_lambda(self):
		try:
			lambda1 = rospy.ServiceProxy('/attention_toast/GetLambda', GetCurrentLambda)

			response = numpy.exponential(1/lambda1,1)
			return response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

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


