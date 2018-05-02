#!/usr/bin/env python2

import sys
import rospy
from tlx_ui.msg import TLX
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *


class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('tlx_id')
		self.r = rospy.Rate(1)
		self.pub = rospy.Publisher('part_id', TLX, queue_size = 10, latch=True)
		self.msg = TLX()


		self.horiz_layout = QHBoxLayout()
		self.vert_layout = QVBoxLayout()
		self.setWindowTitle("TLX")
		self.setGeometry(500,200,200,200)

		self.lbl_box = QLabel(self)
		self.lbl_box.setText('Participant ID:')
		self.vert_layout.addWidget(self.lbl_box)

		self.textbox = QLineEdit(self)
		self.textbox.resize(280,40)
		self.vert_layout.addWidget(self.textbox)
		
		self.smt_btn = QPushButton('Submit',self)
		self.smt_btn.clicked.connect(self.submit_data)
		self.smt_btn.setMaximumWidth(120)
		self.horiz_layout.addWidget(self.smt_btn)

		self.buildWidgets()
		

	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.pub.publish(self.msg)
		self.hide()



	def buildWidgets(self):
		self.vert_layout.addLayout(self.horiz_layout)
		self.vert_layout.addSpacing(10)
		self.vert_layout.setContentsMargins(20,20,20,30)
		self.vert_layout.addStretch()
		self.setLayout(self.vert_layout)
		self.show()


if __name__ == '__main__':
	app = QApplication(sys.argv)
	coretools_app = ParameterWindow()
	sys.exit(app.exec_())


