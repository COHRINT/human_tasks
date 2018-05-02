#!/usr/bin/env python2

import sys
import rospy
from tlx_ui.msg import TLX
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

class TLXSlider(QSlider):
	def __init__(self):
		super(QSlider, self).__init__(Qt.Horizontal)

	def mousePressEvent(self, ev):
		if isinstance(self,QSlider):
			self.setValue(ev.x()/65)

class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('tlx')

		self.pub = rospy.Publisher('tlx', TLX, queue_size = 10)
		self.msg = TLX()

		self.horiz_layout = QHBoxLayout()
		self.vert_layout = QVBoxLayout()
		self.setWindowTitle("TLX")
		self.setGeometry(0,0,600,700)


		self.smt_btn = QPushButton('Submit',self)
		self.smt_btn.clicked.connect(self.submit_data)
		self.smt_btn.setMaximumWidth(120)
		self.horiz_layout.addWidget(self.smt_btn)


		self.mnt_sl = TLXSlider()
		self.phys_sl = TLXSlider()
		self.temp_sl = TLXSlider()
		self.per_sl = TLXSlider()
		self.fru_sl = TLXSlider()
		self.eff_sl = TLXSlider()
		self.createSlider()


		self.lbl_mnt = QLabel(self)
		self.lbl_mnt.setText('Mental Demand')
		self.lbl_phys= QLabel(self)
		self.lbl_phys.setText('Physical Demand')
		self.lbl_tmp= QLabel(self)
		self.lbl_tmp.setText('Temporal Demand')
		self.lbl_per= QLabel(self)
		self.lbl_per.setText('Performance')
		self.lbl_eff= QLabel(self)
		self.lbl_eff.setText('Effort')
		self.lbl_fru= QLabel(self)
		self.lbl_fru.setText('Frustration')


		self.list = [self.lbl_mnt,self.mnt_sl, self.lbl_phys,self.phys_sl, self.lbl_tmp,self.temp_sl, self.lbl_per, self.per_sl, self.lbl_eff,self.eff_sl, self.lbl_fru,self.fru_sl]

		self.buildWidgets()


	def createSlider(self):
 
		sliders = [self.mnt_sl, self.phys_sl, self.temp_sl,self.per_sl,self.eff_sl,self.fru_sl]
		for i in range(0,len(sliders)):
			sliders[i].setFocusPolicy(Qt.ClickFocus)
			sliders[i].setTickPosition(QSlider.TicksBothSides)

			sliders[i].setMinimum(1)
			sliders[i].setMaximum(7)
			sliders[i].setTickInterval(1)


	def submit_data(self):
		self.sliderList = [self.mnt_sl,self.phys_sl,self.temp_sl,self.per_sl,self.eff_sl,self.fru_sl]

		for n in self.sliderList:
			print(n.value())
		#self.msg.id = rospy.Subscriber("part_id", TLX, self.callback)
		self.msg.mental_demand = self.mnt_sl.value()
		self.msg.physical_demand = self.phys_sl.value()
		self.msg.temporal_demand = self.temp_sl.value()
		self.msg.performance = self.per_sl.value()
		self.msg.effort = self.eff_sl.value()
		self.msg.frustration = self.fru_sl.value()
		self.msg.header.stamp = rospy.Time.now()

		self.pub.publish(self.msg)

		self.close()
	def callback(data):
		rospy.loginfo(data.id)
	def buildWidgets(self):
		for i in self.list:
			self.vert_layout.addWidget(i)

		self.vert_layout.addLayout(self.horiz_layout)
		self.setLayout(self.vert_layout)
		self.show()


if __name__ == '__main__':
	app = QApplication(sys.argv)
	coretools_app = ParameterWindow()
	sys.exit(app.exec_())


