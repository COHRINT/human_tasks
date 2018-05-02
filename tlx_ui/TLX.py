#!/usr/env/bin python

import sys
import rospy
#from tlx_ui.msg import TLX
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

 
class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):

		self.horiz_layout = QHBoxLayout()
		self.vert_layout = QVBoxLayout()
		self.setWindowTitle("TLX")
		self.setGeometry(0,0,600,700)


		self.smt_btn = QPushButton('Submit',self)
		self.smt_btn.clicked.connect(self.submit_data)
		self.smt_btn.setMaximumWidth(120)
		self.horiz_layout.addWidget(self.smt_btn)


		self.mnt_sl = QSlider(Qt.Horizontal)
		self.phys_sl = QSlider(Qt.Horizontal)
		self.temp_sl = QSlider(Qt.Horizontal)
		self.per_sl = QSlider(Qt.Horizontal)
		self.fru_sl = QSlider(Qt.Horizontal)
		self.eff_sl = QSlider(Qt.Horizontal)
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
		#rospy.subscriber


		self.sliderList = [self.mnt_sl,self.phys_sl,self.temp_sl,self.per_sl,self.eff_sl,self.fru_sl]

		for n in self.sliderList:
			print(n.value())

		rospy.init_node('dop')
			#subscribers
			#publish
		self.close()

	def buildWidgets(self):
		for i in self.list:
			self.vert_layout.addWidget(i)

		self.vert_layout.addLayout(self.horiz_layout)
		self.setLayout(self.vert_layout)
		self.show()

	def mousePressEvent(self, ev):
		print(ev) #figure out the goodies that belong to ev
		if isinstance == QSlider:
			self.setValue((QtGui.QStyle.sliderValueFromPosition(self.minimum(), self.maximum(), ev.x(), self.width())))


if __name__ == '__main__':
	app = QApplication(sys.argv)
	coretools_app = ParameterWindow()
	sys.exit(app.exec_())


