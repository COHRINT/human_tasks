#!/usr/bin/python2
import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *
from tank_task import TankWidget

class mainUI(QSplitter):
        def __init__(self, parent=None):
                super(mainUI, self).__init__(parent)
                self.initUI()
            
        def initUI(self):
                qbtn = QPushButton('Quit')
                qbtn.clicked.connect(self.btnQuit_onclick)


                self.btnQuit = qbtn
                self.tankTask = TankWidget()

                #self.fuelGauge.show()
                layout = QHBoxLayout()

                leftPane = QSplitter()
                leftPane.setOrientation(Qt.Vertical) #vertical stack...
                leftPane.addWidget(qbtn)
                leftPane.addWidget(self.tankTask)
                leftPane.setSizes((leftPane.height()/2, leftPane.height()/2)) 
                #Disable the handles...
                leftPane.handle(1).setEnabled(False)
                
                self.setOrientation(Qt.Horizontal) #horiz stack
                self.addWidget(leftPane)
                self.addWidget(QPushButton('Main task'))
                self.handle(1).setEnabled(False)
                
                self.setSizes((self.width()/4, self.width()*3/4))
                self.showFullScreen()
                #self.show()

        def btnQuit_onclick(self, evt):
            print 'Closing'
            self.close()
        
def main():
        app = QApplication(sys.argv)
        window = mainUI()
        sys.exit(app.exec_())

if __name__ == '__main__':
        main()
