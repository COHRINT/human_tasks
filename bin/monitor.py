#!/usr/bin/python2
'''
ROS client node to monitor the experiment
Check that several nodes are alive and well:
1. Gazebo
2. ControlNode
3. FullscreenUI
4. MSBand
5. Recording

Conops: Make a watchdog timer that resets when a message on a related topic is received

'''

import sys
import subprocess
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from QLabeledValue import *
from human_tasks.msg import *
from msband.msg import *
from nav_msgs.msg import *

class QTopicWatchdog(QWidget):
    message = pyqtSignal()
    
    def __init__(self, parent = None, srcTopic = None, topicType = None, srcLabel = ''):
        super(QTopicWatchdog, self).__init__(parent)
        self.topic = srcTopic
        self.label = QLabel(srcLabel)
        self.label.setAlignment(Qt.AlignRight)
        
        self.status = QLabel('OK')
        self.status.setAlignment(Qt.AlignLeft)
        self.status.setStyleSheet('font-weight: bold; color: green')

        layout = QHBoxLayout()
        layout.setContentsMargins(-1, 0, -1, 0)
        
        layout.addWidget(self.label)
        layout.addWidget(self.status)
        self.setLayout(layout)
        
        self.topicSub = rospy.Subscriber(self.topic, topicType, self.ros_cb)
        #Start the watchdog timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.onTimeout)
        self.message.connect(self.onMessage)
        self.timer.start(6000)
        
    def onMessage(self):
        #Reset the watchdog timer
        self.timer.start(6000) #restart if needed for 6 seconds
        self.status.setStyleSheet('font-weight: bold; color: green')
        self.status.setText('OK')
        
    def onTimeout(self):
        #Watchdog expired, change the font of the status label
        self.status.setStyleSheet('font-weight: bold; color: red')
        self.status.setText('FAIL')
        
    def ros_cb(self, msg):
        #Feed the dog
        self.message.emit()

        
class MonitorWidget(QWidget):
    progress = pyqtSignal(int)
    bagUpdate = pyqtSignal(str)
    
    def __init__(self, parent = None):
        super(MonitorWidget, self).__init__()
        layout = QVBoxLayout()
        layout.addWidget(QTopicWatchdog(self, 'telemetry', UITelemetry, 'UI Telemetry'))
        layout.addWidget(QTopicWatchdog(self, 'odom', Odometry, 'Gazebo Odometry'))
        layout.addWidget(QTopicWatchdog(self, 'heartrate', Heartrate, 'MSBand Heartrate'))
        self.lblProgress = QLabeledValue('Scenario Number:')
        layout.addWidget(self.lblProgress)


        self.lblBagSize = QLabeledValue('Bag Watcher:')
        layout.addWidget(self.lblBagSize)
        self.bagSub = rospy.Subscriber('bag_updates', FileWatcher, self.updateBagWatcher)
        self.bagUpdate.connect(self.onBagUpdate)
        
        
        self.btnRunRQT = QPushButton('Run RQT')
        self.btnRunRQT.clicked.connect(self.btnRunRQT_onclick)
        layout.addWidget(self.btnRunRQT)
        
        self.btnQuit = QPushButton('Exit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)
        self.progressSub = rospy.Subscriber('progress', UIProgress, self.updateProgress)
        self.progress.connect(self.onProgress)
        
        self.setLayout(layout)
        
    def btnRunRQT_onclick(self):
        #Spawn a subprocess to run RQT with the ROS_MASTER_URI env:
        self.rqt_process = subprocess.Popen(['rqt'])
                                   
    def btnQuit_onclick(self):
        self.close()

    def updateBagWatcher(self, msg):
        self.bagUpdate.emit('%s: %d' % (msg.fileName, msg.fileSize))

    def onBagUpdate(self, val):
        self.lblBagSize.updateValue(val)
        
    def updateProgress(self, msg):
        self.progress.emit(msg.current)
    def onProgress(self, val):
        self.lblProgress.updateValue(str(val))
        
def main():
    if len(sys.argv) < 2:
        print 'Usage: ', sys.argv[0], ' <machine name for windowtitle>'
        sys.exit(0)
        
    rospy.init_node('monitor_node')
    
    app = QApplication(sys.argv)
    mainWidget = MonitorWidget(app)
    mainWindow = QMainWindow()
    mainWindow.setWindowTitle('Monitor of:' + sys.argv[1])
    mainWindow.setCentralWidget(mainWidget)
    mainWindow.setStatusBar(QStatusBar())
          
    mainWindow.show()
    app.exec_()


        
if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
	pass
