#!/usr/bin/env python2
import sys
import pdb

from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *
from tank_task import TankWidget
from grasping import GraspingWidget
from sampleHandling import SampleHandlingWidget
from human_tasks.srv import *
from human_tasks.msg import *

import rospy

class ExperimentView(QSplitter):
    def __init__(self, parent=None):
        super(ExperimentView, self).__init__(parent)
        self.initUI()
        self.startServices()
        self.taskLock = QMutex()
        self.taskCondition = QWaitCondition()
    
    def startServices(self):
        self.graspSvc = rospy.Service('~tasks/grasp_task', RunGraspTask, self.svcGraspTask)
        self.sampleSvc = rospy.Service('~tasks/sample_task', RunSampleTask, self.svcSampleTask)
        self.navSvc = rospy.Service('~tasks/nav_task', RunNavTask, self.svcNavTask)
        
    def onTaskComplete(self):
        print 'Waking up service call'
        self.taskCondition.wakeOne()
            
    def svcGraspTask(self, req):
        #print 'Setting grasp'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        #Fire a signal so that the grasp task touches UI in the GUI thread (we're in a ROS thread)
        self.graspTask.reinitialize.emit(QPolygonF()) #replace this with a param from the scenario list
        

        #Basic idea: Pause this thread - (started from ROS, so it's not under Qt or Python...
        #, starting another loop to service the task.
        #When the task is done, it will emit a signal to wake based on the condvar
        #Block on a condvar until the task is complete and scoring is available
        #A better way to do this would be to implement a generic Task that has a few methods:
        #scoreReport = runTask(): blocking, wait for user to click 'done'
        #scoreReport: user's performance

        self.taskLock.lock()
        #print 'Sleeping in service call'
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()

        return RunGraspTaskResponse()

    def svcSampleTask(self, req):
        #print 'Setting sample'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        #post a signal to the sampleTask telling it to re-initialize
        self.sampleTask.reinitialize.emit(20,4)
        


        self.taskLock.lock()
        #print 'Sleeping in service call'
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()
        return RunSampleTaskResponse()

    def svcNavTask(self, req):
        #print 'Setting nav:'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        #self.navTask.setVisible(True)
        self.taskLayout.update()
        self.taskLock.lock()
        #print 'Sleeping in service call'
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()
        return RunNavTaskResponse()


    def initUI(self):
        qbtn = QPushButton('Quit')
        qbtn.clicked.connect(self.btnQuit_onclick)


        self.btnQuit = qbtn
        self.tankTask = TankWidget()

        #self.fuelGauge.show()
        layout = QHBoxLayout()

        #Control layout
        ctlLayout = QVBoxLayout()

        ctlLayout.addWidget(self.btnQuit)

        self.btnSwitch = QPushButton('Swap tasks')
        self.btnSwitch.clicked.connect(self.btnSwitch_onclick)
        ctlLayout.addWidget(self.btnSwitch)

        ctlWidget = QWidget()
        ctlWidget.setLayout(ctlLayout)

        leftPane = QSplitter()
        leftPane.setOrientation(Qt.Vertical) #vertical stack...
        leftPane.addWidget(ctlWidget)
        leftPane.addWidget(self.tankTask)
        leftPane.setSizes((leftPane.height()/2, leftPane.height()/2)) 
        #Disable the handles...
        leftPane.handle(1).setEnabled(False)

        self.setOrientation(Qt.Horizontal) #horiz stack
        self.addWidget(leftPane)




        #Create the task widgets - these will be created once, and then reset for each instantiation
        self.sampleTask = SampleHandlingWidget()
        self.sampleTask.setVisible(False)
        self.sampleTask.scoringComplete.connect(self.onTaskComplete)

        self.graspTask = GraspingWidget(None)
        self.graspTask.setVisible(False)
        #Hook into this task's completion signal
        self.graspTask.scoringComplete.connect(self.onTaskComplete)

        #To avoid having to resize the splitter, create a pane that we will fill on demand
        self.taskPane = QWidget()
        self.taskLayout = QVBoxLayout()
        self.taskLayout.addWidget(self.graspTask)
        self.taskLayout.addWidget(self.sampleTask)

        self.taskPane.setLayout(self.taskLayout)

        self.addWidget(self.taskPane)
        #Disable the sizer handle
        self.handle(1).setEnabled(False)

        self.setSizes((self.width()/4, self.width()*3/4))
        self.showFullScreen()


    def btnQuit_onclick(self, evt):
        print 'Closing'
        self.close()

    def btnSwitch_onclick(self, evt):
        print 'Switching tasks'

        self.taskLayout.addWidget(self.sampleTask)
        self.taskLayout.removeWidget(self.graspTask)

        #print 'Sizes:', self.sizes()
        #self.setSizes((self.width()/4, self.width()*3/4))
def main():
        app = QApplication(sys.argv)
        rospy.init_node('experiment_ui')
        
        window = ExperimentView()
        sys.exit(app.exec_())

if __name__ == '__main__':
        main()
