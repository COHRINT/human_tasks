#!/usr/bin/env python2
import sys
import pdb

from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *
from tank_task import TankWidget
from attention_task import AttentionWidget
from grasping import GraspingWidget
from sampleHandling import SampleHandlingWidget
from nav_task import NavTaskWidget
from QLabeledValue import *

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
        print 'Setting up polygon:', req.polygonID
        

        #Basic idea: Pause this thread - (started from ROS, so it's not under Qt or Python...
        #, starting another loop to service the task.
        #When the task is done, it will emit a signal to wake based on the condvar
        #Block on a condvar until the task is complete and scoring is available
        #A better way to do this would be to implement a generic Task that has a few methods:
        #scoreReport = runTask(): blocking, wait for user to click 'done'
        #scoreReport: user's performance

        self.taskLock.lock()
        #print 'Sleeping in service call'
        #Set up the preconditions:
        startTime = rospy.Time.now()
        self.graspTask.reinitialize.emit(QPolygonF()) #replace this with a param from the scenario list
        
        self.taskCondition.wait(self.taskLock)
        
        self.taskLock.unlock()

        resp = RunGraspTaskResponse()
        resp.report.header.stamp = rospy.Time.now()
        resp.report.score = self.graspTask.score
        totalTime = self.graspTask.finalTime - startTime
        resp.report.duration = totalTime.to_sec()
        resp.report.objectIndex = req.polygonID
        resp.startTime = startTime
        resp.endTime = self.graspTask.finalTime
        return resp

    def svcSampleTask(self, req):
        #print 'Setting sample'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        self.taskLock.lock()
        #post a signal to the sampleTask telling it to re-initialize
        startTime = rospy.Time.now()
        self.sampleTask.reinitialize.emit(req.windDir,req.windVel)
        
        #print 'Sleeping in service call'
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()
        resp = RunSampleTaskResponse()
        totalTime = self.sampleTask.finalTime - startTime #A Duration
        
        resp.report.header.stamp = rospy.Time.now()
        resp.report.score = self.sampleTask.score
        resp.report.windDir = self.sampleTask.windDirNoisy
        resp.report.windVel = self.sampleTask.windVelNoisy
        resp.report.duration = totalTime.to_sec()
        resp.startTime = startTime
        resp.endTime = self.sampleTask.finalTime
        return resp

    def svcNavTask(self, req):
        #print 'Setting nav:'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        self.navTask.setVisible(True)
        self.taskLayout.update()
        self.taskLock.lock()
        #print 'Sleeping in service call'
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()
        return RunNavTaskResponse()


    def initUI(self):

        expControl = QGroupBox('Experiment Control')
        self.expProgressLabel = QLabeledValue('Task')
        self.scenarioTotalLabel = QLabeledValue('of')
        progressLayout = QHBoxLayout()
        progressLayout.addWidget(self.expProgressLabel)
        progressLayout.addWidget(self.scenarioTotalLabel)
        self.btnPause = QPushButton('Pause')
        self.btnPause.clicked.connect(self.btnPause_onclick)
        self.btnPause.setCheckable(True)
        
        self.btnQuit = QPushButton('Quit')
        self.btnQuit.clicked.connect(self.btnQuit_onclick)

        expControlLayout = QVBoxLayout()
        #expControlLayout.addWidget(QLabel('Correlating Human Physiological Responses\n to Task Performance in a Simulated \nSpace Exploration Environment'))
        #expControlLayout.addWidget(QLabel('IRB Protocol 17-0485'))
                                   
        expControlLayout.addLayout(progressLayout)
        expControlLayout.addWidget(self.btnPause)
        expControlLayout.addWidget(self.btnQuit)

        expControl.setLayout(expControlLayout)
        expGroupLayout = QVBoxLayout()
        expGroupLayout.addWidget(expControl)

        self.tankTask = TankWidget()
        self.attentionTask = AttentionWidget()

        layout = QHBoxLayout()


        leftPane = QSplitter()
        leftPane.setOrientation(Qt.Vertical) #vertical stack...
        leftPane.addWidget(expControl)
        leftPane.addWidget(self.attentionTask)
        leftPane.addWidget(self.tankTask)
        leftPane.setSizes((leftPane.height()/3, leftPane.height()/6, leftPane.height()/3))

        
        #Disable the handles...
        leftPane.handle(1).setEnabled(False)
        leftPane.handle(2).setEnabled(False)
        
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


        self.navTask = NavTaskWidget()
        self.navTask.setVisible(False)
        self.navTask.scoringComplete.connect(self.onTaskComplete)
        
        #To avoid having to resize the splitter, create a pane that we will fill on demand
        self.taskPane = QWidget()
        self.taskLayout = QVBoxLayout()
        self.taskLayout.addWidget(self.graspTask)
        self.taskLayout.addWidget(self.sampleTask)
        self.taskLayout.addWidget(self.navTask)

        self.taskPane.setLayout(self.taskLayout)

        self.addWidget(self.taskPane)
        #Disable the sizer handle
        self.handle(1).setEnabled(False)

        self.setSizes((self.width()/4, self.width()*3/4))
        self.showFullScreen()

    def btnPause_onclick(self, evt):
        #Pause the experiment, including the attention / tank tasks, etc
        if self.btnPause.isChecked():
            print 'Pausing'
            self.attentionTask.pause()
            self.tankTask.pause()
        else:
            print 'Resuming'
            self.attentionTask.resume()
            self.tankTask.resume()
            
    def btnQuit_onclick(self, evt):
        print 'Closing'
        self.close()

def main():
        app = QApplication(sys.argv)
        rospy.init_node('experiment_ui')
        
        window = ExperimentView()
        sys.exit(app.exec_())

if __name__ == '__main__':
        main()
