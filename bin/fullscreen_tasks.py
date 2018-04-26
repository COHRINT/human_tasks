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
    visibilityChanged = pyqtSignal(bool)
    progressChanged = pyqtSignal(int, int)
    layoutChanged = pyqtSignal(QWidget)
    
    def __init__(self, parent=None):
        super(ExperimentView, self).__init__(parent)
        print 'Starting subject UI'
        self.initUI()
        self.startServices()
        self.taskLock = QMutex()
        self.taskCondition = QWaitCondition()
        self.visibilityChanged.connect(self.onVisibilityChanged)
        self.progressChanged.connect(self.onProgressChanged)
        self.layoutChanged.connect(self.onLayoutChanged)
        
        self.telemetryPub = rospy.Publisher('~telemetry', UITelemetry, queue_size=10)
        
    def startServices(self):
        self.graspSvc = rospy.Service('~tasks/grasp_task', RunGraspTask, self.svcGraspTask)
        self.sampleSvc = rospy.Service('~tasks/sample_task', RunSampleTask, self.svcSampleTask)
        self.navSvc = rospy.Service('~tasks/nav_task', RunNavTask, self.svcNavTask)
        self.stateSvc = rospy.Service('~set_visibility', SetUIState, self.svcUIState)
        self.progressSvc = rospy.Service('~set_progress', SetProgress, self.svcProgress)
        
    def onTaskComplete(self):
        print 'Waking up service call'
        self.taskCondition.wakeOne() 
        
    def svcUIState(self, req):
        print 'Setting UI state:', req.visible
        self.visibilityChanged.emit(req.visible)
        return SetUIStateResponse()

    def onProgressChanged(self, current, total):
        self.expProgressLabel.updateValue(str(current))
        self.scenarioTotalLabel.updateValue(str(total))
        
    def onVisibilityChanged(self, visible):
        if visible:
            #Show widgets
            for index in range(0, self.count()):
                self.widget(index).setVisible(True)
            self.contactExp.setVisible(False)
            print 'Resuming'
            self.attentionTask.resume()
            self.tankTask.resume()
        else:
            #Hide the widgets and such
            for index in range(0, self.count()):
                self.widget(index).setVisible(False)

            #Show the Done message
            self.contactExp.setVisible(True)
            print 'Pausing'
            self.attentionTask.pause()
            self.tankTask.pause()

    def onLayoutChanged(self, desiredWidget):
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)
        desiredWidget.setVisible(True)
        self.taskLayout.update()
        
    def svcProgress(self, req):
        self.progressChanged.emit(req.current, req.total)
        return SetProgressResponse()
    
    def svcGraspTask(self, req):
        #print 'Setting grasp'
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)

        #Fire a signal so that the grasp task touches UI in the GUI thread (we're in a ROS thread)
        print 'Setting up polygon:', req.objectIndex
        
        obj = QPolygonF()
        for vertIndex in range(0, len(req.polygonX)):
            obj.append(QPointF(req.polygonX[vertIndex], req.polygonY[vertIndex]))
            
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
        self.graspTask.reinitialize.emit(obj) #replace this with a param from the scenario list
        
        self.taskCondition.wait(self.taskLock)
        
        self.taskLock.unlock()

        resp = RunGraspTaskResponse()
        resp.report.header.stamp = rospy.Time.now()
        resp.report.score = self.graspTask.score
        totalTime = self.graspTask.finalTime - startTime
        resp.report.duration = totalTime.to_sec()
        resp.report.objectIndex = req.objectIndex
        resp.report.startTime = startTime
        resp.report.endTime = self.graspTask.finalTime
        return resp

    def svcSampleTask(self, req):
        #print 'Setting sample'

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
        resp.report.startTime = startTime
        resp.report.endTime = self.sampleTask.finalTime
        return resp

    def svcNavTask(self, req):
        #print 'Setting nav:'

        self.layoutChanged.emit(self.navTask)
        
        self.taskLock.lock()
        #print 'Sleeping in service call'

        startTime = rospy.Time.now()
        #print 'Request:', req
        #print 'Posting goal:', req.goal
        self.navTask.reinitialize.emit(req.start, req.goal)
        
        self.taskCondition.wait(self.taskLock)
        self.taskLock.unlock()
        resp = RunNavTaskResponse()
        totalTime = self.navTask.finalTime - startTime #A Duration
        
        resp.report.header.stamp = rospy.Time.now()
        resp.report.score = self.navTask.score
        resp.report.start = req.start
        resp.report.goal = req.goal
        resp.report.duration = totalTime.to_sec()
        resp.report.startTime = startTime
        resp.report.endTime = self.navTask.finalTime
        return resp


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
        #expControlLayout.addWidget(self.btnQuit)

        expControl.setLayout(expControlLayout)
        expGroupLayout = QVBoxLayout()
        expGroupLayout.addWidget(expControl)

        self.tankTask = TankWidget()
        self.attentionTask = AttentionWidget()

        layout = QHBoxLayout()


        leftPane = QSplitter()
        leftPane.setOrientation(Qt.Vertical) #vertical stack...
        leftPane.addWidget(expControl)

        #Add a filler Widget:
        filler = QWidget()
        #filler.setFixedHeight(leftPane.height()/6)
        leftPane.addWidget(filler)
        leftPane.addWidget(self.attentionTask)
        leftPane.addWidget(self.tankTask)
        leftPane.setSizes((leftPane.height()/8, leftPane.height()/4, leftPane.height()/8, leftPane.height()/2))

        
        #Disable the handles...
        leftPane.handle(1).setEnabled(False)
        leftPane.handle(2).setEnabled(False)
        leftPane.handle(3).setEnabled(False)

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

        self.setSizes((self.width()/4, self.width()*3/4, 0))

        self.contactExp = QLabel('Please contact the experimenter')
        self.contactExp.setStyleSheet('font-weight: bold; font-size:40pt')
        self.contactExp.setAlignment(Qt.AlignCenter)
        self.addWidget(self.contactExp)
        
        for index in range(0, self.count()):
            self.widget(index).setVisible(False)
        
        self.contactExp.setVisible(True)
        print 'Pausing'
        self.attentionTask.pause()
        self.tankTask.pause()


        #Connect the telemetry recorders:
        self.navTask.onTelemetry.connect(self.onTelemetry)
        self.graspTask.onTelemetry.connect(self.onTelemetry)
        self.sampleTask.onTelemetry.connect(self.onTelemetry)
        self.tankTask.onTelemetry.connect(self.onTelemetry)
        self.attentionTask.onTelemetry.connect(self.onTelemetry)
        
        self.showFullScreen()

    def onTelemetry(self, src, jsonState):
        #Publish the given telemetry message:
        msg = UITelemetry()
        msg.header.stamp = rospy.Time.now()
        msg.uiElement = src
        msg.jsonState = jsonState

        self.telemetryPub.publish(msg)
        
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
