#!/usr/bin/env python2
import sys
import pdb
import json

from PyQt5.QtGui import *
from PyQt5.QtCore import * 
from PyQt5.QtWidgets import *
from tank_task import TankWidget
from attention_task import AttentionWidget
from grasping import GraspingWidget
from sampleHandling import SampleHandlingWidget
from nav_task import NavTaskWidget
from QLabeledValue import *
from taskEnums import *

from human_tasks.srv import *
from human_tasks.msg import *

import rospy

class QLabeledTLXSlider(QWidget):
    valueChanged = pyqtSignal(int)
    
    def __init__(self, labelText, initVal):
	super(QWidget, self).__init__()
        self.layout = QHBoxLayout()
        self.heading = QLabel(labelText)
        self.label = QLabel(str(initVal))
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet('font-weight:bold')
        self.layout.addWidget(self.heading)
        self.layout.addWidget(self.label)

        self.minVal = 1
        self.maxVal = 7

      
        self.value = QSlider(Qt.Horizontal)
        #self.value.setFrameShape(QFrame.Panel)
        #self.value.setFrameShadow(QFrame.Sunken)
        #self.value.setLineWidth(1)
        self.value.setMinimum(self.minVal)
        self.value.setMaximum(self.maxVal)
        self.value.setValue(initVal)
        self.value.setTickInterval(1)
        self.value.setTickPosition(QSlider.TicksBelow)
        self.value.setPageStep(1)
        self.value.setSingleStep(1)
        self.value.mousePressEvent = self.slider_mousePressEvent
        self.value.valueChanged.connect(self.onValueChanged)
        self.layout.addWidget(self.value)
        
        self.setLayout(self.layout)

    def onValueChanged(self, newValue):
        self.label.setText('%d' % (newValue))
        self.valueChanged.emit(newValue)

    def getValue(self):
        return int(self.label.text())
    
    def setValue(self, value):
        self.value.setValue(value)

    def slider_mousePressEvent(self, ev):
        
	self.value.setValue(int(float(ev.x())/self.value.width()*self.maxVal + 1))
        ev.accept()
        
    def triggerAction(self, action):
        self.value.triggerAction(action)
        
class QTLXDialog(QDialog):
    def __init__(self, parent=None):
        super(QTLXDialog, self).__init__(parent)
        self.setWindowTitle('TLX Survey')
        layout  = QVBoxLayout()
        self.mental = QLabeledTLXSlider('Mental Demand', 3)
        layout.addWidget(self.mental)

        self.physical = QLabeledTLXSlider('Physical Demand', 3)
        layout.addWidget(self.physical)

        self.temporal = QLabeledTLXSlider('Temporal Demand', 3)
        layout.addWidget(self.temporal)

        self.performance = QLabeledTLXSlider('Performance', 3)
        layout.addWidget(self.performance)

        self.effort = QLabeledTLXSlider('Effort', 3)
        layout.addWidget(self.effort)

        self.frustration = QLabeledTLXSlider('Frustration', 3)
        layout.addWidget(self.frustration)

        self.btnDone = QPushButton('Done')
        self.btnDone.clicked.connect(self.btnDone_onclick)

        layout.addWidget(self.btnDone)
        self.setLayout(layout)
        
    def reset(self):
        self.mental.setValue(3)
        self.physical.setValue(3)
        self.temporal.setValue(3)
        self.performance.setValue(3)
        self.effort.setValue(3)
        self.frustration.setValue(3)
        
    def btnDone_onclick(self):
        self.close()

    def getResults(self):
        return [self.mental.getValue(),
                self.physical.getValue(),
                self.temporal.getValue(),
                self.performance.getValue(),
                self.effort.getValue(),
                self.frustration.getValue()]
    
class QFeedbackDialog(QDialog):
    def __init__(self, parent=None):
        super(QFeedbackDialog, self).__init__(parent)
        self.setWindowTitle('Subject Feedback')
        layout = QVBoxLayout()
        lblPrompt = QLabel('Please enter any comments or questions about the experiment:')
        layout.addWidget(lblPrompt)

        self.txtComments = QTextEdit(self)
        layout.addWidget(self.txtComments)
        self.btnDone = QPushButton('Done')
        self.btnDone.clicked.connect(self.btnDone_onclick)

        layout.addWidget(self.btnDone)
        self.setLayout(layout)

    def btnDone_onclick(self):
        self.close()
        
    def getFeedback(self):
        return self.txtComments.toPlainText()
    
class QTaskRatingDialog(QDialog):
    def __init__(self, parent=None):
        super(QTaskRatingDialog, self).__init__(parent)

        layout = QVBoxLayout()
        #group = QGroupBox()
        self.chkEasy = QRadioButton('Easy')
        self.chkMed = QRadioButton('Medium')
        self.chkHard = QRadioButton('Hard')

        layout.addWidget(QLabel('Please rate the difficulty:'))
        layout.addWidget(self.chkEasy)
        layout.addWidget(self.chkMed)
        layout.addWidget(self.chkHard)

        self.btnDone = QPushButton('Done')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        layout.addWidget(self.btnDone)
        
        self.setLayout(layout)

        self.setWindowTitle('Task Rating')
        
    def btnDone_onclick(self):
        self.close()

    def reset(self):
        self.chkEasy.setChecked(False)
        self.chkMed.setChecked(False)
        self.chkHard.setChecked(False)
        
    def getRating(self):
        if self.chkEasy.isChecked():
            return TaskDifficulty.Easy
        elif self.chkMed.isChecked():
            return TaskDifficulty.Medium
        elif self.chkHard.isChecked():
            return TaskDifficulty.Hard
        else:
            return 
        
class QSubjectParamSelector(QDialog):
    def __init__(self, parent=None):
        super(QSubjectParamDialog, self).__init__(parent)
        
class ExperimentView(QSplitter):
    visibilityChanged = pyqtSignal(str, bool)
    progressChanged = pyqtSignal(int, int)
    layoutChanged = pyqtSignal(QWidget)
    paramSelectorChanged = pyqtSignal(int)
    
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
        self.paramSelectorChanged.connect(self.onParamSelectorChanged)
        self.paramLock = QMutex()
        self.paramCondition = QWaitCondition()
        
        self.telemetryPub = rospy.Publisher('~telemetry', UITelemetry, queue_size=10)
        self.feedbackPub = rospy.Publisher('~feedback', Feedback, queue_size=10)
        self.progressPub = rospy.Publisher('~progress', UIProgress, queue_size=10, latch=True)
        self.ratingPub = rospy.Publisher('~user_rating', TaskRating, queue_size=10)
        self.tlxPub = rospy.Publisher('~tlx', TLXRating, queue_size=10)
        
    def startServices(self):
        self.graspSvc = rospy.Service('~tasks/grasp_task', RunGraspTask, self.svcGraspTask)
        self.sampleSvc = rospy.Service('~tasks/sample_task', RunSampleTask, self.svcSampleTask)
        self.navSvc = rospy.Service('~tasks/nav_task', RunNavTask, self.svcNavTask)
        self.stateSvc = rospy.Service('~set_visibility', SetUIState, self.svcUIState)
        self.progressSvc = rospy.Service('~set_progress', SetProgress, self.svcProgress)
        self.paramIndexSvc = rospy.Service('~get_subject_index', GetSubjectParamIndex, self.svcGetParamIndex)

        #These are handled by their widgets, not by the main UI
        #self.attentionSvc = rospy.Service('~set_attention_params', SetAttentionParams, self.svcAttentionParams)
        #self.workloadSvc = rospy.Service('~set_workload_params', SetWorkloadParams, self.svcWorkloadParams)
        
    def onTaskComplete(self):
        self.runRatingDialog()
        print 'Waking up service call'
        self.taskCondition.wakeOne() 

    def svcGetParamIndex(self, req):
        #Display some UI to get the subject parameter index in use:
        #The request has the upper bound in it:
        self.paramSelectorChanged.emit(req.paramsCount)

        #Wait on a condition variable before returning:
        self.paramLock.lock()
        self.paramCondition.wait(self.paramLock)

        #Woken up from the button press to select
        resp = GetSubjectParamIndexResponse()
        resp.selectedParams = self.selectedParams
        return resp
    
    def svcUIState(self, req):
        #print 'Setting UI component:', req.component, ' to state:', req.visible
        self.visibilityChanged.emit(req.component, req.visible)
        return SetUIStateResponse()

    def runFeedbackDialog(self):
        self.feedbackDialog.show()
        #print 'Got feedback:', self.feedbackDialog.getFeedback()
        msg = Feedback()
        msg.header.stamp = rospy.Time.now()
        msg.feedback = self.feedbackDialog.getFeedback()
        self.feedbackPub.publish(msg)

    def runRatingDialog(self):
        self.ratingDialog.reset()
        self.ratingDialog.exec_()
        msg = TaskRating()
        msg.header.stamp = rospy.Time.now()
        msg.taskRating = self.ratingDialog.getRating().value
        msg.scenarioIndex = int(self.expProgressLabel.getValue())
        self.ratingPub.publish(msg)

        
    def runTLXDialog(self):
        self.tlxDialog.reset()
        self.tlxDialog.exec_()
        msg = TLXRating()
        msg.header.stamp = rospy.Time.now()
        tlxFeedback = self.tlxDialog.getResults()
        
        msg.mental = tlxFeedback[0]
        msg.physical = tlxFeedback[1]
        msg.temporal = tlxFeedback[2]
        msg.performance = tlxFeedback[3]
        msg.effort = tlxFeedback[4]
        msg.frustration = tlxFeedback[5]

        self.tlxPub.publish(msg)
        
    def onParamSelectorChanged(self, paramCount):
        #Twiddle visibility of the UI to show the selector thing
        #The signal includes the number of scenarios (so we don't touch UI from the service call)
        #Also, add items to the QComboBox
        self.contactExp.setVisible(False)

        self.paramIndexBox.clear()
        for index in range(0, paramCount):
            self.paramIndexBox.addItem(str(index))
            
        self.paramDialog.exec_() #run an event loop for the wee dialog
        
        
    def onProgressChanged(self, current, total):
        self.expProgressLabel.updateValue(str(current))
        self.scenarioTotalLabel.updateValue(str(total))
        msg = UIProgress()
        msg.header.stamp = rospy.Time.now()
        msg.current = current
        msg.total = total
        self.progressPub.publish(msg)
        
    def onVisibilityChanged(self, component, visible):

        #This isn't the best way to do this, but effective...
    
        if visible:
            if component == 'ExperimentView':

                #Contact experimenter is a widget, preserve its visibilty
                contactVis = self.contactExp.isVisible()
                
                for index in range(0, self.count()):
                    self.widget(index).setVisible(True)
                self.contactExp.setVisible(contactVis)
                
                print 'Resuming'
                self.attentionTask.resume()
                self.tankTask.resume()
            elif component == 'contactExp':
                self.contactExp.setVisible(True)
            elif component == 'QFeedbackDialog':
                self.runFeedbackDialog()
            elif component == 'QTaskRatingDialog':
                self.runRatingDialog()
            elif component == 'QTLXDialog':
                self.runTLXDialog()
            else:
                print 'Set visible: Unknown component:', component
        else:
            if component == 'ExperimentView':
                contactVis = self.contactExp.isVisible()
                
                for index in range(0, self.count()):
                    self.widget(index).setVisible(False)
                    
                self.contactExp.setVisible(contactVis)
                
                print 'Pausing'
                self.attentionTask.pause()
                self.tankTask.pause()
            elif component == 'contactExp':
                self.contactExp.setVisible(False)

            else:
                print 'Set visible: Unknown component:', component


    def onLayoutChanged(self, desiredWidget):
        for index in range(0, self.taskLayout.count()):
            self.taskLayout.itemAt(index).widget().setVisible(False)
        desiredWidget.setVisible(True)
        self.taskLayout.update()
        
    def svcProgress(self, req):
        self.progressChanged.emit(req.current, req.total)
        return SetProgressResponse()
    
    def svcGraspTask(self, req):
        self.layoutChanged.emit(self.graspTask)
        #Fire a signal so that the grasp task touches UI in the GUI thread (we're in a ROS thread in the service call)

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
        self.layoutChanged.emit(self.sampleTask)
        
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
        commBox = QGroupBox('Communications Console')
        commLayout = QVBoxLayout()
        commLayout.addWidget(self.attentionTask)
        commBox.setLayout(commLayout)
        leftPane.addWidget(commBox)

        tankBox = QGroupBox('Hydroponics Control')
        tankLayout = QVBoxLayout()
        tankLayout.addWidget(self.tankTask)
        tankBox.setLayout(tankLayout)
        leftPane.addWidget(tankBox)
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

        #Make the subject parameter requester UI
        paramIndexLayout = QVBoxLayout()
        paramIndexMsg = QLabel('Select a subject profile:')
        paramIndexMsg.setStyleSheet('font-weight: bold; font-size:20pt')
        paramIndexMsg.setAlignment(Qt.AlignCenter)
        paramIndexLayout.addWidget(paramIndexMsg)

        self.paramIndexBox = QComboBox()
        paramIndexLayout.addWidget(self.paramIndexBox)

        paramIndexSelect = QPushButton('Select')
        paramIndexSelect.clicked.connect(self.btnParamIndexSelect_clicked)
        
        paramIndexLayout.addWidget(paramIndexSelect)

        #self.paramIndex = QWidget()
        #self.paramIndex.setLayout(paramIndexLayout)

        #Parameter selection is done via QDialog
        #self.addWidget(self.paramIndex)
        #self.paramIndex.setVisible(False)
        self.paramDialog = QDialog()
        self.paramDialog.setWindowTitle('Subject Index')
        self.paramDialog.setLayout(paramIndexLayout)

        #Make the feedback dialog for later:
        self.feedbackDialog = QFeedbackDialog()

        #Task rating dialog:
        self.ratingDialog = QTaskRatingDialog()

        #TLX dialog
        self.tlxDialog = QTLXDialog()
        
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

    def btnParamIndexSelect_clicked(self):
        print 'Selected:', self.paramIndexBox.currentIndex()
        self.selectedParams = self.paramIndexBox.currentIndex()
        self.paramDialog.done(0) #stop the local event loop
        self.paramCondition.wakeOne()
        
        
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
            state = dict()
            state['paused'] = True
            self.onTelemetry('main_ui', json.dumps(state))
            
        else:
            print 'Resuming'
            self.attentionTask.resume()
            self.tankTask.resume()
            state = dict()
            state['paused'] = False
            self.onTelemetry('main_ui', json.dumps(state))
            
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
