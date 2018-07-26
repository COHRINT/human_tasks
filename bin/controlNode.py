#!/usr/bin/python2
#Master Control node

#Loads configurations for subtasks, makes blocking service calls to change UI state
import os, sys, string, csv
import rospy
import time
import pdb
import pickle

import subprocess

from taskEnums import *

from human_tasks.msg import *
from human_tasks.srv import *

from geometry_msgs.msg import *

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface

from std_srvs.srv import *

import copy

import std_srvs.srv


    
class ControlNode(object):
    def __init__(self):
        self.experimentNS = '/experiment_ui'
        #Grab files from the rospy param system
        scenarioFile = rospy.get_param('~scenarios', None)
        navTaskFile = rospy.get_param('~navTasks', None)
        pathPlanTaskFile = rospy.get_param('~pathTasks', None)
        
        graspTaskFile = rospy.get_param('~graspTasks', None)
        handlingTaskFile = rospy.get_param('~handlingTasks', None)

        subjectParamFile = rospy.get_param('~subjectParams', None)
        
        #Fire up a publisher for each task type:
        self.navPub = rospy.Publisher('~nav_task', Navigation, queue_size=10)
        self.pathPub = rospy.Publisher('~path_task', PathPlanning, queue_size=10)
        self.graspPub = rospy.Publisher('~grasp_task', Grasping, queue_size=10)
        self.handlingPub = rospy.Publisher('~sample_task', SampleHandling, queue_size=10)
        self.workloadPub = rospy.Publisher('~workload_task', Workload, queue_size=10)

        self.shouldEnd = False
        self.earlyEndSvc = rospy.Service('~end_experiment', Empty, self.svcEndExperiment)
        
        #For each task type, each row represents a scenario that can be run
        #So for example, for grasping, each row in the polygons.csv file is a scenario
        #The control node is where the assignment of easy/medium/hard is made and passed onto the score reporting
        self.ppTasks = self.loadTaskFile(pathPlanTaskFile)
        self.navTasks = self.loadTaskFile(navTaskFile)
        self.graspTasks = self.loadTaskFile(graspTaskFile)
        self.handlingTasks = self.loadTaskFile(handlingTaskFile)
        self.scenarios = self.loadTaskFile(scenarioFile)
        self.loadSubjectParams(subjectParamFile)

    def runHandlingTask(self, index):
        #Make the service call for the scenario with index
        try:
            sample_task = rospy.ServiceProxy(self.experimentNS+'/tasks/sample_task', RunSampleTask)
            req = RunSampleTaskRequest()
            print 'Running sample handling task:', index
            print self.handlingTasks[index]
            #Parse the CSV file of handling tasks
            #Elements:
            #0: difficulty
            #1: wind direction
            #2: wind velocity

            req.windDir = float(self.handlingTasks[index][1])
            req.windVel = float(self.handlingTasks[index][2])
            
            resp = sample_task(req)

            #Publish a report for this subtask
            msg = copy.deepcopy(resp.report)
            #Add the difficulty
            msg.difficulty = int(self.handlingTasks[index][0])
            msg.scenarioIndex = index
            
            self.handlingPub.publish(msg)
            
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        
    def runGraspTask(self, index):
        #Make the service call for the scenario with index
        try:
            grasp_task = rospy.ServiceProxy(self.experimentNS+'/tasks/grasp_task', RunGraspTask)
            req = RunGraspTaskRequest()
            print 'Running grasp task:', index
            print self.graspTasks[index]
            #Parse the CSV file of nav tasks
            #Elements:
            #0: difficulty
            #1: vertex count
            #2-3, 4-5, 6-7, etc: polygon vertices in (x,y) order

            for vertex in range(0,int(self.graspTasks[index][1])):
                req.polygonX.append(float(self.graspTasks[index][2*vertex + 2]))
                req.polygonY.append(float(self.graspTasks[index][2*vertex + 1 + 2]))
            req.objectIndex = index
            
            resp = grasp_task(req)

            #Publish a report for this subtask
            msg = copy.deepcopy(resp.report)
            #Add the difficulty
            msg.difficulty = int(self.graspTasks[index][0])
            
            self.graspPub.publish(msg)
            
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def runNavTask(self, index):
        #Make the service call for the scenario with index
        try:
            nav_task = rospy.ServiceProxy(self.experimentNS+'/tasks/nav_task', RunNavTask)
            req = RunNavTaskRequest()
            print 'Running nav task:', index
            print self.navTasks[index]
            #Parse the CSV file of nav tasks
            #Elements:
            #0: difficulty
            #1-3: start pos (x,y,z)
            #4-5: goal pos (x,y)
            
            req.start.position.x = float(self.navTasks[index][1])
            req.start.position.y = float(self.navTasks[index][2])
            req.start.position.z = float(self.navTasks[index][3])
            req.goal.position.x = float(self.navTasks[index][4])
            req.goal.position.y = float(self.navTasks[index][5])

            resp = nav_task(req)

            #Publish a report for this subtask
            msg = copy.deepcopy(resp.report)
            #Add the difficulty
            msg.difficulty = int(self.navTasks[index][0])
            
            self.navPub.publish(msg)
            
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            
    def runPathPlanTask(self, index):
        #Make the service call for the scenario with index
        try:
            pp_task = rospy.ServiceProxy(self.experimentNS+'/tasks/path_plan_task', RunPathPlanningTask)
            req = RunPathPlanningTaskRequest()
            print 'Running path plan task:', index
            #print self.ppTasks[index]
            #Parse the CSV file of path plan tasks
            #Elements:
            #0: difficulty
            #1: number of hazards
            #for each hazard:
            #0,1: x/y of mean
            #2,3,4: covariance entries, 00, 01, 11

            #startx, starty
            #endx, endy
            req.scenario.difficulty = int(self.ppTasks[index][0])
            numHazards = int(self.ppTasks[index][1])
            for i in range(2, 2+(numHazards*5), 5):
                mean = Point()
                mean.x = float(self.ppTasks[index][i])
                mean.y = float(self.ppTasks[index][i+1])
                req.scenario.means.append(mean)

                cov = Covariance2D()
                cov.covariance[0] = float(self.ppTasks[index][i+2])
                cov.covariance[1] = float(self.ppTasks[index][i+3])
                cov.covariance[2] = float(self.ppTasks[index][i+3])
                cov.covariance[3] = float(self.ppTasks[index][i+4])
                req.scenario.covs.append(cov)

            #Fill in the start/goal:
            req.scenario.start.position.x = float(self.ppTasks[index][2+(numHazards*5)])
            req.scenario.start.position.y = float(self.ppTasks[index][2+(numHazards*5) + 1])
            req.scenario.goal.position.x = float(self.ppTasks[index][2+(numHazards*5) + 2])
            req.scenario.goal.position.y = float(self.ppTasks[index][2+(numHazards*5) + 3])

            
            #print req
            
            resp = pp_task(req)

            #Publish a report for this subtask
            msg = copy.deepcopy(resp.report)
            #Add the difficulty
            msg.difficulty = int(self.ppTasks[index][0])
            
            self.pathPub.publish(msg)
            
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def loadSubjectParams(self, fileName):
        #Load the subject params file:
        paramsFile  = open(fileName, 'rb')
        masterParams = pickle.load(paramsFile)

        #populate the particulars;
        self.blockSize = masterParams['tasksPerBlock']
        self.workloadParams = masterParams['workloadParams']
        self.attentionParams = masterParams['attentionParams']
        
    def loadTaskFile(self, fileName):
        #Given a filename of tasks, load it
        #Each task file is a csv file of parameters
        #Return a list of lists, indexed by subtask ID first, with all parameters

        srcFile = open(fileName, 'r')
        reader = csv.reader(srcFile)
        tasks = []
        for row in reader:
            tasks.append(row) #All parameters are added as a list for this index
        return tasks

    def getSubjectParamMapping(self, attention, workload):
        #Map from easy/medium/hard to actual values for the attention / workload activities
        att = 0.0
        work = 0.0
        
        if attention == TaskDifficulty.Easy:
            att = 10.0
        elif attention == TaskDifficulty.Medium:
            att = 5.0
        elif attention == TaskDifficulty.Hard:
            att = 1.0

        if workload == TaskDifficulty.Easy:
            work = 0.001
        elif workload == TaskDifficulty.Medium:
            work = 0.01
        elif workload == TaskDifficulty.Hard:
            work = 0.05

        return (att, work)
    
    def setSubjectParams(self, attention, workload):
        #Get actual values from the enums:
        (att, work) = self.getSubjectParamMapping(attention, workload)
        
        #Set the attention and workload via service calls:
        try:
            set_attention = rospy.ServiceProxy(self.experimentNS+'/set_attention_params', SetAttentionParams)
            req = SetAttentionParamsRequest()
            req.newParam = att
            resp = set_attention(req)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            set_workload = rospy.ServiceProxy(self.experimentNS+'/set_workload_params', SetWorkloadParams)
            req = SetWorkloadParamsRequest()
            req.pClose = work
            resp = set_workload(req)

            #Publish the report from the workload widget

            msg = Workload()
            msg.header.stamp = rospy.Time.now()
            msg.pClose = resp.pClose
            msg.totalTicks = resp.totalTicks
            msg.lowTicks = resp.lowTicks
            msg.highTicks = resp.highTicks
            self.workloadPub.publish(msg)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def getUISubjectParams(self):
        try:
            get_paramIndex = rospy.ServiceProxy(self.experimentNS+'/get_subject_index', GetSubjectParamIndex)
            req = GetSubjectParamIndexRequest()
            req.paramsCount = len(self.workloadParams) #indexed by subject count first, then block index second

            resp = get_paramIndex(req)
            return resp.selectedParams
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def setUIState(self, component, state):
        try:
            set_visible = rospy.ServiceProxy(self.experimentNS+'/set_visibility', SetUIState)
            req = SetUIStateRequest()
            print 'Setting UI component ', component, ' visible: ', state
            req.component = component
            req.visible = state
            resp = set_visible(req)
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def setUIProgress(self, current, total):
        try:
            set_progress = rospy.ServiceProxy(self.experimentNS+'/set_progress', SetProgress)
            req = SetProgressRequest()
            print 'Setting progress to ', current+1, ' of ', total

            #Don't confuse humans with 0-based indexing...
            req.current = current+1
            req.total = total
            resp = set_progress(req)
            return
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def svcEndExperiment(self, req):
        self.shouldEnd = True
        return std_srvs.srv.EmptyResponse()
        
    def runTasks(self):
        #Wait for the UI to be available:
        rospy.wait_for_service(self.experimentNS+'/set_visibility', 20)

        #Get the subject params:
        paramIndex = self.getUISubjectParams()

        print 'Using subject param index:', paramIndex

        #hide the contact box:
        self.setUIState('contactExp', False)
      
        
        #Enable the task execution UI:
        self.setUIState('ExperimentView', True)
        
        lastBlockIndex = -1
        for index, scenario  in enumerate(self.scenarios):
            #Set the workload / attention params (if needed)
            newBlockIndex = int(index / self.blockSize)
            if newBlockIndex <> lastBlockIndex:
                print 'Updating block ', newBlockIndex, ' w/ attention:',  self.attentionParams[paramIndex][newBlockIndex], ' workload:', self.workloadParams[paramIndex][newBlockIndex]
                self.setSubjectParams(self.attentionParams[paramIndex][newBlockIndex],
                                      self.workloadParams[paramIndex][newBlockIndex])
                lastBlockIndex = newBlockIndex
                #get a TLX survey for this block
                if index > 0:
                    self.setUIState('QTLXDialog', True)
                
            self.setUIProgress(index, len(self.scenarios))
            #first parameter is the subtask type, second is the scenario index of that subtask
            if int(scenario[0]) == TaskType.Navigation.value:
                self.runPathPlanTask(int(scenario[1])) #recycle the nav task enum
            elif int(scenario[0]) == TaskType.Grasping.value:
                self.runGraspTask(int(scenario[1]))
            elif int(scenario[0]) == TaskType.Handling.value:
                self.runHandlingTask(int(scenario[1]))
            else:
                print 'Unknown scenario type: ', scenario[0]
                continue

            if self.shouldEnd:
                break

           

        #Disable the task execution UI:
        self.setUIState('ExperimentView', False)
        #Show the contact box:
        self.setUIState('contactExp', True)
        #Show the feedback window
        self.setUIState('QFeedbackDialog', True)

        
class GazeboInterface():
    def __init__(self, gzNS='/gazebo'):
        self.gazeboNamespace = gzNS
        rospy.wait_for_service(gzNS + '/unpause_physics', timeout=10)
    
    def getModelList(self):
        try:
            modelList = rospy.wait_for_message(self.gazeboNamespace + '/model_states', ModelStates, timeout=2)
        except (rospy.ROSException), e:
            print 'Gazebo model_states not available'
            return list()
        return modelList.name
    
    def resumePhysics(self):
         try:
            pause_physics = rospy.ServiceProxy(self.gazeboNamespace+'/unpause_physics', std_srvs.srv.Empty)
            rospy.loginfo("Calling service %s/unpause_physics" % self.gazeboNamespace)
                        
            resp = pause_physics(std_srvs.srv.EmptyRequest())
            return 
         except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pausePhysics(self):
        try:
            pause_physics = rospy.ServiceProxy(self.gazeboNamespace+'/pause_physics', std_srvs.srv.Empty)
            rospy.loginfo("Calling service %s/pause_physics" % self.gazeboNamespace)
                        
            resp = pause_physics(std_srvs.srv.EmptyRequest())
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def spawnURDF(self, modelName, modelXml, pose):
        # setting initial pose
        initial_pose = Pose()
        initial_pose.position.x = pose[0]
        initial_pose.position.y = pose[1]
        initial_pose.position.z = pose[2]
        # convert rpy to quaternion for Pose message
        #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])

        #q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
        q = Quaternion(0,0,0,1)
        initial_pose.orientation = q;

        # spawn model
        success = gazebo_interface.spawn_urdf_model_client(modelName, modelXml, rospy.get_namespace(),
                                                            initial_pose, "", self.gazeboNamespace)
        print 'Spawned robot ', modelName
      
def getRobotURDF(robotName, robotXacro):
    #Run some xacro for the given robot type:
    #'/home/smcguire/human_ws/src/scenario_runner/husky_cam_laser.xacro'
    modelURDF = subprocess.check_output(['/opt/ros/lunar/share/xacro/xacro.py','--inorder',robotXacro, 'robot_namespace:=' + robotName])
    #print 'Using modelURDF:', modelURDF
    return modelURDF
    
def main():
    rospy.init_node('control_node')
    #Instantiate a master gazebo interface:
    '''
    gzInt = GazeboInterface()
   
    gzInt.resumePhysics()
    models = gzInt.getModelList()
    if 'robot0' not in models:
        gzInt.spawnURDF('robot0', getRobotURDF('robot0', rospy.get_param('~robotURDFFile')), [0,0,0])

    #Wait for the heightmap to load and render from disk...
    #Must be a better way than busy wai
    rospy.sleep(10.0)
        
    gzInt.pausePhysics()
    '''
    
    #Fire up the master control node to run our scenarios
    ctl = ControlNode()
    ctl.runTasks()
    
    return


if __name__ == "__main__":
    """If this script is run as stand alone then call main() function."""
    main();
