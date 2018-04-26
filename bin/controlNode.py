#!/usr/bin/python2
#Master Control node

#Loads configurations for subtasks, makes blocking service calls to change UI state
import os, sys, string, csv
import rospy
import time
import pdb
import subprocess

from enum import Enum

from human_tasks.msg import *
from human_tasks.srv import *

from geometry_msgs.msg import *

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface

from std_srvs.srv import *

import copy

import std_srvs.srv

class TaskType(Enum):
    Navigation = 0
    Grasping = 1
    Handling = 2
    
class TaskDifficulty(Enum):
    Easy = 0
    Medium = 1
    Hard = 2
    
class ControlNode(object):
    def __init__(self):
        self.experimentNS = '/experiment_ui'
        #Grab files from the rospy param system
        scenarioFile = rospy.get_param('~scenarios', None)
        navTaskFile = rospy.get_param('~navTasks', None)
        graspTaskFile = rospy.get_param('~graspTasks', None)
        handlingTaskFile = rospy.get_param('~handlingTasks', None)

        #Fire up a publisher for each task type:
        self.navPub = rospy.Publisher('~nav_task', Navigation, queue_size=10)
        self.graspPub = rospy.Publisher('~grasp_task', Grasping, queue_size=10)
        self.handlingPub = rospy.Publisher('~sample_task', SampleHandling, queue_size=10)
        
        #For each task type, each row represents a scenario that can be run
        #So for example, for grasping, each row in the polygons.csv file is a scenario
        #The control node is where the assignment of easy/medium/hard is made and passed onto the score reporting
        self.navTasks = self.loadTaskFile(navTaskFile)
        self.graspTasks = self.loadTaskFile(graspTaskFile)
        self.handlingTasks = self.loadTaskFile(handlingTaskFile)
        self.scenarios = self.loadTaskFile(scenarioFile)
        
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

            req.windDir = int(self.handlingTasks[index][1])
            req.windVel = int(self.handlingTasks[index][2])
            
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

            for vertex in range(0,int(self.graspTasks[index][1]),2):
                req.polygonX.append(float(self.graspTasks[index][2 + vertex]))
                req.polygonY.append(float(self.graspTasks[index][2 + vertex + 1]))
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

    def setUIState(self, state):
        try:
            set_visible = rospy.ServiceProxy(self.experimentNS+'/set_visibility', SetUIState)
            req = SetUIStateRequest()
            print 'Setting UI visible: ', state

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
            
    def runTasks(self):
        #Wait for the UI to be available:
        rospy.wait_for_service(self.experimentNS+'/set_visibility', 20)
        
        #Enable the UI:
        self.setUIState(True)
        
        for index, scenario  in enumerate(self.scenarios):
            self.setUIProgress(index, len(self.scenarios))
            #first parameter is the subtask type, second is the scenario index of that subtask
            if int(scenario[0]) == TaskType.Navigation.value:
                self.runNavTask(int(scenario[1]))
            elif int(scenario[0]) == TaskType.Grasping.value:
                self.runGraspTask(int(scenario[1]))
            elif int(scenario[0]) == TaskType.Handling.value:
                self.runHandlingTask(int(scenario[1]))
            else:
                print 'Unknown scenario type: ', scenario[0]

        self.setUIState(False)
        
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
    gzInt = GazeboInterface()
   
    gzInt.resumePhysics()
    models = gzInt.getModelList()
    if 'robot0' not in models:
        gzInt.spawnURDF('robot0', getRobotURDF('robot0', rospy.get_param('~robotURDFFile')), [0,0,0])

    #Wait for the heightmap to load and render from disk...
    #Must be a better way than busy wai
    rospy.sleep(10.0)
        
    gzInt.pausePhysics()
    
    #Fire up the master control node to run our scenarios
    ctl = ControlNode()
    ctl.runTasks()
    
    return


if __name__ == "__main__":
    """If this script is run as stand alone then call main() function."""
    main();
